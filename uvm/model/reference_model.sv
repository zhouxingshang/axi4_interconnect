//=============================================================================
// Reference Model: golden predictor for interconnect behavior
// - Shadow memory (W data captured from M-side W beats)
// - AW→W ordering per master (handles outstanding + out-of-order)
// - W-before-AW buffering
// - 4KB split awareness
// - Address decode (routing prediction)
// - Public API for scoreboard to query expected values
//=============================================================================
class reference_model extends uvm_component;
    `uvm_component_utils(reference_model)

    // Input: master-side AW/AR/W from channel monitors
    uvm_analysis_export #(axi_aw_item) aw_ap;
    uvm_analysis_export #(axi_ar_item) ar_ap;
    uvm_analysis_export #(axi_w_item)  w_ap;
    uvm_tlm_analysis_fifo #(axi_aw_item) aw_fifo;
    uvm_tlm_analysis_fifo #(axi_ar_item) ar_fifo;
    uvm_tlm_analysis_fifo #(axi_w_item)  w_fifo;

    //---- Shadow memory: word-addressable expected data ----
    bit[31:0] shadow[bit[31:0]];

    //---- AW tracking: per-master ordered for W beat address calculation ----
    typedef struct {
        bit[3:0] id; bit[31:0] addr; bit[7:0] len; bit[2:0] size; bit[1:0] burst;
    } aw_info_t;
    aw_info_t aw_info_pool[int];   // key={mst_id[1:0], id[3:0]} → AW info
    typedef int int_q[$];
    int_q     aw_key_order[4];     // per-master[0..3] ordered key FIFO
    int       aw_beat_cnt[int];    // key={mst_id,id} → beats written so far

    //---- W-before-AW tolerance: buffer orphan W beats, replay when AW arrives ----
    axi_w_item w_pending[int][$];  // key=mst_id → buffered W beats

    //---- Statistics ----
    int wr_cnt=0, rd_cnt=0, split_cnt=0;

    function new(string name="reference_model", uvm_component parent);
        super.new(name,parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        aw_ap=new("aw_ap",this); ar_ap=new("ar_ap",this); w_ap=new("w_ap",this);
        aw_fifo=new("aw_fifo",this); ar_fifo=new("ar_fifo",this); w_fifo=new("w_fifo",this);
        aw_ap.connect(aw_fifo.analysis_export);
        ar_ap.connect(ar_fifo.analysis_export);
        w_ap.connect(w_fifo.analysis_export);
    endfunction

    task run_phase(uvm_phase phase);
        fork 
            process_aw(); 
            process_w(); 
            process_ar(); 
        join
    endtask

    //---- Process AW: register write transaction, predict split ----
    task process_aw();
        axi_aw_item t;
        addr_decoder::split_info_t split;
        int key;

        forever begin
            aw_fifo.get(t);
            if(!t.is_master_side) continue;

            wr_cnt++;
            key = {t.mst_id[1:0], t.id[3:0]};

            // Store AW info for W beat address calculation
            aw_info_pool[key].id    = t.id;
            aw_info_pool[key].addr  = t.addr;
            aw_info_pool[key].len   = t.len;
            aw_info_pool[key].size  = t.size;
            aw_info_pool[key].burst = t.burst;

            // Push to per-master order queue
            aw_key_order[t.mst_id[1:0]].push_back(key);

            // Replay any buffered W-before-AW beats for this master
            if (w_pending.exists(t.mst_id)) begin
                foreach (w_pending[t.mst_id][i])
                    replay_w_beat(w_pending[t.mst_id][i]);
                w_pending[t.mst_id].delete();
            end

            // Track 4KB splits
            split = addr_decoder::predict_split(t.addr, t.len, t.size);
            if(split.will_split) split_cnt++;

            `uvm_info("REFM", $sformatf("AW reg M[%0d] id=%0d addr=0x%08h len=%0d key=%0d",
                       t.mst_id, t.id, t.addr, t.len, key), UVM_HIGH)
        end
    endtask

    // Replay a buffered W beat using current AW order
    function void replay_w_beat(axi_w_item t);
        int key; 
        aw_info_t info; 
        bit[31:0] addr; 
        int bpb;
        if (aw_key_order[t.mst_id].size() == 0) return;
        key = aw_key_order[t.mst_id][0];
        info = aw_info_pool[key];
        if (!aw_beat_cnt.exists(key)) aw_beat_cnt[key] = 0;
        bpb = 1 << info.size;
        addr = (info.burst == 2'b00) ? info.addr : (info.addr + (aw_beat_cnt[key] * bpb));
        shadow[addr[31:2]] = t.data;
        aw_beat_cnt[key]++;
        if (t.last && aw_beat_cnt[key] == info.len + 1) begin
            aw_info_pool.delete(key);
            void'(aw_key_order[t.mst_id].pop_front());
            aw_beat_cnt.delete(key);
        end
    endfunction

    //---- Process W: write beat data into shadow memory ----
    task process_w();
        axi_w_item t;
        int key; aw_info_t info;
        bit[31:0] addr; int bpb;

        forever begin
            w_fifo.get(t);
            if (!t.is_master_side) continue;

            // W-before-AW: buffer until AW arrives
            if (aw_key_order[t.mst_id].size() == 0) begin
                w_pending[t.mst_id].push_back(t);
                `uvm_info("REFM", $sformatf("W buffered M[%0d] (before AW)", t.mst_id), UVM_HIGH)
                continue;
            end

            key = aw_key_order[t.mst_id][0];  // oldest pending AW
            info = aw_info_pool[key];
            if (!aw_beat_cnt.exists(key)) 
                aw_beat_cnt[key] = 0;

            bpb = 1 << info.size;
            addr = (info.burst == 2'b00) ? info.addr : (info.addr + (aw_beat_cnt[key] * bpb));
            shadow[addr[31:2]] = t.data;

            `uvm_info("REFM", $sformatf("W M[%0d] key=%0d beat=%0d addr=0x%08h data=0x%08h last=%b burst=%0d",
                       t.mst_id, key, aw_beat_cnt[key], addr, t.data, t.last, info.burst), UVM_HIGH)

            aw_beat_cnt[key]++;

            if (t.last) begin
                if (aw_beat_cnt[key] != info.len + 1) begin
                    // 4KB split aware: remaining beats cross boundary → update addr/len
                    int bytes_sent = aw_beat_cnt[key] * bpb;
                    bit crosses_4k = ((info.addr[11:0] + bytes_sent) >= 13'h1000);
                    if (crosses_4k) begin
                        info.addr = info.addr + bytes_sent;
                        info.len  = info.len - aw_beat_cnt[key];
                        aw_info_pool[key] = info;
                        aw_beat_cnt.delete(key);
                        `uvm_info("REFM", $sformatf("W 4KB split M[%0d] key=%0d new_addr=0x%08h new_len=%0d",
                                   t.mst_id, key, info.addr, info.len), UVM_HIGH)
                    end else begin
                        `uvm_error("REFM", $sformatf("WLAST but beats_done(%0d) != len+1(%0d) key=%0d",
                            aw_beat_cnt[key], info.len + 1, key))
                        aw_info_pool.delete(key);
                        void'(aw_key_order[t.mst_id].pop_front());
                        aw_beat_cnt.delete(key);
                    end
                end else begin
                    aw_info_pool.delete(key);
                    void'(aw_key_order[t.mst_id].pop_front());
                    aw_beat_cnt.delete(key);
                end
            end
        end
    endtask

    //---- Process AR: register read transaction ----
    task process_ar();
        axi_ar_item t;
        forever begin
            ar_fifo.get(t);
            if(!t.is_master_side) continue;
            rd_cnt++;
            `uvm_info("REFM", $sformatf("AR reg M[%0d] id=%0d addr=0x%08h len=%0d",
                       t.mst_id, t.id, t.addr, t.len), UVM_HIGH)
        end
    endtask

    //---- Public API: look up expected read data from shadow ----
    function bit[31:0] read_shadow(bit[31:0] addr);
        if(shadow.exists(addr[31:2])) return shadow[addr[31:2]];
        return 32'hDEAD_BEEF;
    endfunction

    //---- Public API: predict which slave an address routes to ----
    function int predict_slv_id(bit[31:0] addr);
        return addr_decoder::decode(addr);
    endfunction

    function void report_phase(uvm_phase phase);
        $display("REF MODEL: %0d writes, %0d reads, %0d 4KB splits predicted", wr_cnt, rd_cnt, split_cnt);
    endfunction

endclass
