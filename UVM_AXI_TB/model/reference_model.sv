//=============================================================================
// Reference Model: models interconnect behavior
// - Address decode (routing prediction)
// - 4KB Split prediction (cross_4k_if behavior)
// - ID extension (c4k prefix + mst index concatenation)
// - Outstanding transaction tracking
// - Write: updates shadow memory
// - Read:  reads from shadow memory, predicts RID
//=============================================================================
class reference_model extends uvm_component;
    `uvm_component_utils(reference_model)

    // Input: master-side AW/AR from channel monitors
    uvm_analysis_export #(axi_aw_item) aw_ap;
    uvm_analysis_export #(axi_ar_item) ar_ap;
    uvm_tlm_analysis_fifo #(axi_aw_item) aw_fifo;
    uvm_tlm_analysis_fifo #(axi_ar_item) ar_fifo;

    // Output: predicted slave-side AW/AR + expected B/R info
    uvm_analysis_port #(axi_aw_item) pred_aw_ap;   // predicted slave-side AWs
    uvm_analysis_port #(axi_ar_item) pred_ar_ap;   // predicted slave-side ARs

    // Shadow memory
    bit[31:0] shadow[bit[31:0]];

    // Outstanding transaction tables
    typedef struct {
        bit[1:0] mst_idx; bit[3:0] orig_id;
        bit[31:0] addr; bit[7:0] len; bit[2:0] size;
        bit will_split; int beats_done; bit[3:0] data_q[$];  // W data queue
    } wr_entry_t;
    typedef struct {
        bit[1:0] mst_idx; bit[3:0] orig_id;
        bit[31:0] addr; bit[7:0] len; bit[2:0] size;
        bit will_split; int beats_done;
    } rd_entry_t;

    wr_entry_t wr_tbl[int];  // key={mst_idx, orig_id}
    rd_entry_t rd_tbl[int];  // key={mst_idx, orig_id}

    int wr_cnt=0, rd_cnt=0, split_cnt=0;

    function new(string name="reference_model", uvm_component parent);
        super.new(name,parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        aw_ap=new("aw_ap",this); ar_ap=new("ar_ap",this);
        aw_fifo=new("aw_fifo",this); ar_fifo=new("ar_fifo",this);
        pred_aw_ap=new("pred_aw_ap",this); pred_ar_ap=new("pred_ar_ap",this);
        aw_ap.connect(aw_fifo.analysis_export);
        ar_ap.connect(ar_fifo.analysis_export);
    endfunction

    task run_phase(uvm_phase phase);
        fork process_aw(); process_ar(); join
    endtask

    //---- Process AW: predict split, compute expected S-side AW ----
    task process_aw();
        axi_aw_item t;
        addr_decoder::split_info_t split;
        axi_aw_item pred;
        int key;
        bit[31:0] beat_addr; int b;

        forever begin
            aw_fifo.get(t);
            if(!t.is_master_side) continue;

            wr_cnt++;
            key = {t.mst_id[1:0], t.id[3:0]};

            // Predict 4KB split
            split = addr_decoder::predict_split(t.addr, t.len, t.size);
            if(split.will_split) split_cnt++;

            // Store outstanding entry
            wr_tbl[key].mst_idx = t.mst_id[1:0];
            wr_tbl[key].orig_id = t.id[3:0];
            wr_tbl[key].addr    = t.addr;
            wr_tbl[key].len     = t.len;
            wr_tbl[key].size    = t.size;
            wr_tbl[key].will_split = split.will_split;
            wr_tbl[key].beats_done = 0;

            // Predict slave-side AW for sub-transaction 1
            pred = axi_aw_item::type_id::create("pred_aw1");
            pred.is_master_side = 0;
            pred.slv_id = addr_decoder::decode(split.sub1_addr);
            pred.mst_id = t.mst_id[1:0];
            pred.id     = t.id;   // original ID
            pred.addr   = split.sub1_addr;
            pred.len    = split.sub1_len;
            pred.size   = t.size;
            pred.burst  = t.burst;
            pred_aw_ap.write(pred);
            `uvm_info("REFM",$sformatf("AW split1: M[%0d] id=%0d -> S[%0d] addr=0x%08h len=%0d",
                      t.mst_id,t.id,pred.slv_id,split.sub1_addr,split.sub1_len),UVM_HIGH)

            // If split: predict slave-side AW for sub-transaction 2
            if(split.will_split) begin
                pred = axi_aw_item::type_id::create("pred_aw2");
                pred.is_master_side = 0;
                pred.slv_id = addr_decoder::decode(split.sub2_addr);
                pred.mst_id = t.mst_id[1:0];
                pred.id     = t.id;
                pred.addr   = split.sub2_addr;
                pred.len    = split.sub2_len;
                pred.size   = t.size;
                pred.burst  = t.burst;
                pred_aw_ap.write(pred);
                `uvm_info("REFM",$sformatf("AW split2: M[%0d] id=%0d -> S[%0d] addr=0x%08h len=%0d",
                          t.mst_id,t.id,pred.slv_id,split.sub2_addr,split.sub2_len),UVM_HIGH)
            end

            // Update shadow memory (predict write data effect)
            // sub-transaction 1
            for(b=0; b<=split.sub1_len; b++) begin
                beat_addr = split.sub1_addr + (b << t.size);
                shadow[beat_addr[31:2]] = 32'h0000_0000;  // placeholder
            end
            // sub-transaction 2 (if split)
            if(split.will_split) begin
                for(b=0; b<=split.sub2_len; b++) begin
                    beat_addr = split.sub2_addr + (b << t.size);
                    shadow[beat_addr[31:2]] = 32'h0000_0000;
                end
            end
        end
    endtask

    //---- Process AR: predict expected R path ----
    task process_ar();
        axi_ar_item t;
        addr_decoder::split_info_t split;
        axi_ar_item pred;
        int key;

        forever begin
            ar_fifo.get(t);
            if(!t.is_master_side) continue;

            rd_cnt++;
            key = {t.mst_id[1:0], t.id[3:0]};

            split = addr_decoder::predict_split(t.addr, t.len, t.size);

            rd_tbl[key].mst_idx = t.mst_id[1:0];
            rd_tbl[key].orig_id = t.id[3:0];
            rd_tbl[key].addr    = t.addr;
            rd_tbl[key].len     = t.len;
            rd_tbl[key].size    = t.size;
            rd_tbl[key].will_split = split.will_split;
            rd_tbl[key].beats_done = 0;

            // Predict slave-side AR
            pred = axi_ar_item::type_id::create("pred_ar");
            pred.is_master_side = 0;
            pred.slv_id = addr_decoder::decode(split.sub1_addr);
            pred.mst_id = t.mst_id[1:0];
            pred.id     = t.id;
            pred.addr   = t.addr;
            pred.len    = t.len;
            pred.size   = t.size;
            pred.burst  = t.burst;
            pred_ar_ap.write(pred);
        end
    endtask

    //---- Public API: predict expected RID from outstanding read table ----
    function bit[7:0] predict_rid(bit[1:0] mst_idx, bit[3:0] orig_id);
        return addr_decoder::predict_sid(mst_idx, 2'b00, orig_id);
    endfunction

    //---- Public API: look up shadow memory for expected read data ----
    function bit[31:0] read_shadow(bit[31:0] addr);
        if(shadow.exists(addr[31:2])) return shadow[addr[31:2]];
        return 32'hDEAD_BEEF;
    endfunction

    function void report_phase(uvm_phase phase);
        $display("REF MODEL: %0d writes, %0d reads, %0d 4KB splits predicted", wr_cnt, rd_cnt, split_cnt);
    endfunction

endclass
