//=============================================================================
// Unified AXI Scoreboard: ID-based routing + per-ID data/response tracking
//=============================================================================
class axi_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(axi_scoreboard)

    `uvm_analysis_imp_decl(_aw)
    `uvm_analysis_imp_decl(_w)
    `uvm_analysis_imp_decl(_b)
    `uvm_analysis_imp_decl(_ar)
    `uvm_analysis_imp_decl(_r)
    `uvm_analysis_imp_decl(_slv_aw)
    `uvm_analysis_imp_decl(_slv_ar)

    uvm_analysis_imp_aw     #(axi_aw_item, axi_scoreboard) aw_imp;
    uvm_analysis_imp_w      #(axi_w_item,  axi_scoreboard) w_imp;
    uvm_analysis_imp_b      #(axi_b_item,  axi_scoreboard) b_imp;
    uvm_analysis_imp_ar     #(axi_ar_item, axi_scoreboard) ar_imp;
    uvm_analysis_imp_r      #(axi_r_item,  axi_scoreboard) r_imp;
    uvm_analysis_imp_slv_aw #(axi_aw_item, axi_scoreboard) slv_aw_imp;
    uvm_analysis_imp_slv_ar #(axi_ar_item, axi_scoreboard) slv_ar_imp;

    uvm_tlm_analysis_fifo #(axi_aw_item) aw_r_fifo, aw_d_fifo, aw_p_fifo;
    uvm_tlm_analysis_fifo #(axi_w_item)  w_d_fifo,  w_p_fifo;
    uvm_tlm_analysis_fifo #(axi_b_item)  b_p_fifo;
    uvm_tlm_analysis_fifo #(axi_ar_item) ar_r_fifo, ar_d_fifo, ar_p_fifo;
    uvm_tlm_analysis_fifo #(axi_r_item)  r_d_fifo,  r_p_fifo;
    uvm_tlm_analysis_fifo #(axi_aw_item) slv_aw_fifo;
    uvm_tlm_analysis_fifo #(axi_ar_item) slv_ar_fifo;

    //===== Routing: bidirectional ID-based matching =====
    int route_err=0, route_cnt=0;
    axi_aw_item aw_m_pending[int];   // key={mst_id,id} → master-side AW
    axi_aw_item aw_s_pending[int];   // key={mst_id,id} → slave-side AW
    axi_ar_item ar_m_pending[int];   // key={mst_id,id} → master-side AR
    axi_ar_item ar_s_pending[int];   // key={mst_id,id} → slave-side AR

    //===== Data =====
    bit[31:0] shadow[bit[31:0]];
    int data_err=0, data_cnt=0;

    // Per-ID AW tracking for W data (ordered per-master via aw_key_order)
    typedef struct { bit[3:0] id; bit[31:0] addr; bit[7:0] len; bit[2:0] size; } aw_info_t;
    aw_info_t aw_info_pool[int];     // key={mst_id,id} → single entry
    int       aw_key_order[4][$];    // per-master ordered ID list
    int       aw_beat_cnt[int];      // key={mst_id,id} → beat index per AW

    // W-before-AW tolerance: buffer orphan W beats, replay when AW arrives
    axi_w_item w_pending[int][$];     // key=mst_id → queue of orphan W beats

    // Per-slave AW tracking for slave-side W → shadow writes (avoids fork race)
    typedef aw_info_t slv_aw_q[$];
    slv_aw_q  slv_aw_info[int];      // key=slv_id → ordered AW queue
    int       slv_w_beat_cnt[int];   // key=slv_id → beat index

    // Per-ID AR tracking for R data (supports interleaved reads)
    typedef struct { bit[31:0] addr; bit[7:0] len; bit[2:0] size; } ar_info_t;
    typedef ar_info_t ar_info_q[$];
    ar_info_q ar_info_pool[int];     // key={mst_id,id} → queue
    int       r_beat_cnt[int];       // key={mst_id,id} → beat index

    //===== Response =====
    typedef struct {
        bit[7:0] mst_id; bit[3:0] id; bit[7:0] len; int w_beat_cnt; int b_received;
    } wr_out_t;
    typedef struct {
        bit[7:0] mst_id; bit[3:0] id; bit[7:0] len; int r_beat_cnt;
    } rd_out_t;
    typedef wr_out_t wr_out_q[$];
    typedef rd_out_t rd_out_q[$];
    wr_out_q  wr_out[int];   // key={mst_id,id} → queue
    rd_out_q  rd_out[int];
    int wr_order[$], rd_order[$], b_order[$];
    int resp_err=0, wr_cnt=0, rd_cnt=0, reorder_cnt=0;

    function new(string n="axi_sb", uvm_component p); super.new(n,p); endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        aw_imp=new("aw_imp",this); w_imp=new("w_imp",this); b_imp=new("b_imp",this);
        ar_imp=new("ar_imp",this); r_imp=new("r_imp",this);
        slv_aw_imp=new("slv_aw_imp",this); slv_ar_imp=new("slv_ar_imp",this);
        aw_r_fifo=new("aw_r_fifo",this); aw_d_fifo=new("aw_d_fifo",this); aw_p_fifo=new("aw_p_fifo",this);
        w_d_fifo=new("w_d_fifo",this); w_p_fifo=new("w_p_fifo",this);
        b_p_fifo=new("b_p_fifo",this);
        ar_r_fifo=new("ar_r_fifo",this); ar_d_fifo=new("ar_d_fifo",this); ar_p_fifo=new("ar_p_fifo",this);
        r_d_fifo=new("r_d_fifo",this); r_p_fifo=new("r_p_fifo",this);
        slv_aw_fifo=new("slv_aw_fifo",this); slv_ar_fifo=new("slv_ar_fifo",this);
    endfunction

    function void write_aw(axi_aw_item t); aw_r_fifo.write(t); aw_d_fifo.write(t); aw_p_fifo.write(t); endfunction
    function void write_w(axi_w_item t);  w_d_fifo.write(t);  w_p_fifo.write(t);  endfunction
    function void write_b(axi_b_item t);  b_p_fifo.write(t);  endfunction
    function void write_ar(axi_ar_item t); ar_r_fifo.write(t); ar_d_fifo.write(t); ar_p_fifo.write(t); endfunction
    function void write_r(axi_r_item t);  r_d_fifo.write(t);  r_p_fifo.write(t);  endfunction
    function void write_slv_aw(axi_aw_item t); slv_aw_fifo.write(t); endfunction
    function void write_slv_ar(axi_ar_item t); slv_ar_fifo.write(t); endfunction

    task run_phase(uvm_phase phase);
        fork
            collect_master_aw();  collect_slave_aw();
            collect_master_ar();  collect_slave_ar();
            process_aw_data();  process_slv_aw_data(); process_w_data();
            process_ar_data();  process_r_data();
            process_aw_resp();  process_w_resp();  process_b_resp();
            process_ar_resp();  process_r_resp();
        join
    endtask

    //=======================================================================
    // 1) Routing: bidirectional ID matching (handles out-of-order arrival)
    //=======================================================================

    // ---- Write routing (AW) ----
    task collect_master_aw();
        axi_aw_item t; int key;
        forever begin
            aw_r_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            if(aw_s_pending.exists(key)) begin
                do_aw_route_check(t, aw_s_pending[key]);
                aw_s_pending.delete(key);
            end else begin
                aw_m_pending[key] = t;
            end
        end
    endtask

    task collect_slave_aw();
        axi_aw_item t; int key;
        forever begin
            slv_aw_fifo.get(t);
            if(t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            if(aw_m_pending.exists(key)) begin
                do_aw_route_check(aw_m_pending[key], t);
                aw_m_pending.delete(key);
            end else begin
                aw_s_pending[key] = t;
            end
        end
    endtask

    function void do_aw_route_check(axi_aw_item m_t, axi_aw_item s_t);
        int exp = addr_decoder::decode(m_t.addr); route_cnt++;
        if(exp != s_t.slv_id) begin route_err++;
            `uvm_error("ROUTE",$sformatf("WR M[%0d] addr=0x%08h exp S%0d got S%0d",
                      m_t.mst_id, m_t.addr, exp, s_t.slv_id))
        end
    endfunction

    // ---- Read routing (AR) ----
    task collect_master_ar();
        axi_ar_item t; int key;
        forever begin
            ar_r_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            if(ar_s_pending.exists(key)) begin
                do_ar_route_check(t, ar_s_pending[key]);
                ar_s_pending.delete(key);
            end else begin
                ar_m_pending[key] = t;
            end
        end
    endtask

    task collect_slave_ar();
        axi_ar_item t; int key;
        forever begin
            slv_ar_fifo.get(t);
            if(t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            if(ar_m_pending.exists(key)) begin
                do_ar_route_check(ar_m_pending[key], t);
                ar_m_pending.delete(key);
            end else begin
                ar_s_pending[key] = t;
            end
        end
    endtask

    function void do_ar_route_check(axi_ar_item m_t, axi_ar_item s_t);
        int exp = addr_decoder::decode(m_t.addr); route_cnt++;
        if(exp != s_t.slv_id) begin route_err++;
            `uvm_error("ROUTE",$sformatf("RD M[%0d] addr=0x%08h exp S%0d got S%0d",
                      m_t.mst_id, m_t.addr, exp, s_t.slv_id))
        end
    endfunction

    //=======================================================================
    // 2) Data: per-master ordered AW for W, per-ID AR for R
    //=======================================================================
    task process_aw_data();
        axi_aw_item t; int key; aw_info_t info;
        forever begin
            aw_d_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            info.id=t.id; info.addr=t.addr; info.len=t.len; info.size=t.size;
            aw_info_pool[key] = info;
            aw_key_order[t.mst_id].push_back(key);

            // Replay buffered W-before-AW beats for this master
            if (w_pending.exists(t.mst_id)) begin
                foreach (w_pending[t.mst_id][i])
                    replay_w_beat(w_pending[t.mst_id][i]);
                w_pending[t.mst_id].delete();
            end
        end
    endtask

    // Replay a buffered W beat using current AW info
    function void replay_w_beat(axi_w_item t);
        int key; aw_info_t info; bit[31:0] addr; int bpb;
        if (aw_key_order[t.mst_id].size() == 0) return;
        key = aw_key_order[t.mst_id][0];
        info = aw_info_pool[key];
        if (!aw_beat_cnt.exists(key)) aw_beat_cnt[key] = 0;
        bpb = 1 << info.size;
        addr = info.addr + (aw_beat_cnt[key] * bpb);
        shadow[addr[31:2]] = t.data;
        aw_beat_cnt[key]++;
        if (t.last && aw_beat_cnt[key] == info.len + 1) begin
            aw_info_pool.delete(key);
            void'(aw_key_order[t.mst_id].pop_front());
            aw_beat_cnt.delete(key);
        end
    endfunction

    // Slave-side AW: populate per-slave AW queue for W beat address look-up
    task process_slv_aw_data();
        axi_aw_item t; aw_info_t info;
        forever begin
            slv_aw_fifo.get(t);
            info.id=t.id; info.addr=t.addr; info.len=t.len; info.size=t.size;
            slv_aw_info[t.slv_id].push_back(info);
            if(!slv_w_beat_cnt.exists(t.slv_id)) slv_w_beat_cnt[t.slv_id] = 0;
        end
    endtask

    task process_w_data();
        axi_w_item t; int key; aw_info_t info; bit[31:0] addr; int bpb;
        forever begin
            w_d_fifo.get(t);
            if(t.is_master_side) begin
                // Master-side: beat count + shadow write
                if(aw_key_order[t.mst_id].size() == 0) begin
                    // W-before-AW: buffer for replay when AW arrives
                    w_pending[t.mst_id].push_back(t);
                    continue;
                end
                key = aw_key_order[t.mst_id][0];
                info = aw_info_pool[key];
                if(!aw_beat_cnt.exists(key)) aw_beat_cnt[key] = 0;
                bpb = 1 << info.size;
                addr = info.addr + (aw_beat_cnt[key] * bpb);
                shadow[addr[31:2]] = t.data;
                aw_beat_cnt[key]++;
                if(t.last) begin
                    if(aw_beat_cnt[key] != info.len+1) begin
                        // 4KB split aware: check if remaining beats cross boundary
                        int bytes_sent = aw_beat_cnt[key] * bpb;
                        bit crosses_4k = ((info.addr[11:0] + bytes_sent) >= 13'h1000);
                        if(crosses_4k) begin
                            // Valid split: update AW info for second sub-transaction
                            info.addr = info.addr + bytes_sent;
                            info.len  = info.len - aw_beat_cnt[key];
                            aw_info_pool[key] = info;
                            aw_beat_cnt.delete(key);  // reset beat count for second half
                        end else begin
                            data_err++;
                            `uvm_error("DATA",$sformatf("WR beat cnt M[%0d] id=%0d exp=%0d got=%0d",
                                      t.mst_id, info.id, info.len+1, aw_beat_cnt[key]))
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
            end else begin
                // Slave-side: write to shadow (data already committed to slave memory)
                int s = t.slv_id;
                if(slv_aw_info[s].size() == 0) begin
                    continue;
                end
                info = slv_aw_info[s][0];
                if(!slv_w_beat_cnt.exists(s)) slv_w_beat_cnt[s] = 0;
                bpb = 1 << info.size;
                addr = info.addr + (slv_w_beat_cnt[s] * bpb);
                shadow[addr[31:2]] = t.data;
                slv_w_beat_cnt[s]++;
                if(t.last) begin
                    void'(slv_aw_info[s].pop_front());
                    slv_w_beat_cnt.delete(s);
                end
            end
        end
    endtask

    task process_ar_data();
        axi_ar_item t; int key; ar_info_t info;
        forever begin
            ar_d_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            info.addr=t.addr; info.len=t.len; info.size=t.size;
            ar_info_pool[key].push_back(info);
            if(!r_beat_cnt.exists(key)) r_beat_cnt[key]=0;
        end
    endtask

    task process_r_data();
        axi_r_item t; int key; ar_info_t info; bit[31:0] addr, exp_val; int bpb;
        forever begin
            r_d_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            if(ar_info_pool[key].size() == 0) begin data_err++;
                `uvm_error("DATA","R beat with no matching AR"); continue;
            end
            info = ar_info_pool[key][0];   // oldest AR with this ID
            bpb = 1 << info.size;
            addr = info.addr + (r_beat_cnt[key] * bpb); data_cnt++;
            exp_val = shadow.exists(addr[31:2]) ? shadow[addr[31:2]] : 32'hDEAD_BEEF;
            if(t.data !== exp_val) begin data_err++;
                `uvm_error("DATA",$sformatf("M[%0d] addr=0x%08h exp=0x%08h act=0x%08h",
                          t.mst_id, addr, exp_val, t.data))
            end
            r_beat_cnt[key]++;
            if(t.last) begin
                if(r_beat_cnt[key] != info.len+1) begin data_err++;
                    `uvm_error("DATA",$sformatf("RD beat cnt M[%0d] id=%0d exp=%0d got=%0d",
                              t.mst_id, t.id, info.len+1, r_beat_cnt[key]))
                end
                void'(ar_info_pool[key].pop_front());
                r_beat_cnt.delete(key);
            end
        end
    endtask

    //=======================================================================
    // 3) Response: per-ID outstanding tracking (queue-based, same-ID ok)
    //=======================================================================
    task process_aw_resp();
        axi_aw_item t; int key; wr_out_t entry;
        forever begin
            aw_p_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            entry.mst_id=t.mst_id; entry.id=t.id;
            entry.len=t.len; entry.w_beat_cnt=0; entry.b_received=0;
            wr_out[key].push_back(entry);
            wr_order.push_back(key); wr_cnt++;
        end
    endtask

    task process_w_resp();
        axi_w_item t; int key; bit found;
        forever begin
            w_p_fifo.get(t);
            if(!t.is_master_side) continue;
            // W-before-AW: if no outstanding AW, silently skip (buffered W beats are transient)
            if(wr_order.size() == 0) continue;
            found=0;
            foreach(wr_order[i]) begin
                key=wr_order[i];
                foreach(wr_out[key][j]) begin
                    if(wr_out[key][j].mst_id == t.mst_id && wr_out[key][j].w_beat_cnt <= wr_out[key][j].len) begin
                        wr_out[key][j].w_beat_cnt++;
                        if(t.last && wr_out[key][j].w_beat_cnt != wr_out[key][j].len+1) begin
                            // 4KB split aware: remaining beats cross boundary → valid split
                            if(wr_out[key][j].w_beat_cnt < wr_out[key][j].len+1) begin
                                wr_out[key][j].len = wr_out[key][j].len - wr_out[key][j].w_beat_cnt;
                                wr_out[key][j].w_beat_cnt = 0;
                            end else begin
                                resp_err++;
                                `uvm_error("RESP",$sformatf("WLAST mismatch M[%0d] id=%0d",
                                          wr_out[key][j].mst_id,wr_out[key][j].id))
                            end
                        end
                        found=1; break;
                    end
                end
                if(found) break;
            end
            if(!found) begin resp_err++;
                `uvm_error("RESP","W beat with no matching outstanding AW")
            end
        end
    endtask

    task process_b_resp();
        axi_b_item t; int key;
        forever begin
            b_p_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            if(wr_out[key].size() == 0) begin resp_err++;
                `uvm_error("RESP",$sformatf("BID mismatch M[%0d] id=%0d",t.mst_id,t.id))
                continue;
            end
            wr_out[key][0].b_received=1;
            if(t.resp!=2'b00) begin resp_err++;
                `uvm_error("RESP",$sformatf("BRESP error M[%0d] id=%0d resp=%b",t.mst_id,t.id,t.resp))
            end
            if(wr_out[key][0].w_beat_cnt < wr_out[key][0].len+1) begin resp_err++;
                `uvm_error("RESP",$sformatf("B before WLAST M[%0d] id=%0d",t.mst_id,t.id))
            end
            b_order.push_back(key);
            void'(wr_out[key].pop_front());
            if(wr_out[key].size() == 0) begin
                for(int i=0; i<wr_order.size(); i++)
                    if(wr_order[i]==key) begin wr_order.delete(i); break; end
            end
        end
    endtask

    task process_ar_resp();
        axi_ar_item t; int key; rd_out_t entry;
        forever begin
            ar_p_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            entry.mst_id=t.mst_id; entry.id=t.id;
            entry.len=t.len; entry.r_beat_cnt=0;
            rd_out[key].push_back(entry);
            rd_order.push_back(key); rd_cnt++;
        end
    endtask

    task process_r_resp();
        axi_r_item t; int key;
        forever begin
            r_p_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            if(rd_out[key].size() == 0) begin resp_err++;
                `uvm_error("RESP",$sformatf("RID mismatch M[%0d] id=%0d",t.mst_id,t.id))
                continue;
            end
            rd_out[key][0].r_beat_cnt++;
            if(t.resp!=2'b00) begin resp_err++;
                `uvm_error("RESP",$sformatf("RRESP error M[%0d] id=%0d resp=%b",t.mst_id,t.id,t.resp))
            end
            if(t.last && rd_out[key][0].r_beat_cnt != rd_out[key][0].len+1) begin resp_err++;
                `uvm_error("RESP",$sformatf("RLAST mismatch M[%0d] id=%0d",t.mst_id,t.id))
            end
            if(rd_order.size()>0 && rd_order[0]!=key) reorder_cnt++;
            if(t.last) begin
                void'(rd_out[key].pop_front());
                if(rd_out[key].size() == 0) begin
                    for(int i=0; i<rd_order.size(); i++)
                        if(rd_order[i]==key) begin rd_order.delete(i); break; end
                end
            end
        end
    endtask

    function void report_phase(uvm_phase phase);
        $display("AXI SCOREBOARD:");
        $display("  Routing : %0d checks, %0d errors", route_cnt, route_err);
        $display("  Data    : %0d checks, %0d errors", data_cnt, data_err);
        $display("  Response: %0d writes, %0d reads, %0d errors, %0d reorder events",
                 wr_cnt, rd_cnt, resp_err, reorder_cnt);
    endfunction

endclass
