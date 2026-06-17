//=============================================================================
// Unified AXI Scoreboard: comparison only
// - Route checking: refm prediction vs slave-side actual
// - Data checking: refm shadow vs R channel actual
// - Response checking: BRESP/RRESP, beat count validation
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
    `uvm_analysis_imp_decl(_slv_w)
    `uvm_analysis_imp_decl(_slv_b)
    `uvm_analysis_imp_decl(_slv_r)

    //---- Master-side imports ----
    uvm_analysis_imp_aw     #(axi_aw_item, axi_scoreboard) aw_imp;
    uvm_analysis_imp_w      #(axi_w_item,  axi_scoreboard) w_imp;
    uvm_analysis_imp_b      #(axi_b_item,  axi_scoreboard) b_imp;
    uvm_analysis_imp_ar     #(axi_ar_item, axi_scoreboard) ar_imp;
    uvm_analysis_imp_r      #(axi_r_item,  axi_scoreboard) r_imp;
    //---- Slave-side imports ----
    uvm_analysis_imp_slv_aw #(axi_aw_item, axi_scoreboard) slv_aw_imp;
    uvm_analysis_imp_slv_ar #(axi_ar_item, axi_scoreboard) slv_ar_imp;
    uvm_analysis_imp_slv_w  #(axi_w_item,  axi_scoreboard) slv_w_imp;
    uvm_analysis_imp_slv_b  #(axi_b_item,  axi_scoreboard) slv_b_imp;
    uvm_analysis_imp_slv_r  #(axi_r_item,  axi_scoreboard) slv_r_imp;

    //===== Master-side FIFOs =====
    uvm_tlm_analysis_fifo #(axi_aw_item) aw_r_fifo, aw_p_fifo;
    uvm_tlm_analysis_fifo #(axi_w_item)  w_p_fifo;
    uvm_tlm_analysis_fifo #(axi_b_item)  b_p_fifo;
    uvm_tlm_analysis_fifo #(axi_ar_item) ar_r_fifo, ar_d_fifo, ar_p_fifo;
    uvm_tlm_analysis_fifo #(axi_r_item)  r_d_fifo,  r_p_fifo;
    //===== Slave-side FIFOs =====
    uvm_tlm_analysis_fifo #(axi_aw_item) slv_aw_fifo;
    uvm_tlm_analysis_fifo #(axi_ar_item) slv_ar_fifo;
    uvm_tlm_analysis_fifo #(axi_w_item)  slv_w_fifo;
    uvm_tlm_analysis_fifo #(axi_b_item)  slv_b_fifo;
    uvm_tlm_analysis_fifo #(axi_r_item)  slv_r_fifo;

    //===== Reference Model handle (set by env) =====
    reference_model refm;

    //===== Route checking: bidirectional ID-based matching =====
    int route_err=0, route_cnt=0;
    axi_aw_item aw_m_pending[int];   // key={mst_id,id} → master-side AW
    axi_aw_item aw_s_pending[int];   // key={mst_id,id} → slave-side AW
    axi_ar_item ar_m_pending[int];   // key={mst_id,id} → master-side AR
    axi_ar_item ar_s_pending[int];   // key={mst_id,id} → slave-side AR
    int        ar_pair_exp[int];     // key → expected AR pairs (1=normal, 2=split)
    int        ar_pair_got[int];     // key → AR pairs completed

    //===== Data checking: AR tracking for R beat address =====
    int data_err=0, data_cnt=0;
    typedef struct { bit[31:0] addr; bit[7:0] len; bit[2:0] size; bit[1:0] burst; } ar_info_t;
    typedef ar_info_t ar_info_q[$];
    ar_info_q ar_info_pool[int];     // key={mst_id,id} → AR queue
    int       r_beat_cnt[int];       // key={mst_id,id} → beat index

    //===== Response checking =====
    typedef struct {
        bit[7:0] mst_id; bit[3:0] id; bit[7:0] len; int w_beat_cnt; int b_received;
        bit[31:0] m_w_data[$];  // M-side W data beats for cross-check
        bit[31:0] s_w_data[$];  // S-side W data beats for cross-check
    } wr_out_t;
    typedef struct {
        bit[7:0] mst_id; bit[3:0] id; bit[7:0] len; int r_beat_cnt;
        bit[31:0] m_r_data[$];  // M-side R data beats for cross-check
        bit[1:0]  m_r_resp[$];  // M-side RRESP
        bit[31:0] s_r_data[$];  // S-side R data beats for cross-check
        bit[1:0]  s_r_resp[$];  // S-side RRESP
    } rd_out_t;
    typedef wr_out_t wr_out_q[$];
    typedef rd_out_t rd_out_q[$];
    wr_out_q  wr_out[int];   // key={mst_id,id} → queue
    rd_out_q  rd_out[int];
    int wr_order[$], rd_order[$], b_order[$];
    int resp_err=0, wr_cnt=0, rd_cnt=0, reorder_cnt=0;

    //===== W/B/R cross-check: M-side vs S-side data comparison =====
    int w_xchk_err=0, b_xchk_err=0, r_xchk_err=0;
    int w_xchk_cnt=0, b_xchk_cnt=0, r_xchk_cnt=0;
    typedef int int_q[$];
    int_q slv_aw_order[4];    // per-slave[0..3] ordered key queue for S-side W matching

    // B cross-check: split-aware (2 S-side Bs merged → 1 M-side B)
    int        b_s_exp[int];     // key → expected S-side B count (1=normal, 2=split)
    int        b_s_got[int];     // key → received S-side B count
    bit[1:0]   b_s_mresp[int];   // key → merged S-side BRESP (bitwise OR)
    bit[3:0]   b_s_id[int];      // key → S-side BID (all subs share same orig_id)
    axi_b_item b_m_pending[int]; // key → M-side B

    // R cross-check: split-aware (2 S-side RLASTs merged → 1 M-side RLAST)
    int        r_s_exp[int];      // key → expected S-side RLAST count (1=normal, 2=split)
    int        r_s_got[int];      // key → received S-side RLAST count
    bit[1:0]   r_s_merge_resp[int]; // key → OR merged S-side RRESP
    bit        r_m_done[int];     // key → M-side RLAST received (data complete)
    bit        r_seen[int];       // key → transaction was registered (persists past cleanup)
    bit[3:0]   s_r_id[int];       // key → S-side RID
    bit[3:0]   m_r_id[int];       // key → M-side RID

    // AW-not-ready buffers: hold S-side W/B until M-side AW is processed
    axi_w_item slv_w_buf[int][$];  // key → buffered S-side W beats
    axi_b_item b_buf[int][$];      // key → buffered B items (AW or W not ready)

    function new(string n="axi_sb", uvm_component p); 
        super.new(n,p); 
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        aw_imp=new("aw_imp",this); w_imp=new("w_imp",this); b_imp=new("b_imp",this);
        ar_imp=new("ar_imp",this); r_imp=new("r_imp",this);
        slv_aw_imp=new("slv_aw_imp",this); slv_ar_imp=new("slv_ar_imp",this);
        slv_w_imp=new("slv_w_imp",this); slv_b_imp=new("slv_b_imp",this); slv_r_imp=new("slv_r_imp",this);
        aw_r_fifo=new("aw_r_fifo",this); aw_p_fifo=new("aw_p_fifo",this);
        w_p_fifo=new("w_p_fifo",this);
        b_p_fifo=new("b_p_fifo",this);
        ar_r_fifo=new("ar_r_fifo",this); ar_d_fifo=new("ar_d_fifo",this); ar_p_fifo=new("ar_p_fifo",this);
        r_d_fifo=new("r_d_fifo",this); r_p_fifo=new("r_p_fifo",this);
        slv_aw_fifo=new("slv_aw_fifo",this); slv_ar_fifo=new("slv_ar_fifo",this);
        slv_w_fifo=new("slv_w_fifo",this); slv_b_fifo=new("slv_b_fifo",this); slv_r_fifo=new("slv_r_fifo",this);
    endfunction

    // write_*: fan out to comparison FIFOs only (modeling FIFOs removed)
    function void write_aw(axi_aw_item t); 
        aw_r_fifo.write(t); 
        aw_p_fifo.write(t); 
    endfunction

    function void write_w(axi_w_item t);  
        w_p_fifo.write(t);  
    endfunction

    function void write_b(axi_b_item t);  
        b_p_fifo.write(t);  
    endfunction

    function void write_ar(axi_ar_item t); 
        ar_r_fifo.write(t); 
        ar_d_fifo.write(t); 
        ar_p_fifo.write(t); 
    endfunction

    function void write_r(axi_r_item t);  
        r_d_fifo.write(t);  
        r_p_fifo.write(t);  
    endfunction

    function void write_slv_aw(axi_aw_item t);
        slv_aw_fifo.write(t);
    endfunction

    function void write_slv_ar(axi_ar_item t);
        slv_ar_fifo.write(t);
    endfunction

    function void write_slv_w(axi_w_item t);
        slv_w_fifo.write(t);
    endfunction

    function void write_slv_b(axi_b_item t);
        slv_b_fifo.write(t);
    endfunction

    function void write_slv_r(axi_r_item t);
        slv_r_fifo.write(t);
    endfunction

    task run_phase(uvm_phase phase);
        fork
            // Route comparison
            collect_master_aw();  collect_slave_aw();
            collect_master_ar();  collect_slave_ar();
            // Data comparison (M-side)
            process_ar_data();  process_r_data();
            // Response comparison (M-side)
            process_aw_resp();  process_w_resp();  process_b_resp();
            process_ar_resp();  process_r_resp();
            // Slave-side monitoring
            process_slv_w();  process_slv_b();  process_slv_r();
        join
    endtask

    //=======================================================================
    // 1) Route comparison: refm prediction vs slave-side actual
    //=======================================================================

    task collect_master_aw();
        axi_aw_item t; int key;
        forever begin
            aw_r_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
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
            key = {t.mst_id[1:0], t.id[3:0]};
            if(aw_m_pending.exists(key)) begin
                do_aw_route_check(aw_m_pending[key], t);
                aw_m_pending.delete(key);
            end else begin
                aw_s_pending[key] = t;
            end
        end
    endtask

    function void do_aw_route_check(axi_aw_item m_t, axi_aw_item s_t);
        int key = {m_t.mst_id[1:0], m_t.id[3:0]};
        int exp;
        bit is_split;
        bit[31:0] sub2_addr;
        int sub2_slv;
        exp = refm.predict_slv_id(m_t.addr); route_cnt++;
        if(exp != s_t.slv_id) begin
            route_err++;
            `uvm_error("ROUTE",$sformatf("WR M[%0d] addr=0x%08h exp S%0d got S%0d",
                      m_t.mst_id, m_t.addr, exp, s_t.slv_id))
        end
        // W cross-check: record routing for S-side W beat ordering
        is_split = addr_decoder::crosses_4k(m_t.addr, m_t.len, m_t.size);
        // Push key to the current S-side AW's slave
        slv_aw_order[s_t.slv_id].push_back(key);
        // If split: also push key to sub2's slave (not covered by the paired S-side AW)
        if (is_split) begin
            sub2_addr = {m_t.addr[31:12] + 1'b1, 12'h0};
            sub2_slv  = refm.predict_slv_id(sub2_addr);
            if (sub2_slv != s_t.slv_id) begin
                slv_aw_order[sub2_slv].push_back(key);
            end
        end
        // B cross-check: track split for B response merge (1 or 2 S-side Bs)
        if (!b_s_exp.exists(key)) begin
            b_s_exp[key] = is_split ? 2 : 1;
        end
    endfunction

    task collect_master_ar();
        axi_ar_item t; int key;
        forever begin
            ar_r_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            if(ar_s_pending.exists(key)) begin
                do_ar_route_check(t, ar_s_pending[key]);
                // Only delete S-side pending when all expected pairs completed
                if (ar_pair_got[key] == ar_pair_exp[key]) begin
                    ar_s_pending.delete(key);
                    ar_pair_exp.delete(key);
                    ar_pair_got.delete(key);
                end
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
            key = {t.mst_id[1:0], t.id[3:0]};
            if(ar_m_pending.exists(key)) begin
                do_ar_route_check(ar_m_pending[key], t);
                // Only delete M-side pending when all expected pairs completed
                if (ar_pair_got[key] == ar_pair_exp[key]) begin
                    ar_m_pending.delete(key);
                    ar_pair_exp.delete(key);
                    ar_pair_got.delete(key);
                end
            end else begin
                ar_s_pending[key] = t;
            end
        end
    endtask

    function void do_ar_route_check(axi_ar_item m_t, axi_ar_item s_t);
        int key = {m_t.mst_id[1:0], m_t.id[3:0]};
        int exp = refm.predict_slv_id(m_t.addr); route_cnt++;
        if(exp != s_t.slv_id) begin route_err++;
            `uvm_error("ROUTE",$sformatf("RD M[%0d] addr=0x%08h exp S%0d got S%0d",
                      m_t.mst_id, m_t.addr, exp, s_t.slv_id))
        end
        // AR 1:2 pairing: track expected pairs for split reads
        if (!ar_pair_exp.exists(key)) begin
            ar_pair_exp[key] = addr_decoder::crosses_4k(m_t.addr, m_t.len, m_t.size) ? 2 : 1;
        end
        if (!ar_pair_got.exists(key))
            ar_pair_got[key] = 1;
        else
            ar_pair_got[key]++;
        // R cross-check: track split for RLAST/RRESP merge (1 or 2 S-side RLASTs)
        if (!r_s_exp.exists(key)) begin
            r_s_exp[key] = ar_pair_exp[key];
        end
    endfunction

    //=======================================================================
    // 2) Data comparison: AR→R address tracking + refm.shadow vs R actual
    //=======================================================================

    task process_ar_data();
        axi_ar_item t; int key; ar_info_t info;
        forever begin
            ar_d_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            info.addr=t.addr; info.len=t.len; info.size=t.size; info.burst=t.burst;
            ar_info_pool[key].push_back(info);
            if(!r_beat_cnt.exists(key)) r_beat_cnt[key]=0;
        end
    endtask

    task process_r_data();
        axi_r_item t; int key; ar_info_t info; bit[31:0] addr, exp_val; int bpb;
        forever begin
            r_d_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            if(ar_info_pool[key].size() == 0) begin 
                data_err++;
                `uvm_error("DATA","R beat with no matching AR"); continue;
            end
            info = ar_info_pool[key][0];   // oldest AR with this ID
            bpb = 1 << info.size;
            addr = (info.burst == 2'b00) ? info.addr : (info.addr + (r_beat_cnt[key] * bpb)); 
            data_cnt++;
            exp_val = refm.read_shadow(addr);
            if(t.data !== exp_val) begin data_err++;
                `uvm_error("DATA",$sformatf("M[%0d] addr=0x%08h exp=0x%08h act=0x%08h",
                          t.mst_id, addr, exp_val, t.data))
            end
            // R cross-check: store M-side R data in rd_out
            if(rd_out[key].size() > 0) begin
                rd_out[key][0].m_r_data.push_back(t.data);
                rd_out[key][0].m_r_resp.push_back(t.resp);
                if (!m_r_id.exists(key)) 
                    m_r_id[key] = t.id;
                if (m_r_id[key] !== t.id) begin r_xchk_err++;
                    `uvm_error("R_XCHK", $sformatf("M-side RID inconsistent key=%0d exp=%0d got=%0d",
                              key, m_r_id[key], t.id))
                end
                if(t.last) begin
                    r_m_done[key] = 1;
                    r_try_xchk(key);
                end
            end
            r_beat_cnt[key]++;
            if(t.last) begin
                if(r_beat_cnt[key] != info.len+1) begin
                    data_err++;
                    `uvm_error("DATA",$sformatf("RD beat cnt M[%0d] id=%0d exp=%0d got=%0d",
                              t.mst_id, t.id, info.len+1, r_beat_cnt[key]))
                end
                void'(ar_info_pool[key].pop_front());
                r_beat_cnt.delete(key);
            end
        end
    endtask

    //=======================================================================
    // 3) Response checking: BRESP/RRESP validation, beat count tracking
    //=======================================================================

    task process_aw_resp();
        axi_aw_item t; int key; wr_out_t entry;
        forever begin
            aw_p_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            entry.mst_id=t.mst_id; 
            entry.id=t.id;
            entry.len=t.len; 
            entry.w_beat_cnt=0; 
            entry.b_received=0;
            wr_out[key].push_back(entry);
            wr_order.push_back(key);
            wr_cnt++;
            // Replay any buffered S-side W beats that arrived before AW was ready
            if (slv_w_buf.exists(key)) begin
                foreach (slv_w_buf[key][i])
                    replay_slv_w_beat(slv_w_buf[key][i]);
                slv_w_buf[key].delete();
            end
            // Replay any buffered B items that arrived before AW was ready
            if (b_buf.exists(key)) begin
                foreach (b_buf[key][i])
                    replay_b_item(b_buf[key][i]);
                b_buf[key].delete();
            end
        end
    endtask

    task process_w_resp();
        axi_w_item t; int key; bit found;
        forever begin
            w_p_fifo.get(t);
            if(wr_order.size() == 0) continue;
            found=0;
            foreach(wr_order[i]) begin
                key=wr_order[i];
                foreach(wr_out[key][j]) begin
                    if(wr_out[key][j].mst_id == t.mst_id && wr_out[key][j].w_beat_cnt <= wr_out[key][j].len) begin
                        wr_out[key][j].w_beat_cnt++;
                        wr_out[key][j].m_w_data.push_back(t.data);  // store M-side W data
                        if(t.last && wr_out[key][j].w_beat_cnt != wr_out[key][j].len+1) begin
                            if(wr_out[key][j].w_beat_cnt < wr_out[key][j].len+1) begin
                                // 4KB split: remaining beats cross boundary
                                wr_out[key][j].len = wr_out[key][j].len - wr_out[key][j].w_beat_cnt;
                                wr_out[key][j].w_beat_cnt = 0;
                            end else begin
                                resp_err++;
                                `uvm_error("RESP",$sformatf("WLAST mismatch M[%0d] id=%0d",
                                          wr_out[key][j].mst_id,wr_out[key][j].id))
                            end
                        end
                        // W cross-check: trigger if S-side data also complete
                        if(t.last && wr_out[key][j].s_w_data.size() == wr_out[key][j].m_w_data.size())
                            w_do_xchk(key, j);
                        // Replay buffered B items that were waiting for W completion
                        if (t.last && b_buf.exists(key)) begin
                            foreach (b_buf[key][k])
                                replay_b_item(b_buf[key][k]);
                            b_buf[key].delete();
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
            key = {t.mst_id[1:0], t.id[3:0]};
            if(wr_out[key].size() == 0) begin
                // AW not yet processed → buffer for later replay
                b_buf[key].push_back(t);
                `uvm_info("RESP", $sformatf("B buffered M[%0d] id=%0d (AW not ready)", t.mst_id, t.id), UVM_HIGH)
                continue;
            end
            wr_out[key][0].b_received=1;
            if(t.resp!=2'b00) begin 
                resp_err++;
                `uvm_error("RESP",$sformatf("BRESP error M[%0d] id=%0d resp=%b",t.mst_id,t.id,t.resp))
            end
            if(wr_out[key][0].w_beat_cnt < wr_out[key][0].len+1) begin
                // W beats not yet complete → buffer for later replay
                b_buf[key].push_back(t);
                `uvm_info("RESP", $sformatf("B buffered M[%0d] id=%0d (W not done, w_beat=%0d)", t.mst_id, t.id, wr_out[key][0].w_beat_cnt), UVM_HIGH)
                continue;
            end
            // B cross-check: store M-side B, compare if all S-side Bs received
            if (b_s_got.exists(key) && b_s_got[key] == b_s_exp[key]) begin
                do_b_xchk(t, b_s_mresp[key], b_s_id[key], key);
            end else begin
                b_m_pending[key] = t;
            end
            b_order.push_back(key);
            // Only pop wr_out when S-side W data fully received (split-safe)
            if (wr_out[key][0].s_w_data.size() == wr_out[key][0].len + 1) begin
                void'(wr_out[key].pop_front());
                if(wr_out[key].size() == 0) begin
                    for(int i=0; i<wr_order.size(); i++)
                        if(wr_order[i]==key) begin wr_order.delete(i); break; end
                end
            end
        end
    endtask

    task process_ar_resp();
        axi_ar_item t; int key; rd_out_t entry;
        forever begin
            ar_p_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            entry.mst_id=t.mst_id; 
            entry.id=t.id;
            entry.len=t.len; 
            entry.r_beat_cnt=0;
            rd_out[key].push_back(entry);
            rd_order.push_back(key); r_seen[key] = 1; rd_cnt++;
        end
    endtask

    task process_r_resp();
        axi_r_item t; int key;
        forever begin
            r_p_fifo.get(t);
            key = {t.mst_id[1:0], t.id[3:0]};
            if(rd_out[key].size() == 0) begin
                // May have been cleaned up by r_try_xchk before process_r_resp ran
                if (!r_seen.exists(key)) begin
                    resp_err++;
                    `uvm_error("RESP",$sformatf("RID mismatch M[%0d] id=%0d",t.mst_id,t.id))
                end
                continue;
            end
            rd_out[key][0].r_beat_cnt++;
            if(t.resp!=2'b00) begin 
                resp_err++;
                `uvm_error("RESP",$sformatf("RRESP error M[%0d] id=%0d resp=%b",t.mst_id,t.id,t.resp))
            end
            if(t.last && rd_out[key][0].r_beat_cnt != rd_out[key][0].len+1) begin 
                resp_err++;
                `uvm_error("RESP",$sformatf("RLAST mismatch M[%0d] id=%0d",t.mst_id,t.id))
            end
            if(rd_order.size()>0 && rd_order[0]!=key) reorder_cnt++;
            if(t.last) begin
                // M-side RLAST: cleanup happens in r_try_xchk() when both sides done
            end
        end
    endtask

    //=======================================================================
    // 4) Slave-side monitoring + cross-check against M-side
    //=======================================================================

    // S-side W: cross-check W data against M-side using slv_aw_order for matching
    task process_slv_w();
        axi_w_item t;
        int key;
        bit found;
        forever begin
            slv_w_fifo.get(t);
            if (slv_aw_order[t.slv_id].size() == 0) begin
                `uvm_error("SLV_W", $sformatf("S[%0d] W beat with no pending AW in slv_aw_order", t.slv_id))
                continue;
            end
            key = slv_aw_order[t.slv_id][0];  // peek: key of current write transaction
            // Find the first incomplete entry in wr_out[key] for S-side data
            found = 0;
            if (wr_out.exists(key)) begin
                foreach (wr_out[key][j]) begin
                    if (wr_out[key][j].s_w_data.size() < wr_out[key][j].len + 1) begin
                        wr_out[key][j].s_w_data.push_back(t.data);
                        `uvm_info("SLV_W", $sformatf("S[%0d] W key=%0d beat=%0d data=0x%08h last=%b",
                                   t.slv_id, key, wr_out[key][j].s_w_data.size()-1, t.data, t.last), UVM_HIGH)
                        if (t.last) begin
                            void'(slv_aw_order[t.slv_id].pop_front());
                            // Trigger cross-check if M-side data also complete
                            if (wr_out[key][j].m_w_data.size() == wr_out[key][j].len + 1)
                                w_do_xchk(key, j);
                        end
                        found = 1;
                        break;
                    end
                end
            end
            if (!found) begin
                // AW not yet processed → buffer for later replay
                slv_w_buf[key].push_back(t);
                `uvm_info("SLV_W", $sformatf("S[%0d] W buffered key=%0d (AW not ready)", t.slv_id, key), UVM_HIGH)
            end
        end
    endtask

    // S-side B: merge BRESP (split-aware) then cross-check against M-side B
    task process_slv_b();
        axi_b_item t; int key;
        forever begin
            slv_b_fifo.get(t);
            key = {t.mst_id[1:0], t.id[3:0]};
            if (t.resp != 2'b00) begin
                `uvm_error("SLV_RESP", $sformatf("S[%0d] BRESP error mst=%0d resp=%b",
                          t.slv_id, t.mst_id, t.resp))
            end
            // B cross-check: accumulate S-side B responses
            if (!b_s_exp.exists(key)) 
                b_s_exp[key] = 1;  // default non-split if AW not yet routed

            if (!b_s_got.exists(key)) begin
                b_s_got[key]   = 1;
                b_s_mresp[key] = t.resp;
                b_s_id[key]    = t.id;
            end else begin
                b_s_got[key]++;
                b_s_mresp[key] = b_s_mresp[key] | t.resp;  // OR merge
                // verify all S-side BIDs share same orig_id
                if (b_s_id[key] !== t.id) begin 
                    b_xchk_err++;
                    `uvm_error("B_XCHK", $sformatf("S-side BID inconsistent key=%0d exp=%0d got=%0d",
                              key, b_s_id[key], t.id))
                end
            end
            `uvm_info("SLV_B", $sformatf("S[%0d] B mst=%0d id=%0d resp=%0d got=%0d/exp=%0d",
                       t.slv_id, t.mst_id, t.id, t.resp, b_s_got[key], b_s_exp[key]), UVM_HIGH)
            // All S-side Bs received → cross-check if M-side B also arrived
            if (b_s_got[key] == b_s_exp[key] && b_m_pending.exists(key)) begin
                do_b_xchk(b_m_pending[key], b_s_mresp[key], b_s_id[key], key);
                b_m_pending.delete(key);
            end
        end
    endtask

    // S-side R: cross-check R data + RRESP against M-side R
    task process_slv_r();
        axi_r_item t; int key;
        forever begin
            slv_r_fifo.get(t);
            key = {t.mst_id[1:0], t.id[3:0]};
            if (t.resp != 2'b00) begin
                `uvm_error("SLV_RESP", $sformatf("S[%0d] RRESP error mst=%0d resp=%b",
                          t.slv_id, t.mst_id, t.resp))
            end
            // R cross-check: accumulate S-side R beats + track RLASTs for merge
            if (rd_out[key].size() > 0) begin
                rd_out[key][0].s_r_data.push_back(t.data);
                rd_out[key][0].s_r_resp.push_back(t.resp);
                if (!s_r_id.exists(key))
                    s_r_id[key] = t.id;
                if (s_r_id[key] !== t.id) begin
                    r_xchk_err++;
                    `uvm_error("R_XCHK", $sformatf("S-side RID inconsistent key=%0d exp=%0d got=%0d",
                              key, s_r_id[key], t.id))
                end
                // Accumulate merged RRESP (OR across sub-transactions)
                if (!r_s_exp.exists(key)) r_s_exp[key] = 1;  // default if AR not yet routed
                if (!r_s_got.exists(key)) begin
                    r_s_got[key]        = 0;
                    r_s_merge_resp[key] = 2'b00;
                end
                r_s_merge_resp[key] = r_s_merge_resp[key] | t.resp;
                if (t.last) begin
                    r_s_got[key]++;
                    r_try_xchk(key);
                end
            end
            `uvm_info("SLV_R", $sformatf("S[%0d] R mst=%0d id=%0d beat=%0d data=0x%08h last=%b",
                       t.slv_id, t.mst_id, t.id, rd_out.exists(key) ? rd_out[key][0].s_r_data.size() : 0, t.data, t.last), UVM_HIGH)
        end
    endtask

    //---- Cross-check helper functions ----

    function void w_do_xchk(int key, int j);
        automatic int len = wr_out[key][j].len + 1;
        w_xchk_cnt++;
        for (int i = 0; i < len; i++) begin
            if (i < wr_out[key][j].m_w_data.size() && i < wr_out[key][j].s_w_data.size()) begin
                if (wr_out[key][j].m_w_data[i] !== wr_out[key][j].s_w_data[i]) begin
                    w_xchk_err++;
                    `uvm_error("W_XCHK", $sformatf("W data mismatch key=%0d beat=%0d M=0x%08h S=0x%08h",
                        key, i, wr_out[key][j].m_w_data[i], wr_out[key][j].s_w_data[i]))
                end
            end
        end
        // If B already received, clean up wr_out now (otherwise process_b_resp will do it)
        if (wr_out[key][j].b_received) begin
            void'(wr_out[key].pop_front());
            if (wr_out[key].size() == 0) begin
                for (int i = 0; i < wr_order.size(); i++)
                    if (wr_order[i] == key) begin wr_order.delete(i); break; end
            end
        end
    endfunction

    function void do_b_xchk(axi_b_item m_t, bit[1:0] s_merged_resp, bit[3:0] s_id, int key);
        b_xchk_cnt++;
        if (m_t.resp !== s_merged_resp) begin b_xchk_err++;
            `uvm_error("B_XCHK", $sformatf("BRESP mismatch key=%0d M=%0d S_merged=%0d",
                      key, m_t.resp, s_merged_resp))
        end
        if (m_t.id !== s_id) begin b_xchk_err++;
            `uvm_error("B_XCHK", $sformatf("BID mismatch key=%0d M=%0d S=%0d",
                      key, m_t.id, s_id))
        end
    endfunction

    function void r_do_xchk(int key, bit[1:0] merged_resp);
        automatic int len = rd_out[key][0].len + 1;
        bit[1:0] m_final_resp;
        r_xchk_cnt++;
        // RID: compare M-side vs S-side orig_id
        if (m_r_id.exists(key) && s_r_id.exists(key)) begin
            if (m_r_id[key] !== s_r_id[key]) begin
                r_xchk_err++;
                `uvm_error("R_XCHK", $sformatf("RID mismatch key=%0d M=%0d S=%0d",
                          key, m_r_id[key], s_r_id[key]))
            end
        end
        // RRESP: compare M-side final RRESP vs S-side merged RRESP
        if (rd_out[key][0].m_r_resp.size() > 0) begin
            m_final_resp = rd_out[key][0].m_r_resp[rd_out[key][0].m_r_resp.size()-1];
            if (m_final_resp !== merged_resp) begin
                r_xchk_err++;
                `uvm_error("R_XCHK", $sformatf("Merged RRESP mismatch key=%0d M=%0d S=%0d",
                          key, m_final_resp, merged_resp))
            end
        end
        // R data: beat-by-beat
        for (int i = 0; i < len; i++) begin
            if (i < rd_out[key][0].m_r_data.size() && i < rd_out[key][0].s_r_data.size()) begin
                if (rd_out[key][0].m_r_data[i] !== rd_out[key][0].s_r_data[i]) begin
                    r_xchk_err++;
                    `uvm_error("R_XCHK", $sformatf("R data mismatch key=%0d beat=%0d M=0x%08h S=0x%08h",
                        key, i, rd_out[key][0].m_r_data[i], rd_out[key][0].s_r_data[i]))
                end
            end
        end
    endfunction

    // R cross-check trigger: bilateral sync — only fires when M-side RLAST + all
    // S-side RLASTs are both received and both data queues are fully populated.
    function void r_try_xchk(int key);
        // Both sides must be done
        if (!r_m_done.exists(key) || !r_m_done[key]) return;
        if (!r_s_got.exists(key) || r_s_got[key] != r_s_exp[key]) return;
        if (rd_out[key].size() == 0) return;
        // Both data queues must be fully populated
        if (rd_out[key][0].m_r_data.size() != rd_out[key][0].len + 1) return;
        if (rd_out[key][0].s_r_data.size() != rd_out[key][0].len + 1) return;

        // All conditions met → cross-check
        r_do_xchk(key, r_s_merge_resp[key]);

        // Cleanup: rd_out + rd_order + all R state
        void'(rd_out[key].pop_front());
        if (rd_out[key].size() == 0) begin
            for (int i = 0; i < rd_order.size(); i++)
                if (rd_order[i] == key) begin rd_order.delete(i); break; end
        end
        r_s_exp.delete(key);
        r_s_got.delete(key);
        r_s_merge_resp.delete(key);
        r_m_done.delete(key);
        s_r_id.delete(key);
        m_r_id.delete(key);
    endfunction

    //---- Replay functions for buffered items (AW-not-ready recovery) ----

    function void replay_slv_w_beat(axi_w_item t);
        int key; bit found;
        if (slv_aw_order[t.slv_id].size() == 0) return;
        key = slv_aw_order[t.slv_id][0];
        found = 0;
        if (wr_out.exists(key)) begin
            foreach (wr_out[key][j]) begin
                if (wr_out[key][j].s_w_data.size() < wr_out[key][j].len + 1) begin
                    wr_out[key][j].s_w_data.push_back(t.data);
                    if (t.last) begin
                        void'(slv_aw_order[t.slv_id].pop_front());
                        if (wr_out[key][j].m_w_data.size() == wr_out[key][j].len + 1)
                            w_do_xchk(key, j);
                    end
                    found = 1;
                    break;
                end
            end
        end
        if (!found) begin
            slv_w_buf[key].push_back(t);  // still not ready, re-buffer
        end
    endfunction

    function void replay_b_item(axi_b_item t);
        int key; key = {t.mst_id[1:0], t.id[3:0]};
        if (wr_out[key].size() == 0) begin
            b_buf[key].push_back(t);  // still not ready
            return;
        end
        if (wr_out[key][0].w_beat_cnt < wr_out[key][0].len + 1) begin
            b_buf[key].push_back(t);  // W not done yet
            return;
        end
        // Conditions met → process B normally
        wr_out[key][0].b_received = 1;
        if (t.resp != 2'b00) begin resp_err++;
            `uvm_error("RESP", $sformatf("BRESP error M[%0d] id=%0d resp=%b", t.mst_id, t.id, t.resp))
        end
        // B cross-check
        if (b_s_got.exists(key) && b_s_got[key] == b_s_exp[key]) begin
            do_b_xchk(t, b_s_mresp[key], b_s_id[key], key);
        end else begin
            b_m_pending[key] = t;
        end
        b_order.push_back(key);
        // Pop wr_out if S-side W data fully received
        if (wr_out[key][0].s_w_data.size() == wr_out[key][0].len + 1) begin
            void'(wr_out[key].pop_front());
            if (wr_out[key].size() == 0) begin
                for (int i = 0; i < wr_order.size(); i++)
                    if (wr_order[i] == key) begin wr_order.delete(i); break; end
            end
        end
    endfunction

    function void report_phase(uvm_phase phase);
        $display("AXI SCOREBOARD:");
        $display("  Routing : %0d checks, %0d errors", route_cnt, route_err);
        $display("  Data    : %0d checks, %0d errors", data_cnt, data_err);
        $display("  Response: %0d writes, %0d reads, %0d errors, %0d reorder events",
                 wr_cnt, rd_cnt, resp_err, reorder_cnt);
        $display("  W Cross-Check: %0d txn, %0d errors", w_xchk_cnt, w_xchk_err);
        $display("  B Cross-Check: %0d txn, %0d errors", b_xchk_cnt, b_xchk_err);
        $display("  R Cross-Check: %0d txn, %0d errors", r_xchk_cnt, r_xchk_err);
    endfunction

endclass
