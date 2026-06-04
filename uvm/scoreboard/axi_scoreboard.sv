//=============================================================================
// Unified AXI Scoreboard: routing + data integrity + response tracking
// Uses per-master AW/AR queues + {mst_id} matching to avoid fork-join bugs
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

    //===== Routing =====
    int route_err=0, route_cnt=0;

    //===== Data =====
    bit[31:0] shadow[bit[31:0]];
    int data_err=0, data_cnt=0;

    // Per-master AW/AR queues: avoid fork-join pairing bugs
    typedef struct { bit[3:0] id; bit[31:0] addr; bit[7:0] len; bit[2:0] size; } aw_info_t;
    typedef struct { bit[3:0] id; bit[31:0] addr; bit[7:0] len; bit[2:0] size; } ar_info_t;
    aw_info_t aw_data_q[4][$];
    ar_info_t ar_data_q[4][$];
    int       aw_beat_cnt[4];   // beat index within current AW transaction

    //===== Response =====
    typedef struct {
        bit[7:0] mst_id; bit[3:0] id; bit[7:0] len; int w_beat_cnt; int b_received;
    } wr_out_t;
    typedef struct {
        bit[7:0] mst_id; bit[3:0] id; bit[7:0] len; int r_beat_cnt;
    } rd_out_t;
    wr_out_t wr_out[int];
    rd_out_t rd_out[int];
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
            check_wr_routing(); check_rd_routing();
            process_aw_data();  process_w_data();
            process_ar_data();  process_r_data();
            process_aw_resp();  process_w_resp();  process_b_resp();
            process_ar_resp();  process_r_resp();
        join
    endtask

    //=======================================================================
    // 1) Routing
    //=======================================================================
    task check_wr_routing();
        axi_aw_item m_t, s_t; int exp;
        forever begin
            fork aw_r_fifo.get(m_t); slv_aw_fifo.get(s_t); join
            exp = addr_decoder::decode(m_t.addr); route_cnt++;
            if(exp != s_t.slv_id) begin route_err++;
                `uvm_error("ROUTE",$sformatf("WR M[%0d] addr=0x%08h exp S%0d got S%0d",m_t.mst_id,m_t.addr,exp,s_t.slv_id))
            end
        end
    endtask
    task check_rd_routing();
        axi_ar_item m_t, s_t; int exp;
        forever begin
            fork ar_r_fifo.get(m_t); slv_ar_fifo.get(s_t); join
            exp = addr_decoder::decode(m_t.addr); route_cnt++;
            if(exp != s_t.slv_id) begin route_err++;
                `uvm_error("ROUTE",$sformatf("RD M[%0d] addr=0x%08h exp S%0d got S%0d",m_t.mst_id,m_t.addr,exp,s_t.slv_id))
            end
        end
    endtask

    //=======================================================================
    // 2) Data: per-master AW/W queues — no fork-join pairing
    //=======================================================================
    task process_aw_data();
        axi_aw_item t; aw_info_t info;
        forever begin
            aw_d_fifo.get(t);
            if(!t.is_master_side) continue;
            info.id=t.id; info.addr=t.addr; info.len=t.len; info.size=t.size;
            aw_data_q[t.mst_id].push_back(info);
        end
    endtask

    task process_w_data();
        axi_w_item t; int m; aw_info_t info; bit[31:0] addr; int bpb;
        forever begin
            w_d_fifo.get(t);
            if(!t.is_master_side) continue;
            m = t.mst_id;
            if(aw_data_q[m].size() == 0) begin data_err++;
                `uvm_error("DATA","W beat with no matching AW"); continue;
            end
            info = aw_data_q[m][0];
            bpb = 1 << info.size;
            addr = info.addr + (aw_beat_cnt[m] * bpb);
            shadow[addr[31:2]] = t.data;
            aw_beat_cnt[m]++;
            if(t.last) begin
                if(aw_beat_cnt[m] != info.len+1) begin data_err++;
                    `uvm_error("DATA",$sformatf("WR beat cnt M[%0d] id=%0d exp=%0d got=%0d",
                              m, info.id, info.len+1, aw_beat_cnt[m]))
                end
                void'(aw_data_q[m].pop_front());
                aw_beat_cnt[m] = 0;
            end
        end
    endtask

    task process_ar_data();
        axi_ar_item t; ar_info_t info;
        forever begin
            ar_d_fifo.get(t);
            if(!t.is_master_side) continue;
            info.id=t.id; info.addr=t.addr; info.len=t.len; info.size=t.size;
            ar_data_q[t.mst_id].push_back(info);
        end
    endtask

    task process_r_data();
        axi_r_item t; int m; ar_info_t info; bit[31:0] addr, exp;
        int r_beat[int]; int key; int bpb;
        forever begin
            r_d_fifo.get(t);
            if(!t.is_master_side) continue;
            m = t.mst_id;
            key = {8'(m), t.id};
            if(!r_beat.exists(key)) r_beat[key]=0;
            // Match R by mst_id: use first pending AR for this master
            if(ar_data_q[m].size() == 0) begin data_err++;
                `uvm_error("DATA","R beat with no matching AR"); continue;
            end
            info = ar_data_q[m][0];
            bpb = 1 << info.size;
            addr = info.addr + (r_beat[key] * bpb); data_cnt++;
            exp = shadow.exists(addr[31:2]) ? shadow[addr[31:2]] : 32'hDEAD_BEEF;
            if(t.data !== exp) begin data_err++;
                `uvm_error("DATA",$sformatf("M[%0d] addr=0x%08h exp=0x%08h act=0x%08h",m,addr,exp,t.data))
            end
            r_beat[key]++;
            if(t.last) begin
                if(r_beat[key] != info.len+1) begin data_err++;
                    `uvm_error("DATA",$sformatf("RD beat cnt M[%0d] id=%0d",m,info.id))
                end
                void'(ar_data_q[m].pop_front());
                r_beat.delete(key);
            end
        end
    endtask

    //=======================================================================
    // 3) Response: per-master outstanding tracking
    //=======================================================================
    task process_aw_resp();
        axi_aw_item t; int key;
        forever begin
            aw_p_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            if(wr_out.exists(key)) begin resp_err++;
                `uvm_error("RESP",$sformatf("DUPLICATE AW M[%0d] id=%0d",t.mst_id,t.id))
            end
            wr_out[key].mst_id=t.mst_id; wr_out[key].id=t.id;
            wr_out[key].len=t.len; wr_out[key].w_beat_cnt=0; wr_out[key].b_received=0;
            wr_order.push_back(key); wr_cnt++;
        end
    endtask

    task process_w_resp();
        axi_w_item t; int key; bit found;
        forever begin
            w_p_fifo.get(t);
            if(!t.is_master_side) continue;
            found=0;
            // Search wr_order for first pending write belonging to this master
            foreach(wr_order[i]) begin
                key=wr_order[i];
                if(wr_out[key].mst_id == t.mst_id && wr_out[key].w_beat_cnt <= wr_out[key].len) begin
                    wr_out[key].w_beat_cnt++;
                    if(t.last && wr_out[key].w_beat_cnt != wr_out[key].len+1) begin resp_err++;
                        `uvm_error("RESP",$sformatf("WLAST mismatch M[%0d] id=%0d",
                                  wr_out[key].mst_id,wr_out[key].id))
                    end
                    found=1; break;
                end
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
            if(!wr_out.exists(key)) begin resp_err++;
                `uvm_error("RESP",$sformatf("BID mismatch M[%0d] id=%0d",t.mst_id,t.id))
                continue;
            end
            wr_out[key].b_received=1;
            if(t.resp!=2'b00) begin resp_err++;
                `uvm_error("RESP",$sformatf("BRESP error M[%0d] id=%0d resp=%b",t.mst_id,t.id,t.resp))
            end
            b_order.push_back(key);
            if(wr_out[key].w_beat_cnt < wr_out[key].len+1) begin resp_err++;
                `uvm_error("RESP",$sformatf("B before WLAST M[%0d] id=%0d",t.mst_id,t.id))
            end
        end
    endtask

    task process_ar_resp();
        axi_ar_item t; int key;
        forever begin
            ar_p_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            rd_out[key].mst_id=t.mst_id; rd_out[key].id=t.id;
            rd_out[key].len=t.len; rd_out[key].r_beat_cnt=0;
            rd_order.push_back(key); rd_cnt++;
        end
    endtask

    task process_r_resp();
        axi_r_item t; int key;
        forever begin
            r_p_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            if(!rd_out.exists(key)) begin resp_err++;
                `uvm_error("RESP",$sformatf("RID mismatch M[%0d] id=%0d",t.mst_id,t.id))
                continue;
            end
            rd_out[key].r_beat_cnt++;
            if(t.resp!=2'b00) begin resp_err++;
                `uvm_error("RESP",$sformatf("RRESP error M[%0d] id=%0d resp=%b",t.mst_id,t.id,t.resp))
            end
            if(t.last && rd_out[key].r_beat_cnt != rd_out[key].len+1) begin resp_err++;
                `uvm_error("RESP",$sformatf("RLAST mismatch M[%0d] id=%0d",t.mst_id,t.id))
            end
            if(rd_order.size()>0 && rd_order[0]!=key) reorder_cnt++;
            if(t.last) begin
                for(int i=0; i<rd_order.size(); i++)
                    if(rd_order[i]==key) begin rd_order.delete(i); break; end
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
