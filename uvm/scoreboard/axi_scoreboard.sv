//=============================================================================
// Unified AXI Scoreboard: comparison only
//   axi_scoreboard_routing.sv — Route checking
//   axi_scoreboard_data.sv    — Data + cross-check
//   axi_scoreboard_resp.sv    — Response checking
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

    //===== Route checking =====
    int route_err=0, route_cnt=0;
    axi_aw_item aw_m_pending[int];
    typedef axi_aw_item aw_item_q[$];
    aw_item_q   aw_s_pending[int];
    axi_ar_item ar_m_pending[int];
    typedef axi_ar_item ar_item_q[$];
    ar_item_q   ar_s_pending[int];
    int        ar_pair_exp[int];
    int        ar_pair_got[int];

    //===== Data checking =====
    int data_err=0, data_cnt=0;
    typedef struct { bit[31:0] addr; bit[7:0] len; bit[2:0] size; bit[1:0] burst; } ar_info_t;
    typedef ar_info_t ar_info_q[$];
    ar_info_q ar_info_pool[int];
    int       r_beat_cnt[int];

    //===== Response checking =====
    typedef struct {
        bit[7:0] mst_id; bit[3:0] id; bit[7:0] len; int w_beat_cnt; int b_received;
        bit[31:0] m_w_data[$];
        bit[31:0] s_w_data[$];
    } wr_out_t;
    typedef struct {
        bit[7:0] mst_id; bit[3:0] id; bit[7:0] len; int r_beat_cnt;
        bit[31:0] m_r_data[$];
        bit[1:0]  m_r_resp[$];
        bit[31:0] s_r_data[$];
        bit[1:0]  s_r_resp[$];
    } rd_out_t;
    typedef wr_out_t wr_out_q[$];
    typedef rd_out_t rd_out_q[$];
    wr_out_q  wr_out[int];
    rd_out_q  rd_out[int];
    int wr_order[$], rd_order[$], b_order[$];
    int resp_err=0, wr_cnt=0, rd_cnt=0, reorder_cnt=0;

    //===== W/B/R cross-check =====
    int w_xchk_err=0, b_xchk_err=0, r_xchk_err=0;
    int w_xchk_cnt=0, b_xchk_cnt=0, r_xchk_cnt=0;
    typedef int int_q[$];
    int_q slv_aw_order[4];

    // B cross-check
    int        b_s_exp[int];
    int        b_s_got[int];
    bit[1:0]   b_s_mresp[int];
    bit[3:0]   b_s_id[int];
    axi_b_item b_m_pending[int];

    // R cross-check
    int        r_s_exp[int];
    int        r_s_got[int];
    bit[1:0]   r_s_merge_resp[int];
    bit        r_m_done[int];
    bit        r_seen[int];
    bit[3:0]   s_r_id[int];
    bit[3:0]   m_r_id[int];

    // AW-not-ready buffers
    axi_w_item slv_w_buf[int][$];
    axi_b_item b_buf[int][$];

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
            // Route
            collect_master_aw();  collect_slave_aw();
            collect_master_ar();  collect_slave_ar();
            // Data
            process_ar_data();  process_r_data();
            // Response
            process_aw_resp();  process_w_resp();  process_b_resp();
            process_ar_resp();  process_r_resp();
            // Slave-side
            process_slv_w();  process_slv_b();  process_slv_r();
        join
    endtask

    `include "axi_scoreboard_routing.sv"
    `include "axi_scoreboard_data.sv"
    `include "axi_scoreboard_resp.sv"

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
