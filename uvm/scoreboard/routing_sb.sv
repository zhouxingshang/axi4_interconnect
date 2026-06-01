//=============================================================================
// Routing Scoreboard: matches M-side AW/AR to S-side AW/AR
// Checks: expected slave (addr decode) == actual slave, detects routing errors
//=============================================================================
class routing_sb extends uvm_scoreboard;
    `uvm_component_utils(routing_sb)
    uvm_analysis_export #(axi_aw_item) mst_aw_ap, slv_aw_ap;
    uvm_analysis_export #(axi_ar_item) mst_ar_ap, slv_ar_ap;
    uvm_tlm_analysis_fifo #(axi_aw_item) mst_aw_fifo, slv_aw_fifo;
    uvm_tlm_analysis_fifo #(axi_ar_item) mst_ar_fifo, slv_ar_fifo;
    int error_cnt=0, check_cnt=0;
    function new(string name="routing_sb", uvm_component parent); super.new(name,parent); endfunction
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        mst_aw_ap=new("mst_aw_ap",this); slv_aw_ap=new("slv_aw_ap",this);
        mst_ar_ap=new("mst_ar_ap",this); slv_ar_ap=new("slv_ar_ap",this);
        mst_aw_fifo=new("mst_aw_fifo",this); slv_aw_fifo=new("slv_aw_fifo",this);
        mst_ar_fifo=new("mst_ar_fifo",this); slv_ar_fifo=new("slv_ar_fifo",this);
        mst_aw_ap.connect(mst_aw_fifo.analysis_export); slv_aw_ap.connect(slv_aw_fifo.analysis_export);
        mst_ar_ap.connect(mst_ar_fifo.analysis_export); slv_ar_ap.connect(slv_ar_fifo.analysis_export);
    endfunction
    task run_phase(uvm_phase phase); fork check_wr_routing(); check_rd_routing(); join endtask

    task check_wr_routing();
        axi_aw_item m_t, s_t; int exp;
        forever begin
            fork mst_aw_fifo.get(m_t); slv_aw_fifo.get(s_t); join
            exp = addr_decoder::decode(m_t.addr); check_cnt++;
            if(exp != s_t.slv_id) begin error_cnt++;
                `uvm_error("ROUTE",$sformatf("WR M[%0d] id=%0d addr=0x%08h exp S%0d got S%0d",
                          m_t.mst_id,m_t.id,m_t.addr,exp,s_t.slv_id))
            end
        end
    endtask

    task check_rd_routing();
        axi_ar_item m_t, s_t; int exp;
        forever begin
            fork mst_ar_fifo.get(m_t); slv_ar_fifo.get(s_t); join
            exp = addr_decoder::decode(m_t.addr); check_cnt++;
            if(exp != s_t.slv_id) begin error_cnt++;
                `uvm_error("ROUTE",$sformatf("RD M[%0d] id=%0d addr=0x%08h exp S%0d got S%0d",
                          m_t.mst_id,m_t.id,m_t.addr,exp,s_t.slv_id))
            end
        end
    endtask
    function void report_phase(uvm_phase phase);
        $display("ROUTING SB: %0d checks, %0d errors", check_cnt, error_cnt);
    endfunction
endclass
