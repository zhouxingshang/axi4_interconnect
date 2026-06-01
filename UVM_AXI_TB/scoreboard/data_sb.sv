//=============================================================================
// Data Scoreboard: shadow memory – W beats update, R beats compare
// Uses W/R channel monitors for per-beat data tracking
// AW/AR provide address context
//=============================================================================
class data_sb extends uvm_scoreboard;
    `uvm_component_utils(data_sb)
    uvm_analysis_export #(axi_aw_item) aw_ap; uvm_analysis_export #(axi_w_item) w_ap;
    uvm_analysis_export #(axi_ar_item) ar_ap; uvm_analysis_export #(axi_r_item) r_ap;
    uvm_tlm_analysis_fifo #(axi_aw_item) aw_fifo; uvm_tlm_analysis_fifo #(axi_w_item) w_fifo;
    uvm_tlm_analysis_fifo #(axi_ar_item) ar_fifo; uvm_tlm_analysis_fifo #(axi_r_item) r_fifo;

    bit[31:0] shadow[bit[31:0]];
    int error_cnt=0, check_cnt=0;

    // Track active write data by {mst_id, id} for address calculation
    bit[31:0] wr_addr[int]; bit[2:0] wr_size[int]; int wr_beat[int];

    function new(string n="data_sb", uvm_component p); super.new(n,p); endfunction
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        aw_ap=new("aw_ap",this); w_ap=new("w_ap",this); ar_ap=new("ar_ap",this); r_ap=new("r_ap",this);
        aw_fifo=new("aw_fifo",this); w_fifo=new("w_fifo",this); ar_fifo=new("ar_fifo",this); r_fifo=new("r_fifo",this);
        aw_ap.connect(aw_fifo.analysis_export); w_ap.connect(w_fifo.analysis_export);
        ar_ap.connect(ar_fifo.analysis_export); r_ap.connect(r_fifo.analysis_export);
    endfunction
    task run_phase(uvm_phase phase); fork process_wr(); process_rd(); join endtask

    task process_wr();
        axi_aw_item at; axi_w_item wt; int key; bit[31:0] addr;
        forever begin
            fork aw_fifo.get(at); w_fifo.get(wt); join
            key = {8'(at.mst_id), at.id};
            addr = at.addr + (wr_beat[key] << at.size);
            shadow[addr[31:2]] = wt.data;
            wr_beat[key]++;
            if(wt.last && wr_beat[key] != at.len+1) begin error_cnt++;
                `uvm_error("DATA",$sformatf("WR beat count mismatch M[%0d] id=%0d",at.mst_id,at.id))
            end
            if(wt.last) begin wr_beat.delete(key); end
        end
    endtask

    task process_rd();
        axi_ar_item at; axi_r_item rt; int key; bit[31:0] addr, exp; int r_beat;
        forever begin
            fork ar_fifo.get(at); r_fifo.get(rt); join
            key = {8'(at.mst_id), at.id};
            if(!r_beat.exists(key)) r_beat[key]=0;
            addr = at.addr + (r_beat[key] << at.size);
            check_cnt++;
            if(shadow.exists(addr[31:2])) exp=shadow[addr[31:2]]; else exp=32'hDEAD_BEEF;
            if(rt.data !== exp) begin error_cnt++;
                `uvm_error("DATA",$sformatf("M[%0d] addr=0x%08h exp=0x%08h act=0x%08h id=%0d",
                          at.mst_id,addr,exp,rt.data,at.id))
            end
            r_beat[key]++;
            if(rt.last && r_beat[key] != at.len+1) begin error_cnt++;
                `uvm_error("DATA",$sformatf("RD beat count mismatch M[%0d] id=%0d",at.mst_id,at.id))
            end
            if(rt.last) r_beat.delete(key);
        end
    endtask

    function void report_phase(uvm_phase phase);
        $display("DATA SB: %0d checks, %0d errors", check_cnt, error_cnt);
    endfunction
endclass
