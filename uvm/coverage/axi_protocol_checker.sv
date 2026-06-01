//=============================================================================
// AXI Protocol Checker: immediate assertions on AXI bus protocol rules
// Checks: VALID stability, WLAST/RLAST, ID matching, B uniqueness
//=============================================================================
class axi_protocol_checker extends uvm_component;
    `uvm_component_utils(axi_protocol_checker)

    // Inputs from channel monitors
    uvm_analysis_export #(axi_aw_item) aw_ap; uvm_analysis_export #(axi_w_item) w_ap;
    uvm_analysis_export #(axi_b_item)  b_ap;
    uvm_analysis_export #(axi_ar_item) ar_ap; uvm_analysis_export #(axi_r_item) r_ap;
    uvm_tlm_analysis_fifo #(axi_aw_item) aw_fifo; uvm_tlm_analysis_fifo #(axi_w_item) w_fifo;
    uvm_tlm_analysis_fifo #(axi_b_item)  b_fifo;
    uvm_tlm_analysis_fifo #(axi_ar_item) ar_fifo; uvm_tlm_analysis_fifo #(axi_r_item) r_fifo;

    // Outstanding tables for ID matching
    typedef struct { int beats_done; int beats_exp; bit done; } wr_entry_t;
    typedef struct { int beats_done; int beats_exp; bit done; } rd_entry_t;
    wr_entry_t wr_tbl[int];   // key = {mst, id}
    rd_entry_t rd_tbl[int];   // key = {mst, id}
    int wr_total=0, rd_total=0;

    int error_cnt=0;

    function new(string name="axi_protocol_checker", uvm_component parent);
        super.new(name,parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        aw_ap=new("aw_ap",this); w_ap=new("w_ap",this); b_ap=new("b_ap",this);
        ar_ap=new("ar_ap",this); r_ap=new("r_ap",this);
        aw_fifo=new("aw_fifo",this); w_fifo=new("w_fifo",this); b_fifo=new("b_fifo",this);
        ar_fifo=new("ar_fifo",this); r_fifo=new("r_fifo",this);
        aw_ap.connect(aw_fifo.analysis_export); w_ap.connect(w_fifo.analysis_export);
        b_ap.connect(b_fifo.analysis_export); ar_ap.connect(ar_fifo.analysis_export);
        r_ap.connect(r_fifo.analysis_export);
    endfunction

    task run_phase(uvm_phase phase);
        fork
            check_aw(); check_w(); check_b();
            check_ar(); check_r();
        join
    endtask

    //=====================================================================
    // AW Channel: track outstanding, check for duplicate AWID
    //=====================================================================
    task check_aw();
        axi_aw_item t; int key;
        forever begin
            aw_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            // VALID must not be re-asserted until B completes
            if(wr_tbl.exists(key) && !wr_tbl[key].done) begin
                error_cnt++;
                `uvm_error("PROTO", $sformatf("DUPLICATE AWID: M[%0d] id=%0d before B", t.mst_id, t.id))
            end
            wr_tbl[key].beats_exp = t.len + 1;
            wr_tbl[key].beats_done = 0;
            wr_tbl[key].done = 0;
            wr_total++;
        end
    endtask

    //=====================================================================
    // W Channel: check WLAST, beat count, per-beat data presence
    //=====================================================================
    task check_w();
        axi_w_item t; int key; bit found;
        forever begin
            w_fifo.get(t);
            if(!t.is_master_side) continue;
            // Find matching outstanding write by searching all entries
            found = 0;
            foreach(wr_tbl[k]) begin
                if(!wr_tbl[k].done && wr_tbl[k].beats_done < wr_tbl[k].beats_exp) begin
                    // WLAST must match beat position
                    if(t.last && wr_tbl[k].beats_done != wr_tbl[k].beats_exp-1) begin
                        error_cnt++;
                        `uvm_error("PROTO", $sformatf("WLAST early: beat %0d/%0d exp_last=%0d",
                                  wr_tbl[k].beats_done, wr_tbl[k].beats_exp, wr_tbl[k].beats_exp-1))
                    end
                    if(!t.last && wr_tbl[k].beats_done == wr_tbl[k].beats_exp-1) begin
                        error_cnt++;
                        `uvm_error("PROTO", $sformatf("WLAST missing: beat %0d/%0d should be last",
                                  wr_tbl[k].beats_done, wr_tbl[k].beats_exp))
                    end
                    wr_tbl[k].beats_done++;
                    found=1; break;
                end
            end
            if(!found) begin
                error_cnt++;
                `uvm_error("PROTO", "W beat without matching outstanding AW")
            end
        end
    endtask

    //=====================================================================
    // B Channel: check BID matches AWID, B uniqueness (one B per AW)
    //=====================================================================
    task check_b();
        axi_b_item t; int key;
        forever begin
            b_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            if(!wr_tbl.exists(key)) begin
                error_cnt++;
                `uvm_error("PROTO", $sformatf("BID mismatch: M[%0d] id=%0d no matching AW", t.mst_id, t.id))
                continue;
            end
            if(wr_tbl[key].done) begin
                error_cnt++;
                `uvm_error("PROTO", $sformatf("DUPLICATE B: M[%0d] id=%0d already completed", t.mst_id, t.id))
            end
            // Check W beats completed
            if(wr_tbl[key].beats_done < wr_tbl[key].beats_exp) begin
                error_cnt++;
                `uvm_error("PROTO", $sformatf("B before WLAST: M[%0d] id=%0d w=%0d/%0d",
                          t.mst_id, t.id, wr_tbl[key].beats_done, wr_tbl[key].beats_exp))
            end
            wr_tbl[key].done = 1;
        end
    endtask

    //=====================================================================
    // AR Channel: track outstanding, check duplicate ARID
    //=====================================================================
    task check_ar();
        axi_ar_item t; int key;
        forever begin
            ar_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            if(rd_tbl.exists(key) && !rd_tbl[key].done) begin
                error_cnt++;
                `uvm_error("PROTO", $sformatf("DUPLICATE ARID: M[%0d] id=%0d before RLAST", t.mst_id, t.id))
            end
            rd_tbl[key].beats_exp = t.len + 1;
            rd_tbl[key].beats_done = 0;
            rd_tbl[key].done = 0;
            rd_total++;
        end
    endtask

    //=====================================================================
    // R Channel: check RID matches ARID, RLAST correctness
    //=====================================================================
    task check_r();
        axi_r_item t; int key; bit found;
        forever begin
            r_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            if(!rd_tbl.exists(key)) begin
                error_cnt++;
                `uvm_error("PROTO", $sformatf("RID mismatch: M[%0d] id=%0d no matching AR", t.mst_id, t.id))
                continue;
            end
            if(rd_tbl[key].done) begin
                error_cnt++;
                `uvm_error("PROTO", $sformatf("R after RLAST: M[%0d] id=%0d", t.mst_id, t.id))
            end
            // RLAST position check
            if(t.last && rd_tbl[key].beats_done != rd_tbl[key].beats_exp-1) begin
                error_cnt++;
                `uvm_error("PROTO", $sformatf("RLAST early: M[%0d] id=%0d beat %0d/%0d exp_last=%0d",
                          t.mst_id, t.id, rd_tbl[key].beats_done, rd_tbl[key].beats_exp, rd_tbl[key].beats_exp-1))
            end
            if(!t.last && rd_tbl[key].beats_done == rd_tbl[key].beats_exp-1) begin
                error_cnt++;
                `uvm_error("PROTO", $sformatf("RLAST missing: M[%0d] id=%0d beat %0d/%0d",
                          t.mst_id, t.id, rd_tbl[key].beats_done, rd_tbl[key].beats_exp))
            end
            rd_tbl[key].beats_done++;
            if(t.last) rd_tbl[key].done = 1;
        end
    endtask

    function void report_phase(uvm_phase phase);
        $display("PROTOCOL CHECKER: %0d writes, %0d reads, %0d protocol errors",
                 wr_total, rd_total, error_cnt);
    endfunction

endclass
