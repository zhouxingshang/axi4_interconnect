//=============================================================================
// Unified AXI Monitor: 5 channels (AW/W/B/AR/R), master-side or slave-side
// Placed inside each agent, monitors its own master or slave port only
//=============================================================================
`ifndef AXI_MONITOR_SV
`define AXI_MONITOR_SV

class axi_monitor extends uvm_monitor;
    `uvm_component_utils(axi_monitor)

    virtual axi_if vif;

    // Side selector: mst_id>=0 → master-side, slv_id>=0 → slave-side
    int mst_id = -1;
    int slv_id = -1;

    // Per-channel analysis ports
    uvm_analysis_port #(axi_aw_item) ap_aw;
    uvm_analysis_port #(axi_w_item)  ap_w;
    uvm_analysis_port #(axi_b_item)  ap_b;
    uvm_analysis_port #(axi_ar_item) ap_ar;
    uvm_analysis_port #(axi_r_item)  ap_r;

    function new(string name = "axi_monitor", uvm_component parent);
        super.new(name, parent);
        ap_aw = new("ap_aw", this);
        ap_w  = new("ap_w",  this);
        ap_b  = new("ap_b",  this);
        ap_ar = new("ap_ar", this);
        ap_r  = new("ap_r",  this);
    endfunction

    task run_phase(uvm_phase phase);
        if (mst_id >= 0) begin
            fork
                monitor_master_aw();
                monitor_master_w();
                monitor_master_b();
                monitor_master_ar();
                monitor_master_r();
            join
        end else if (slv_id >= 0) begin
            fork
                monitor_slave_aw();
                monitor_slave_w();
                monitor_slave_b();
                monitor_slave_ar();
                monitor_slave_r();
            join
        end
    endtask

    //===================================================================
    // Master-side monitors (watch one master's M_* signals)
    //===================================================================

    task monitor_master_aw();
        axi_aw_item t;
        forever begin
            @(posedge vif.ACLK);
            if (vif.M_AWVALID[mst_id] && vif.M_AWREADY[mst_id]) begin
                t = axi_aw_item::type_id::create("t");
                t.is_master_side = 1; t.mst_id = mst_id; t.slv_id = -1;
                t.id    = vif.M_AWID[mst_id*4+:4];
                t.addr  = vif.M_AWADDR[mst_id*32+:32];
                t.len   = vif.M_AWLEN[mst_id*8+:8];
                t.size  = vif.M_AWSIZE[mst_id*3+:3];
                t.burst = vif.M_AWBURST[mst_id*2+:2];
                t.timestamp = $time;
                ap_aw.write(t);
            end
        end
    endtask

    task monitor_master_w();
        axi_w_item t;
        forever begin
            @(posedge vif.ACLK);
            if (vif.M_WVALID[mst_id] && vif.M_WREADY[mst_id]) begin
                t = axi_w_item::type_id::create("t");
                t.is_master_side = 1; t.mst_id = mst_id; t.slv_id = -1;
                t.data  = vif.M_WDATA[mst_id*32+:32];
                t.strb  = vif.M_WSTRB[mst_id*4+:4];
                t.last  = vif.M_WLAST[mst_id];
                t.timestamp = $time;
                ap_w.write(t);
            end
        end
    endtask

    task monitor_master_b();
        axi_b_item t;
        forever begin
            @(posedge vif.ACLK);
            if (vif.M_BVALID[mst_id] && vif.M_BREADY[mst_id]) begin
                t = axi_b_item::type_id::create("t");
                t.is_master_side = 1; t.mst_id = mst_id; t.slv_id = -1;
                t.id   = vif.M_BID[mst_id*4+:4];
                t.resp = vif.M_BRESP[mst_id*2+:2];
                t.timestamp = $time;
                ap_b.write(t);
                `uvm_info("B_MON", $sformatf("M[%0d] B id=%0d resp=%0d", mst_id, t.id, t.resp), UVM_HIGH)
            end
        end
    endtask

    task monitor_master_ar();
        axi_ar_item t;
        forever begin
            @(posedge vif.ACLK);
            if (vif.M_ARVALID[mst_id] && vif.M_ARREADY[mst_id]) begin
                t = axi_ar_item::type_id::create("t");
                t.is_master_side = 1; t.mst_id = mst_id; t.slv_id = -1;
                t.id    = vif.M_ARID[mst_id*4+:4];
                t.addr  = vif.M_ARADDR[mst_id*32+:32];
                t.len   = vif.M_ARLEN[mst_id*8+:8];
                t.size  = vif.M_ARSIZE[mst_id*3+:3];
                t.burst = vif.M_ARBURST[mst_id*2+:2];
                t.timestamp = $time;
                ap_ar.write(t);
                `uvm_info("AR_MON", $sformatf("M[%0d] AR id=%0d addr=0x%08h len=%0d", mst_id, t.id, t.addr, t.len), UVM_HIGH)
            end
        end
    endtask

    task monitor_master_r();
        axi_r_item t;
        forever begin
            @(posedge vif.ACLK);
            if (vif.M_RVALID[mst_id] && vif.M_RREADY[mst_id]) begin
                t = axi_r_item::type_id::create("t");
                t.is_master_side = 1; t.mst_id = mst_id; t.slv_id = -1;
                t.id   = vif.M_RID[mst_id*4+:4];
                t.data = vif.M_RDATA[mst_id*32+:32];
                t.resp = vif.M_RRESP[mst_id*2+:2];
                t.last = vif.M_RLAST[mst_id];
                t.timestamp = $time;
                ap_r.write(t);
            end
        end
    endtask

    //===================================================================
    // Slave-side monitors (watch one slave's S_* signals)
    //===================================================================

    task monitor_slave_aw();
        axi_aw_item t;
        forever begin
            @(posedge vif.ACLK);
            if (vif.S_AWVALID[slv_id] && vif.S_AWREADY[slv_id]) begin
                t = axi_aw_item::type_id::create("t");
                t.is_master_side = 0; t.slv_id = slv_id;
                t.mst_id = vif.S_AWID[slv_id*8+6+:2];   // upper 2 bits = mst index
                t.id     = vif.S_AWID[slv_id*8+:4];      // lower 4 bits = original ID
                t.addr   = vif.S_AWADDR[slv_id*32+:32];
                t.len    = vif.S_AWLEN[slv_id*8+:8];
                t.size   = vif.S_AWSIZE[slv_id*3+:3];
                t.burst  = vif.S_AWBURST[slv_id*2+:2];
                t.timestamp = $time;
                ap_aw.write(t);
            end
        end
    endtask

    task monitor_slave_w();
        axi_w_item t;
        forever begin
            @(posedge vif.ACLK);
            if (vif.S_WVALID[slv_id] && vif.S_WREADY[slv_id]) begin
                t = axi_w_item::type_id::create("t");
                t.is_master_side = 0; t.slv_id = slv_id; t.mst_id = -1;
                t.data  = vif.S_WDATA[slv_id*32+:32];
                t.strb  = vif.S_WSTRB[slv_id*4+:4];
                t.last  = vif.S_WLAST[slv_id];
                t.timestamp = $time;
                ap_w.write(t);
            end
        end
    endtask

    task monitor_slave_b();
        axi_b_item t;
        forever begin
            @(posedge vif.ACLK);
            if (vif.S_BVALID[slv_id] && vif.S_BREADY[slv_id]) begin
                t = axi_b_item::type_id::create("t");
                t.is_master_side = 0; t.slv_id = slv_id;
                t.mst_id = vif.S_BID[slv_id*8+6+:2];
                t.id     = vif.S_BID[slv_id*8+:4];
                t.resp   = vif.S_BRESP[slv_id*2+:2];
                t.timestamp = $time;
                ap_b.write(t);
                `uvm_info("B_MON", $sformatf("S[%0d] B id=%0d mst=%0d", slv_id, t.id, t.mst_id), UVM_HIGH)
            end
        end
    endtask

    task monitor_slave_ar();
        axi_ar_item t;
        forever begin
            @(posedge vif.ACLK);
            if (vif.S_ARVALID[slv_id] && vif.S_ARREADY[slv_id]) begin
                t = axi_ar_item::type_id::create("t");
                t.is_master_side = 0; t.slv_id = slv_id;
                t.mst_id = vif.S_ARID[slv_id*8+6+:2];
                t.id     = vif.S_ARID[slv_id*8+:4];
                t.addr   = vif.S_ARADDR[slv_id*32+:32];
                t.len    = vif.S_ARLEN[slv_id*8+:8];
                t.size   = vif.S_ARSIZE[slv_id*3+:3];
                t.burst  = vif.S_ARBURST[slv_id*2+:2];
                t.timestamp = $time;
                ap_ar.write(t);
                `uvm_info("AR_MON", $sformatf("S[%0d] AR id=%0d mst=%0d addr=0x%08h", slv_id, t.id, t.mst_id, t.addr), UVM_HIGH)
            end
        end
    endtask

    task monitor_slave_r();
        axi_r_item t;
        forever begin
            @(posedge vif.ACLK);
            if (vif.S_RVALID[slv_id] && vif.S_RREADY[slv_id]) begin
                t = axi_r_item::type_id::create("t");
                t.is_master_side = 0; t.slv_id = slv_id;
                t.mst_id = vif.S_RID[slv_id*8+6+:2];
                t.id     = vif.S_RID[slv_id*8+:4];
                t.data   = vif.S_RDATA[slv_id*32+:32];
                t.resp   = vif.S_RRESP[slv_id*2+:2];
                t.last   = vif.S_RLAST[slv_id];
                t.timestamp = $time;
                ap_r.write(t);
            end
        end
    endtask

endclass

`endif
