//=============================================================================
// AXI Monitor: observes both master-side and slave-side buses, sends
// transactions to analysis ports for scoreboard / coverage.
//=============================================================================
`ifndef AXI_MONITOR_SV
`define AXI_MONITOR_SV

class axi_monitor extends uvm_monitor;

    virtual axi_if vif;
    int mst_id;  // -1 = slave-side monitor, 0..3 = master-side

    uvm_analysis_port #(axi_transaction) ap;

    `uvm_component_utils(axi_monitor)

    function new(string name = "axi_monitor", uvm_component parent);
        super.new(name, parent);
        ap = new("ap", this);
    endfunction

    task run_phase(uvm_phase phase);
        if (mst_id >= 0)
            monitor_master_side();
        else
            monitor_slave_side();
    endtask

    //---- Master-side: capture AW/W/AR transactions ----
    task monitor_master_side();
        axi_transaction t;
        forever begin
            @(posedge vif.ACLK);
            // AW channel
            if (vif.M_AWVALID[mst_id] && vif.M_AWREADY[mst_id]) begin
                t = axi_transaction::type_id::create("t");
                t.is_write = 1;
                t.mst_id   = mst_id;
                t.id       = vif.M_AWID[mst_id*4+:4];
                t.addr     = vif.M_AWADDR[mst_id*32+:32];
                t.len      = vif.M_AWLEN[mst_id*8+:8];
                t.size     = vif.M_AWSIZE[mst_id*3+:3];
                t.burst    = vif.M_AWBURST[mst_id*2+:2];
                ap.write(t);
                `uvm_info("MONITOR", $sformatf("M[%0d] AW: %s", mst_id, t.convert2string()), UVM_HIGH)
            end
            // AR channel
            if (vif.M_ARVALID[mst_id] && vif.M_ARREADY[mst_id]) begin
                t = axi_transaction::type_id::create("t");
                t.is_write = 0;
                t.mst_id   = mst_id;
                t.id       = vif.M_ARID[mst_id*4+:4];
                t.addr     = vif.M_ARADDR[mst_id*32+:32];
                t.len      = vif.M_ARLEN[mst_id*8+:8];
                t.size     = vif.M_ARSIZE[mst_id*3+:3];
                t.burst    = vif.M_ARBURST[mst_id*2+:2];
                ap.write(t);
                `uvm_info("MONITOR", $sformatf("M[%0d] AR: %s", mst_id, t.convert2string()), UVM_HIGH)
            end
        end
    endtask

    //---- Slave-side: capture routed AW/AR (for routing check) ----
    task monitor_slave_side();
        axi_transaction t;
        int s;
        forever begin
            @(posedge vif.ACLK);
            for (s = 0; s < 4; s++) begin
                if (vif.S_AWVALID[s] && vif.S_AWREADY[s]) begin
                    t = axi_transaction::type_id::create("t");
                    t.is_write = 1;
                    t.slv_id   = s;
                    t.id       = vif.S_AWID[s*8+:4];   // lower 4 bits = original ID
                    t.addr     = vif.S_AWADDR[s*32+:32];
                    t.len      = vif.S_AWLEN[s*8+:8];
                    t.size     = vif.S_AWSIZE[s*3+:3];
                    t.burst    = vif.S_AWBURST[s*2+:2];
                    t.mst_id   = vif.S_AWID[s*8+4+:2]; // upper bits = master index
                    ap.write(t);
                    `uvm_info("MONITOR", $sformatf("S[%0d] AW: %s", s, t.convert2string()), UVM_HIGH)
                end
                if (vif.S_ARVALID[s] && vif.S_ARREADY[s]) begin
                    t = axi_transaction::type_id::create("t");
                    t.is_write = 0;
                    t.slv_id   = s;
                    t.id       = vif.S_ARID[s*8+:4];
                    t.addr     = vif.S_ARADDR[s*32+:32];
                    t.len      = vif.S_ARLEN[s*8+:8];
                    t.size     = vif.S_ARSIZE[s*3+:3];
                    t.burst    = vif.S_ARBURST[s*2+:2];
                    t.mst_id   = vif.S_ARID[s*8+4+:2];
                    ap.write(t);
                    `uvm_info("MONITOR", $sformatf("S[%0d] AR: %s", s, t.convert2string()), UVM_HIGH)
                end
            end
        end
    endtask

endclass

`endif
