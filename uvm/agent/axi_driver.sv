//=============================================================================
// AXI Master Driver: drives AXI bus from sequencer transactions
// Uses negedge-set / posedge-handshake pattern to avoid class-RTL race
//=============================================================================
`ifndef AXI_DRIVER_SV
`define AXI_DRIVER_SV

class axi_driver extends uvm_driver #(axi_transaction);

    virtual axi_if vif;
    int mst_id;

    `uvm_component_utils(axi_driver)

    function new(string name = "axi_driver", uvm_component parent);
        super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
        forever begin
            seq_item_port.get_next_item(req);
            `uvm_info("DRIVER", $sformatf("M[%0d] driving: %s", mst_id, req.convert2string()), UVM_MEDIUM)
            if (req.is_write)
                drive_write(req);
            else
                drive_read(req);
            seq_item_port.item_done();
        end
    endtask

    //===================================================================
    // Write: AW (negedge) + W (negedge per beat) + wait B
    //===================================================================
    task drive_write(axi_transaction t);
        // AW channel: drive at negedge, handshake at posedge
        @(negedge vif.ACLK);
        vif.M_AWVALID[mst_id]   = 1'b1;
        vif.M_AWID[mst_id*4+:4] = t.id;
        vif.M_AWADDR[mst_id*32+:32] = t.addr;
        vif.M_AWLEN[mst_id*8+:8]    = t.len;
        vif.M_AWSIZE[mst_id*3+:3]   = t.size;
        vif.M_AWBURST[mst_id*2+:2]  = t.burst;
        `uvm_info("TRACE", $sformatf("M[%0d] DRV AW start addr=0x%08h len=%0d", mst_id, t.addr, t.len), UVM_MEDIUM)
        @(posedge vif.ACLK);
        while (!vif.M_AWREADY[mst_id]) @(posedge vif.ACLK);
        `uvm_info("TRACE", $sformatf("M[%0d] DRV AW handshake done", mst_id), UVM_MEDIUM)
        @(negedge vif.ACLK);
        vif.M_AWVALID[mst_id] = 1'b0;

        // W channel: each beat driven at negedge
        for (int b = 0; b <= t.len; b++) begin
            @(negedge vif.ACLK);
            vif.M_WVALID[mst_id]   = 1'b1;
            vif.M_WDATA[mst_id*32+:32]  = t.data[b];
            vif.M_WSTRB[mst_id*4+:4]    = t.strb[b];
            vif.M_WLAST[mst_id]   = (b == t.len) ? 1'b1 : 1'b0;
            `uvm_info("TRACE", $sformatf("M[%0d] DRV W beat %0d/%0d last=%b", mst_id, b, t.len, vif.M_WLAST[mst_id]), UVM_MEDIUM)
            @(posedge vif.ACLK);
            while (!vif.M_WREADY[mst_id]) @(posedge vif.ACLK);
            @(negedge vif.ACLK);
            vif.M_WVALID[mst_id] = 1'b0;
        end

        // B channel: wait for response
        `uvm_info("TRACE", $sformatf("M[%0d] DRV B wait", mst_id), UVM_MEDIUM)
        @(negedge vif.ACLK);
        vif.M_BREADY[mst_id] = 1'b1;
        @(posedge vif.ACLK);
        while (!vif.M_BVALID[mst_id]) @(posedge vif.ACLK);
        @(negedge vif.ACLK);
        t.resp = vif.M_BRESP[mst_id*2+:2];
        `uvm_info("TRACE", $sformatf("M[%0d] DRV B resp=%0d", mst_id, t.resp), UVM_MEDIUM)
        vif.M_BREADY[mst_id] = 1'b0;
    endtask

    //===================================================================
    // Read: AR (negedge) + R beats (negedge per beat)
    //===================================================================
    task drive_read(axi_transaction t);
        // AR channel: drive at negedge, handshake at posedge
        @(negedge vif.ACLK);
        vif.M_ARVALID[mst_id]   = 1'b1;
        vif.M_ARID[mst_id*4+:4] = t.id;
        vif.M_ARADDR[mst_id*32+:32] = t.addr;
        vif.M_ARLEN[mst_id*8+:8]    = t.len;
        vif.M_ARSIZE[mst_id*3+:3]   = t.size;
        vif.M_ARBURST[mst_id*2+:2]  = t.burst;
        `uvm_info("TRACE", $sformatf("M[%0d] DRV AR start addr=0x%08h len=%0d", mst_id, t.addr, t.len), UVM_MEDIUM)
        @(posedge vif.ACLK);
        while (!vif.M_ARREADY[mst_id]) @(posedge vif.ACLK);
        `uvm_info("TRACE", $sformatf("M[%0d] DRV AR handshake done", mst_id), UVM_MEDIUM)
        @(negedge vif.ACLK);
        vif.M_ARVALID[mst_id] = 1'b0;

        // R channel: each beat driven at negedge
        for (int b = 0; b <= t.len; b++) begin
            `uvm_info("TRACE", $sformatf("M[%0d] DRV R wait beat %0d/%0d", mst_id, b, t.len), UVM_MEDIUM)
            @(negedge vif.ACLK);
            vif.M_RREADY[mst_id] = 1'b1;
            @(posedge vif.ACLK);
            while (!vif.M_RVALID[mst_id]) @(posedge vif.ACLK);
            @(negedge vif.ACLK);
            t.data[b] = vif.M_RDATA[mst_id*32+:32];
            t.resp    = vif.M_RRESP[mst_id*2+:2];
            t.rlast   = vif.M_RLAST[mst_id];
            vif.M_RREADY[mst_id] = 1'b0;
            if (t.rlast && b != t.len) begin
                `uvm_warning("DRIVER", $sformatf("M[%0d] early RLAST at beat %0d/%0d", mst_id, b, t.len))
            end
        end
    endtask

endclass

`endif
