//=============================================================================
// AXI Master Driver: drives AXI bus from sequencer transactions
// Supports back-pressure injection via configurable ready delays
//=============================================================================
`ifndef AXI_DRIVER_SV
`define AXI_DRIVER_SV

class axi_driver extends uvm_driver #(axi_transaction);

    virtual axi_if vif;
    int mst_id;

    // Back-pressure control: min/max random wait cycles before asserting VALID
    int aw_valid_delay_min = 0, aw_valid_delay_max = 3;
    int  w_valid_delay_min = 0,  w_valid_delay_max = 3;
    int ar_valid_delay_min = 0, ar_valid_delay_max = 3;

    `uvm_component_utils(axi_driver)

    function new(string name = "axi_driver", uvm_component parent);
        super.new(name, parent);
    endfunction

    //---- Main run phase ----
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

    //---- Write: AW + W, wait for B ----
    task drive_write(axi_transaction t);
        // AW channel
        random_delay(aw_valid_delay_min, aw_valid_delay_max);
        vif.M_AWVALID[mst_id]   <= 1'b1;
        vif.M_AWID[mst_id*4+:4] <= t.id;
        vif.M_AWADDR[mst_id*32+:32] <= t.addr;
        vif.M_AWLEN[mst_id*8+:8]    <= t.len;
        vif.M_AWSIZE[mst_id*3+:3]   <= t.size;
        vif.M_AWBURST[mst_id*2+:2]  <= t.burst;
        @(posedge vif.ACLK);
        while (!vif.M_AWREADY[mst_id]) @(posedge vif.ACLK);
        vif.M_AWVALID[mst_id] <= 1'b0;

        // W channel: send each beat
        for (int b = 0; b <= t.len; b++) begin
            random_delay(w_valid_delay_min, w_valid_delay_max);
            vif.M_WVALID[mst_id]   <= 1'b1;
            vif.M_WDATA[mst_id*32+:32]  <= t.data[b];
            vif.M_WSTRB[mst_id*4+:4]    <= t.strb[b];
            vif.M_WLAST[mst_id]   <= (b == t.len);
            @(posedge vif.ACLK);
            while (!vif.M_WREADY[mst_id]) @(posedge vif.ACLK);
        end
        vif.M_WVALID[mst_id] <= 1'b0;
        vif.M_WLAST[mst_id]  <= 1'b0;

        // B channel: wait for response
        vif.M_BREADY[mst_id] <= 1'b1;
        @(posedge vif.ACLK);
        while (!vif.M_BVALID[mst_id]) @(posedge vif.ACLK);
        t.resp = vif.M_BRESP[mst_id*2+:2];
        vif.M_BREADY[mst_id] <= 1'b0;
    endtask

    //---- Read: AR, wait for R beats ----
    task drive_read(axi_transaction t);
        // AR channel
        random_delay(ar_valid_delay_min, ar_valid_delay_max);
        vif.M_ARVALID[mst_id]   <= 1'b1;
        vif.M_ARID[mst_id*4+:4] <= t.id;
        vif.M_ARADDR[mst_id*32+:32] <= t.addr;
        vif.M_ARLEN[mst_id*8+:8]    <= t.len;
        vif.M_ARSIZE[mst_id*3+:3]   <= t.size;
        vif.M_ARBURST[mst_id*2+:2]  <= t.burst;
        @(posedge vif.ACLK);
        while (!vif.M_ARREADY[mst_id]) @(posedge vif.ACLK);
        vif.M_ARVALID[mst_id] <= 1'b0;

        // R channel: receive each beat (may be out-of-order via RID)
        for (int b = 0; b <= t.len; b++) begin
            vif.M_RREADY[mst_id] <= 1'b1;
            @(posedge vif.ACLK);
            while (!vif.M_RVALID[mst_id]) @(posedge vif.ACLK);
            t.data[b] = vif.M_RDATA[mst_id*32+:32];
            t.resp    = vif.M_RRESP[mst_id*2+:2];
            t.rlast   = vif.M_RLAST[mst_id];
            if (t.rlast && b != t.len) begin
                `uvm_warning("DRIVER", $sformatf("M[%0d] early RLAST at beat %0d/%0d", mst_id, b, t.len))
            end
        end
        vif.M_RREADY[mst_id] <= 1'b0;
    endtask

    //---- Random delay for back-pressure ----
    task random_delay(int min_d, int max_d);
        int d;
        if (max_d > 0) begin
            d = min_d + ($urandom % (max_d - min_d + 1));
            repeat(d) @(posedge vif.ACLK);
        end
    endtask

endclass

`endif
