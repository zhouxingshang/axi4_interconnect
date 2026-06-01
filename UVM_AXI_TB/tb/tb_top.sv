// Top-level testbench: instantiates DUT + interface + runs test
module tb_top;

    import uvm_pkg::*;
    import axi_pkg::*;

    // Clock & Reset
    reg ACLK = 0;
    reg ARSTn = 0;
    always #5 ACLK = ~ACLK;
    initial begin
        repeat(10) @(posedge ACLK);
        ARSTn = 1;
    end

    // Interface
    axi_if vif(.ACLK(ACLK), .ARSTn(ARSTn));

    // DUT: AXI Interconnect
    axi_interconnect #(
        .MST_AMT(4), .SLV_AMT(4), .OUTSTANDING_AMT(8),
        .W_ID(4), .DATA_WIDTH(32), .ADDR_WIDTH(32),
        .LEN_W(8), .SIZE_W(3), .BURST_W(2), .RESP_W(2),
        .SLV_ADDR_BASE({32'h0000_6000,32'h0000_4000,32'h0000_2000,32'h0000_0000}),
        .SLV_ADDR_LEN({8'd13,8'd13,8'd13,8'd13}),
        .DEFAULT_SLV_EN(1'b0)
    ) dut (
        .AXI_CLK(ACLK), .AXI_RSTn(ARSTn),
        .M_AXI_AWID_i(vif.M_AWID), .M_AXI_AWADDR_i(vif.M_AWADDR),
        .M_AXI_AWLEN_i(vif.M_AWLEN), .M_AXI_AWSIZE_i(vif.M_AWSIZE),
        .M_AXI_AWBURST_i(vif.M_AWBURST), .M_AXI_AWVALID_i(vif.M_AWVALID), .M_AXI_AWREADY_o(vif.M_AWREADY),
        .M_AXI_WDATA_i(vif.M_WDATA), .M_AXI_WSTRB_i(vif.M_WSTRB),
        .M_AXI_WLAST_i(vif.M_WLAST), .M_AXI_WVALID_i(vif.M_WVALID), .M_AXI_WREADY_o(vif.M_WREADY),
        .M_AXI_BID_o(vif.M_BID), .M_AXI_BRESP_o(vif.M_BRESP),
        .M_AXI_BVALID_o(vif.M_BVALID), .M_AXI_BREADY_i(vif.M_BREADY),
        .M_AXI_ARID_i(vif.M_ARID), .M_AXI_ARADDR_i(vif.M_ARADDR),
        .M_AXI_ARLEN_i(vif.M_ARLEN), .M_AXI_ARSIZE_i(vif.M_ARSIZE),
        .M_AXI_ARBURST_i(vif.M_ARBURST), .M_AXI_ARVALID_i(vif.M_ARVALID), .M_AXI_ARREADY_o(vif.M_ARREADY),
        .M_AXI_RID_o(vif.M_RID), .M_AXI_RDATA_o(vif.M_RDATA), .M_AXI_RRESP_o(vif.M_RRESP),
        .M_AXI_RLAST_o(vif.M_RLAST), .M_AXI_RVALID_o(vif.M_RVALID), .M_AXI_RREADY_i(vif.M_RREADY),
        .S_AXI_AWID_o(vif.S_AWID), .S_AXI_AWADDR_o(vif.S_AWADDR),
        .S_AXI_AWLEN_o(vif.S_AWLEN), .S_AXI_AWSIZE_o(vif.S_AWSIZE),
        .S_AXI_AWBURST_o(vif.S_AWBURST), .S_AXI_AWVALID_o(vif.S_AWVALID), .S_AXI_AWREADY_i(vif.S_AWREADY),
        .S_AXI_WDATA_o(vif.S_WDATA), .S_AXI_WSTRB_o(vif.S_WSTRB),
        .S_AXI_WLAST_o(vif.S_WLAST), .S_AXI_WVALID_o(vif.S_WVALID), .S_AXI_WREADY_i(vif.S_WREADY),
        .S_AXI_BID_i(vif.S_BID), .S_AXI_BRESP_i(vif.S_BRESP),
        .S_AXI_BVALID_i(vif.S_BVALID), .S_AXI_BREADY_o(vif.S_BREADY),
        .S_AXI_ARID_o(vif.S_ARID), .S_AXI_ARADDR_o(vif.S_ARADDR),
        .S_AXI_ARLEN_o(vif.S_ARLEN), .S_AXI_ARSIZE_o(vif.S_ARSIZE),
        .S_AXI_ARBURST_o(vif.S_ARBURST), .S_AXI_ARVALID_o(vif.S_ARVALID), .S_AXI_ARREADY_i(vif.S_ARREADY),
        .S_AXI_RID_i(vif.S_RID), .S_AXI_RDATA_i(vif.S_RDATA), .S_AXI_RRESP_i(vif.S_RRESP),
        .S_AXI_RLAST_i(vif.S_RLAST), .S_AXI_RVALID_i(vif.S_RVALID), .S_AXI_RREADY_o(vif.S_RREADY),
        .slv_en_i(4'b1111), .arbiter_type(1'b0)
    );

    // Connect interface to UVM config DB
    initial begin
        uvm_config_db #(virtual axi_if)::set(null, "*", "vif", vif);
        run_test();
    end

endmodule
