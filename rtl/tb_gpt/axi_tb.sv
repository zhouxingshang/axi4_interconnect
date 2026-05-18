`timescale 1ns/1ps

module axi_tb;

    //=========================================================
    // Parameters
    //=========================================================
    localparam MST_AMT    = 2;
    localparam SLV_AMT    = 2;

    localparam W_ID       = 4;
    localparam DATA_WIDTH = 32;
    localparam ADDR_WIDTH = 32;

    localparam LEN_W      = 8;
    localparam SIZE_W     = 3;
    localparam BURST_W    = 2;
    localparam RESP_W     = 2;

    localparam W_STRB     = DATA_WIDTH/8;

    localparam W_CID      = $clog2(MST_AMT);
    localparam W_MID      = W_ID + 2;
    localparam W_SID      = W_CID + W_MID;

    //=========================================================
    // Clock / Reset
    //=========================================================
    reg AXI_CLK;
    reg AXI_RSTn;

    initial begin
        AXI_CLK = 0;
        forever #5 AXI_CLK = ~AXI_CLK;
    end

    initial begin
        AXI_RSTn = 0;
        repeat(20) @(posedge AXI_CLK);
        AXI_RSTn = 1;
    end

    //=========================================================
    // FSDB
    //=========================================================
    initial begin
        $fsdbDumpfile("axi_tb.fsdb");
        $fsdbDumpvars(0, axi_tb);
        $fsdbDumpMDA();
    end

    //=========================================================
    // Master Interface
    //=========================================================

    reg  [MST_AMT*W_ID-1:0]       M_AXI_AWID_i;
    reg  [MST_AMT*ADDR_WIDTH-1:0] M_AXI_AWADDR_i;
    reg  [MST_AMT*LEN_W-1:0]      M_AXI_AWLEN_i;
    reg  [MST_AMT*SIZE_W-1:0]     M_AXI_AWSIZE_i;
    reg  [MST_AMT*BURST_W-1:0]    M_AXI_AWBURST_i;
    reg  [MST_AMT-1:0]            M_AXI_AWVALID_i;
    wire [MST_AMT-1:0]            M_AXI_AWREADY_o;

    reg  [MST_AMT*DATA_WIDTH-1:0] M_AXI_WDATA_i;
    reg  [MST_AMT*W_STRB-1:0]     M_AXI_WSTRB_i;
    reg  [MST_AMT-1:0]            M_AXI_WLAST_i;
    reg  [MST_AMT-1:0]            M_AXI_WVALID_i;
    wire [MST_AMT-1:0]            M_AXI_WREADY_o;

    wire [MST_AMT*W_ID-1:0]       M_AXI_BID_o;
    wire [MST_AMT*RESP_W-1:0]     M_AXI_BRESP_o;
    wire [MST_AMT-1:0]            M_AXI_BVALID_o;
    reg  [MST_AMT-1:0]            M_AXI_BREADY_i;

    reg  [MST_AMT*W_ID-1:0]       M_AXI_ARID_i;
    reg  [MST_AMT*ADDR_WIDTH-1:0] M_AXI_ARADDR_i;
    reg  [MST_AMT*LEN_W-1:0]      M_AXI_ARLEN_i;
    reg  [MST_AMT*SIZE_W-1:0]     M_AXI_ARSIZE_i;
    reg  [MST_AMT*BURST_W-1:0]    M_AXI_ARBURST_i;
    reg  [MST_AMT-1:0]            M_AXI_ARVALID_i;
    wire [MST_AMT-1:0]            M_AXI_ARREADY_o;

    wire [MST_AMT*W_ID-1:0]       M_AXI_RID_o;
    wire [MST_AMT*DATA_WIDTH-1:0] M_AXI_RDATA_o;
    wire [MST_AMT*RESP_W-1:0]     M_AXI_RRESP_o;
    wire [MST_AMT-1:0]            M_AXI_RLAST_o;
    wire [MST_AMT-1:0]            M_AXI_RVALID_o;
    reg  [MST_AMT-1:0]            M_AXI_RREADY_i;

    //=========================================================
    // DUT
    //=========================================================

    axi_interconnect #(
        .MST_AMT(MST_AMT),
        .SLV_AMT(SLV_AMT),
        .W_ID(W_ID),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) dut (

        .AXI_CLK           (AXI_CLK),
        .AXI_RSTn          (AXI_RSTn),

        .M_AXI_AWID_i      (M_AXI_AWID_i),
        .M_AXI_AWADDR_i    (M_AXI_AWADDR_i),
        .M_AXI_AWLEN_i     (M_AXI_AWLEN_i),
        .M_AXI_AWSIZE_i    (M_AXI_AWSIZE_i),
        .M_AXI_AWBURST_i   (M_AXI_AWBURST_i),
        .M_AXI_AWVALID_i   (M_AXI_AWVALID_i),
        .M_AXI_AWREADY_o   (M_AXI_AWREADY_o),

        .M_AXI_WDATA_i     (M_AXI_WDATA_i),
        .M_AXI_WSTRB_i     (M_AXI_WSTRB_i),
        .M_AXI_WLAST_i     (M_AXI_WLAST_i),
        .M_AXI_WVALID_i    (M_AXI_WVALID_i),
        .M_AXI_WREADY_o    (M_AXI_WREADY_o),

        .M_AXI_BID_o       (M_AXI_BID_o),
        .M_AXI_BRESP_o     (M_AXI_BRESP_o),
        .M_AXI_BVALID_o    (M_AXI_BVALID_o),
        .M_AXI_BREADY_i    (M_AXI_BREADY_i),

        .M_AXI_ARID_i      (M_AXI_ARID_i),
        .M_AXI_ARADDR_i    (M_AXI_ARADDR_i),
        .M_AXI_ARLEN_i     (M_AXI_ARLEN_i),
        .M_AXI_ARSIZE_i    (M_AXI_ARSIZE_i),
        .M_AXI_ARBURST_i   (M_AXI_ARBURST_i),
        .M_AXI_ARVALID_i   (M_AXI_ARVALID_i),
        .M_AXI_ARREADY_o   (M_AXI_ARREADY_o),

        .M_AXI_RID_o       (M_AXI_RID_o),
        .M_AXI_RDATA_o     (M_AXI_RDATA_o),
        .M_AXI_RRESP_o     (M_AXI_RRESP_o),
        .M_AXI_RLAST_o     (M_AXI_RLAST_o),
        .M_AXI_RVALID_o    (M_AXI_RVALID_o),
        .M_AXI_RREADY_i    (M_AXI_RREADY_i),

        // 新增控制端口
        .arbiter_type      (1'b0),
        .r_order_grant_i   ({SLV_AMT{1'b1}})
    );

    //=========================================================
    // INIT
    //=========================================================

    initial begin

        M_AXI_AWID_i      = 0;
        M_AXI_AWADDR_i    = 0;
        M_AXI_AWLEN_i     = 0;
        M_AXI_AWSIZE_i    = 0;
        M_AXI_AWBURST_i   = 0;
        M_AXI_AWVALID_i   = 0;

        M_AXI_WDATA_i     = 0;
        M_AXI_WSTRB_i     = 0;
        M_AXI_WLAST_i     = 0;
        M_AXI_WVALID_i    = 0;

        M_AXI_BREADY_i    = 0;

        M_AXI_ARID_i      = 0;
        M_AXI_ARADDR_i    = 0;
        M_AXI_ARLEN_i     = 0;
        M_AXI_ARSIZE_i    = 0;
        M_AXI_ARBURST_i   = 0;
        M_AXI_ARVALID_i   = 0;

        M_AXI_RREADY_i    = 0;

        wait(AXI_RSTn);

        repeat(10) @(posedge AXI_CLK);

        //=====================================================
        // TEST1 WRITE
        //=====================================================

        axi_write(
            0,
            4'h1,
            32'h1000_0000,
            32'hAAAA_5555
        );

        //=====================================================
        // TEST2 READ
        //=====================================================

        axi_read(
            0,
            4'h2,
            32'h1000_0000
        );

        repeat(100) @(posedge AXI_CLK);

        $display("=================================");
        $display(" AXI TB PASS ");
        $display("=================================");

        $finish;

    end

    //=========================================================
    // AXI WRITE TASK
    //=========================================================

    task automatic axi_write;

        input integer mst;
        input [W_ID-1:0] id;
        input [31:0] addr;
        input [31:0] data;

        begin

            @(posedge AXI_CLK);

            M_AXI_AWID_i[mst*W_ID +: W_ID]       <= id;
            M_AXI_AWADDR_i[mst*ADDR_WIDTH +: ADDR_WIDTH] <= addr;
            M_AXI_AWLEN_i[mst*LEN_W +: LEN_W]   <= 0;
            M_AXI_AWSIZE_i[mst*SIZE_W +: SIZE_W]<= 3'b010;
            M_AXI_AWBURST_i[mst*BURST_W +: BURST_W] <= 2'b01;
            M_AXI_AWVALID_i[mst] <= 1'b1;

            wait(M_AXI_AWREADY_o[mst]);

            @(posedge AXI_CLK);

            M_AXI_AWVALID_i[mst] <= 0;

            M_AXI_WDATA_i[mst*DATA_WIDTH +: DATA_WIDTH] <= data;
            M_AXI_WSTRB_i[mst*W_STRB +: W_STRB] <= '1;
            M_AXI_WLAST_i[mst] <= 1'b1;
            M_AXI_WVALID_i[mst] <= 1'b1;

            wait(M_AXI_WREADY_o[mst]);

            @(posedge AXI_CLK);

            M_AXI_WVALID_i[mst] <= 0;
            M_AXI_WLAST_i[mst]  <= 0;

            M_AXI_BREADY_i[mst] <= 1'b1;

            fork
                begin
                    wait(M_AXI_BVALID_o[mst]);
                end
                begin
                    repeat(1000) @(posedge AXI_CLK);
                    $fatal("WRITE TIMEOUT");
                end
            join_any

            disable fork;

            $display("[%0t] WRITE OK addr=%h data=%h",
                        $time,
                        addr,
                        data);

            @(posedge AXI_CLK);

            M_AXI_BREADY_i[mst] <= 0;

        end
    endtask

    //=========================================================
    // AXI READ TASK
    //=========================================================

    task automatic axi_read;

        input integer mst;
        input [W_ID-1:0] id;
        input [31:0] addr;

        begin

            @(posedge AXI_CLK);

            M_AXI_ARID_i[mst*W_ID +: W_ID] <= id;
            M_AXI_ARADDR_i[mst*ADDR_WIDTH +: ADDR_WIDTH] <= addr;
            M_AXI_ARLEN_i[mst*LEN_W +: LEN_W] <= 0;
            M_AXI_ARSIZE_i[mst*SIZE_W +: SIZE_W] <= 3'b010;
            M_AXI_ARBURST_i[mst*BURST_W +: BURST_W] <= 2'b01;
            M_AXI_ARVALID_i[mst] <= 1'b1;

            wait(M_AXI_ARREADY_o[mst]);

            @(posedge AXI_CLK);

            M_AXI_ARVALID_i[mst] <= 0;

            M_AXI_RREADY_i[mst] <= 1'b1;

            fork
                begin
                    wait(M_AXI_RVALID_o[mst]);
                end
                begin
                    repeat(1000) @(posedge AXI_CLK);
                    $fatal("READ TIMEOUT");
                end
            join_any

            disable fork;

            $display("[%0t] READ OK addr=%h data=%h",
                        $time,
                        addr,
                        M_AXI_RDATA_o[mst*DATA_WIDTH +: DATA_WIDTH]);

            @(posedge AXI_CLK);

            M_AXI_RREADY_i[mst] <= 0;

        end
    endtask

endmodule