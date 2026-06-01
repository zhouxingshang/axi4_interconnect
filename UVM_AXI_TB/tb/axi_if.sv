//=============================================================================
// AXI Interface: parameterized 4-Master x 4-Slave interconnect ports
//=============================================================================
`ifndef AXI_IF_SV
`define AXI_IF_SV

interface axi_if #(
    parameter MST_AMT       = 4,
    parameter SLV_AMT       = 4,
    parameter W_ID          = 4,
    parameter W_CID         = 2,
    parameter W_SID         = $clog2(MST_AMT) + W_CID + W_ID,
    parameter ADDR_WIDTH    = 32,
    parameter DATA_WIDTH    = 32,
    parameter LEN_W         = 8,
    parameter SIZE_W        = 3,
    parameter BURST_W       = 2,
    parameter RESP_W        = 2,
    parameter W_STRB        = DATA_WIDTH / 8
)(
    input wire ACLK,
    input wire ARSTn
);

    //---- Master-side (interconnect M_AXI ports) ----
    wire [W_ID*MST_AMT-1:0]            M_AWID;
    wire [ADDR_WIDTH*MST_AMT-1:0]      M_AWADDR;
    wire [LEN_W*MST_AMT-1:0]           M_AWLEN;
    wire [SIZE_W*MST_AMT-1:0]          M_AWSIZE;
    wire [BURST_W*MST_AMT-1:0]         M_AWBURST;
    wire [MST_AMT-1:0]                 M_AWVALID;
    wire [MST_AMT-1:0]                 M_AWREADY;

    wire [DATA_WIDTH*MST_AMT-1:0]      M_WDATA;
    wire [W_STRB*MST_AMT-1:0]          M_WSTRB;
    wire [MST_AMT-1:0]                 M_WLAST;
    wire [MST_AMT-1:0]                 M_WVALID;
    wire [MST_AMT-1:0]                 M_WREADY;

    wire [W_ID*MST_AMT-1:0]            M_BID;
    wire [RESP_W*MST_AMT-1:0]          M_BRESP;
    wire [MST_AMT-1:0]                 M_BVALID;
    wire [MST_AMT-1:0]                 M_BREADY;

    wire [W_ID*MST_AMT-1:0]            M_ARID;
    wire [ADDR_WIDTH*MST_AMT-1:0]      M_ARADDR;
    wire [LEN_W*MST_AMT-1:0]           M_ARLEN;
    wire [SIZE_W*MST_AMT-1:0]          M_ARSIZE;
    wire [BURST_W*MST_AMT-1:0]         M_ARBURST;
    wire [MST_AMT-1:0]                 M_ARVALID;
    wire [MST_AMT-1:0]                 M_ARREADY;

    wire [W_ID*MST_AMT-1:0]            M_RID;
    wire [DATA_WIDTH*MST_AMT-1:0]      M_RDATA;
    wire [RESP_W*MST_AMT-1:0]          M_RRESP;
    wire [MST_AMT-1:0]                 M_RLAST;
    wire [MST_AMT-1:0]                 M_RVALID;
    wire [MST_AMT-1:0]                 M_RREADY;

    //---- Slave-side (interconnect S_AXI ports) ----
    wire [W_SID*SLV_AMT-1:0]           S_AWID;
    wire [ADDR_WIDTH*SLV_AMT-1:0]      S_AWADDR;
    wire [LEN_W*SLV_AMT-1:0]           S_AWLEN;
    wire [SIZE_W*SLV_AMT-1:0]          S_AWSIZE;
    wire [BURST_W*SLV_AMT-1:0]         S_AWBURST;
    wire [SLV_AMT-1:0]                 S_AWVALID;
    wire [SLV_AMT-1:0]                 S_AWREADY;

    wire [DATA_WIDTH*SLV_AMT-1:0]      S_WDATA;
    wire [W_STRB*SLV_AMT-1:0]          S_WSTRB;
    wire [SLV_AMT-1:0]                 S_WLAST;
    wire [SLV_AMT-1:0]                 S_WVALID;
    wire [SLV_AMT-1:0]                 S_WREADY;

    wire [W_SID*SLV_AMT-1:0]           S_BID;
    wire [RESP_W*SLV_AMT-1:0]          S_BRESP;
    wire [SLV_AMT-1:0]                 S_BVALID;
    wire [SLV_AMT-1:0]                 S_BREADY;

    wire [W_SID*SLV_AMT-1:0]           S_ARID;
    wire [ADDR_WIDTH*SLV_AMT-1:0]      S_ARADDR;
    wire [LEN_W*SLV_AMT-1:0]           S_ARLEN;
    wire [SIZE_W*SLV_AMT-1:0]          S_ARSIZE;
    wire [BURST_W*SLV_AMT-1:0]         S_ARBURST;
    wire [SLV_AMT-1:0]                 S_ARVALID;
    wire [SLV_AMT-1:0]                 S_ARREADY;

    wire [W_SID*SLV_AMT-1:0]           S_RID;
    wire [DATA_WIDTH*SLV_AMT-1:0]      S_RDATA;
    wire [RESP_W*SLV_AMT-1:0]          S_RRESP;
    wire [SLV_AMT-1:0]                 S_RLAST;
    wire [SLV_AMT-1:0]                 S_RVALID;
    wire [SLV_AMT-1:0]                 S_RREADY;

    //---- Modports ----
    modport master_mp (
        input ACLK, ARSTn,
        output M_AWID, M_AWADDR, M_AWLEN, M_AWSIZE, M_AWBURST, M_AWVALID,
        input  M_AWREADY,
        output M_WDATA, M_WSTRB, M_WLAST, M_WVALID,
        input  M_WREADY,
        input  M_BID, M_BRESP, M_BVALID,
        output M_BREADY,
        output M_ARID, M_ARADDR, M_ARLEN, M_ARSIZE, M_ARBURST, M_ARVALID,
        input  M_ARREADY,
        input  M_RID, M_RDATA, M_RRESP, M_RLAST, M_RVALID,
        output M_RREADY
    );

    modport slave_mp (
        input ACLK, ARSTn,
        input  S_AWID, S_AWADDR, S_AWLEN, S_AWSIZE, S_AWBURST, S_AWVALID,
        output S_AWREADY,
        input  S_WDATA, S_WSTRB, S_WLAST, S_WVALID,
        output S_WREADY,
        output S_BID, S_BRESP, S_BVALID,
        input  S_BREADY,
        input  S_ARID, S_ARADDR, S_ARLEN, S_ARSIZE, S_ARBURST, S_ARVALID,
        output S_ARREADY,
        output S_RID, S_RDATA, S_RRESP, S_RLAST, S_RVALID,
        input  S_RREADY
    );

    modport monitor_mp (
        input ACLK, ARSTn,
        input M_AWID, M_AWADDR, M_AWLEN, M_AWSIZE, M_AWBURST, M_AWVALID, M_AWREADY,
        input M_WDATA, M_WSTRB, M_WLAST, M_WVALID, M_WREADY,
        input M_BID, M_BRESP, M_BVALID, M_BREADY,
        input M_ARID, M_ARADDR, M_ARLEN, M_ARSIZE, M_ARBURST, M_ARVALID, M_ARREADY,
        input M_RID, M_RDATA, M_RRESP, M_RLAST, M_RVALID, M_RREADY,
        input S_AWID, S_AWADDR, S_AWLEN, S_AWSIZE, S_AWBURST, S_AWVALID, S_AWREADY,
        input S_WDATA, S_WSTRB, S_WLAST, S_WVALID, S_WREADY,
        input S_BID, S_BRESP, S_BVALID, S_BREADY,
        input S_ARID, S_ARADDR, S_ARLEN, S_ARSIZE, S_ARBURST, S_ARVALID, S_ARREADY,
        input S_RID, S_RDATA, S_RRESP, S_RLAST, S_RVALID, S_RREADY
    );

endinterface

`endif
