//=============================================================================
// Module: axi_interconnect
// Desc  : Top-level AXI Interconnect Wrapper with symmetric FIFOs on both sides
//         of the crossbar.
//         - Master side: pre-FIFO for AW/AR/W, post-FIFO for B/R
//         - Slave side:  post-FIFO for AW/AR/W, pre-FIFO for B/R
//=============================================================================
module axi_interconnect
#(
    parameter MST_AMT           = 4,
    parameter SLV_AMT           = 4,
    parameter OUTSTANDING_AMT   = 8,
    parameter W_ID              = 4,
    parameter DATA_WIDTH        = 32,
    parameter ADDR_WIDTH        = 32,
    parameter LEN_W             = 8,
    parameter SIZE_W            = 3,
    parameter BURST_W           = 2,
    parameter RESP_W            = 2,
    parameter W_STRB            = DATA_WIDTH / 8,
    parameter W_CID             = 2,               // cross_4k_if ID extension width
    parameter W_MID             = W_CID + W_ID,          // master-side ID width after cross_4k_if
    parameter W_SID             = $clog2(MST_AMT) + W_CID + W_ID,  // slave-side ID width

    // Address mapping (default: upper bits select slave)
    parameter [(SLV_AMT*ADDR_WIDTH)-1:0]  SLV_ADDR_BASE = '0,
    parameter [(SLV_AMT*8)-1:0]           SLV_ADDR_LEN  = {SLV_AMT{8'd12}}
)
(
    input   wire                      AXI_CLK,
    input   wire                      AXI_RSTn,

    // ========== Master Flattened Interface ==========
    // AW
    input   wire  [W_ID*MST_AMT-1             : 0]  M_AXI_AWID_i,
    input   wire  [ADDR_WIDTH*MST_AMT-1       : 0]  M_AXI_AWADDR_i,
    input   wire  [LEN_W*MST_AMT-1            : 0]  M_AXI_AWLEN_i,
    input   wire  [SIZE_W*MST_AMT-1           : 0]  M_AXI_AWSIZE_i,
    input   wire  [BURST_W*MST_AMT-1          : 0]  M_AXI_AWBURST_i,
    input   wire  [MST_AMT-1                  : 0]  M_AXI_AWVALID_i,
    output  wire  [MST_AMT-1                  : 0]  M_AXI_AWREADY_o,
    // W
    input   wire  [DATA_WIDTH*MST_AMT-1       : 0]  M_AXI_WDATA_i,
    input   wire  [W_STRB*MST_AMT-1           : 0]  M_AXI_WSTRB_i,
    input   wire  [MST_AMT-1                  : 0]  M_AXI_WLAST_i,
    input   wire  [MST_AMT-1                  : 0]  M_AXI_WVALID_i,
    output  wire  [MST_AMT-1                  : 0]  M_AXI_WREADY_o,
    // B
    output  wire  [W_ID*MST_AMT-1             : 0]  M_AXI_BID_o,
    output  wire  [RESP_W*MST_AMT-1           : 0]  M_AXI_BRESP_o,
    output  wire  [MST_AMT-1                  : 0]  M_AXI_BVALID_o,
    input   wire  [MST_AMT-1                  : 0]  M_AXI_BREADY_i,
    // AR
    input   wire  [W_ID*MST_AMT-1             : 0]  M_AXI_ARID_i,
    input   wire  [ADDR_WIDTH*MST_AMT-1       : 0]  M_AXI_ARADDR_i,
    input   wire  [LEN_W*MST_AMT-1            : 0]  M_AXI_ARLEN_i,
    input   wire  [SIZE_W*MST_AMT-1           : 0]  M_AXI_ARSIZE_i,
    input   wire  [BURST_W*MST_AMT-1          : 0]  M_AXI_ARBURST_i,
    input   wire  [MST_AMT-1                  : 0]  M_AXI_ARVALID_i,
    output  wire  [MST_AMT-1                  : 0]  M_AXI_ARREADY_o,
    // R
    output  wire  [W_ID*MST_AMT-1             : 0]  M_AXI_RID_o,
    output  wire  [DATA_WIDTH*MST_AMT-1       : 0]  M_AXI_RDATA_o,
    output  wire  [RESP_W*MST_AMT-1           : 0]  M_AXI_RRESP_o,
    output  wire  [MST_AMT-1                  : 0]  M_AXI_RLAST_o,
    output  wire  [MST_AMT-1                  : 0]  M_AXI_RVALID_o,
    input   wire  [MST_AMT-1                  : 0]  M_AXI_RREADY_i,

    // ========== Slave Flattened Interface ==========
    // AW
    output  wire  [W_SID*SLV_AMT-1            : 0]  S_AXI_AWID_o,
    output  wire  [ADDR_WIDTH*SLV_AMT-1       : 0]  S_AXI_AWADDR_o,
    output  wire  [LEN_W*SLV_AMT-1            : 0]  S_AXI_AWLEN_o,
    output  wire  [SIZE_W*SLV_AMT-1           : 0]  S_AXI_AWSIZE_o,
    output  wire  [BURST_W*SLV_AMT-1          : 0]  S_AXI_AWBURST_o,
    output  wire  [SLV_AMT-1                  : 0]  S_AXI_AWVALID_o,
    input   wire  [SLV_AMT-1                  : 0]  S_AXI_AWREADY_i,
    // W
    output  wire  [DATA_WIDTH*SLV_AMT-1       : 0]  S_AXI_WDATA_o,
    output  wire  [W_STRB*SLV_AMT-1           : 0]  S_AXI_WSTRB_o,
    output  wire  [SLV_AMT-1                  : 0]  S_AXI_WLAST_o,
    output  wire  [SLV_AMT-1                  : 0]  S_AXI_WVALID_o,
    input   wire  [SLV_AMT-1                  : 0]  S_AXI_WREADY_i,
    // B
    input   wire  [W_SID*SLV_AMT-1            : 0]  S_AXI_BID_i,
    input   wire  [RESP_W*SLV_AMT-1           : 0]  S_AXI_BRESP_i,
    input   wire  [SLV_AMT-1                  : 0]  S_AXI_BVALID_i,
    output  wire  [SLV_AMT-1                  : 0]  S_AXI_BREADY_o,
    // AR
    output  wire  [W_SID*SLV_AMT-1            : 0]  S_AXI_ARID_o,
    output  wire  [ADDR_WIDTH*SLV_AMT-1       : 0]  S_AXI_ARADDR_o,
    output  wire  [LEN_W*SLV_AMT-1            : 0]  S_AXI_ARLEN_o,
    output  wire  [SIZE_W*SLV_AMT-1           : 0]  S_AXI_ARSIZE_o,
    output  wire  [BURST_W*SLV_AMT-1          : 0]  S_AXI_ARBURST_o,
    output  wire  [SLV_AMT-1                  : 0]  S_AXI_ARVALID_o,
    input   wire  [SLV_AMT-1                  : 0]  S_AXI_ARREADY_i,
    // R
    input   wire  [W_SID*SLV_AMT-1            : 0]  S_AXI_RID_i,
    input   wire  [DATA_WIDTH*SLV_AMT-1       : 0]  S_AXI_RDATA_i,
    input   wire  [RESP_W*SLV_AMT-1           : 0]  S_AXI_RRESP_i,
    input   wire  [SLV_AMT-1                  : 0]  S_AXI_RLAST_i,
    input   wire  [SLV_AMT-1                  : 0]  S_AXI_RVALID_i,
    output  wire  [SLV_AMT-1                  : 0]  S_AXI_RREADY_o,

    // ========== Control Ports ==========
    input   wire                      arbiter_type
);

//=============================================================================
// Internal Arrays: Unpacked per Master
//=============================================================================
wire [W_ID-1:0]             m_awid      [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]       m_awaddr    [0:MST_AMT-1];
wire [LEN_W-1:0]            m_awlen     [0:MST_AMT-1];
wire [SIZE_W-1:0]           m_awsize    [0:MST_AMT-1];
wire [BURST_W-1:0]          m_awburst   [0:MST_AMT-1];
wire                        m_awvalid   [0:MST_AMT-1];
wire                        m_awready   [0:MST_AMT-1];

wire [DATA_WIDTH-1:0]       m_wdata     [0:MST_AMT-1];
wire [W_STRB-1:0]           m_wstrb     [0:MST_AMT-1];
wire                        m_wlast     [0:MST_AMT-1];
wire                        m_wvalid    [0:MST_AMT-1];
wire                        m_wready    [0:MST_AMT-1];

wire [W_ID-1:0]             m_bid       [0:MST_AMT-1];
wire [RESP_W-1:0]           m_bresp     [0:MST_AMT-1];
wire                        m_bvalid    [0:MST_AMT-1];
wire                        m_bready    [0:MST_AMT-1];

wire [W_ID-1:0]             m_arid      [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]       m_araddr    [0:MST_AMT-1];
wire [LEN_W-1:0]            m_arlen     [0:MST_AMT-1];
wire [SIZE_W-1:0]           m_arsize    [0:MST_AMT-1];
wire [BURST_W-1:0]          m_arburst   [0:MST_AMT-1];
wire                        m_arvalid   [0:MST_AMT-1];
wire                        m_arready   [0:MST_AMT-1];

wire [W_ID-1:0]             m_rid       [0:MST_AMT-1];
wire [DATA_WIDTH-1:0]       m_rdata     [0:MST_AMT-1];
wire [RESP_W-1:0]           m_rresp     [0:MST_AMT-1];
wire                        m_rlast     [0:MST_AMT-1];
wire                        m_rvalid    [0:MST_AMT-1];
wire                        m_rready    [0:MST_AMT-1];

// R channel: between R post-FIFO and axi_split_r_merge
wire [W_SID-1:0]            r_fifo_rid   [0:MST_AMT-1];
wire [DATA_WIDTH-1:0]       r_fifo_rdata [0:MST_AMT-1];
wire [RESP_W-1:0]           r_fifo_rresp [0:MST_AMT-1];
wire                        r_fifo_rlast [0:MST_AMT-1];
wire                        r_fifo_rvalid [0:MST_AMT-1];
wire                        r_fifo_rready [0:MST_AMT-1];

//=============================================================================
// Pre-FIFO buses: between cross_4k_if (or master for W) and pre-FIFO input
//=============================================================================
wire [W_MID-1:0]            c4k_awid    [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]       c4k_awaddr  [0:MST_AMT-1];
wire [LEN_W-1:0]            c4k_awlen   [0:MST_AMT-1];
wire [SIZE_W-1:0]           c4k_awsize  [0:MST_AMT-1];
wire [BURST_W-1:0]          c4k_awburst [0:MST_AMT-1];
wire                        c4k_awvalid [0:MST_AMT-1];
wire                        c4k_awready [0:MST_AMT-1];

wire [W_MID-1:0]            c4k_arid    [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]       c4k_araddr  [0:MST_AMT-1];
wire [LEN_W-1:0]            c4k_arlen   [0:MST_AMT-1];
wire [SIZE_W-1:0]           c4k_arsize  [0:MST_AMT-1];
wire [BURST_W-1:0]          c4k_arburst [0:MST_AMT-1];
wire                        c4k_arvalid [0:MST_AMT-1];
wire                        c4k_arready [0:MST_AMT-1];

wire [DATA_WIDTH-1:0]       c4k_wdata   [0:MST_AMT-1];
wire [W_STRB-1:0]           c4k_wstrb   [0:MST_AMT-1];
wire                        c4k_wlast   [0:MST_AMT-1];
wire                        c4k_wvalid  [0:MST_AMT-1];
wire                        c4k_wready  [0:MST_AMT-1];

wire [W_SID-1:0]            c4k_bid     [0:MST_AMT-1];
wire [RESP_W-1:0]           c4k_bresp   [0:MST_AMT-1];
wire                        c4k_bvalid  [0:MST_AMT-1];
wire                        c4k_bready  [0:MST_AMT-1];

//=============================================================================
// Post-FIFO buses (Master side): between crossbar output and FIFO → master
//=============================================================================
wire [W_MID-1:0]            M_AWID     [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]       M_AWADDR   [0:MST_AMT-1];
wire [LEN_W-1:0]            M_AWLEN    [0:MST_AMT-1];
wire [SIZE_W-1:0]           M_AWSIZE   [0:MST_AMT-1];
wire [BURST_W-1:0]          M_AWBURST  [0:MST_AMT-1];
wire                        M_AWVALID  [0:MST_AMT-1];
wire                        M_AWREADY  [0:MST_AMT-1];

wire [DATA_WIDTH-1:0]       M_WDATA    [0:MST_AMT-1];
wire [W_STRB-1:0]           M_WSTRB    [0:MST_AMT-1];
wire                        M_WLAST    [0:MST_AMT-1];
wire                        M_WVALID   [0:MST_AMT-1];
wire                        M_WREADY   [0:MST_AMT-1];

wire [W_SID-1:0]            M_BID      [0:MST_AMT-1];
wire [RESP_W-1:0]           M_BRESP    [0:MST_AMT-1];
wire                        M_BVALID   [0:MST_AMT-1];
wire                        M_BREADY   [0:MST_AMT-1];

wire [W_MID-1:0]            M_ARID     [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]       M_ARADDR   [0:MST_AMT-1];
wire [LEN_W-1:0]            M_ARLEN    [0:MST_AMT-1];
wire [SIZE_W-1:0]           M_ARSIZE   [0:MST_AMT-1];
wire [BURST_W-1:0]          M_ARBURST  [0:MST_AMT-1];
wire                        M_ARVALID  [0:MST_AMT-1];
wire                        M_ARREADY  [0:MST_AMT-1];

wire [W_SID-1:0]            M_RID      [0:MST_AMT-1];
wire [DATA_WIDTH-1:0]       M_RDATA    [0:MST_AMT-1];
wire [RESP_W-1:0]           M_RRESP    [0:MST_AMT-1];
wire                        M_RLAST    [0:MST_AMT-1];
wire                        M_RVALID   [0:MST_AMT-1];
wire                        M_RREADY   [0:MST_AMT-1];
wire                        M_RREADY_FIFO [0:MST_AMT-1];  // R FIFO wr_rdy before clear-gate

wire [W_SID*MST_AMT-1:0]    M_RSID_PACKED;
wire [W_SID-1:0]            M_RSID     [0:MST_AMT-1];

//=============================================================================
// NEW: Slave side internal signals (crossbar output → slave FIFO input)
//=============================================================================
wire [W_SID-1:0]            S_AWID     [0:SLV_AMT-1];
wire [ADDR_WIDTH-1:0]       S_AWADDR   [0:SLV_AMT-1];
wire [LEN_W-1:0]            S_AWLEN    [0:SLV_AMT-1];
wire [SIZE_W-1:0]           S_AWSIZE   [0:SLV_AMT-1];
wire [BURST_W-1:0]          S_AWBURST  [0:SLV_AMT-1];
wire                        S_AWVALID  [0:SLV_AMT-1];
wire                        S_AWREADY  [0:SLV_AMT-1];

wire [DATA_WIDTH-1:0]       S_WDATA    [0:SLV_AMT-1];
wire [W_STRB-1:0]           S_WSTRB    [0:SLV_AMT-1];
wire                        S_WLAST    [0:SLV_AMT-1];
wire                        S_WVALID   [0:SLV_AMT-1];
wire                        S_WREADY   [0:SLV_AMT-1];

wire [W_SID-1:0]            S_ARID     [0:SLV_AMT-1];
wire [ADDR_WIDTH-1:0]       S_ARADDR   [0:SLV_AMT-1];
wire [LEN_W-1:0]            S_ARLEN    [0:SLV_AMT-1];
wire [SIZE_W-1:0]           S_ARSIZE   [0:SLV_AMT-1];
wire [BURST_W-1:0]          S_ARBURST  [0:SLV_AMT-1];
wire                        S_ARVALID  [0:SLV_AMT-1];
wire                        S_ARREADY  [0:SLV_AMT-1];

// Slave side B/R pre-FIFO (slave → crossbar)
wire [W_SID-1:0]            s_bid_fifo      [0:SLV_AMT-1];
wire [RESP_W-1:0]           s_bresp_fifo    [0:SLV_AMT-1];
wire                        s_bvalid_fifo   [0:SLV_AMT-1];
wire                        s_bready_fifo   [0:SLV_AMT-1];

wire [W_SID-1:0]            s_rid_fifo      [0:SLV_AMT-1];
wire [DATA_WIDTH-1:0]       s_rdata_fifo    [0:SLV_AMT-1];
wire [RESP_W-1:0]           s_rresp_fifo    [0:SLV_AMT-1];
wire                        s_rlast_fifo    [0:SLV_AMT-1];
wire                        s_rvalid_fifo   [0:SLV_AMT-1];
wire                        s_rready_fifo   [0:SLV_AMT-1];

// Slave side FIFO output to top (or from top for B/R)
wire [W_SID-1:0]            s_awid_out      [0:SLV_AMT-1];
wire [ADDR_WIDTH-1:0]       s_awaddr_out    [0:SLV_AMT-1];
wire [LEN_W-1:0]            s_awlen_out     [0:SLV_AMT-1];
wire [SIZE_W-1:0]           s_awsize_out    [0:SLV_AMT-1];
wire [BURST_W-1:0]          s_awburst_out   [0:SLV_AMT-1];
wire                        s_awvalid_out   [0:SLV_AMT-1];
wire                        s_awready_out   [0:SLV_AMT-1];   // from top

wire [DATA_WIDTH-1:0]       s_wdata_out     [0:SLV_AMT-1];
wire [W_STRB-1:0]           s_wstrb_out     [0:SLV_AMT-1];
wire                        s_wlast_out     [0:SLV_AMT-1];
wire                        s_wvalid_out    [0:SLV_AMT-1];
wire                        s_wready_out    [0:SLV_AMT-1];   // from top

wire [W_SID-1:0]            s_arid_out      [0:SLV_AMT-1];
wire [ADDR_WIDTH-1:0]       s_araddr_out    [0:SLV_AMT-1];
wire [LEN_W-1:0]            s_arlen_out     [0:SLV_AMT-1];
wire [SIZE_W-1:0]           s_arsize_out    [0:SLV_AMT-1];
wire [BURST_W-1:0]          s_arburst_out   [0:SLV_AMT-1];
wire                        s_arvalid_out   [0:SLV_AMT-1];
wire                        s_arready_out   [0:SLV_AMT-1];   // from top

// B/R from top to pre-FIFO
wire [W_SID-1:0]            s_bid_in        [0:SLV_AMT-1];
wire [RESP_W-1:0]           s_bresp_in      [0:SLV_AMT-1];
wire                        s_bvalid_in     [0:SLV_AMT-1];
wire                        s_bready_in     [0:SLV_AMT-1];   // to top

wire [W_SID-1:0]            s_rid_in        [0:SLV_AMT-1];
wire [DATA_WIDTH-1:0]       s_rdata_in      [0:SLV_AMT-1];
wire [RESP_W-1:0]           s_rresp_in      [0:SLV_AMT-1];
wire                        s_rlast_in      [0:SLV_AMT-1];
wire                        s_rvalid_in     [0:SLV_AMT-1];
wire                        s_rready_in     [0:SLV_AMT-1];   // to top

//=============================================================================
// Read Reorder Signals — single shared sid_buffer + reorder
//=============================================================================
wire [W_SID-1:0]            sid_buf_out         [0:3];          // DEPTH=4
wire [SLV_AMT-1:0]          reorder_grant;

// sid_buffer backpressure: s_push_rdy per slave, gated AR ready
wire [SLV_AMT-1:0]          sid_buf_push_rdy;                   // per-slave
wire [SLV_AMT-1:0]          S_ARREADY_FIFO;                // AR FIFO wr_rdy before gate

//=============================================================================
// Port Flattening (Unpack Master Inputs / Pack Master Outputs)
//=============================================================================
genvar m, s;
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : UNPACK_MST
        assign m_awid[m]    = M_AXI_AWID_i[W_ID*(m+1)-1 -: W_ID];
        assign m_awaddr[m]  = M_AXI_AWADDR_i[ADDR_WIDTH*(m+1)-1 -: ADDR_WIDTH];
        assign m_awlen[m]   = M_AXI_AWLEN_i[LEN_W*(m+1)-1 -: LEN_W];
        assign m_awsize[m]  = M_AXI_AWSIZE_i[SIZE_W*(m+1)-1 -: SIZE_W];
        assign m_awburst[m] = M_AXI_AWBURST_i[BURST_W*(m+1)-1 -: BURST_W];
        assign m_awvalid[m] = M_AXI_AWVALID_i[m];
        assign M_AXI_AWREADY_o[m] = m_awready[m];

        assign m_wdata[m]   = M_AXI_WDATA_i[DATA_WIDTH*(m+1)-1 -: DATA_WIDTH];
        assign m_wstrb[m]   = M_AXI_WSTRB_i[W_STRB*(m+1)-1 -: W_STRB];
        assign m_wlast[m]   = M_AXI_WLAST_i[m];
        assign m_wvalid[m]  = M_AXI_WVALID_i[m];
        assign M_AXI_WREADY_o[m] = m_wready[m];

        assign m_bready[m]  = M_AXI_BREADY_i[m];
        assign M_AXI_BID_o[W_ID*(m+1)-1 -: W_ID] = m_bid[m];
        assign M_AXI_BRESP_o[RESP_W*(m+1)-1 -: RESP_W] = m_bresp[m];
        assign M_AXI_BVALID_o[m] = m_bvalid[m];

        assign m_arid[m]    = M_AXI_ARID_i[W_ID*(m+1)-1 -: W_ID];
        assign m_araddr[m]  = M_AXI_ARADDR_i[ADDR_WIDTH*(m+1)-1 -: ADDR_WIDTH];
        assign m_arlen[m]   = M_AXI_ARLEN_i[LEN_W*(m+1)-1 -: LEN_W];
        assign m_arsize[m]  = M_AXI_ARSIZE_i[SIZE_W*(m+1)-1 -: SIZE_W];
        assign m_arburst[m] = M_AXI_ARBURST_i[BURST_W*(m+1)-1 -: BURST_W];
        assign m_arvalid[m] = M_AXI_ARVALID_i[m];
        assign M_AXI_ARREADY_o[m] = m_arready[m];

        assign m_rready[m]  = M_AXI_RREADY_i[m];
        assign M_AXI_RID_o[W_ID*(m+1)-1 -: W_ID] = m_rid[m];
        assign M_AXI_RDATA_o[DATA_WIDTH*(m+1)-1 -: DATA_WIDTH] = m_rdata[m];
        assign M_AXI_RRESP_o[RESP_W*(m+1)-1 -: RESP_W] = m_rresp[m];
        assign M_AXI_RLAST_o[m] = m_rlast[m];
        assign M_AXI_RVALID_o[m] = m_rvalid[m];

    end
endgenerate

//=============================================================================
// Unpack Slave Flattened Ports (connect to internal FIFO interfaces)
//=============================================================================
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : UNPACK_SLV
        // Outputs (from slave FIFO to top)
        assign S_AXI_AWID_o   [W_SID*(s+1)-1 -: W_SID]   = s_awid_out[s];
        assign S_AXI_AWADDR_o [ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH] = s_awaddr_out[s];
        assign S_AXI_AWLEN_o  [LEN_W*(s+1)-1 -: LEN_W]   = s_awlen_out[s];
        assign S_AXI_AWSIZE_o [SIZE_W*(s+1)-1 -: SIZE_W] = s_awsize_out[s];
        assign S_AXI_AWBURST_o[BURST_W*(s+1)-1 -: BURST_W] = s_awburst_out[s];
        assign S_AXI_AWVALID_o[s] = s_awvalid_out[s];
        assign s_awready_out[s] = S_AXI_AWREADY_i[s];

        assign S_AXI_WDATA_o [DATA_WIDTH*(s+1)-1 -: DATA_WIDTH] = s_wdata_out[s];
        assign S_AXI_WSTRB_o [W_STRB*(s+1)-1 -: W_STRB] = s_wstrb_out[s];
        assign S_AXI_WLAST_o [s] = s_wlast_out[s];
        assign S_AXI_WVALID_o[s] = s_wvalid_out[s];
        assign s_wready_out[s] = S_AXI_WREADY_i[s];

        assign S_AXI_ARID_o    [W_SID*(s+1)-1 -: W_SID]   = s_arid_out[s];
        assign S_AXI_ARADDR_o  [ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH] = s_araddr_out[s];
        assign S_AXI_ARLEN_o   [LEN_W*(s+1)-1 -: LEN_W]   = s_arlen_out[s];
        assign S_AXI_ARSIZE_o  [SIZE_W*(s+1)-1 -: SIZE_W] = s_arsize_out[s];
        assign S_AXI_ARBURST_o [BURST_W*(s+1)-1 -: BURST_W] = s_arburst_out[s];
        assign S_AXI_ARVALID_o [s] = s_arvalid_out[s];
        assign s_arready_out[s] = S_AXI_ARREADY_i[s];
            // TRACE: slave AR valid
            always @(posedge AXI_CLK) begin
                if (s_arvalid_out[s])
                    $display("[TRACE] %0t AR at slave s=%0d addr=0x%08h", $time, s, s_araddr_out[s]);
            end

        // Inputs (from top to slave pre-FIFO)
        assign s_bid_in[s]   = S_AXI_BID_i[W_SID*(s+1)-1 -: W_SID];
        assign s_bresp_in[s] = S_AXI_BRESP_i[RESP_W*(s+1)-1 -: RESP_W];
        assign s_bvalid_in[s]= S_AXI_BVALID_i[s];
        assign S_AXI_BREADY_o[s] = s_bready_in[s];

        assign s_rid_in[s]   = S_AXI_RID_i[W_SID*(s+1)-1 -: W_SID];
        assign s_rdata_in[s] = S_AXI_RDATA_i[DATA_WIDTH*(s+1)-1 -: DATA_WIDTH];
        assign s_rresp_in[s] = S_AXI_RRESP_i[RESP_W*(s+1)-1 -: RESP_W];
        assign s_rlast_in[s] = S_AXI_RLAST_i[s];
        assign s_rvalid_in[s]= S_AXI_RVALID_i[s];
        assign S_AXI_RREADY_o[s] = s_rready_in[s];
    end
endgenerate

//=============================================================================
// 1. cross_4k_if: 4KB Boundary & WRAP/INCR Splitting (per Master)
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : INST_CROSS_4K
        cross_4k_if #(
            .W_ID   (W_ID),
            .W_ADDR (ADDR_WIDTH),
            .W_LEN  (LEN_W),
            .W_DATA (DATA_WIDTH),
            .W_STRB (W_STRB)
        ) u_cross_4k (
            .clk             (AXI_CLK),
            .rst_n           (AXI_RSTn),
            // AR channel
            .m_axi_arid      (m_arid[m]),
            .m_axi_araddr    (m_araddr[m]),
            .m_axi_arlen     (m_arlen[m]),
            .m_axi_arsize    (m_arsize[m]),
            .m_axi_arburst   (m_arburst[m]),
            .m_axi_arvalid   (m_arvalid[m]),
            .m_axi_arready   (m_arready[m]),
            // AW channel
            .m_axi_awid      (m_awid[m]),
            .m_axi_awaddr    (m_awaddr[m]),
            .m_axi_awlen     (m_awlen[m]),
            .m_axi_awsize    (m_awsize[m]),
            .m_axi_awburst   (m_awburst[m]),
            .m_axi_awvalid   (m_awvalid[m]),
            .m_axi_awready   (m_awready[m]),
            // W channel (WLAST insertion for split transactions)
            .m_axi_wdata     (m_wdata[m]),
            .m_axi_wstrb     (m_wstrb[m]),
            .m_axi_wlast     (m_wlast[m]),
            .m_axi_wvalid    (m_wvalid[m]),
            .m_axi_wready    (m_wready[m]),
            .s_axi_wdata     (c4k_wdata[m]),
            .s_axi_wstrb     (c4k_wstrb[m]),
            .s_axi_wlast     (c4k_wlast[m]),
            .s_axi_wvalid    (c4k_wvalid[m]),
            .s_axi_wready    (c4k_wready[m]),
            // AR slave side
            .s_axi_arid      (c4k_arid[m]),
            .s_axi_araddr    (c4k_araddr[m]),
            .s_axi_arlen     (c4k_arlen[m]),
            .s_axi_arsize    (c4k_arsize[m]),
            .s_axi_arburst   (c4k_arburst[m]),
            .s_axi_arvalid   (c4k_arvalid[m]),
            .s_axi_arready   (c4k_arready[m]),
            // AW slave side
            .s_axi_awid      (c4k_awid[m]),
            .s_axi_awaddr    (c4k_awaddr[m]),
            .s_axi_awlen     (c4k_awlen[m]),
            .s_axi_awsize    (c4k_awsize[m]),
            .s_axi_awburst   (c4k_awburst[m]),
            .s_axi_awvalid   (c4k_awvalid[m]),
            .s_axi_awready   (c4k_awready[m])
        );
    end
endgenerate

//=============================================================================
// 2. Pre-Crossbar FIFOs: Master -> Crossbar (AW/W/AR)
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : INST_FIFO_AW_W_AR
        // AW FIFO: valid from rd_vld only (not stored in data)
        axi_fifo_sync #(
            .FDW(W_MID + ADDR_WIDTH + LEN_W + SIZE_W + BURST_W),
            .FAW(2)
        ) u_fifo_aw (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (c4k_awready[m]),
            .wr_vld (c4k_awvalid[m]),
            .wr_din ({c4k_awid[m], c4k_awaddr[m], c4k_awlen[m],
                      c4k_awsize[m], c4k_awburst[m]}),
            .rd_rdy (M_AWREADY[m]),
            .rd_vld (M_AWVALID[m]),
            .rd_dout ({M_AWID[m], M_AWADDR[m], M_AWLEN[m],
                       M_AWSIZE[m], M_AWBURST[m]})
        );

        // W FIFO (from cross_4k_if output)
        axi_fifo_sync #(
            .FDW(DATA_WIDTH + W_STRB + 1),
            .FAW(2)
        ) u_fifo_w (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (c4k_wready[m]),
            .wr_vld (c4k_wvalid[m]),
            .wr_din ({c4k_wdata[m], c4k_wstrb[m], c4k_wlast[m]}),
            .rd_rdy (M_WREADY[m]),
            .rd_vld (M_WVALID[m]),
            .rd_dout ({M_WDATA[m], M_WSTRB[m], M_WLAST[m]})
        );

        // AR FIFO: valid from rd_vld only (not stored in data)
        axi_fifo_sync #(
            .FDW(W_MID + ADDR_WIDTH + LEN_W + SIZE_W + BURST_W),
            .FAW(2)
        ) u_fifo_ar (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (c4k_arready[m]),
            .wr_vld (c4k_arvalid[m]),
            .wr_din ({c4k_arid[m], c4k_araddr[m], c4k_arlen[m],
                      c4k_arsize[m], c4k_arburst[m]}),
            .rd_rdy (M_ARREADY[m]),
            .rd_vld (M_ARVALID[m]),
            .rd_dout ({M_ARID[m], M_ARADDR[m], M_ARLEN[m],
                       M_ARSIZE[m], M_ARBURST[m]})
        );
            // TRACE: pre-crossbar AR FIFO output
            reg _prev_arv;
            always @(posedge AXI_CLK) begin
                _prev_arv <= M_ARVALID[m];
                if (AXI_RSTn && M_ARVALID[m] !== _prev_arv)
                    $display("[TRACE] %0t AR pre-FIFO vld m=%0d vld=%b rdy=%b", $time, m, M_ARVALID[m], M_ARREADY[m]);
                if (M_ARVALID[m] && M_ARREADY[m])
                    $display("[TRACE] %0t AR pre-FIFO out m=%0d addr=0x%08h", $time, m, M_ARADDR[m]);
            end
    end
endgenerate

//=============================================================================
// 3. New: Post-Crossbar FIFOs: Crossbar -> Slave (AW/W/AR)
//=============================================================================
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : INST_POST_FIFO_AW_W_AR
        // AW FIFO
        axi_fifo_sync #(
            .FDW(W_SID + ADDR_WIDTH + LEN_W + SIZE_W + BURST_W),
            .FAW(2)
        ) u_fifo_aw_slv (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (S_AWREADY[s]),
            .wr_vld (S_AWVALID[s]),
            .wr_din ({S_AWID[s], S_AWADDR[s], S_AWLEN[s],
                      S_AWSIZE[s], S_AWBURST[s]}),
            .rd_rdy (s_awready_out[s]),
            .rd_vld (s_awvalid_out[s]),
            .rd_dout ({s_awid_out[s], s_awaddr_out[s], s_awlen_out[s],
                       s_awsize_out[s], s_awburst_out[s]})
        );

        // W FIFO
        axi_fifo_sync #(
            .FDW(DATA_WIDTH + W_STRB + 1),
            .FAW(2)
        ) u_fifo_w_slv (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (S_WREADY[s]),
            .wr_vld (S_WVALID[s]),
            .wr_din ({S_WDATA[s], S_WSTRB[s], S_WLAST[s]}),
            .rd_rdy (s_wready_out[s]),
            .rd_vld (s_wvalid_out[s]),
            .rd_dout ({s_wdata_out[s], s_wstrb_out[s], s_wlast_out[s]})
        );

        // AR FIFO: wr_rdy → intermediate, gated by sid_buffer backpressure
        axi_fifo_sync #(
            .FDW(W_SID + ADDR_WIDTH + LEN_W + SIZE_W + BURST_W),
            .FAW(2)
        ) u_fifo_ar_slv (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (S_ARREADY_FIFO[s]),
            .wr_vld (S_ARVALID[s]),
            .wr_din ({S_ARID[s], S_ARADDR[s], S_ARLEN[s],
                      S_ARSIZE[s], S_ARBURST[s]}),
            .rd_rdy (s_arready_out[s]),
            .rd_vld (s_arvalid_out[s]),
            .rd_dout ({s_arid_out[s], s_araddr_out[s], s_arlen_out[s],
                       s_arsize_out[s], s_arburst_out[s]})
        );
    end
endgenerate

// TRACE: slave W pre-FIFO state (wr_rdy back to crossbar)
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : TRACE_W_SLV_FIFO
        always @(posedge AXI_CLK) begin
            if(AXI_RSTn) begin
                if(S_WVALID[s] || s_wvalid_out[s])
                    $display("[TRACE_W_PREFIFO] %0t s=%0d wr_hs=%b (vld=%b rdy=%b) rd_hs=%b (vld=%b rdy=%b)",
                             $time, s,
                             S_WVALID[s] && S_WREADY[s], S_WVALID[s], S_WREADY[s],
                             s_wvalid_out[s] && s_wready_out[s], s_wvalid_out[s], s_wready_out[s]);
            end
        end
    end
endgenerate

// Gate AR ready to crossbar: use s_push_rdy from shared sid_buffer per slave
// rf-style push_rdy = ~(select ^ grant) & clr_allowed
// When buffer has space: push_rdy[s]=1 → arready passes
// When buffer full or clearing: push_rdy[s]=0 → arready gated
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : GEN_ARREADY_GATE
        assign S_ARREADY[s] = S_ARREADY_FIFO[s] & sid_buf_push_rdy[s];
        // TRACE: ARREADY gate (every cycle for s=2)
        //always @(posedge AXI_CLK) begin
            //if (AXI_RSTn)
                //$display("[TRACE] %0t S_ARREADY s=%0d fifo=%b sid=%b final=%b", $time, s, S_ARREADY_FIFO[s], sid_buf_push_rdy[s], S_ARREADY[s]);
        //end
        // TRACE: ARREADY gate
        always @(posedge AXI_CLK) begin
            if (S_ARREADY_FIFO[s] !== sid_buf_push_rdy[s])
                $display("[TRACE] %0t S_ARREADY s=%0d fifo=%b sid=%b final=%b",
                         $time, s, S_ARREADY_FIFO[s], sid_buf_push_rdy[s], S_ARREADY[s]);
        end
    end
endgenerate

//=============================================================================
// 4. New: Pre-Crossbar FIFOs: Slave -> Crossbar (B/R)
//=============================================================================
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : INST_PRE_FIFO_B_R
        // B FIFO
        axi_fifo_sync #(
            .FDW(W_SID + RESP_W),
            .FAW(2)
        ) u_fifo_b_slv (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (s_bready_in[s]),
            .wr_vld (s_bvalid_in[s]),
            .wr_din ({s_bid_in[s], s_bresp_in[s]}),
            .rd_rdy (s_bready_fifo[s]),
            .rd_vld (s_bvalid_fifo[s]),
            .rd_dout ({s_bid_fifo[s], s_bresp_fifo[s]})
        );

        // R FIFO
        axi_fifo_sync #(
            .FDW(W_SID + DATA_WIDTH + RESP_W + 1),
            .FAW(2)
        ) u_fifo_r_slv (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (s_rready_in[s]),
            .wr_vld (s_rvalid_in[s]),
            .wr_din ({s_rid_in[s], s_rdata_in[s], s_rresp_in[s], s_rlast_in[s]}),
            .rd_rdy (s_rready_fifo[s]),
            .rd_vld (s_rvalid_fifo[s]),
            .rd_dout ({s_rid_fifo[s], s_rdata_fifo[s], s_rresp_fifo[s], s_rlast_fifo[s]})
        );
    end
endgenerate

//=============================================================================
// 5. Post-Crossbar FIFOs: Crossbar -> Master (B/R)
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : INST_FIFO_B_R_MST
        // B FIFO: output to c4k_bid/bresp, then routed to axi_split_b_merge
        axi_fifo_sync #(
            .FDW(W_SID + RESP_W),
            .FAW(2)
        ) u_fifo_b_mst (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (M_BREADY[m]),
            .wr_vld (M_BVALID[m]),
            .wr_din ({M_BID[m], M_BRESP[m]}),
            .rd_rdy (c4k_bready[m]),
            .rd_vld (c4k_bvalid[m]),
            .rd_dout ({c4k_bid[m], c4k_bresp[m]})
        );

        // R FIFO: widened to carry W_SID RID (includes mst_idx + split_st for r_merge)
        axi_fifo_sync #(
            .FDW(W_SID + DATA_WIDTH + RESP_W + 1),
            .FAW(2)
        ) u_fifo_r_mst (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (M_RREADY_FIFO[m]),
            .wr_vld (M_RVALID[m]),
            .wr_din ({M_RID[m], M_RDATA[m],
                      M_RRESP[m], M_RLAST[m]}),
            .rd_rdy (r_fifo_rready[m]),
            .rd_vld (r_fifo_rvalid[m]),
            .rd_dout ({r_fifo_rid[m], r_fifo_rdata[m], r_fifo_rresp[m], r_fifo_rlast[m]})
        );
    end
endgenerate

// ----- Clear port packing (one port per master) -----
wire [MST_AMT-1:0]       clr_last_packed;
wire [W_SID*MST_AMT-1:0] clr_sid_packed;
wire [MST_AMT-1:0]       clr_sid_vld_packed;
wire [MST_AMT-1:0]       sid_buf_s_clr_rdy;

// Gate R ready to crossbar: use s_clr_rdy from shared sid_buffer per master
// rf-style s_clr_rdy = ~(clr_select ^ clr_grant)
// When master has no clear request or wins arbitration: s_clr_rdy=1 → rready passes
// When master requests clear but loses arbitration: s_clr_rdy=0 → rready gated → retry
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : GEN_RREADY_GATE
        assign M_RREADY[m] = M_RREADY_FIFO[m] & sid_buf_s_clr_rdy[m];
    end
endgenerate

//=============================================================================
// 6. B/R Merge Modules: axi_split_b_merge / axi_split_r_merge (per Master)
//    Placed on master side after S2M has stripped mst_idx.
//    BID/RID format: {split_st[1:0], orig_id[W_ID-1:0]} → M_ID_W=0.
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : INST_SPLIT_MERGE
        // B merge: replaces cross_4k_if built-in B merge
        axi_split_b_merge #(
            .W_ID     (W_ID),
            .M_ID_W   ($clog2(MST_AMT)),
            .MAX_SPLIT(4)
        ) u_b_merge (
            .clk            (AXI_CLK),
            .rst_n          (AXI_RSTn),
            .s_axi_bvalid   (c4k_bvalid[m]),
            .s_axi_bid      (c4k_bid[m]),
            .s_axi_bresp    (c4k_bresp[m]),
            .s_axi_bready   (c4k_bready[m]),
            .m_axi_bvalid   (m_bvalid[m]),
            .m_axi_bid      (m_bid[m]),
            .m_axi_bresp    (m_bresp[m]),
            .m_axi_bready   (m_bready[m])
        );

        // R merge: handles split read response merging
        axi_split_r_merge #(
            .W_ID     (W_ID),
            .M_ID_W   ($clog2(MST_AMT)),
            .W_DATA   (DATA_WIDTH),
            .MAX_SPLIT(4)
        ) u_r_merge (
            .clk            (AXI_CLK),
            .rst_n          (AXI_RSTn),
            .s_axi_rvalid   (r_fifo_rvalid[m]),
            .s_axi_rid      (r_fifo_rid[m]),
            .s_axi_rdata    (r_fifo_rdata[m]),
            .s_axi_rresp    (r_fifo_rresp[m]),
            .s_axi_rlast    (r_fifo_rlast[m]),
            .s_axi_rready   (r_fifo_rready[m]),
            .m_axi_rvalid   (m_rvalid[m]),
            .m_axi_rid      (m_rid[m]),
            .m_axi_rdata    (m_rdata[m]),
            .m_axi_rresp    (m_rresp[m]),
            .m_axi_rlast    (m_rlast[m]),
            .m_axi_rready   (m_rready[m])
        );
    end
endgenerate

//=============================================================================
// 7. Read Reorder Logic (sid_buffer write/clear + reorder)
//=============================================================================

//=============================================================================
// 7. Single shared sid_buffer + reorder (not per-master)
//    - Write: per-slave AR handshake captures s_arid
//    - Clear: per-master R completion (one port per master, priority-arbitrated)
//    - reorder: compares S_RID against shared rob_buffer → per-slave order_grant
//    - order_grant shared to ALL S2M instances (each filters by MASTER_ID)
//=============================================================================

// ----- Write port packing (per slave) -----
wire [W_SID*SLV_AMT-1:0] sid_buf_wr_id_packed;
wire [SLV_AMT-1:0]       sid_buf_wr_vld;
wire [SLV_AMT-1:0]       sid_buf_wr_rdy;

genvar si_wr;
generate
    for(si_wr = 0; si_wr < SLV_AMT; si_wr = si_wr + 1) begin : SID_BUF_WR_PACK
        assign sid_buf_wr_id_packed[W_SID*(si_wr+1)-1 -: W_SID] = s_arid_out[si_wr];
        assign sid_buf_wr_vld[si_wr] = s_arvalid_out[si_wr];
        assign sid_buf_wr_rdy[si_wr] = s_arready_out[si_wr];
    end
endgenerate

// ----- Clear port packing (one port per master) -----
// Each master's R completion drives one clear port; priority_sel_clr picks winner

genvar si_clr;
generate
    for(si_clr = 0; si_clr < MST_AMT; si_clr = si_clr + 1) begin : SID_BUF_CLR_PACK
        assign clr_last_packed[si_clr]    = r_fifo_rlast[si_clr];
        assign clr_sid_packed[W_SID*(si_clr+1)-1 -: W_SID] = r_fifo_rid[si_clr];
        assign clr_sid_vld_packed[si_clr] = r_fifo_rvalid[si_clr] & r_fifo_rready[si_clr] & r_fifo_rlast[si_clr];
    end
endgenerate

// ----- Shared sid_buffer -----
sid_buffer #(
    .NUM_WR (SLV_AMT),
    .NUM_CLR(MST_AMT),
    .W_ID   (W_SID),
    .DEPTH  (4)
) u_sid_buffer (
    .clk         (AXI_CLK),
    .rstn        (AXI_RSTn),
    .s_axid      (sid_buf_wr_id_packed),
    .s_axid_vld  (sid_buf_wr_vld),
    .s_fifo_rdy  (sid_buf_wr_rdy),
    .s_push_rdy  (sid_buf_push_rdy),
    .clr_last    (clr_last_packed),
    .clr_sid     (clr_sid_packed),
    .clr_sid_vld (clr_sid_vld_packed),
    .s_clr_rdy   (sid_buf_s_clr_rdy),
    .sid_buffer  (sid_buf_out)
);

// ----- Shared reorder -----
wire [W_SID*SLV_AMT-1:0] reorder_sid_packed;
wire [SLV_AMT-1:0]       reorder_sid_vld;

genvar si_ro;
generate
    for(si_ro = 0; si_ro < SLV_AMT; si_ro = si_ro + 1) begin : REORDER_IN_PACK
        assign reorder_sid_packed[W_SID*(si_ro+1)-1 -: W_SID] = s_rid_fifo[si_ro];
        assign reorder_sid_vld[si_ro] = s_rvalid_fifo[si_ro];
    end
endgenerate

reorder #(
    .NUM    (SLV_AMT),
    .M_ID_W ($clog2(MST_AMT)),
    .W_ID   (W_MID),
    .DEPTH  (4)
) u_reorder (
    .clk         (AXI_CLK),
    .rstn        (AXI_RSTn),
    .s_sid       (reorder_sid_packed),
    .s_sid_vld   (reorder_sid_vld),
    .rob_buffer  (sid_buf_out),
    .order_grant (reorder_grant)
);


//=============================================================================
// Unpack crossbar M_RSID output
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : UNPACK_RSID
        assign M_RSID[m] = M_RSID_PACKED[W_SID*(m+1)-1 -: W_SID];
    end
endgenerate

//=============================================================================
// 8. axi_crossbar: Core Routing Engine
//    Pack 2D arrays → flat vectors, connect to crossbar, unpack back.
//=============================================================================

// ----- Master-side packed wires (crossbar interface) -----
wire [W_MID*MST_AMT-1:0]        x_m_AWID;
wire [ADDR_WIDTH*MST_AMT-1:0]   x_m_AWADDR;
wire [BURST_W*MST_AMT-1:0]      x_m_AWBURST;
wire [LEN_W*MST_AMT-1:0]        x_m_AWLEN;
wire [SIZE_W*MST_AMT-1:0]       x_m_AWSIZE;
wire [MST_AMT-1:0]              x_m_AWVALID;
wire [MST_AMT-1:0]              x_m_AWREADY;
wire [DATA_WIDTH*MST_AMT-1:0]   x_m_WDATA;
wire [W_STRB*MST_AMT-1:0]       x_m_WSTRB;
wire [MST_AMT-1:0]              x_m_WLAST;
wire [MST_AMT-1:0]              x_m_WVALID;
wire [MST_AMT-1:0]              x_m_WREADY;
wire [W_SID*MST_AMT-1:0]        x_m_BID;
wire [RESP_W*MST_AMT-1:0]       x_m_BRESP;
wire [MST_AMT-1:0]              x_m_BVALID;
wire [MST_AMT-1:0]              x_m_BREADY;
wire [W_MID*MST_AMT-1:0]        x_m_ARID;
wire [ADDR_WIDTH*MST_AMT-1:0]   x_m_ARADDR;
wire [BURST_W*MST_AMT-1:0]      x_m_ARBURST;
wire [LEN_W*MST_AMT-1:0]        x_m_ARLEN;
wire [SIZE_W*MST_AMT-1:0]       x_m_ARSIZE;
wire [MST_AMT-1:0]              x_m_ARVALID;
wire [MST_AMT-1:0]              x_m_ARREADY;
wire [W_SID*MST_AMT-1:0]        x_m_RID;
wire [DATA_WIDTH*MST_AMT-1:0]   x_m_RDATA;
wire [RESP_W*MST_AMT-1:0]       x_m_RRESP;
wire [MST_AMT-1:0]              x_m_RLAST;
wire [MST_AMT-1:0]              x_m_RVALID;
wire [MST_AMT-1:0]              x_m_RREADY;

// ----- Slave-side packed wires (crossbar interface) -----
wire [W_SID*SLV_AMT-1:0]        x_s_AWID;
wire [ADDR_WIDTH*SLV_AMT-1:0]   x_s_AWADDR;
wire [BURST_W*SLV_AMT-1:0]      x_s_AWBURST;
wire [LEN_W*SLV_AMT-1:0]        x_s_AWLEN;
wire [SIZE_W*SLV_AMT-1:0]       x_s_AWSIZE;
wire [SLV_AMT-1:0]              x_s_AWVALID;
wire [SLV_AMT-1:0]              x_s_AWREADY;
wire [DATA_WIDTH*SLV_AMT-1:0]   x_s_WDATA;
wire [W_STRB*SLV_AMT-1:0]       x_s_WSTRB;
wire [SLV_AMT-1:0]              x_s_WLAST;
wire [SLV_AMT-1:0]              x_s_WVALID;
wire [SLV_AMT-1:0]              x_s_WREADY;
wire [W_SID*SLV_AMT-1:0]        x_s_BID;
wire [RESP_W*SLV_AMT-1:0]       x_s_BRESP;
wire [SLV_AMT-1:0]              x_s_BVALID;
wire [SLV_AMT-1:0]              x_s_BREADY;
wire [W_SID*SLV_AMT-1:0]        x_s_ARID;
wire [ADDR_WIDTH*SLV_AMT-1:0]   x_s_ARADDR;
wire [BURST_W*SLV_AMT-1:0]      x_s_ARBURST;
wire [LEN_W*SLV_AMT-1:0]        x_s_ARLEN;
wire [SIZE_W*SLV_AMT-1:0]       x_s_ARSIZE;
wire [SLV_AMT-1:0]              x_s_ARVALID;
wire [SLV_AMT-1:0]              x_s_ARREADY;
wire [W_SID*SLV_AMT-1:0]        x_s_RID;
wire [DATA_WIDTH*SLV_AMT-1:0]   x_s_RDATA;
wire [RESP_W*SLV_AMT-1:0]       x_s_RRESP;
wire [SLV_AMT-1:0]              x_s_RLAST;
wire [SLV_AMT-1:0]              x_s_RVALID;
wire [SLV_AMT-1:0]              x_s_RREADY;

// ----- Master-side packing: M_* arrays → flat x_m_* -----
genvar xm;
generate
    for (xm = 0; xm < MST_AMT; xm = xm + 1) begin : PACK_MST
        // AW channel
        assign x_m_AWID   [W_MID*(xm+1)-1     -: W_MID]      = M_AWID[xm];
        assign x_m_AWADDR [ADDR_WIDTH*(xm+1)-1 -: ADDR_WIDTH] = M_AWADDR[xm];
        assign x_m_AWBURST[BURST_W*(xm+1)-1   -: BURST_W]    = M_AWBURST[xm];
        assign x_m_AWLEN  [LEN_W*(xm+1)-1     -: LEN_W]      = M_AWLEN[xm];
        assign x_m_AWSIZE [SIZE_W*(xm+1)-1    -: SIZE_W]     = M_AWSIZE[xm];
        assign x_m_AWVALID[xm] = M_AWVALID[xm];
        // W channel
        assign x_m_WDATA  [DATA_WIDTH*(xm+1)-1 -: DATA_WIDTH] = M_WDATA[xm];
        assign x_m_WSTRB  [W_STRB*(xm+1)-1     -: W_STRB]     = M_WSTRB[xm];
        assign x_m_WLAST [xm]  = M_WLAST[xm];
        assign x_m_WVALID[xm]  = M_WVALID[xm];
        // B channel
        assign x_m_BREADY[xm]  = M_BREADY[xm];
        // AR channel
        assign x_m_ARID   [W_MID*(xm+1)-1     -: W_MID]      = M_ARID[xm];
        assign x_m_ARADDR [ADDR_WIDTH*(xm+1)-1 -: ADDR_WIDTH] = M_ARADDR[xm];
        assign x_m_ARBURST[BURST_W*(xm+1)-1   -: BURST_W]    = M_ARBURST[xm];
        assign x_m_ARLEN  [LEN_W*(xm+1)-1     -: LEN_W]      = M_ARLEN[xm];
        assign x_m_ARSIZE [SIZE_W*(xm+1)-1    -: SIZE_W]     = M_ARSIZE[xm];
        assign x_m_ARVALID[xm] = M_ARVALID[xm];
        // R channel
        assign x_m_RREADY[xm]  = M_RREADY[xm];
    end
endgenerate

// ----- Master-side unpacking: flat x_m_* → M_* arrays -----
generate
    for (xm = 0; xm < MST_AMT; xm = xm + 1) begin : UNPACK_MST_XBAR
        assign M_AWREADY[xm] = x_m_AWREADY[xm];
        assign M_WREADY[xm]  = x_m_WREADY[xm];
        assign M_BID[xm]     = x_m_BID   [W_SID*(xm+1)-1  -: W_SID];
        assign M_BRESP[xm]   = x_m_BRESP [RESP_W*(xm+1)-1 -: RESP_W];
        assign M_BVALID[xm]  = x_m_BVALID[xm];
        assign M_ARREADY[xm] = x_m_ARREADY[xm];
        assign M_RID[xm]     = x_m_RID   [W_SID*(xm+1)-1     -: W_SID];
        assign M_RDATA[xm]   = x_m_RDATA [DATA_WIDTH*(xm+1)-1 -: DATA_WIDTH];
        assign M_RRESP[xm]   = x_m_RRESP [RESP_W*(xm+1)-1    -: RESP_W];
        assign M_RLAST[xm]   = x_m_RLAST[xm];
        assign M_RVALID[xm]  = x_m_RVALID[xm];
    end
endgenerate

// ----- Slave-side packing: S_* / s_*_fifo arrays → flat x_s_* -----
genvar xs;
generate
    for (xs = 0; xs < SLV_AMT; xs = xs + 1) begin : PACK_SLV
        // AW channel (inputs to crossbar)
        assign x_s_AWREADY[xs]  = S_AWREADY[xs];
        // W channel (inputs to crossbar)
        assign x_s_WREADY[xs]   = S_WREADY[xs];
        // B channel (inputs to crossbar, from s_*_fifo)
        assign x_s_BID   [W_SID*(xs+1)-1  -: W_SID]  = s_bid_fifo[xs];
        assign x_s_BRESP [RESP_W*(xs+1)-1 -: RESP_W] = s_bresp_fifo[xs];
        assign x_s_BVALID[xs]   = s_bvalid_fifo[xs];
        // AR channel (inputs to crossbar)
        assign x_s_ARREADY[xs]  = S_ARREADY[xs];
        // R channel (inputs to crossbar, from s_*_fifo)
        assign x_s_RID   [W_SID*(xs+1)-1     -: W_SID]      = s_rid_fifo[xs];
        assign x_s_RDATA [DATA_WIDTH*(xs+1)-1 -: DATA_WIDTH] = s_rdata_fifo[xs];
        assign x_s_RRESP [RESP_W*(xs+1)-1    -: RESP_W]     = s_rresp_fifo[xs];
        assign x_s_RLAST[xs]    = s_rlast_fifo[xs];
        assign x_s_RVALID[xs]   = s_rvalid_fifo[xs];
    end
endgenerate

// ----- Slave-side unpacking: flat x_s_* → S_* / s_*_fifo arrays -----
generate
    for (xs = 0; xs < SLV_AMT; xs = xs + 1) begin : UNPACK_SLV_XBAR
        // AW channel (outputs from crossbar)
        assign S_AWID[xs]       = x_s_AWID   [W_SID*(xs+1)-1     -: W_SID];
        assign S_AWADDR[xs]     = x_s_AWADDR [ADDR_WIDTH*(xs+1)-1 -: ADDR_WIDTH];
        assign S_AWBURST[xs]    = x_s_AWBURST[BURST_W*(xs+1)-1   -: BURST_W];
        assign S_AWLEN[xs]      = x_s_AWLEN  [LEN_W*(xs+1)-1     -: LEN_W];
        assign S_AWSIZE[xs]     = x_s_AWSIZE [SIZE_W*(xs+1)-1    -: SIZE_W];
        assign S_AWVALID[xs]    = x_s_AWVALID[xs];
        // W channel (outputs from crossbar)
        assign S_WDATA[xs]      = x_s_WDATA  [DATA_WIDTH*(xs+1)-1 -: DATA_WIDTH];
        assign S_WSTRB[xs]      = x_s_WSTRB  [W_STRB*(xs+1)-1     -: W_STRB];
        assign S_WLAST[xs]      = x_s_WLAST[xs];
        assign S_WVALID[xs]     = x_s_WVALID[xs];
        // B channel (outputs from crossbar)
        assign s_bready_fifo[xs] = x_s_BREADY[xs];
        // AR channel (outputs from crossbar)
        assign S_ARID[xs]       = x_s_ARID   [W_SID*(xs+1)-1     -: W_SID];
        assign S_ARADDR[xs]     = x_s_ARADDR [ADDR_WIDTH*(xs+1)-1 -: ADDR_WIDTH];
        assign S_ARBURST[xs]    = x_s_ARBURST[BURST_W*(xs+1)-1   -: BURST_W];
        assign S_ARLEN[xs]      = x_s_ARLEN  [LEN_W*(xs+1)-1     -: LEN_W];
        assign S_ARSIZE[xs]     = x_s_ARSIZE [SIZE_W*(xs+1)-1    -: SIZE_W];
        assign S_ARVALID[xs]    = x_s_ARVALID[xs];
        // R channel (outputs from crossbar)
        assign s_rready_fifo[xs] = x_s_RREADY[xs];
    end
endgenerate

// ----- Crossbar instantiation -----
axi_crossbar #(
    .MST_AMT(MST_AMT),
    .SLV_AMT(SLV_AMT),
    .OUTSTANDING_AMT(OUTSTANDING_AMT),
    .W_ID(W_ID),
    .TRANS_BURST_W(BURST_W),
    .TRANS_DATA_LEN_W(LEN_W),
    .TRANS_DATA_SIZE_W(SIZE_W),
    .TRANS_WR_RESP_W(RESP_W),
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .SLV_ADDR_BASE(SLV_ADDR_BASE),
    .SLV_ADDR_LEN(SLV_ADDR_LEN),
    .DEFAULT_SLV_IDX(0),
    .DEFAULT_SLV_EN(1'b0)
) u_axi_crossbar (
    .AXI_CLK(AXI_CLK),
    .AXI_RSTn(AXI_RSTn),

    // Master Ports
    .m_AWID_i(x_m_AWID),
    .m_AWADDR_i(x_m_AWADDR),
    .m_AWLEN_i(x_m_AWLEN),
    .m_AWSIZE_i(x_m_AWSIZE),
    .m_AWBURST_i(x_m_AWBURST),
    .m_AWVALID_i(x_m_AWVALID),
    .m_AWREADY_o(x_m_AWREADY),
    .m_WDATA_i(x_m_WDATA),
    .m_WSTRB_i(x_m_WSTRB),
    .m_WLAST_i(x_m_WLAST),
    .m_WVALID_i(x_m_WVALID),
    .m_WREADY_o(x_m_WREADY),
    .m_BID_o(x_m_BID),
    .m_BRESP_o(x_m_BRESP),
    .m_BVALID_o(x_m_BVALID),
    .m_BREADY_i(x_m_BREADY),
    .m_ARID_i(x_m_ARID),
    .m_ARADDR_i(x_m_ARADDR),
    .m_ARLEN_i(x_m_ARLEN),
    .m_ARSIZE_i(x_m_ARSIZE),
    .m_ARBURST_i(x_m_ARBURST),
    .m_ARVALID_i(x_m_ARVALID),
    .m_ARREADY_o(x_m_ARREADY),
    .m_RID_o(x_m_RID),
    .m_RDATA_o(x_m_RDATA),
    .m_RRESP_o(x_m_RRESP),
    .m_RLAST_o(x_m_RLAST),
    .m_RVALID_o(x_m_RVALID),
    .m_RREADY_i(x_m_RREADY),
    .m_RSID_o(M_RSID_PACKED),

    // Slave Ports
    .s_AWID_o(x_s_AWID),
    .s_AWADDR_o(x_s_AWADDR),
    .s_AWLEN_o(x_s_AWLEN),
    .s_AWSIZE_o(x_s_AWSIZE),
    .s_AWBURST_o(x_s_AWBURST),
    .s_AWVALID_o(x_s_AWVALID),
    .s_AWREADY_i(x_s_AWREADY),
    .s_WDATA_o(x_s_WDATA),
    .s_WSTRB_o(x_s_WSTRB),
    .s_WLAST_o(x_s_WLAST),
    .s_WVALID_o(x_s_WVALID),
    .s_WREADY_i(x_s_WREADY),
    .s_BID_i(x_s_BID),
    .s_BRESP_i(x_s_BRESP),
    .s_BVALID_i(x_s_BVALID),
    .s_BREADY_o(x_s_BREADY),
    .s_ARID_o(x_s_ARID),
    .s_ARADDR_o(x_s_ARADDR),
    .s_ARLEN_o(x_s_ARLEN),
    .s_ARSIZE_o(x_s_ARSIZE),
    .s_ARBURST_o(x_s_ARBURST),
    .s_ARVALID_o(x_s_ARVALID),
    .s_ARREADY_i(x_s_ARREADY),
    .s_RID_i(x_s_RID),
    .s_RDATA_i(x_s_RDATA),
    .s_RRESP_i(x_s_RRESP),
    .s_RLAST_i(x_s_RLAST),
    .s_RVALID_i(x_s_RVALID),
    .s_RREADY_o(x_s_RREADY),

    // Control
    .arbiter_type(arbiter_type),
    .slv_en_i({SLV_AMT{1'b1}})
);

//===================================================================
// TRACE: 5-channel handshake monitoring for ALL masters and slaves
//===================================================================

genvar tm, ts;
// ---- Master side: pre-FIFO output + response per master ----
generate
    for (tm = 0; tm < MST_AMT; tm = tm + 1) begin : TRACE_MST
        always @(posedge AXI_CLK) begin
            if (AXI_RSTn) begin
                if (M_AWVALID[tm] && M_AWREADY[tm])
                    $display("[TRACE] %0t AW pre-FIFO m=%0d addr=0x%08h len=%0d", $time, tm, M_AWADDR[tm], M_AWLEN[tm]);
                if (M_WVALID[tm] && M_WREADY[tm])
                    $display("[TRACE_W] %0t W pre-FIFO HS m=%0d last=%b data=0x%08h strb=0x%01h",
                             $time, tm, M_WLAST[tm], M_WDATA[tm], M_WSTRB[tm]);
                else if (M_WVALID[tm] && !M_WREADY[tm])
                    $display("[TRACE_W] %0t W pre-FIFO STALL m=%0d vld=1 rdy=0 last=%b",
                             $time, tm, M_WLAST[tm]);
                if (M_BVALID[tm] && M_BREADY[tm])
                    $display("[TRACE] %0t B  to master m=%0d", $time, tm);
                if (M_ARVALID[tm] && M_ARREADY[tm])
                    $display("[TRACE] %0t AR pre-FIFO m=%0d addr=0x%08h len=%0d", $time, tm, M_ARADDR[tm], M_ARLEN[tm]);
                if (M_RVALID[tm] && M_RREADY[tm])
                    $display("[TRACE] %0t R  to master m=%0d", $time, tm);
            end
        end
    end
endgenerate

// ---- Slave side: request arrival + response per slave ----
generate
    for (ts = 0; ts < SLV_AMT; ts = ts + 1) begin : TRACE_SLV
        always @(posedge AXI_CLK) begin
            if (AXI_RSTn) begin
                if (s_awvalid_out[ts])
                    $display("[TRACE] %0t AW at slave s=%0d addr=0x%08h len=%0d", $time, ts, s_awaddr_out[ts], s_awlen_out[ts]);
                if (s_wvalid_out[ts] && s_wready_out[ts])
                    $display("[TRACE_W] %0t W slave HS s=%0d last=%b data=0x%08h",
                             $time, ts, s_wlast_out[ts], s_wdata_out[ts]);
                else if (s_wvalid_out[ts] && !s_wready_out[ts])
                    $display("[TRACE_W] %0t W slave STALL s=%0d vld=1 rdy=0 last=%b",
                             $time, ts, s_wlast_out[ts]);
                if (s_bvalid_in[ts] && s_bready_in[ts])
                    $display("[TRACE] %0t B  from slave s=%0d", $time, ts);
                if (s_arvalid_out[ts])
                    $display("[TRACE] %0t AR at slave s=%0d addr=0x%08h len=%0d", $time, ts, s_araddr_out[ts], s_arlen_out[ts]);
                if (s_rvalid_in[ts] && s_rready_in[ts])
                    $display("[TRACE] %0t R  from slave s=%0d", $time, ts);
            end
        end
    end
endgenerate

// TRACE_W: W FIFO per master (after cross_4k, before xbar)
generate
    for (tm = 0; tm < MST_AMT; tm = tm + 1) begin : TRACE_W_FIFO
        always @(posedge AXI_CLK) begin
            if (AXI_RSTn) begin
                // W FIFO write: cross_4k output → pre-FIFO input
                if (c4k_wvalid[tm] && c4k_wready[tm])
                    $display("[TRACE_W_FIFO] %0t W_FIFO_WR m=%0d last=%b data=0x%08h",
                             $time, tm, c4k_wlast[tm], c4k_wdata[tm]);
                else if (c4k_wvalid[tm] && !c4k_wready[tm])
                    $display("[TRACE_W_FIFO] %0t W_FIFO_WR_STALL m=%0d vld=1 rdy=0 last=%b",
                             $time, tm, c4k_wlast[tm]);

                // W post-FIFO: pre-FIFO output → crossbar input
                if (M_WVALID[tm] && M_WREADY[tm])
                    $display("[TRACE_W_FIFO] %0t W_FIFO_RD m=%0d last=%b data=0x%08h",
                             $time, tm, M_WLAST[tm], M_WDATA[tm]);
                else if (M_WVALID[tm] && !M_WREADY[tm])
                    $display("[TRACE_W_FIFO] %0t W_FIFO_RD_STALL m=%0d vld=1 rdy=0 last=%b",
                             $time, tm, M_WLAST[tm]);
            end
        end
    end
endgenerate

// TRACE_W: Crossbar internal W — master side (all masters)
generate
    for (tm = 0; tm < MST_AMT; tm = tm + 1) begin : TRACE_W_XBAR_M
        always @(posedge AXI_CLK) begin
            if (AXI_RSTn) begin
                if (x_m_WVALID[tm] && x_m_WREADY[tm])
                    $display("[TRACE_W_XBAR] %0t W_XBAR_M m=%0d last=%b data=0x%08h",
                             $time, tm, x_m_WLAST[tm], x_m_WDATA[tm]);
                else if (x_m_WVALID[tm] && !x_m_WREADY[tm])
                    $display("[TRACE_W_XBAR] %0t W_XBAR_M_STALL m=%0d vld=1 rdy=0",
                             $time, tm);
            end
        end
    end
endgenerate

// TRACE_W: Crossbar internal W — slave side (all slaves)
generate
    for (ts = 0; ts < SLV_AMT; ts = ts + 1) begin : TRACE_W_XBAR_S
        always @(posedge AXI_CLK) begin
            if (AXI_RSTn) begin
                if (x_s_WVALID[ts] && x_s_WREADY[ts])
                    $display("[TRACE_W_XBAR] %0t W_XBAR_S s=%0d last=%b data=0x%08h",
                             $time, ts, x_s_WLAST[ts], x_s_WDATA[ts]);
                else if (x_s_WVALID[ts] && !x_s_WREADY[ts])
                    $display("[TRACE_W_XBAR] %0t W_XBAR_S_STALL s=%0d vld=1 rdy=0",
                             $time, ts);
            end
        end
    end
endgenerate

// TRACE: B merge monitor for m=0
always @(posedge AXI_CLK) begin
    if (AXI_RSTn) begin
        if (c4k_bvalid[0] && c4k_bready[0])
            $display("[TRACE] %0t B merge in m=0 id=0x%02h resp=%b", $time, c4k_bid[0], c4k_bresp[0]);
        if (m_bvalid[0] && m_bready[0])
            $display("[TRACE] %0t B merge out m=0 id=0x%02h resp=%b", $time, m_bid[0], m_bresp[0]);
    end
end

// TRACE: Crossbar B channel interface (slave side → master side)
always @(posedge AXI_CLK) begin
    if (AXI_RSTn) begin
        // Slave side: pre-FIFO output → crossbar input
        if (x_s_BVALID[0] || x_s_BREADY[0])
            $display("[TRACE_XBAR_B] %0t S side s=0 s_bvalid=%b s_bready=%b",
                     $time, x_s_BVALID[0], x_s_BREADY[0]);
        // Master side: crossbar output → post-FIFO input
        if (x_m_BVALID[0] || x_m_BREADY[0])
            $display("[TRACE_XBAR_B] %0t M side m=0 m_bvalid=%b m_bready=%b",
                     $time, x_m_BVALID[0], x_m_BREADY[0]);
    end
end

// TRACE: B path from slave to merge (for slave 0)
always @(posedge AXI_CLK) begin
    if (AXI_RSTn) begin
        // Slave drives B (pre-FIFO write)
        if (s_bvalid_in[0] && s_bready_in[0])
            $display("[TRACE] %0t B slave handshake s=0 id=0x%02h", $time, s_bid_in[0]);
        // Pre-FIFO output to crossbar (pre-FIFO read)
        if (s_bvalid_fifo[0] && s_bready_fifo[0])
            $display("[TRACE] %0t B pre-FIFO out s=0 id=0x%02h", $time, s_bid_fifo[0]);
        // Pre-FIFO has data but crossbar not reading
        if (s_bvalid_fifo[0] && !s_bready_fifo[0])
            $display("[TRACE] %0t B pre-FIFO stuck s=0 vld=1 rdy=0", $time);
        // Post-FIFO (master side) status: wr_rdy determines if crossbar can write
        if (M_BVALID[0] && !M_BREADY[0])
            $display("[TRACE] %0t B post-FIFO stuck m=0 wr_vld=1 wr_rdy=0", $time);
        if (M_BVALID[0] && M_BREADY[0])
            $display("[TRACE] %0t B post-FIFO out m=0 id=0x%02h", $time, M_BID[0]);
    end
end


endmodule
