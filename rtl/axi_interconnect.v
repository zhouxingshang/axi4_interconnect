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
    parameter [0:(SLV_AMT*ADDR_WIDTH)-1]  SLV_ADDR_BASE = {SLV_AMT{ADDR_WIDTH{1'b0}}},
    parameter [0:(SLV_AMT*8)-1]           SLV_ADDR_LEN  = {SLV_AMT{8'd12}}
)
(
    input   wire                      AXI_CLK,
    input   wire                      AXI_RSTn,

    // ========== Master Flattened Interface ==========
    // AW
    input   wire  [W_ID*MST_AMT-1   : 0]  M_AXI_AWID_i,
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
    output  wire  [W_ID*MST_AMT-1   : 0]  M_AXI_BID_o,
    output  wire  [RESP_W*MST_AMT-1           : 0]  M_AXI_BRESP_o,
    output  wire  [MST_AMT-1                  : 0]  M_AXI_BVALID_o,
    input   wire  [MST_AMT-1                  : 0]  M_AXI_BREADY_i,
    // AR
    input   wire  [W_ID*MST_AMT-1   : 0]  M_AXI_ARID_i,
    input   wire  [ADDR_WIDTH*MST_AMT-1       : 0]  M_AXI_ARADDR_i,
    input   wire  [LEN_W*MST_AMT-1            : 0]  M_AXI_ARLEN_i,
    input   wire  [SIZE_W*MST_AMT-1           : 0]  M_AXI_ARSIZE_i,
    input   wire  [BURST_W*MST_AMT-1          : 0]  M_AXI_ARBURST_i,
    input   wire  [MST_AMT-1                  : 0]  M_AXI_ARVALID_i,
    output  wire  [MST_AMT-1                  : 0]  M_AXI_ARREADY_o,
    // R
    output  wire  [W_ID*MST_AMT-1   : 0]  M_AXI_RID_o,
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
    input   wire                      arbiter_type,
    input   wire  [SLV_AMT-1              : 0]  r_order_grant_i
);

//=============================================================================
// Internal Arrays: Unpacked per Master
//=============================================================================
wire [W_ID-1:0]   m_awid      [0:MST_AMT-1];
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

wire [W_ID-1:0]   m_bid       [0:MST_AMT-1];
wire [RESP_W-1:0]           m_bresp     [0:MST_AMT-1];
wire                        m_bvalid    [0:MST_AMT-1];
wire                        m_bready    [0:MST_AMT-1];

wire [W_ID-1:0]   m_arid      [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]       m_araddr    [0:MST_AMT-1];
wire [LEN_W-1:0]            m_arlen     [0:MST_AMT-1];
wire [SIZE_W-1:0]           m_arsize    [0:MST_AMT-1];
wire [BURST_W-1:0]          m_arburst   [0:MST_AMT-1];
wire                        m_arvalid   [0:MST_AMT-1];
wire                        m_arready   [0:MST_AMT-1];

wire [W_ID-1:0]   m_rid       [0:MST_AMT-1];
wire [DATA_WIDTH-1:0]       m_rdata     [0:MST_AMT-1];
wire [RESP_W-1:0]           m_rresp     [0:MST_AMT-1];
wire                        m_rlast     [0:MST_AMT-1];
wire                        m_rvalid    [0:MST_AMT-1];
wire                        m_rready    [0:MST_AMT-1];

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

wire [W_MID-1:0]            c4k_bid     [0:MST_AMT-1];
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

wire [W_MID-1:0]            M_BID      [0:MST_AMT-1];
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

wire [W_ID-1:0]   M_RID      [0:MST_AMT-1];
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
wire [SLV_AMT-1:0]          internal_r_order_grant;
wire [SLV_AMT-1:0]          effective_r_order_grant;

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
            .W_ID   (W_MID),
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
            // B channel (response merging for split writes)
            .s_axi_bid       (c4k_bid[m]),
            .s_axi_bresp     (c4k_bresp[m]),
            .s_axi_bvalid    (c4k_bvalid[m]),
            .s_axi_bready    (c4k_bready[m]),
            .m_axi_bid       (m_bid[m]),
            .m_axi_bresp     (m_bresp[m]),
            .m_axi_bvalid    (m_bvalid[m]),
            .m_axi_bready    (m_bready[m]),
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

// Gate AR ready to crossbar: use s_push_rdy from shared sid_buffer per slave
// rf-style push_rdy = ~(select ^ grant) & clr_allowed
// When buffer has space: push_rdy[s]=1 → arready passes
// When buffer full or clearing: push_rdy[s]=0 → arready gated
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : GEN_ARREADY_GATE
        assign S_ARREADY[s] = S_ARREADY_FIFO[s] & sid_buf_push_rdy[s];
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
// 5. Post-Crossbar FIFOs: Crossbar -> Master (B/R) – unchanged
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : INST_FIFO_B_R_MST
        // B FIFO
        axi_fifo_sync #(
            .FDW(W_MID + RESP_W),
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

        // R FIFO: wr_rdy gated by sid_buffer s_clr_rdy (clear backpressure)
        axi_fifo_sync #(
            .FDW(W_ID + DATA_WIDTH + RESP_W + 1),
            .FAW(2)
        ) u_fifo_r_mst (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (M_RREADY_FIFO[m]),
            .wr_vld (M_RVALID[m]),
            .wr_din ({M_RID[m], M_RDATA[m],
                      M_RRESP[m], M_RLAST[m]}),
            .rd_rdy (m_rready[m]),
            .rd_vld (m_rvalid[m]),
            .rd_dout ({m_rid[m], m_rdata[m], m_rresp[m], m_rlast[m]})
        );
    end
endgenerate

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
// 6. Read Reorder Logic (sid_buffer write/clear + reorder)
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
wire [MST_AMT-1:0]       clr_last_packed;
wire [W_SID*MST_AMT-1:0] clr_sid_packed;
wire [MST_AMT-1:0]       clr_sid_vld_packed;
wire [MST_AMT-1:0]       sid_buf_s_clr_rdy;

genvar si_clr;
generate
    for(si_clr = 0; si_clr < MST_AMT; si_clr = si_clr + 1) begin : SID_BUF_CLR_PACK
        assign clr_last_packed[si_clr]    = M_RLAST[si_clr];
        assign clr_sid_packed[W_SID*(si_clr+1)-1 -: W_SID] = M_RSID[si_clr];
        assign clr_sid_vld_packed[si_clr] = M_RVALID[si_clr] & m_rready[si_clr];
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
// Bypass logic: external r_order_grant_i overrides internal reorder
//=============================================================================
assign internal_r_order_grant = reorder_grant;
assign effective_r_order_grant = (|r_order_grant_i) ? r_order_grant_i : internal_r_order_grant;

//=============================================================================
// Unpack crossbar M_RSID output
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : UNPACK_RSID
        assign M_RSID[m] = M_RSID_PACKED[W_SID*(m+1)-1 -: W_SID];
    end
endgenerate

//=============================================================================
// 7. axi_crossbar: Core Routing Engine (connect to new slave side FIFOs)
//=============================================================================
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
    .DEFAULT_SLV_EN(1'b1)
) u_axi_crossbar (
    .AXI_CLK(AXI_CLK),
    .AXI_RSTn(AXI_RSTn),

    // Master Ports (connected to pre-FIFO outputs)
    .m_AWID_i({M_AWID[MST_AMT-1:0]}),
    .m_AWADDR_i({M_AWADDR[MST_AMT-1:0]}),
    .m_AWLEN_i({M_AWLEN[MST_AMT-1:0]}),
    .m_AWSIZE_i({M_AWSIZE[MST_AMT-1:0]}),
    .m_AWBURST_i({M_AWBURST[MST_AMT-1:0]}),
    .m_AWVALID_i({M_AWVALID[MST_AMT-1:0]}),
    .m_AWREADY_o({M_AWREADY[MST_AMT-1:0]}),
    .m_WDATA_i({M_WDATA[MST_AMT-1:0]}),
    .m_WSTRB_i({M_WSTRB[MST_AMT-1:0]}),
    .m_WLAST_i({M_WLAST[MST_AMT-1:0]}),
    .m_WVALID_i({M_WVALID[MST_AMT-1:0]}),
    .m_WREADY_o({M_WREADY[MST_AMT-1:0]}),
    .m_BID_o({M_BID[MST_AMT-1:0]}),
    .m_BRESP_o({M_BRESP[MST_AMT-1:0]}),
    .m_BVALID_o({M_BVALID[MST_AMT-1:0]}),
    .m_BREADY_i({M_BREADY[MST_AMT-1:0]}),
    .m_ARID_i({M_ARID[MST_AMT-1:0]}),
    .m_ARADDR_i({M_ARADDR[MST_AMT-1:0]}),
    .m_ARLEN_i({M_ARLEN[MST_AMT-1:0]}),
    .m_ARSIZE_i({M_ARSIZE[MST_AMT-1:0]}),
    .m_ARBURST_i({M_ARBURST[MST_AMT-1:0]}),
    .m_ARVALID_i({M_ARVALID[MST_AMT-1:0]}),
    .m_ARREADY_o({M_ARREADY[MST_AMT-1:0]}),
    .m_RID_o({M_RID[MST_AMT-1:0]}),
    .m_RDATA_o({M_RDATA[MST_AMT-1:0]}),
    .m_RRESP_o({M_RRESP[MST_AMT-1:0]}),
    .m_RLAST_o({M_RLAST[MST_AMT-1:0]}),
    .m_RVALID_o({M_RVALID[MST_AMT-1:0]}),
    .m_RREADY_i({M_RREADY[MST_AMT-1:0]}),
    .m_RSID_o(M_RSID_PACKED),

    // Slave Ports (connected to new slave-side FIFOs)
    .s_AWID_o({S_AWID[SLV_AMT-1:0]}),
    .s_AWADDR_o({S_AWADDR[SLV_AMT-1:0]}),
    .s_AWLEN_o({S_AWLEN[SLV_AMT-1:0]}),
    .s_AWSIZE_o({S_AWSIZE[SLV_AMT-1:0]}),
    .s_AWBURST_o({S_AWBURST[SLV_AMT-1:0]}),
    .s_AWVALID_o({S_AWVALID[SLV_AMT-1:0]}),
    .s_AWREADY_i({S_AWREADY[SLV_AMT-1:0]}),
    .s_WDATA_o({S_WDATA[SLV_AMT-1:0]}),
    .s_WSTRB_o({S_WSTRB[SLV_AMT-1:0]}),
    .s_WLAST_o({S_WLAST[SLV_AMT-1:0]}),
    .s_WVALID_o({S_WVALID[SLV_AMT-1:0]}),
    .s_WREADY_i({S_WREADY[SLV_AMT-1:0]}),
    .s_BID_i({s_bid_fifo[SLV_AMT-1:0]}),
    .s_BRESP_i({s_bresp_fifo[SLV_AMT-1:0]}),
    .s_BVALID_i({s_bvalid_fifo[SLV_AMT-1:0]}),
    .s_BREADY_o({s_bready_fifo[SLV_AMT-1:0]}),
    .s_ARID_o({S_ARID[SLV_AMT-1:0]}),
    .s_ARADDR_o({S_ARADDR[SLV_AMT-1:0]}),
    .s_ARLEN_o({S_ARLEN[SLV_AMT-1:0]}),
    .s_ARSIZE_o({S_ARSIZE[SLV_AMT-1:0]}),
    .s_ARBURST_o({S_ARBURST[SLV_AMT-1:0]}),
    .s_ARVALID_o({S_ARVALID[SLV_AMT-1:0]}),
    .s_ARREADY_i({S_ARREADY[SLV_AMT-1:0]}),
    .s_RID_i({s_rid_fifo[SLV_AMT-1:0]}),
    .s_RDATA_i({s_rdata_fifo[SLV_AMT-1:0]}),
    .s_RRESP_i({s_rresp_fifo[SLV_AMT-1:0]}),
    .s_RLAST_i({s_rlast_fifo[SLV_AMT-1:0]}),
    .s_RVALID_i({s_rvalid_fifo[SLV_AMT-1:0]}),
    .s_RREADY_o({s_rready_fifo[SLV_AMT-1:0]}),

    // Control
    .arbiter_type(arbiter_type),
    .r_order_grant_i(effective_r_order_grant),
    .slv_en_i({SLV_AMT{1'b1}})
);

endmodule