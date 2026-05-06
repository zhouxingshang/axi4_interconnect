//=============================================================================
// Module: axi_interconnect
// Desc  : Top-level AXI Interconnect Wrapper
//         Hierarchy: Master -> cross_4k_if -> pre_FIFO -> axi_crossbar -> Slave
//         - Supports arbitrary MST_AMT / SLV_AMT via flattened interfaces
//         - 4KB crossing & WRAP/INCR burst splitting handled by cross_4k_if
//         - Channel decoupling & backpressure handled by axi_fifo_sync
//         - Read reorder via sid_buffer + reorder per master
//         - W-follows-AW handled inside axi_crossbar submodules
//=============================================================================
module axi_interconnect
#(
    parameter MST_AMT           = 4,
    parameter SLV_AMT           = 4,
    parameter OUTSTANDING_AMT   = 8,
    parameter TRANS_MST_ID_W    = 4,
    parameter DATA_WIDTH        = 32,
    parameter ADDR_WIDTH        = 32,
    parameter LEN_W             = 8,
    parameter SIZE_W            = 3,
    parameter BURST_W           = 2,
    parameter RESP_W            = 2,
    parameter W_STRB            = DATA_WIDTH / 8,
    parameter W_SID             = $clog2(MST_AMT) + TRANS_MST_ID_W,

    // Address mapping (default: upper bits select slave)
    parameter [0:(SLV_AMT*ADDR_WIDTH)-1]  SLV_ADDR_BASE = {SLV_AMT{ADDR_WIDTH{1'b0}}},
    parameter [0:(SLV_AMT*8)-1]           SLV_ADDR_LEN  = {SLV_AMT{8'd12}}
)
(
    input   wire                      AXI_CLK,
    input   wire                      AXI_RSTn,

    // ========== Master Flattened Interface ==========
    // AW
    input   wire  [TRANS_MST_ID_W*MST_AMT-1   : 0]  m_AWID_i,
    input   wire  [ADDR_WIDTH*MST_AMT-1       : 0]  m_AWADDR_i,
    input   wire  [LEN_W*MST_AMT-1            : 0]  m_AWLEN_i,
    input   wire  [SIZE_W*MST_AMT-1           : 0]  m_AWSIZE_i,
    input   wire  [BURST_W*MST_AMT-1          : 0]  m_AWBURST_i,
    input   wire  [MST_AMT-1                  : 0]  m_AWVALID_i,
    output  wire  [MST_AMT-1                  : 0]  m_AWREADY_o,
    // W
    input   wire  [DATA_WIDTH*MST_AMT-1       : 0]  m_WDATA_i,
    input   wire  [W_STRB*MST_AMT-1           : 0]  m_WSTRB_i,
    input   wire  [MST_AMT-1                  : 0]  m_WLAST_i,
    input   wire  [MST_AMT-1                  : 0]  m_WVALID_i,
    output  wire  [MST_AMT-1                  : 0]  m_WREADY_o,
    // B
    output  wire  [TRANS_MST_ID_W*MST_AMT-1   : 0]  m_BID_o,
    output  wire  [RESP_W*MST_AMT-1           : 0]  m_BRESP_o,
    output  wire  [MST_AMT-1                  : 0]  m_BVALID_o,
    input   wire  [MST_AMT-1                  : 0]  m_BREADY_i,
    // AR
    input   wire  [TRANS_MST_ID_W*MST_AMT-1   : 0]  m_ARID_i,
    input   wire  [ADDR_WIDTH*MST_AMT-1       : 0]  m_ARADDR_i,
    input   wire  [LEN_W*MST_AMT-1            : 0]  m_ARLEN_i,
    input   wire  [SIZE_W*MST_AMT-1           : 0]  m_ARSIZE_i,
    input   wire  [BURST_W*MST_AMT-1          : 0]  m_ARBURST_i,
    input   wire  [MST_AMT-1                  : 0]  m_ARVALID_i,
    output  wire  [MST_AMT-1                  : 0]  m_ARREADY_o,
    // R
    output  wire  [TRANS_MST_ID_W*MST_AMT-1   : 0]  m_RID_o,
    output  wire  [DATA_WIDTH*MST_AMT-1       : 0]  m_RDATA_o,
    output  wire  [RESP_W*MST_AMT-1           : 0]  m_RRESP_o,
    output  wire  [MST_AMT-1                  : 0]  m_RLAST_o,
    output  wire  [MST_AMT-1                  : 0]  m_RVALID_o,
    input   wire  [MST_AMT-1                  : 0]  m_RREADY_i,

    // ========== Slave Flattened Interface ==========
    // AW
    output  wire  [W_SID*SLV_AMT-1            : 0]  s_AWID_o,
    output  wire  [ADDR_WIDTH*SLV_AMT-1       : 0]  s_AWADDR_o,
    output  wire  [LEN_W*SLV_AMT-1            : 0]  s_AWLEN_o,
    output  wire  [SIZE_W*SLV_AMT-1           : 0]  s_AWSIZE_o,
    output  wire  [BURST_W*SLV_AMT-1          : 0]  s_AWBURST_o,
    output  wire  [SLV_AMT-1                  : 0]  s_AWVALID_o,
    input   wire  [SLV_AMT-1                  : 0]  s_AWREADY_i,
    // W
    output  wire  [DATA_WIDTH*SLV_AMT-1       : 0]  s_WDATA_o,
    output  wire  [W_STRB*SLV_AMT-1           : 0]  s_WSTRB_o,
    output  wire  [SLV_AMT-1                  : 0]  s_WLAST_o,
    output  wire  [SLV_AMT-1                  : 0]  s_WVALID_o,
    input   wire  [SLV_AMT-1                  : 0]  s_WREADY_i,
    // B
    input   wire  [W_SID*SLV_AMT-1            : 0]  s_BID_i,
    input   wire  [RESP_W*SLV_AMT-1           : 0]  s_BRESP_i,
    input   wire  [SLV_AMT-1                  : 0]  s_BVALID_i,
    output  wire  [SLV_AMT-1                  : 0]  s_BREADY_o,
    // AR
    output  wire  [W_SID*SLV_AMT-1            : 0]  s_ARID_o,
    output  wire  [ADDR_WIDTH*SLV_AMT-1       : 0]  s_ARADDR_o,
    output  wire  [LEN_W*SLV_AMT-1            : 0]  s_ARLEN_o,
    output  wire  [SIZE_W*SLV_AMT-1           : 0]  s_ARSIZE_o,
    output  wire  [BURST_W*SLV_AMT-1          : 0]  s_ARBURST_o,
    output  wire  [SLV_AMT-1                  : 0]  s_ARVALID_o,
    input   wire  [SLV_AMT-1                  : 0]  s_ARREADY_i,
    // R
    input   wire  [W_SID*SLV_AMT-1            : 0]  s_RID_i,
    input   wire  [DATA_WIDTH*SLV_AMT-1       : 0]  s_RDATA_i,
    input   wire  [RESP_W*SLV_AMT-1           : 0]  s_RRESP_i,
    input   wire  [SLV_AMT-1                  : 0]  s_RLAST_i,
    input   wire  [SLV_AMT-1                  : 0]  s_RVALID_i,
    output  wire  [SLV_AMT-1                  : 0]  s_RREADY_o,

    // ========== Control Ports ==========
    input   wire                      arbiter_type,
    input   wire  [MST_AMT*SLV_AMT-1  : 0]  r_order_grant_i
);

//=============================================================================
// Internal Arrays: Unpacked per Master
//=============================================================================
wire [TRANS_MST_ID_W-1:0]   m_awid      [0:MST_AMT-1];
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

wire [TRANS_MST_ID_W-1:0]   m_bid       [0:MST_AMT-1];
wire [RESP_W-1:0]           m_bresp     [0:MST_AMT-1];
wire                        m_bvalid    [0:MST_AMT-1];
wire                        m_bready    [0:MST_AMT-1];

wire [TRANS_MST_ID_W-1:0]   m_arid      [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]       m_araddr    [0:MST_AMT-1];
wire [LEN_W-1:0]            m_arlen     [0:MST_AMT-1];
wire [SIZE_W-1:0]           m_arsize    [0:MST_AMT-1];
wire [BURST_W-1:0]          m_arburst   [0:MST_AMT-1];
wire                        m_arvalid   [0:MST_AMT-1];
wire                        m_arready   [0:MST_AMT-1];

wire [TRANS_MST_ID_W-1:0]   m_rid       [0:MST_AMT-1];
wire [DATA_WIDTH-1:0]       m_rdata     [0:MST_AMT-1];
wire [RESP_W-1:0]           m_rresp     [0:MST_AMT-1];
wire                        m_rlast     [0:MST_AMT-1];
wire                        m_rvalid    [0:MST_AMT-1];
wire                        m_rready    [0:MST_AMT-1];

//=============================================================================
// Pre-FIFO buses: between cross_4k_if (or master for W) and pre-FIFO input
//=============================================================================
wire [TRANS_MST_ID_W-1:0]   pre_awid    [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]       pre_awaddr  [0:MST_AMT-1];
wire [LEN_W-1:0]            pre_awlen   [0:MST_AMT-1];
wire [SIZE_W-1:0]           pre_awsize  [0:MST_AMT-1];
wire [BURST_W-1:0]          pre_awburst [0:MST_AMT-1];
wire                        pre_awvalid [0:MST_AMT-1];
wire                        pre_awready [0:MST_AMT-1];

wire [DATA_WIDTH-1:0]       pre_wdata   [0:MST_AMT-1];
wire [W_STRB-1:0]           pre_wstrb   [0:MST_AMT-1];
wire                        pre_wlast   [0:MST_AMT-1];
wire                        pre_wvalid  [0:MST_AMT-1];
wire                        pre_wready  [0:MST_AMT-1];

wire [TRANS_MST_ID_W-1:0]   pre_arid    [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]       pre_araddr  [0:MST_AMT-1];
wire [LEN_W-1:0]            pre_arlen   [0:MST_AMT-1];
wire [SIZE_W-1:0]           pre_arsize  [0:MST_AMT-1];
wire [BURST_W-1:0]          pre_arburst [0:MST_AMT-1];
wire                        pre_arvalid [0:MST_AMT-1];
wire                        pre_arready [0:MST_AMT-1];

//=============================================================================
// Post-FIFO buses: between crossbar output and FIFO input → FIFO output to master
//=============================================================================
wire [TRANS_MST_ID_W-1:0]   cbar_m_awid     [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]       cbar_m_awaddr   [0:MST_AMT-1];
wire [LEN_W-1:0]            cbar_m_awlen    [0:MST_AMT-1];
wire [SIZE_W-1:0]           cbar_m_awsize   [0:MST_AMT-1];
wire [BURST_W-1:0]          cbar_m_awburst  [0:MST_AMT-1];
wire                        cbar_m_awvalid  [0:MST_AMT-1];
wire                        cbar_m_awready  [0:MST_AMT-1];

wire [DATA_WIDTH-1:0]       cbar_m_wdata    [0:MST_AMT-1];
wire [W_STRB-1:0]           cbar_m_wstrb    [0:MST_AMT-1];
wire                        cbar_m_wlast    [0:MST_AMT-1];
wire                        cbar_m_wvalid   [0:MST_AMT-1];
wire                        cbar_m_wready   [0:MST_AMT-1];

wire [TRANS_MST_ID_W-1:0]   cbar_m_bid      [0:MST_AMT-1];
wire [RESP_W-1:0]           cbar_m_bresp    [0:MST_AMT-1];
wire                        cbar_m_bvalid   [0:MST_AMT-1];
wire                        cbar_m_bready   [0:MST_AMT-1];

wire [TRANS_MST_ID_W-1:0]   cbar_m_arid     [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]       cbar_m_araddr   [0:MST_AMT-1];
wire [LEN_W-1:0]            cbar_m_arlen    [0:MST_AMT-1];
wire [SIZE_W-1:0]           cbar_m_arsize   [0:MST_AMT-1];
wire [BURST_W-1:0]          cbar_m_arburst  [0:MST_AMT-1];
wire                        cbar_m_arvalid  [0:MST_AMT-1];
wire                        cbar_m_arready  [0:MST_AMT-1];

wire [TRANS_MST_ID_W-1:0]   cbar_m_rid      [0:MST_AMT-1];
wire [DATA_WIDTH-1:0]       cbar_m_rdata    [0:MST_AMT-1];
wire [RESP_W-1:0]           cbar_m_rresp    [0:MST_AMT-1];
wire                        cbar_m_rlast    [0:MST_AMT-1];
wire                        cbar_m_rvalid   [0:MST_AMT-1];
wire                        cbar_m_rready   [0:MST_AMT-1];

//=============================================================================
// Crossbar M_RSID output (full slave-side RID, for sid_buffer clear)
//=============================================================================
wire [W_SID*MST_AMT-1:0]    cbar_m_rsid_packed;
wire [W_SID-1:0]            cbar_m_rsid     [0:MST_AMT-1];

//=============================================================================
// Reorder / sid_buffer interconnect signals
//=============================================================================
// Per-slave unpacked AR outputs (from crossbar, for sid_buffer write)
wire [W_SID-1:0]            s_arid_unpk     [0:SLV_AMT-1];
wire                        s_arvalid_unpk  [0:SLV_AMT-1];
wire                        s_arready_unpk  [0:SLV_AMT-1];

// Per-slave unpacked R inputs (from slaves, for reorder)
wire [W_SID-1:0]            s_rid_unpk      [0:SLV_AMT-1];
wire                        s_rvalid_unpk   [0:SLV_AMT-1];

// Per-master sid_buffer output (connected to reorder rob_buffer)
wire [W_SID-1:0]            sid_buf_out     [0:MST_AMT-1][0:3];  // DEPTH=4

// Per-master reorder grant outputs
wire [SLV_AMT-1:0]          reorder_grant_m [0:MST_AMT-1];

// Packed internal reorder grant (from reorder modules)
wire [MST_AMT*SLV_AMT-1:0]  internal_r_order_grant;

//=============================================================================
// Port Flattening (Unpack Master Inputs / Pack Master Outputs)
//=============================================================================
genvar m, s;
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : UNPACK_MST
        assign m_awid[m]    = m_AWID_i[TRANS_MST_ID_W*(m+1)-1 -: TRANS_MST_ID_W];
        assign m_awaddr[m]  = m_AWADDR_i[ADDR_WIDTH*(m+1)-1 -: ADDR_WIDTH];
        assign m_awlen[m]   = m_AWLEN_i[LEN_W*(m+1)-1 -: LEN_W];
        assign m_awsize[m]  = m_AWSIZE_i[SIZE_W*(m+1)-1 -: SIZE_W];
        assign m_awburst[m] = m_AWBURST_i[BURST_W*(m+1)-1 -: BURST_W];
        assign m_awvalid[m] = m_AWVALID_i[m];
        assign m_AWREADY_o[m] = m_awready[m];

        assign m_wdata[m]   = m_WDATA_i[DATA_WIDTH*(m+1)-1 -: DATA_WIDTH];
        assign m_wstrb[m]   = m_WSTRB_i[W_STRB*(m+1)-1 -: W_STRB];
        assign m_wlast[m]   = m_WLAST_i[m];
        assign m_wvalid[m]  = m_WVALID_i[m];
        assign m_WREADY_o[m] = m_wready[m];

        assign m_bready[m]  = m_BREADY_i[m];
        assign m_BID_o[TRANS_MST_ID_W*(m+1)-1 -: TRANS_MST_ID_W] = m_bid[m];
        assign m_BRESP_o[RESP_W*(m+1)-1 -: RESP_W] = m_bresp[m];
        assign m_BVALID_o[m] = m_bvalid[m];

        assign m_arid[m]    = m_ARID_i[TRANS_MST_ID_W*(m+1)-1 -: TRANS_MST_ID_W];
        assign m_araddr[m]  = m_ARADDR_i[ADDR_WIDTH*(m+1)-1 -: ADDR_WIDTH];
        assign m_arlen[m]   = m_ARLEN_i[LEN_W*(m+1)-1 -: LEN_W];
        assign m_arsize[m]  = m_ARSIZE_i[SIZE_W*(m+1)-1 -: SIZE_W];
        assign m_arburst[m] = m_ARBURST_i[BURST_W*(m+1)-1 -: BURST_W];
        assign m_arvalid[m] = m_ARVALID_i[m];
        assign m_ARREADY_o[m] = m_arready[m];

        assign m_rready[m]  = m_RREADY_i[m];
        assign m_RID_o[TRANS_MST_ID_W*(m+1)-1 -: TRANS_MST_ID_W] = m_rid[m];
        assign m_RDATA_o[DATA_WIDTH*(m+1)-1 -: DATA_WIDTH] = m_rdata[m];
        assign m_RRESP_o[RESP_W*(m+1)-1 -: RESP_W] = m_rresp[m];
        assign m_RLAST_o[m] = m_rlast[m];
        assign m_RVALID_o[m] = m_rvalid[m];

        // W channel: direct from master to pre-FIFO (cross_4k_if handles only AW/AR)
        assign pre_wdata[m]  = m_wdata[m];
        assign pre_wstrb[m]  = m_wstrb[m];
        assign pre_wlast[m]  = m_wlast[m];
        assign pre_wvalid[m] = m_wvalid[m];
        assign m_wready[m]   = pre_wready[m];
    end
endgenerate

//=============================================================================
// 1. cross_4k_if: 4KB Boundary & WRAP/INCR Splitting (per Master)
//    Outputs now drive pre_* buses (not cbar_m_*)
//=============================================================================
genvar c4k;
generate
    for(c4k = 0; c4k < MST_AMT; c4k = c4k + 1) begin : INST_CROSS_4K
        cross_4k_if #(
            .W_ID   (TRANS_MST_ID_W),
            .W_CID  ($clog2(SLV_AMT)),
            .W_ADDR (ADDR_WIDTH),
            .W_LEN  (LEN_W),
            .W_DATA (DATA_WIDTH),
            .W_STRB (W_STRB),
            .W_SID  (W_SID)
        ) u_cross_4k (
            .clk             (AXI_CLK),
            .rst_n           (AXI_RSTn),
            // Master AW/AR -> Splitter
            .m_axi_arid      (m_arid[c4k]),
            .m_axi_araddr    (m_araddr[c4k]),
            .m_axi_arlen     (m_arlen[c4k]),
            .m_axi_arsize    (m_arsize[c4k]),
            .m_axi_arburst   (m_arburst[c4k]),
            .m_axi_arvalid   (m_arvalid[c4k]),
            .m_axi_arready   (m_arready[c4k]),
            .m_axi_awid      (m_awid[c4k]),
            .m_axi_awaddr    (m_awaddr[c4k]),
            .m_axi_awlen     (m_awlen[c4k]),
            .m_axi_awsize    (m_awsize[c4k]),
            .m_axi_awburst   (m_awburst[c4k]),
            .m_axi_awvalid   (m_awvalid[c4k]),
            .m_axi_awready   (m_awready[c4k]),
            // Splitter -> pre-FIFO (drives pre_* buses)
            .s_axi_arid      (pre_arid[c4k]),
            .s_axi_araddr    (pre_araddr[c4k]),
            .s_axi_arlen     (pre_arlen[c4k]),
            .s_axi_arsize    (pre_arsize[c4k]),
            .s_axi_arburst   (pre_arburst[c4k]),
            .s_axi_arvalid   (pre_arvalid[c4k]),
            .s_axi_arready   (pre_arready[c4k]),
            .s_axi_awid      (pre_awid[c4k]),
            .s_axi_awaddr    (pre_awaddr[c4k]),
            .s_axi_awlen     (pre_awlen[c4k]),
            .s_axi_awsize    (pre_awsize[c4k]),
            .s_axi_awburst   (pre_awburst[c4k]),
            .s_axi_awvalid   (pre_awvalid[c4k]),
            .s_axi_awready   (pre_awready[c4k])
        );
    end
endgenerate

//=============================================================================
// 2. Pre-Crossbar FIFOs: AW/W/AR (Master -> Crossbar)
//    wr_din from pre_* buses, rd_dout drives cbar_m_* buses
//=============================================================================
genvar fifo_aw_ar;
generate
    for(fifo_aw_ar = 0; fifo_aw_ar < MST_AMT; fifo_aw_ar = fifo_aw_ar + 1) begin : INST_FIFO_AW_W_AR
        // AW FIFO: pre_aw* → cbar_m_aw*
        axi_fifo_sync #(
            .FDW(TRANS_MST_ID_W + ADDR_WIDTH + LEN_W + SIZE_W + BURST_W + 1),
            .FAW(2)
        ) u_fifo_aw (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (pre_awready[fifo_aw_ar]),
            .wr_vld (pre_awvalid[fifo_aw_ar]),
            .wr_din ({pre_awid[fifo_aw_ar], pre_awaddr[fifo_aw_ar],
                      pre_awlen[fifo_aw_ar], pre_awsize[fifo_aw_ar],
                      pre_awburst[fifo_aw_ar], pre_awvalid[fifo_aw_ar]}),
            .rd_rdy (cbar_m_awready[fifo_aw_ar]),
            .rd_vld (cbar_m_awvalid[fifo_aw_ar]),
            .rd_dout ({cbar_m_awid[fifo_aw_ar], cbar_m_awaddr[fifo_aw_ar],
                       cbar_m_awlen[fifo_aw_ar], cbar_m_awsize[fifo_aw_ar],
                       cbar_m_awburst[fifo_aw_ar], cbar_m_awvalid[fifo_aw_ar]})
        );

        // W FIFO: pre_w* → cbar_m_w*
        axi_fifo_sync #(
            .FDW(DATA_WIDTH + W_STRB + 1),
            .FAW(2)
        ) u_fifo_w (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (pre_wready[fifo_aw_ar]),
            .wr_vld (pre_wvalid[fifo_aw_ar]),
            .wr_din ({pre_wdata[fifo_aw_ar], pre_wstrb[fifo_aw_ar], pre_wlast[fifo_aw_ar]}),
            .rd_rdy (cbar_m_wready[fifo_aw_ar]),
            .rd_vld (cbar_m_wvalid[fifo_aw_ar]),
            .rd_dout ({cbar_m_wdata[fifo_aw_ar], cbar_m_wstrb[fifo_aw_ar], cbar_m_wlast[fifo_aw_ar]})
        );

        // AR FIFO: pre_ar* → cbar_m_ar*
        axi_fifo_sync #(
            .FDW(TRANS_MST_ID_W + ADDR_WIDTH + LEN_W + SIZE_W + BURST_W + 1),
            .FAW(2)
        ) u_fifo_ar (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (pre_arready[fifo_aw_ar]),
            .wr_vld (pre_arvalid[fifo_aw_ar]),
            .wr_din ({pre_arid[fifo_aw_ar], pre_araddr[fifo_aw_ar],
                      pre_arlen[fifo_aw_ar], pre_arsize[fifo_aw_ar],
                      pre_arburst[fifo_aw_ar], pre_arvalid[fifo_aw_ar]}),
            .rd_rdy (cbar_m_arready[fifo_aw_ar]),
            .rd_vld (cbar_m_arvalid[fifo_aw_ar]),
            .rd_dout ({cbar_m_arid[fifo_aw_ar], cbar_m_araddr[fifo_aw_ar],
                       cbar_m_arlen[fifo_aw_ar], cbar_m_arsize[fifo_aw_ar],
                       cbar_m_arburst[fifo_aw_ar], cbar_m_arvalid[fifo_aw_ar]})
        );
    end
endgenerate

//=============================================================================
// 3. Post-Crossbar FIFOs: B/R (Crossbar -> Master)
//    Crossbar output cbar_m_b* → FIFO → m_b* (correct pattern, kept)
//=============================================================================
genvar fifo_b_r;
generate
    for(fifo_b_r = 0; fifo_b_r < MST_AMT; fifo_b_r = fifo_b_r + 1) begin : INST_FIFO_B_R
        // B FIFO: stores {bid, bresp}, rd_vld drives m_bvalid
        axi_fifo_sync #(
            .FDW(TRANS_MST_ID_W + RESP_W),
            .FAW(2)
        ) u_fifo_b (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (cbar_m_bready[fifo_b_r]),
            .wr_vld (cbar_m_bvalid[fifo_b_r]),
            .wr_din ({cbar_m_bid[fifo_b_r], cbar_m_bresp[fifo_b_r]}),
            .rd_rdy (m_bready[fifo_b_r]),
            .rd_vld (m_bvalid[fifo_b_r]),
            .rd_dout ({m_bid[fifo_b_r], m_bresp[fifo_b_r]})
        );

        // R FIFO: stores {rid, rdata, rresp, rlast}, rd_vld drives m_rvalid
        axi_fifo_sync #(
            .FDW(TRANS_MST_ID_W + DATA_WIDTH + RESP_W + 1),
            .FAW(2)
        ) u_fifo_r (
            .rstn   (AXI_RSTn), .clr(1'b0), .clk(AXI_CLK),
            .wr_rdy (cbar_m_rready[fifo_b_r]),
            .wr_vld (cbar_m_rvalid[fifo_b_r]),
            .wr_din ({cbar_m_rid[fifo_b_r], cbar_m_rdata[fifo_b_r],
                      cbar_m_rresp[fifo_b_r], cbar_m_rlast[fifo_b_r]}),
            .rd_rdy (m_rready[fifo_b_r]),
            .rd_vld (m_rvalid[fifo_b_r]),
            .rd_dout ({m_rid[fifo_b_r], m_rdata[fifo_b_r],
                       m_rresp[fifo_b_r], m_rlast[fifo_b_r]})
        );
    end
endgenerate

//=============================================================================
// 4. Slave-side signal unpacking (for sid_buffer and reorder)
//=============================================================================
genvar s_unpack;
generate
    for(s_unpack = 0; s_unpack < SLV_AMT; s_unpack = s_unpack + 1) begin : UNPACK_SLV_SID
        // AR outputs from crossbar → sid_buffer write data
        assign s_arid_unpk[s_unpack]    = s_ARID_o[W_SID*(s_unpack+1)-1 -: W_SID];
        assign s_arvalid_unpk[s_unpack] = s_ARVALID_o[s_unpack];
        assign s_arready_unpk[s_unpack] = s_ARREADY_i[s_unpack];

        // R inputs from slaves → reorder
        assign s_rid_unpk[s_unpack]     = s_RID_i[W_SID*(s_unpack+1)-1 -: W_SID];
        assign s_rvalid_unpk[s_unpack]  = s_RVALID_i[s_unpack];
    end
endgenerate

//=============================================================================
// 5. Read Reorder Logic: sid_buffer + reorder per Master
//=============================================================================
genvar rm;
generate
    for(rm = 0; rm < MST_AMT; rm = rm + 1) begin : INST_REORDER_M

        // ----- sid_buffer -----
        // Writes: capture s_ARID on per-slave AR handshake
        // Clear:  from crossbar master-side R completion (m_RSID + m_RVALID + m_RLAST)

        wire [W_SID*SLV_AMT-1:0] sid_buf_wr_id_packed;
        wire [SLV_AMT-1:0]       sid_buf_wr_vld;
        wire [SLV_AMT-1:0]       sid_buf_wr_rdy;

        genvar si_wr;
        for(si_wr = 0; si_wr < SLV_AMT; si_wr = si_wr + 1) begin : SID_BUF_WR_PACK
            assign sid_buf_wr_id_packed[W_SID*(si_wr+1)-1 -: W_SID] = s_arid_unpk[si_wr];
            // Write on AR handshake completion
            assign sid_buf_wr_vld[si_wr] = s_arvalid_unpk[si_wr] & s_arready_unpk[si_wr];
            // Always ready to accept (only enabled when vld & rdy)
            assign sid_buf_wr_rdy[si_wr] = 1'b1;
        end

        // Clear on: master-side R handshake with last beat
        wire clr_last_wire = cbar_m_rlast[rm];
        wire clr_vld_wire  = cbar_m_rvalid[rm] & m_rready[rm];

        sid_buffer #(
            .NUM  (SLV_AMT),
            .W_ID (W_SID),
            .DEPTH(4)
        ) u_sid_buffer (
            .clk         (AXI_CLK),
            .rstn        (AXI_RSTn),
            // Write ports
            .s_axid      (sid_buf_wr_id_packed),
            .s_axid_vld  (sid_buf_wr_vld),
            .s_fifo_rdy  (sid_buf_wr_rdy),
            .s_push_rdy  (),                           // unused
            // Single clear port (from crossbar master-side R)
            .clr_last    (clr_last_wire),
            .clr_sid     (cbar_m_rsid[rm]),
            .clr_sid_vld (clr_vld_wire),
            .clr_rdy     (),                           // unused
            // Buffer output
            .sid_buffer  (sid_buf_out[rm])
        );

        // ----- reorder -----
        // Inputs: per-slave RID/RVALID from slave response channel
        // rob_buffer: from sid_buffer output (expected AR order)

        wire [W_SID*SLV_AMT-1:0] reorder_sid_packed;
        wire [SLV_AMT-1:0]       reorder_sid_vld;

        genvar si_ro;
        for(si_ro = 0; si_ro < SLV_AMT; si_ro = si_ro + 1) begin : REORDER_IN_PACK
            assign reorder_sid_packed[W_SID*(si_ro+1)-1 -: W_SID] = s_rid_unpk[si_ro];
            assign reorder_sid_vld[si_ro] = s_rvalid_unpk[si_ro];
        end

        reorder #(
            .NUM  (SLV_AMT),
            .W_ID (W_SID),
            .DEPTH(4)
        ) u_reorder (
            .clk         (AXI_CLK),
            .rstn        (AXI_RSTn),
            .s_sid       (reorder_sid_packed),
            .s_sid_vld   (reorder_sid_vld),
            .rob_buffer  (sid_buf_out[rm]),
            .order_grant (reorder_grant_m[rm])
        );
    end
endgenerate

//=============================================================================
// 6. Pack internal reorder grants + bypass logic
//=============================================================================
genvar rg;
generate
    for(rg = 0; rg < MST_AMT; rg = rg + 1) begin : PACK_RORDER_GRANT
        assign internal_r_order_grant[SLV_AMT*(rg+1)-1 -: SLV_AMT] = reorder_grant_m[rg];
    end
endgenerate

// External r_order_grant_i can override internal reorder (for debug/bypass):
//   - If r_order_grant_i is all-zeros, use internal reorder grants
//   - Otherwise, use external grants directly
wire [MST_AMT*SLV_AMT-1:0] effective_r_order_grant;
assign effective_r_order_grant = (|r_order_grant_i) ? r_order_grant_i : internal_r_order_grant;

//=============================================================================
// 7. Unpack crossbar M_RSID output
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : UNPACK_RSID
        assign cbar_m_rsid[m] = cbar_m_rsid_packed[W_SID*(m+1)-1 -: W_SID];
    end
endgenerate

//=============================================================================
// 8. axi_crossbar: Core Routing Engine
//=============================================================================
axi_crossbar #(
    .MST_AMT(MST_AMT),
    .SLV_AMT(SLV_AMT),
    .OUTSTANDING_AMT(OUTSTANDING_AMT),
    .TRANS_MST_ID_W(TRANS_MST_ID_W),
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

    // Master Ports
    .m_AWID_i({cbar_m_awid[MST_AMT-1:0]}),
    .m_AWADDR_i({cbar_m_awaddr[MST_AMT-1:0]}),
    .m_AWLEN_i({cbar_m_awlen[MST_AMT-1:0]}),
    .m_AWSIZE_i({cbar_m_awsize[MST_AMT-1:0]}),
    .m_AWBURST_i({cbar_m_awburst[MST_AMT-1:0]}),
    .m_AWVALID_i({cbar_m_awvalid[MST_AMT-1:0]}),
    .m_AWREADY_o({cbar_m_awready[MST_AMT-1:0]}),
    .m_WDATA_i({cbar_m_wdata[MST_AMT-1:0]}),
    .m_WSTRB_i({cbar_m_wstrb[MST_AMT-1:0]}),
    .m_WLAST_i({cbar_m_wlast[MST_AMT-1:0]}),
    .m_WVALID_i({cbar_m_wvalid[MST_AMT-1:0]}),
    .m_WREADY_o({cbar_m_wready[MST_AMT-1:0]}),
    .m_BID_o({cbar_m_bid[MST_AMT-1:0]}),
    .m_BRESP_o({cbar_m_bresp[MST_AMT-1:0]}),
    .m_BVALID_o({cbar_m_bvalid[MST_AMT-1:0]}),
    .m_BREADY_i({cbar_m_bready[MST_AMT-1:0]}),
    .m_ARID_i({cbar_m_arid[MST_AMT-1:0]}),
    .m_ARADDR_i({cbar_m_araddr[MST_AMT-1:0]}),
    .m_ARLEN_i({cbar_m_arlen[MST_AMT-1:0]}),
    .m_ARSIZE_i({cbar_m_arsize[MST_AMT-1:0]}),
    .m_ARBURST_i({cbar_m_arburst[MST_AMT-1:0]}),
    .m_ARVALID_i({cbar_m_arvalid[MST_AMT-1:0]}),
    .m_ARREADY_o({cbar_m_arready[MST_AMT-1:0]}),
    .m_RID_o({cbar_m_rid[MST_AMT-1:0]}),
    .m_RDATA_o({cbar_m_rdata[MST_AMT-1:0]}),
    .m_RRESP_o({cbar_m_rresp[MST_AMT-1:0]}),
    .m_RLAST_o({cbar_m_rlast[MST_AMT-1:0]}),
    .m_RVALID_o({cbar_m_rvalid[MST_AMT-1:0]}),
    .m_RREADY_i({cbar_m_rready[MST_AMT-1:0]}),
    .m_RSID_o(cbar_m_rsid_packed),

    // Slave Ports (Direct to external)
    .s_AWID_o(s_AWID_o), .s_AWADDR_o(s_AWADDR_o), .s_AWLEN_o(s_AWLEN_o),
    .s_AWSIZE_o(s_AWSIZE_o), .s_AWBURST_o(s_AWBURST_o), .s_AWVALID_o(s_AWVALID_o),
    .s_AWREADY_i(s_AWREADY_i),
    .s_WDATA_o(s_WDATA_o), .s_WSTRB_o(s_WSTRB_o), .s_WLAST_o(s_WLAST_o),
    .s_WVALID_o(s_WVALID_o), .s_WREADY_i(s_WREADY_i),
    .s_BID_i(s_BID_i), .s_BRESP_i(s_BRESP_i), .s_BVALID_i(s_BVALID_i),
    .s_BREADY_o(s_BREADY_o),
    .s_ARID_o(s_ARID_o), .s_ARADDR_o(s_ARADDR_o), .s_ARLEN_o(s_ARLEN_o),
    .s_ARSIZE_o(s_ARSIZE_o), .s_ARBURST_o(s_ARBURST_o), .s_ARVALID_o(s_ARVALID_o),
    .s_ARREADY_i(s_ARREADY_i),
    .s_RID_i(s_RID_i), .s_RDATA_i(s_RDATA_i), .s_RRESP_i(s_RRESP_i),
    .s_RLAST_i(s_RLAST_i), .s_RVALID_i(s_RVALID_i), .s_RREADY_o(s_RREADY_o),

    // Control
    .arbiter_type(arbiter_type),
    .r_order_grant_i(effective_r_order_grant),
    .slv_en_i({SLV_AMT{1'b1}})
);

endmodule
