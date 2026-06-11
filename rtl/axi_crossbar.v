module axi_crossbar
#(
    // ========== 互连配置 ==========
    parameter MST_AMT               = 4,              // 主设备数量
    parameter SLV_AMT               = 4,              // 从设备数量
    parameter OUTSTANDING_AMT       = 8,              // 每主设备最大未完成事务数

    // ========== 事务配置 ==========
    parameter W_ID        = 4,              // 主设备事务 ID 位宽
    parameter TRANS_BURST_W         = 2,              // xBURST 字段位宽
    parameter TRANS_DATA_LEN_W      = 8,              // xLEN 字段位宽
    parameter TRANS_DATA_SIZE_W     = 3,              // xSIZE 字段位宽
    parameter TRANS_WR_RESP_W       = 2,              // xRESP 字段位宽
    parameter DATA_WIDTH            = 32,             // 数据总线位宽
    parameter ADDR_WIDTH            = 32,             // 地址总线位宽

    // ========== 派生参数 ==========
    parameter MST_ID_W              = $clog2(MST_AMT),           // 主设备索引编码位宽
    parameter SLV_ID_W              = 2,           // 从设备索引编码位宽
    parameter W_STRB                = DATA_WIDTH / 8,            // 字节使能位宽
    parameter W_MID                 = SLV_ID_W + W_ID, // master-side ID width after cross_4k_if extension
    parameter W_SID                 = MST_ID_W + SLV_ID_W + W_ID, // 从设备侧 ID 位宽 (MST_IDX + C4K_PREFIX + ORIG_ID)

    // ========== 地址映射配置 ==========
    // 默认: 地址高位用于选择从设备
    parameter SLV_ID_MSB_IDX        = ADDR_WIDTH - 1,
    parameter SLV_ID_LSB_IDX        = ADDR_WIDTH - SLV_ID_W,

    // ========== 从设备地址基址数组 ==========
    // 格式: {SLV_AMT{ADDR_WIDTH{1'b0}}} - 用户需按从设备逐一覆写
    parameter [(SLV_AMT*ADDR_WIDTH)-1:0] SLV_ADDR_BASE = 'b0,
    parameter [(SLV_AMT*8)-1:0]          SLV_ADDR_LEN  = {SLV_AMT{8'd12}}, // 默认 12 位译码

    // ========== 默认从设备配置 ==========
    parameter DEFAULT_SLV_IDX       = 0,              // 默认从设备索引 (未命中地址空间的访问)
    parameter DEFAULT_SLV_EN        = 1'b1,           // 使能默认从设备路由
    parameter [SLV_AMT-1:0] SLV_DEFAULT_MASK = ((DEFAULT_SLV_EN) ? (1 << DEFAULT_SLV_IDX) : '0)  // bit[s]=1 表示 slave s 为默认设备
)
(
    // ========== 全局信号 ==========
    input   wire                      AXI_RSTn,
    input   wire                      AXI_CLK,

    // ========== 主设备侧接口 (扁平化) ==========
    // -- 写地址通道 (AW)
    input   wire  [W_MID*MST_AMT-1              : 0]  m_AWID_i,
    input   wire  [ADDR_WIDTH*MST_AMT-1         : 0]  m_AWADDR_i,
    input   wire  [TRANS_BURST_W*MST_AMT-1      : 0]  m_AWBURST_i,
    input   wire  [TRANS_DATA_LEN_W*MST_AMT-1   : 0]  m_AWLEN_i,
    input   wire  [TRANS_DATA_SIZE_W*MST_AMT-1  : 0]  m_AWSIZE_i,
    input   wire  [MST_AMT-1                    : 0]  m_AWVALID_i,
    output  wire  [MST_AMT-1                    : 0]  m_AWREADY_o,

    // -- 写数据通道 (W)
    input   wire  [DATA_WIDTH*MST_AMT-1         : 0]  m_WDATA_i,
    input   wire  [W_STRB*MST_AMT-1             : 0]  m_WSTRB_i,
    input   wire  [MST_AMT-1                    : 0]  m_WLAST_i,
    input   wire  [MST_AMT-1                    : 0]  m_WVALID_i,
    output  wire  [MST_AMT-1                    : 0]  m_WREADY_o,

    // -- 写响应通道 (B)
    output  wire  [W_SID*MST_AMT-1              : 0]  m_BID_o,
    output  wire  [TRANS_WR_RESP_W*MST_AMT-1    : 0]  m_BRESP_o,
    output  wire  [MST_AMT-1                    : 0]  m_BVALID_o,
    input   wire  [MST_AMT-1                    : 0]  m_BREADY_i,

    // -- 读地址通道 (AR)
    input   wire  [W_MID*MST_AMT-1              : 0]  m_ARID_i,
    input   wire  [ADDR_WIDTH*MST_AMT-1         : 0]  m_ARADDR_i,
    input   wire  [TRANS_BURST_W*MST_AMT-1      : 0]  m_ARBURST_i,
    input   wire  [TRANS_DATA_LEN_W*MST_AMT-1   : 0]  m_ARLEN_i,
    input   wire  [TRANS_DATA_SIZE_W*MST_AMT-1  : 0]  m_ARSIZE_i,
    input   wire  [MST_AMT-1                    : 0]  m_ARVALID_i,
    output  wire  [MST_AMT-1                    : 0]  m_ARREADY_o,

    // -- 读数据通道 (R)
    output  wire  [W_SID*MST_AMT-1              : 0]  m_RID_o,
    output  wire  [DATA_WIDTH*MST_AMT-1         : 0]  m_RDATA_o,
    output  wire  [TRANS_WR_RESP_W*MST_AMT-1    : 0]  m_RRESP_o,
    output  wire  [MST_AMT-1                    : 0]  m_RLAST_o,
    output  wire  [MST_AMT-1                    : 0]  m_RVALID_o,
    input   wire  [MST_AMT-1                    : 0]  m_RREADY_i,

    // -- 主设备侧 R SID 输出 (完整 W_SID，用于 reorder/sid_buffer)
    output  wire  [W_SID*MST_AMT-1              : 0]  m_RSID_o,

    // ========== 从设备侧接口 (扁平化) ==========
    // -- 写地址通道 (AW)
    output  wire  [W_SID*SLV_AMT-1              : 0]  s_AWID_o,
    output  wire  [ADDR_WIDTH*SLV_AMT-1         : 0]  s_AWADDR_o,
    output  wire  [TRANS_BURST_W*SLV_AMT-1      : 0]  s_AWBURST_o,
    output  wire  [TRANS_DATA_LEN_W*SLV_AMT-1   : 0]  s_AWLEN_o,
    output  wire  [TRANS_DATA_SIZE_W*SLV_AMT-1  : 0]  s_AWSIZE_o,
    output  wire  [SLV_AMT-1                    : 0]  s_AWVALID_o,
    input   wire  [SLV_AMT-1                    : 0]  s_AWREADY_i,

    // -- 写数据通道 (W)
    output  wire  [DATA_WIDTH*SLV_AMT-1         : 0]  s_WDATA_o,
    output  wire  [W_STRB*SLV_AMT-1             : 0]  s_WSTRB_o,
    output  wire  [SLV_AMT-1                    : 0]  s_WLAST_o,
    output  wire  [SLV_AMT-1                    : 0]  s_WVALID_o,
    input   wire  [SLV_AMT-1                    : 0]  s_WREADY_i,

    // -- 写响应通道 (B)
    input   wire  [W_SID*SLV_AMT-1              : 0]  s_BID_i,
    input   wire  [TRANS_WR_RESP_W*SLV_AMT-1    : 0]  s_BRESP_i,
    input   wire  [SLV_AMT-1                    : 0]  s_BVALID_i,
    output  wire  [SLV_AMT-1                    : 0]  s_BREADY_o,

    // -- 读地址通道 (AR)
    output  wire  [W_SID*SLV_AMT-1              : 0]  s_ARID_o,
    output  wire  [ADDR_WIDTH*SLV_AMT-1         : 0]  s_ARADDR_o,
    output  wire  [TRANS_BURST_W*SLV_AMT-1      : 0]  s_ARBURST_o,
    output  wire  [TRANS_DATA_LEN_W*SLV_AMT-1   : 0]  s_ARLEN_o,
    output  wire  [TRANS_DATA_SIZE_W*SLV_AMT-1  : 0]  s_ARSIZE_o,
    output  wire  [SLV_AMT-1                    : 0]  s_ARVALID_o,
    input   wire  [SLV_AMT-1                    : 0]  s_ARREADY_i,

    // -- 读数据通道 (R)
    input   wire  [W_SID*SLV_AMT-1              : 0]  s_RID_i,
    input   wire  [DATA_WIDTH*SLV_AMT-1         : 0]  s_RDATA_i,
    input   wire  [TRANS_WR_RESP_W*SLV_AMT-1    : 0]  s_RRESP_i,
    input   wire  [SLV_AMT-1                    : 0]  s_RLAST_i,
    input   wire  [SLV_AMT-1                    : 0]  s_RVALID_i,
    output  wire  [SLV_AMT-1                    : 0]  s_RREADY_o,

    // ========== 控制 / 状态端口 ==========
    input   wire                      arbiter_type,           // 0: 轮询 (Round-Robin), 1: 固定优先级 (Fixed-Priority)

    // 可选: 从设备使能掩码
    input   wire  [SLV_AMT-1          : 0]  slv_en_i
);

//=============================================================================
// 本地参数与内部信号声明
//=============================================================================
localparam ADDR_DECODE_W = SLV_ID_MSB_IDX - SLV_ID_LSB_IDX + 1;

// ========== 扁平化内部数组 (主设备侧) ==========
wire [W_MID-1:0]             m_awid      [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]        m_awaddr    [0:MST_AMT-1];
wire [TRANS_BURST_W-1:0]     m_awburst   [0:MST_AMT-1];
wire [TRANS_DATA_LEN_W-1:0]  m_awlen     [0:MST_AMT-1];
wire [TRANS_DATA_SIZE_W-1:0] m_awsize    [0:MST_AMT-1];
wire                         m_awvalid   [0:MST_AMT-1];
wire                         m_awready   [0:MST_AMT-1];

wire [DATA_WIDTH-1:0]        m_wdata     [0:MST_AMT-1];
wire [W_STRB-1:0]            m_wstrb     [0:MST_AMT-1];
wire                         m_wlast     [0:MST_AMT-1];
wire                         m_wvalid    [0:MST_AMT-1];
wire                         m_wready    [0:MST_AMT-1];

wire [W_SID-1:0]              m_bid       [0:MST_AMT-1];
wire [TRANS_WR_RESP_W-1:0]   m_bresp     [0:MST_AMT-1];
wire                         m_bvalid    [0:MST_AMT-1];
wire                         m_bready    [0:MST_AMT-1];

wire [W_MID-1:0]             m_arid      [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]        m_araddr    [0:MST_AMT-1];
wire [TRANS_BURST_W-1:0]     m_arburst   [0:MST_AMT-1];
wire [TRANS_DATA_LEN_W-1:0]  m_arlen     [0:MST_AMT-1];
wire [TRANS_DATA_SIZE_W-1:0] m_arsize    [0:MST_AMT-1];
wire                         m_arvalid   [0:MST_AMT-1];
wire                         m_arready   [0:MST_AMT-1];

wire [W_SID-1:0]             m_rid       [0:MST_AMT-1];
wire [DATA_WIDTH-1:0]        m_rdata     [0:MST_AMT-1];
wire [TRANS_WR_RESP_W-1:0]   m_rresp     [0:MST_AMT-1];
wire                         m_rlast     [0:MST_AMT-1];
wire                         m_rvalid    [0:MST_AMT-1];
wire                         m_rready    [0:MST_AMT-1];

wire [W_SID-1:0]             m_rsid      [0:MST_AMT-1];   // 来自 S2M 的完整从设备侧 RID

// ========== 扁平化内部数组 (从设备侧) ==========
wire [W_SID-1:0]             s_awid      [0:SLV_AMT-1];
wire [ADDR_WIDTH-1:0]        s_awaddr    [0:SLV_AMT-1];
wire [TRANS_BURST_W-1:0]     s_awburst   [0:SLV_AMT-1];
wire [TRANS_DATA_LEN_W-1:0]  s_awlen     [0:SLV_AMT-1];
wire [TRANS_DATA_SIZE_W-1:0] s_awsize    [0:SLV_AMT-1];
wire                         s_awvalid   [0:SLV_AMT-1];
wire                         s_awready   [0:SLV_AMT-1];

wire [DATA_WIDTH-1:0]        s_wdata     [0:SLV_AMT-1];
wire [W_STRB-1:0]            s_wstrb     [0:SLV_AMT-1];
wire                         s_wlast     [0:SLV_AMT-1];
wire                         s_wvalid    [0:SLV_AMT-1];
wire                         s_wready    [0:SLV_AMT-1];

wire [W_SID-1:0]             s_bid       [0:SLV_AMT-1];
wire [TRANS_WR_RESP_W-1:0]   s_bresp     [0:SLV_AMT-1];
wire                         s_bvalid    [0:SLV_AMT-1];
wire                         s_bready    [0:SLV_AMT-1];

wire [W_SID-1:0]             s_arid      [0:SLV_AMT-1];
wire [ADDR_WIDTH-1:0]        s_araddr    [0:SLV_AMT-1];
wire [TRANS_BURST_W-1:0]     s_arburst   [0:SLV_AMT-1];
wire [TRANS_DATA_LEN_W-1:0]  s_arlen     [0:SLV_AMT-1];
wire [TRANS_DATA_SIZE_W-1:0] s_arsize    [0:SLV_AMT-1];
wire                         s_arvalid   [0:SLV_AMT-1];
wire                         s_arready   [0:SLV_AMT-1];

wire [W_SID-1:0]             s_rid       [0:SLV_AMT-1];
wire [DATA_WIDTH-1:0]        s_rdata     [0:SLV_AMT-1];
wire [TRANS_WR_RESP_W-1:0]   s_rresp     [0:SLV_AMT-1];
wire                         s_rlast     [0:SLV_AMT-1];
wire                         s_rvalid    [0:SLV_AMT-1];
wire                         s_rready    [0:SLV_AMT-1];

// ========== M2S 互连信号 ==========
// AWSELECT/ARSELECT: 每从设备的地址译码结果 [MST_AMT-1:0]
wire [MST_AMT-1:0]           awselect_out [0:SLV_AMT-1];
wire [MST_AMT-1:0]           arselect_out [0:SLV_AMT-1];

// 默认从设备 M2S 的 AWSELECT_IN = OR(所有非默认从设备的 awselect_out)
wire [MST_AMT-1:0]           awselect_or_nondefault;
wire [MST_AMT-1:0]           arselect_or_nondefault;

// Ready 聚合: 收集每个 (从设备, 主设备) 对的 M2S ready，然后按主设备 OR 归约
wire                         m_awready_m2s [0:SLV_AMT-1][0:MST_AMT-1];
wire                         m_wready_m2s  [0:SLV_AMT-1][0:MST_AMT-1];
wire                         m_arready_m2s [0:SLV_AMT-1][0:MST_AMT-1];

// ========== S2M 互连信号 ==========
// 收集每个 (主设备, 从设备) 对的 S2M ready，然后按从设备 OR 归约
wire                         s_bready_s2m [0:MST_AMT-1][0:SLV_AMT-1];
wire                         s_rready_s2m [0:MST_AMT-1][0:SLV_AMT-1];

//=============================================================================
// 端口扁平化: 主设备侧 (解包)
//=============================================================================
genvar m, s;
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : UNPACK_MASTER
        // AW 通道
        assign m_awid[m]     = m_AWID_i[W_MID*(m+1)-1 -: W_MID];
        assign m_awaddr[m]   = m_AWADDR_i[ADDR_WIDTH*(m+1)-1 -: ADDR_WIDTH];
        assign m_awburst[m]  = m_AWBURST_i[TRANS_BURST_W*(m+1)-1 -: TRANS_BURST_W];
        assign m_awlen[m]    = m_AWLEN_i[TRANS_DATA_LEN_W*(m+1)-1 -: TRANS_DATA_LEN_W];
        assign m_awsize[m]   = m_AWSIZE_i[TRANS_DATA_SIZE_W*(m+1)-1 -: TRANS_DATA_SIZE_W];
        assign m_awvalid[m]  = m_AWVALID_i[m];
        assign m_AWREADY_o[m] = m_awready[m];
        
        // W 通道
        assign m_wdata[m]    = m_WDATA_i[DATA_WIDTH*(m+1)-1 -: DATA_WIDTH];
        assign m_wstrb[m]    = m_WSTRB_i[W_STRB*(m+1)-1 -: W_STRB];
        assign m_wlast[m]    = m_WLAST_i[m];
        assign m_wvalid[m]   = m_WVALID_i[m];
        assign m_WREADY_o[m] = m_wready[m];
        
        // B 通道
        assign m_BID_o[W_SID*(m+1)-1 -: W_SID] = m_bid[m];
        assign m_BRESP_o[TRANS_WR_RESP_W*(m+1)-1 -: TRANS_WR_RESP_W] = m_bresp[m];
        assign m_BVALID_o[m] = m_bvalid[m];
        assign m_bready[m]   = m_BREADY_i[m];
        
        // AR 通道
        assign m_arid[m]     = m_ARID_i[W_MID*(m+1)-1 -: W_MID];
        assign m_araddr[m]   = m_ARADDR_i[ADDR_WIDTH*(m+1)-1 -: ADDR_WIDTH];
        assign m_arburst[m]  = m_ARBURST_i[TRANS_BURST_W*(m+1)-1 -: TRANS_BURST_W];
        assign m_arlen[m]    = m_ARLEN_i[TRANS_DATA_LEN_W*(m+1)-1 -: TRANS_DATA_LEN_W];
        assign m_arsize[m]   = m_ARSIZE_i[TRANS_DATA_SIZE_W*(m+1)-1 -: TRANS_DATA_SIZE_W];
        assign m_arvalid[m]  = m_ARVALID_i[m];
        assign m_ARREADY_o[m] = m_arready[m];
        // TRACE
        always @(posedge AXI_CLK) begin
            if (m == 0 && m_ARREADY_o[m])
                $display("[TRACE] %0t XBAR M_ARREADY m=%0d", $time, m);
        end
        
        // R 通道
        assign m_rid[m]    = m_rsid[m];                // pass full W_SID, no stripping
        assign m_RSID_o[W_SID*(m+1)-1 -: W_SID] = m_rsid[m];
        assign m_RID_o[W_SID*(m+1)-1 -: W_SID] = m_rid[m];
        assign m_RDATA_o[DATA_WIDTH*(m+1)-1 -: DATA_WIDTH] = m_rdata[m];
        assign m_RRESP_o[TRANS_WR_RESP_W*(m+1)-1 -: TRANS_WR_RESP_W] = m_rresp[m];
        assign m_RLAST_o[m] = m_rlast[m];
        assign m_RVALID_o[m] = m_rvalid[m];
        assign m_rready[m]   = m_RREADY_i[m];
    end
endgenerate

//=============================================================================
// 端口扁平化: 从设备侧 (打包)
// 默认从设备 (SLV_DEFAULT_MASK[s]==1) 的外部端口被 tie-off,
// 其 s_awready / s_wready / s_arready / B / R 信号由 INST_M2S 中的
// axi_default_slave 驱动
//=============================================================================
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : PACK_SLAVE
        if (!SLV_DEFAULT_MASK[s]) begin : NORMAL_SLV
            // AW 通道
            assign s_AWID_o[W_SID*(s+1)-1 -: W_SID]       = s_awid[s];
            assign s_AWADDR_o[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH] = s_awaddr[s];
            assign s_AWBURST_o[TRANS_BURST_W*(s+1)-1 -: TRANS_BURST_W] = s_awburst[s];
            assign s_AWLEN_o[TRANS_DATA_LEN_W*(s+1)-1 -: TRANS_DATA_LEN_W] = s_awlen[s];
            assign s_AWSIZE_o[TRANS_DATA_SIZE_W*(s+1)-1 -: TRANS_DATA_SIZE_W] = s_awsize[s];
            assign s_AWVALID_o[s] = s_awvalid[s];
            assign s_awready[s]   = s_AWREADY_i[s];

            // W 通道
            assign s_WDATA_o[DATA_WIDTH*(s+1)-1 -: DATA_WIDTH] = s_wdata[s];
            assign s_WSTRB_o[W_STRB*(s+1)-1 -: W_STRB] = s_wstrb[s];
            assign s_WLAST_o[s] = s_wlast[s];
            assign s_WVALID_o[s] = s_wvalid[s];
            assign s_wready[s]   = s_WREADY_i[s];

            // B 通道
            assign s_bid[s]      = s_BID_i[W_SID*(s+1)-1 -: W_SID];
            assign s_bresp[s]    = s_BRESP_i[TRANS_WR_RESP_W*(s+1)-1 -: TRANS_WR_RESP_W];
            assign s_bvalid[s]   = s_BVALID_i[s];
            assign s_BREADY_o[s] = s_bready[s];

            // AR 通道
            assign s_ARID_o[W_SID*(s+1)-1 -: W_SID]       = s_arid[s];
        // TRACE
        always @(posedge AXI_CLK) begin
            if (s_ARVALID_o[s])
                $display("[TRACE] %0t XBAR S_ARVALID s=%0d addr=0x%08h", $time, s, s_ARADDR_o[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH]);
        end
            assign s_ARADDR_o[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH] = s_araddr[s];
            assign s_ARBURST_o[TRANS_BURST_W*(s+1)-1 -: TRANS_BURST_W] = s_arburst[s];
            assign s_ARLEN_o[TRANS_DATA_LEN_W*(s+1)-1 -: TRANS_DATA_LEN_W] = s_arlen[s];
            assign s_ARSIZE_o[TRANS_DATA_SIZE_W*(s+1)-1 -: TRANS_DATA_SIZE_W] = s_arsize[s];
            assign s_ARVALID_o[s] = s_arvalid[s];
            assign s_arready[s]   = s_ARREADY_i[s];

            // R 通道
            assign s_rid[s]      = s_RID_i[W_SID*(s+1)-1 -: W_SID];
            assign s_rdata[s]    = s_RDATA_i[DATA_WIDTH*(s+1)-1 -: DATA_WIDTH];
            assign s_rresp[s]    = s_RRESP_i[TRANS_WR_RESP_W*(s+1)-1 -: TRANS_WR_RESP_W];
            assign s_rlast[s]    = s_RLAST_i[s];
            assign s_rvalid[s]   = s_RVALID_i[s];
            assign s_RREADY_o[s] = s_rready[s];
        end else begin : DEFAULT_SLV
            // 默认从设备: 外部 AW/W/AR 端口 tie-off
            assign s_AWID_o[W_SID*(s+1)-1 -: W_SID]       = {W_SID{1'b0}};
            assign s_AWADDR_o[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH] = {ADDR_WIDTH{1'b0}};
            assign s_AWBURST_o[TRANS_BURST_W*(s+1)-1 -: TRANS_BURST_W] = {TRANS_BURST_W{1'b0}};
            assign s_AWLEN_o[TRANS_DATA_LEN_W*(s+1)-1 -: TRANS_DATA_LEN_W] = {TRANS_DATA_LEN_W{1'b0}};
            assign s_AWSIZE_o[TRANS_DATA_SIZE_W*(s+1)-1 -: TRANS_DATA_SIZE_W] = {TRANS_DATA_SIZE_W{1'b0}};
            assign s_AWVALID_o[s] = 1'b0;
            // s_awready[s] 由 INST_M2S 中的 axi_default_slave 驱动

            assign s_WDATA_o[DATA_WIDTH*(s+1)-1 -: DATA_WIDTH] = {DATA_WIDTH{1'b0}};
            assign s_WSTRB_o[W_STRB*(s+1)-1 -: W_STRB] = {W_STRB{1'b0}};
            assign s_WLAST_o[s] = 1'b0;
            assign s_WVALID_o[s] = 1'b0;
            // s_wready[s] 由 INST_M2S 中的 axi_default_slave 驱动

            // B 通道输入由 INST_M2S 中的 axi_default_slave 驱动
            assign s_BREADY_o[s] = 1'b0;

            assign s_ARID_o[W_SID*(s+1)-1 -: W_SID]       = {W_SID{1'b0}};
            assign s_ARADDR_o[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH] = {ADDR_WIDTH{1'b0}};
            assign s_ARBURST_o[TRANS_BURST_W*(s+1)-1 -: TRANS_BURST_W] = {TRANS_BURST_W{1'b0}};
            assign s_ARLEN_o[TRANS_DATA_LEN_W*(s+1)-1 -: TRANS_DATA_LEN_W] = {TRANS_DATA_LEN_W{1'b0}};
            assign s_ARSIZE_o[TRANS_DATA_SIZE_W*(s+1)-1 -: TRANS_DATA_SIZE_W] = {TRANS_DATA_SIZE_W{1'b0}};
            assign s_ARVALID_o[s] = 1'b0;
            // s_arready[s] 由 INST_M2S 中的 axi_default_slave 驱动

            // R 通道输入由 INST_M2S 中的 axi_default_slave 驱动
            assign s_RREADY_o[s] = 1'b0;
        end
    end
endgenerate

//=============================================================================
// 默认从设备 AWSELECT_IN/ARSELECT_IN 归约
// awselect_or_nondefault = bitwise OR(所有非默认从设备的 awselect_out)
// 默认从设备 M2S 内部做 ~AWSELECT_IN, 实现 "未被任何其他从设备选中" 的译码
//=============================================================================
generate
    if (|SLV_DEFAULT_MASK) begin : DEFAULT_OR
        wire [MST_AMT-1:0] aw_chain [0:SLV_AMT];
        wire [MST_AMT-1:0] ar_chain [0:SLV_AMT];
        assign aw_chain[0] = {MST_AMT{1'b0}};
        assign ar_chain[0] = {MST_AMT{1'b0}};
        for (s = 0; s < SLV_AMT; s = s + 1) begin
            if (!SLV_DEFAULT_MASK[s]) begin
                assign aw_chain[s+1] = aw_chain[s] | awselect_out[s];
                assign ar_chain[s+1] = ar_chain[s] | arselect_out[s];
            end else begin
                assign aw_chain[s+1] = aw_chain[s];
                assign ar_chain[s+1] = ar_chain[s];
            end
        end
        assign awselect_or_nondefault = aw_chain[SLV_AMT];
        assign arselect_or_nondefault = ar_chain[SLV_AMT];
    end else begin
        assign awselect_or_nondefault = {MST_AMT{1'b0}};
        assign arselect_or_nondefault = {MST_AMT{1'b0}};
    end
endgenerate

//=============================================================================
// Ready 聚合: 主设备侧 (M2S OR 归约)
// 对于每个主设备 m，只要存在任意一个从设备 s 的 M2S 返回 ready，则该主设备 ready
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : READY_AGG_M
        wire [SLV_AMT-1:0] aw_rdy_vec, w_rdy_vec, ar_rdy_vec;
        for(s = 0; s < SLV_AMT; s = s + 1) begin : AGG_SLV
            assign aw_rdy_vec[s] = m_awready_m2s[s][m];
            assign w_rdy_vec[s]  = m_wready_m2s[s][m];
        // TRACE
        always @(posedge AXI_CLK) begin
            if (m_arready[0])
                $display("[TRACE] %0t XBAR m_arready[0]=1 ar_rdy_vec=%b", $time, ar_rdy_vec);
        end
            assign ar_rdy_vec[s] = m_arready_m2s[s][m];
        end
        assign m_awready[m] = |aw_rdy_vec;
        assign m_wready[m]  = |w_rdy_vec;
        assign m_arready[m] = |ar_rdy_vec;
    end
endgenerate

//=============================================================================
// Ready 聚合: 从设备侧 (S2M OR 归约)
// 对于每个从设备 s，只要存在任意一个主设备 m 的 S2M 返回 ready，则该从设备 ready
//=============================================================================
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : READY_AGG_S
        wire [MST_AMT-1:0] b_rdy_vec, r_rdy_vec;
        for(m = 0; m < MST_AMT; m = m + 1) begin : AGG_MST
            assign b_rdy_vec[m] = s_bready_s2m[m][s];
            assign r_rdy_vec[m] = s_rready_s2m[m][s];
        end
        assign s_bready[s] = |b_rdy_vec;
        assign s_rready[s] = |r_rdy_vec;
    end
endgenerate

//=============================================================================
// M2S 模块实例化: 每个从设备一个 (axi_m2s_m_amt)
//=============================================================================
wire [MST_AMT-1:0] m_w_busy;                          // per-master W busy (aggregated)
wire [MST_AMT-1:0] m2s_w_busy_arr [0:SLV_AMT-1];      // per-slave W busy vectors
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : INST_M2S
        // 构造打包的主设备数组以连接 M2S 模块
        wire [W_MID*MST_AMT-1:0] m_awid_packed;
        wire [ADDR_WIDTH*MST_AMT-1:0]     m_awaddr_packed;
        wire [TRANS_DATA_LEN_W*MST_AMT-1:0] m_awlen_packed;
        wire [TRANS_DATA_SIZE_W*MST_AMT-1:0] m_awsize_packed;
        wire [TRANS_BURST_W*MST_AMT-1:0] m_awburst_packed;
        wire [MST_AMT-1:0] m_awvalid_packed;
        wire [MST_AMT-1:0] m_awready_packed;
        
        wire [DATA_WIDTH*MST_AMT-1:0] m_wdata_packed;
        wire [W_STRB*MST_AMT-1:0]     m_wstrb_packed;
        wire [MST_AMT-1:0] m_wlast_packed;
        wire [MST_AMT-1:0] m_wvalid_packed;
        wire [MST_AMT-1:0] m_wready_packed;
        
        wire [W_MID*MST_AMT-1:0] m_arid_packed;
        wire [ADDR_WIDTH*MST_AMT-1:0]     m_araddr_packed;
        wire [TRANS_DATA_LEN_W*MST_AMT-1:0] m_arlen_packed;
        wire [TRANS_DATA_SIZE_W*MST_AMT-1:0] m_arsize_packed;
        wire [TRANS_BURST_W*MST_AMT-1:0] m_arburst_packed;
        wire [MST_AMT-1:0] m_arvalid_packed;
        wire [MST_AMT-1:0] m_arready_packed;

        // W routing control: signals from M2S (declared before use in PACK_M2S)
        wire [MST_AMT-1:0] m2s_w_accept;  // per-master: this M2S is receiving W beats
        wire [MST_AMT-1:0] m2s_w_busy;    // per-master: this M2S has pending W

        // 将内部解包数组重新打包为扁平化向量
        for(m = 0; m < MST_AMT; m = m + 1) begin : PACK_M2S
            assign m_awid_packed[W_MID*m +: W_MID] = m_awid[m];
            assign m_awaddr_packed[ADDR_WIDTH*m +: ADDR_WIDTH] = m_awaddr[m];
            assign m_awlen_packed[TRANS_DATA_LEN_W*m +: TRANS_DATA_LEN_W] = m_awlen[m];
            assign m_awsize_packed[TRANS_DATA_SIZE_W*m +: TRANS_DATA_SIZE_W] = m_awsize[m];
            assign m_awburst_packed[TRANS_BURST_W*m +: TRANS_BURST_W] = m_awburst[m];
            // AW: passthrough (W gate handles routing; AW-order FIFO handles same-slave pipeline)
            assign m_awvalid_packed[m] = m_awvalid[m];
            assign m_wdata_packed[DATA_WIDTH*m +: DATA_WIDTH] = m_wdata[m];
            assign m_wstrb_packed[W_STRB*m +: W_STRB] = m_wstrb[m];
            assign m_wlast_packed[m] = m_wlast[m];
            // W: only accepted by the slave currently processing this master's W
            assign m_wvalid_packed[m] = m_wvalid[m] && m2s_w_accept[m];
            assign m_arid_packed[W_MID*m +: W_MID] = m_arid[m];
            assign m_araddr_packed[ADDR_WIDTH*m +: ADDR_WIDTH] = m_araddr[m];
            assign m_arlen_packed[TRANS_DATA_LEN_W*m +: TRANS_DATA_LEN_W] = m_arlen[m];
            assign m_arsize_packed[TRANS_DATA_SIZE_W*m +: TRANS_DATA_SIZE_W] = m_arsize[m];
            assign m_arburst_packed[TRANS_BURST_W*m +: TRANS_BURST_W] = m_arburst[m];
            assign m_arvalid_packed[m] = m_arvalid[m];
            
            // Ready 输出: 将 M2S 实例 s 对主设备 m 的 ready 写入 2D 数组
            assign m_awready_m2s[s][m] = m_awready_packed[m];
            assign m_wready_m2s[s][m]  = m_wready_packed[m];
            assign m_arready_m2s[s][m] = m_arready_packed[m];
        end

        // M2S 输出的中间信号 (所有从设备的 awselect_out/arselect_out 均由 M2S 驱动)
        wire [MST_AMT-1:0] m2s_awsel, m2s_arsel;

        axi_m2s_m_amt #(
            .ADDR_BASE(SLV_ADDR_BASE[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH]),
            .ADDR_LENGTH(SLV_ADDR_LEN[8*(s+1)-1 -: 8]),
            .M_ID_W(MST_ID_W),
            .W_ID(W_MID),
            .W_ADDR(ADDR_WIDTH),
            .W_DATA(DATA_WIDTH),
            .W_STRB(W_STRB),
            .W_SID(W_SID),
            .MST_AMT(MST_AMT),
            .OUTSTANDING_AMT(OUTSTANDING_AMT),
            .ALEN_W(TRANS_DATA_LEN_W),
            .ASIZE_W(TRANS_DATA_SIZE_W),
            .ABURST_W(TRANS_BURST_W),
            .SLAVE_DEFAULT(SLV_DEFAULT_MASK[s])
        ) u_axi_m2s (
            .AXI_RSTn(AXI_RSTn),
            .AXI_CLK(AXI_CLK),

            .M_AWID(m_awid_packed),
            .M_AWADDR(m_awaddr_packed),
            .M_AWLEN(m_awlen_packed),
            .M_AWSIZE(m_awsize_packed),
            .M_AWBURST(m_awburst_packed),
            .M_AWVALID(m_awvalid_packed),
            .M_AWREADY(m_awready_packed),

            .M_WDATA(m_wdata_packed),
            .M_WSTRB(m_wstrb_packed),
            .M_WLAST(m_wlast_packed),
            .M_WVALID(m_wvalid_packed),
            .M_WREADY(m_wready_packed),

            .M_ARID(m_arid_packed),
            .M_ARADDR(m_araddr_packed),
            .M_ARLEN(m_arlen_packed),
            .M_ARSIZE(m_arsize_packed),
            .M_ARBURST(m_arburst_packed),
            .M_ARVALID(m_arvalid_packed),
            .M_ARREADY(m_arready_packed),

            .S_AWID(s_awid[s]),
            .S_AWADDR(s_awaddr[s]),
            .S_AWLEN(s_awlen[s]),
            .S_AWSIZE(s_awsize[s]),
            .S_AWBURST(s_awburst[s]),
            .S_AWVALID(s_awvalid[s]),
            .S_AWREADY(s_awready[s]),

            .S_WDATA(s_wdata[s]),
            .S_WSTRB(s_wstrb[s]),
            .S_WLAST(s_wlast[s]),
            .S_WVALID(s_wvalid[s]),
            .S_WREADY(s_wready[s]),

            .S_ARID(s_arid[s]),
            .S_ARADDR(s_araddr[s]),
            .S_ARLEN(s_arlen[s]),
            .S_ARSIZE(s_arsize[s]),
            .S_ARBURST(s_arburst[s]),
            .S_ARVALID(s_arvalid[s]),
            .S_ARREADY(s_arready[s]),

            .AWSELECT_OUT(m2s_awsel),
            .ARSELECT_OUT(m2s_arsel),
            .AWSELECT_IN(awselect_or_nondefault),
            .ARSELECT_IN(arselect_or_nondefault),
            .arbiter_type(arbiter_type),
            .slv_en(slv_en_i[s]),

            .W_ACCEPT(m2s_w_accept),
            .W_BUSY(m2s_w_busy)
        );

        // Export per-slave W busy for crossbar-level aggregation
        assign m2s_w_busy_arr[s] = m2s_w_busy;

        // 默认从设备: 例化 axi_default_slave 驱动 ready / B / R 信号
        if (SLV_DEFAULT_MASK[s]) begin : GEN_DEFAULT_SLV
            axi_default_slave #(
                .W_CID(SLV_ID_W),
                .W_ID(W_MID),
                .W_ADDR(ADDR_WIDTH),
                .W_DATA(DATA_WIDTH),
                .W_STRB(W_STRB),
                .W_SID(W_SID)
            ) u_axi_default_slave (
                .AXI_RSTn(AXI_RSTn),
                .AXI_CLK (AXI_CLK),

                .AWID    (s_awid[s]),
                .AWADDR  (s_awaddr[s]),
                .AWLEN   (s_awlen[s]),
                .AWSIZE  (s_awsize[s]),
                .AWBURST (s_awburst[s]),
                .AWVALID (s_awvalid[s]),
                .AWREADY (s_awready[s]),

                .WDATA   (s_wdata[s]),
                .WSTRB   (s_wstrb[s]),
                .WLAST   (s_wlast[s]),
                .WVALID  (s_wvalid[s]),
                .WREADY  (s_wready[s]),

                .BID     (s_bid[s]),
                .BRESP   (s_bresp[s]),
                .BVALID  (s_bvalid[s]),
                .BREADY  (s_bready[s]),

                .ARID    (s_arid[s]),
                .ARADDR  (s_araddr[s]),
                .ARLEN   (s_arlen[s]),
                .ARSIZE  (s_arsize[s]),
                .ARBURST (s_arburst[s]),
                .ARVALID (s_arvalid[s]),
                .ARREADY (s_arready[s]),

                .RID     (s_rid[s]),
                .RDATA   (s_rdata[s]),
                .RRESP   (s_rresp[s]),
                .RLAST   (s_rlast[s]),
                .RVALID  (s_rvalid[s]),
                .RREADY  (s_rready[s])
            );
        end

        assign awselect_out[s] = m2s_awsel;
        assign arselect_out[s] = m2s_arsel;
    end
endgenerate

//=============================================================================
// Per-master W busy aggregation: any slave has pending W for this master
//=============================================================================
genvar mb;
generate
    for (mb = 0; mb < MST_AMT; mb = mb + 1) begin : GEN_M_W_BUSY
        wire [SLV_AMT-1:0] busy_or;
        for (s = 0; s < SLV_AMT; s = s + 1) begin : BUSY_SLV
            assign busy_or[s] = m2s_w_busy_arr[s][mb];
        end
        assign m_w_busy[mb] = |busy_or;
    end
endgenerate

//=============================================================================
// S2M 模块实例化: 每个主设备一个 (axi_s2m_s_amt)
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : INST_S2M
        // 构造打包的从设备数组以连接 S2M 模块
        wire [W_SID*SLV_AMT-1:0] s_bid_packed;
        wire [TRANS_WR_RESP_W*SLV_AMT-1:0] s_bresp_packed;
        wire [SLV_AMT-1:0] s_bvalid_packed;
        wire [SLV_AMT-1:0] s_bready_packed;
        
        wire [W_SID*SLV_AMT-1:0] s_rid_packed;
        wire [DATA_WIDTH*SLV_AMT-1:0] s_rdata_packed;
        wire [TRANS_WR_RESP_W*SLV_AMT-1:0] s_rresp_packed;
        wire [SLV_AMT-1:0] s_rlast_packed;
        wire [SLV_AMT-1:0] s_rvalid_packed;
        wire [SLV_AMT-1:0] s_rready_packed;
        
        for(s = 0; s < SLV_AMT; s = s + 1) begin : PACK_S2M
            assign s_bid_packed[W_SID*s +: W_SID] = s_bid[s];
            assign s_bresp_packed[TRANS_WR_RESP_W*s +: TRANS_WR_RESP_W] = s_bresp[s];
            assign s_bvalid_packed[s] = s_bvalid[s];
            assign s_rid_packed[W_SID*s +: W_SID] = s_rid[s];
            assign s_rdata_packed[DATA_WIDTH*s +: DATA_WIDTH] = s_rdata[s];
            assign s_rresp_packed[TRANS_WR_RESP_W*s +: TRANS_WR_RESP_W] = s_rresp[s];
            assign s_rlast_packed[s] = s_rlast[s];
            assign s_rvalid_packed[s] = s_rvalid[s];
            
            assign s_bready_s2m[m][s] = s_bready_packed[s];
            assign s_rready_s2m[m][s] = s_rready_packed[s];
        end

        axi_s2m_s_amt #(
            .MASTER_ID(m),
            .W_CID(MST_ID_W),
            .W_ID(W_MID),
            .W_ADDR(ADDR_WIDTH),
            .W_DATA(DATA_WIDTH),
            .W_STRB(W_STRB),
            .W_SID(W_SID),
            .SLV_AMT(SLV_AMT),
            .MST_ID_FIELD_MSB(MST_ID_W + W_MID - 1),
            .MST_ID_FIELD_LSB(W_MID)
        ) u_axi_s2m (
            .AXI_RSTn(AXI_RSTn),
            .AXI_CLK(AXI_CLK),
            
            .M_BID(m_bid[m]),
            .M_BRESP(m_bresp[m]),
            .M_BVALID(m_bvalid[m]),
            .M_BREADY(m_bready[m]),
            
            .M_RSID(m_rsid[m]),
            .M_RDATA(m_rdata[m]),
            .M_RRESP(m_rresp[m]),
            .M_RLAST(m_rlast[m]),
            .M_RVALID(m_rvalid[m]),
            .M_RREADY(m_rready[m]),
            
            .S_BID(s_bid_packed),
            .S_BRESP(s_bresp_packed),
            .S_BVALID(s_bvalid_packed),
            .S_BREADY(s_bready_packed),
            
            .S_RID(s_rid_packed),
            .S_RDATA(s_rdata_packed),
            .S_RRESP(s_rresp_packed),
            .S_RLAST(s_rlast_packed),
            .S_RVALID(s_rvalid_packed),
            .S_RREADY(s_rready_packed),
            
            .arbiter_type(arbiter_type)
        );
    end
endgenerate

endmodule