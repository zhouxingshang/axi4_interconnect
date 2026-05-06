module axi_crossbar
#(
    // ========== Interconnect Configuration ==========
    parameter MST_AMT               = 4,              // Number of masters
    parameter SLV_AMT               = 4,              // Number of slaves
    parameter OUTSTANDING_AMT       = 8,              // Max outstanding transactions per master
    
    // ========== Transaction Configuration ==========
    parameter TRANS_MST_ID_W        = 4,              // Master transaction ID width
    parameter TRANS_BURST_W         = 2,              // xBURST field width
    parameter TRANS_DATA_LEN_W      = 8,              // xLEN field width
    parameter TRANS_DATA_SIZE_W     = 3,              // xSIZE field width
    parameter TRANS_WR_RESP_W       = 2,              // xRESP field width
    parameter DATA_WIDTH            = 32,             // Data bus width
    parameter ADDR_WIDTH            = 32,             // Address bus width
    
    // ========== Derived Parameters ==========
    parameter MST_ID_W              = $clog2(MST_AMT),           // Width to encode master index
    parameter SLV_ID_W              = $clog2(SLV_AMT),           // Width to encode slave index
    parameter W_STRB                = DATA_WIDTH / 8,            // Byte strobe width
    parameter W_SID                 = SLV_ID_W + TRANS_MST_ID_W, // Slave-side ID width (SLV_ID + MST_ID + ORIG_ID)
    
    // ========== Address Mapping Configuration ==========
    // Default: Upper bits of address select slave
    parameter SLV_ID_MSB_IDX        = ADDR_WIDTH - 1,
    parameter SLV_ID_LSB_IDX        = ADDR_WIDTH - SLV_ID_W,
    
    // ========== Slave Address Base Array ==========
    // Format: {SLV_AMT{ADDR_WIDTH{1'b0}}} - user must override per slave
    parameter [0:(SLV_AMT*ADDR_WIDTH)-1] SLV_ADDR_BASE = {SLV_AMT{ADDR_WIDTH{1'b0}}},
    parameter [0:(SLV_AMT*8)-1]          SLV_ADDR_LEN  = {SLV_AMT{8'd12}}, // Default 12-bit decode
    
    // ========== Default Slave Configuration ==========
    parameter DEFAULT_SLV_IDX       = 0,              // Index of default slave (for unmapped accesses)
    parameter DEFAULT_SLV_EN        = 1'b1            // Enable default slave routing
)
(
    // ========== Global Signals ==========
    input   wire                      AXI_RSTn,
    input   wire                      AXI_CLK,
    
    // ========== Master Side Interface (FLATTENED) ==========
    // -- Write Address Channel (AW)
    input   wire  [TRANS_MST_ID_W*MST_AMT-1     : 0]  m_AWID_i,
    input   wire  [ADDR_WIDTH*MST_AMT-1         : 0]  m_AWADDR_i,
    input   wire  [TRANS_BURST_W*MST_AMT-1      : 0]  m_AWBURST_i,
    input   wire  [TRANS_DATA_LEN_W*MST_AMT-1   : 0]  m_AWLEN_i,
    input   wire  [TRANS_DATA_SIZE_W*MST_AMT-1  : 0]  m_AWSIZE_i,
    input   wire  [MST_AMT-1                    : 0]  m_AWVALID_i,
    output  wire  [MST_AMT-1                    : 0]  m_AWREADY_o,
    
    // -- Write Data Channel (W)
    input   wire  [DATA_WIDTH*MST_AMT-1         : 0]  m_WDATA_i,
    input   wire  [W_STRB*MST_AMT-1             : 0]  m_WSTRB_i,
    input   wire  [MST_AMT-1                    : 0]  m_WLAST_i,
    input   wire  [MST_AMT-1                    : 0]  m_WVALID_i,
    output  wire  [MST_AMT-1                    : 0]  m_WREADY_o,
    
    // -- Write Response Channel (B)
    output  wire  [TRANS_MST_ID_W*MST_AMT-1     : 0]  m_BID_o,
    output  wire  [TRANS_WR_RESP_W*MST_AMT-1    : 0]  m_BRESP_o,
    output  wire  [MST_AMT-1                    : 0]  m_BVALID_o,
    input   wire  [MST_AMT-1                    : 0]  m_BREADY_i,
    
    // -- Read Address Channel (AR)
    input   wire  [TRANS_MST_ID_W*MST_AMT-1     : 0]  m_ARID_i,
    input   wire  [ADDR_WIDTH*MST_AMT-1         : 0]  m_ARADDR_i,
    input   wire  [TRANS_BURST_W*MST_AMT-1      : 0]  m_ARBURST_i,
    input   wire  [TRANS_DATA_LEN_W*MST_AMT-1   : 0]  m_ARLEN_i,
    input   wire  [TRANS_DATA_SIZE_W*MST_AMT-1  : 0]  m_ARSIZE_i,
    input   wire  [MST_AMT-1                    : 0]  m_ARVALID_i,
    output  wire  [MST_AMT-1                    : 0]  m_ARREADY_o,
    
    // -- Read Data Channel (R)
    output  wire  [TRANS_MST_ID_W*MST_AMT-1     : 0]  m_RID_o,
    output  wire  [DATA_WIDTH*MST_AMT-1         : 0]  m_RDATA_o,
    output  wire  [TRANS_WR_RESP_W*MST_AMT-1    : 0]  m_RRESP_o,
    output  wire  [MST_AMT-1                    : 0]  m_RLAST_o,
    output  wire  [MST_AMT-1                    : 0]  m_RVALID_o,
    input   wire  [MST_AMT-1                    : 0]  m_RREADY_i,
    
    // ========== Slave Side Interface (FLATTENED) ==========
    // -- Write Address Channel (AW)
    output  wire  [W_SID*SLV_AMT-1              : 0]  s_AWID_o,
    output  wire  [ADDR_WIDTH*SLV_AMT-1         : 0]  s_AWADDR_o,
    output  wire  [TRANS_BURST_W*SLV_AMT-1      : 0]  s_AWBURST_o,
    output  wire  [TRANS_DATA_LEN_W*SLV_AMT-1   : 0]  s_AWLEN_o,
    output  wire  [TRANS_DATA_SIZE_W*SLV_AMT-1  : 0]  s_AWSIZE_o,
    output  wire  [SLV_AMT-1                    : 0]  s_AWVALID_o,
    input   wire  [SLV_AMT-1                    : 0]  s_AWREADY_i,
    
    // -- Write Data Channel (W)
    output  wire  [DATA_WIDTH*SLV_AMT-1         : 0]  s_WDATA_o,
    output  wire  [W_STRB*SLV_AMT-1             : 0]  s_WSTRB_o,
    output  wire  [SLV_AMT-1                    : 0]  s_WLAST_o,
    output  wire  [SLV_AMT-1                    : 0]  s_WVALID_o,
    input   wire  [SLV_AMT-1                    : 0]  s_WREADY_i,
    
    // -- Write Response Channel (B)
    input   wire  [W_SID*SLV_AMT-1              : 0]  s_BID_i,
    input   wire  [TRANS_WR_RESP_W*SLV_AMT-1    : 0]  s_BRESP_i,
    input   wire  [SLV_AMT-1                    : 0]  s_BVALID_i,
    output  wire  [SLV_AMT-1                    : 0]  s_BREADY_o,
    
    // -- Read Address Channel (AR)
    output  wire  [W_SID*SLV_AMT-1              : 0]  s_ARID_o,
    output  wire  [ADDR_WIDTH*SLV_AMT-1         : 0]  s_ARADDR_o,
    output  wire  [TRANS_BURST_W*SLV_AMT-1      : 0]  s_ARBURST_o,
    output  wire  [TRANS_DATA_LEN_W*SLV_AMT-1   : 0]  s_ARLEN_o,
    output  wire  [TRANS_DATA_SIZE_W*SLV_AMT-1  : 0]  s_ARSIZE_o,
    output  wire  [SLV_AMT-1                    : 0]  s_ARVALID_o,
    input   wire  [SLV_AMT-1                    : 0]  s_ARREADY_i,
    
    // -- Read Data Channel (R)
    input   wire  [W_SID*SLV_AMT-1              : 0]  s_RID_i,
    input   wire  [DATA_WIDTH*SLV_AMT-1         : 0]  s_RDATA_i,
    input   wire  [TRANS_WR_RESP_W*SLV_AMT-1    : 0]  s_RRESP_i,
    input   wire  [SLV_AMT-1                    : 0]  s_RLAST_i,
    input   wire  [SLV_AMT-1                    : 0]  s_RVALID_i,
    output  wire  [SLV_AMT-1                    : 0]  s_RREADY_o,
    
    // ========== Control/Status Ports ==========
    input   wire                      arbiter_type,           // 0: Round-Robin, 1: Fixed-Priority
    
    // Read reorder control: per-master grant vector [SLV_AMT-1:0]
    input   wire  [MST_AMT*SLV_AMT-1  : 0]  r_order_grant_i,
    
    // Optional: APB-style configuration (can be tied to constants)
    input   wire  [SLV_AMT-1          : 0]  slv_en_i            // Slave enable mask
);

//=============================================================================
// Local Parameters & Internal Signal Declarations
//=============================================================================
localparam ADDR_DECODE_W = SLV_ID_MSB_IDX - SLV_ID_LSB_IDX + 1;

// ========== Flattened Internal Arrays (Master Side) ==========
wire [TRANS_MST_ID_W-1:0]    m_awid      [0:MST_AMT-1];
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

wire [TRANS_MST_ID_W-1:0]    m_bid       [0:MST_AMT-1];
wire [TRANS_WR_RESP_W-1:0]   m_bresp     [0:MST_AMT-1];
wire                         m_bvalid    [0:MST_AMT-1];
wire                         m_bready    [0:MST_AMT-1];

wire [TRANS_MST_ID_W-1:0]    m_arid      [0:MST_AMT-1];
wire [ADDR_WIDTH-1:0]        m_araddr    [0:MST_AMT-1];
wire [TRANS_BURST_W-1:0]     m_arburst   [0:MST_AMT-1];
wire [TRANS_DATA_LEN_W-1:0]  m_arlen     [0:MST_AMT-1];
wire [TRANS_DATA_SIZE_W-1:0] m_arsize    [0:MST_AMT-1];
wire                         m_arvalid   [0:MST_AMT-1];
wire                         m_arready   [0:MST_AMT-1];

wire [TRANS_MST_ID_W-1:0]    m_rid       [0:MST_AMT-1];
wire [DATA_WIDTH-1:0]        m_rdata     [0:MST_AMT-1];
wire [TRANS_WR_RESP_W-1:0]   m_rresp     [0:MST_AMT-1];
wire                         m_rlast     [0:MST_AMT-1];
wire                         m_rvalid    [0:MST_AMT-1];
wire                         m_rready    [0:MST_AMT-1];

// ========== Flattened Internal Arrays (Slave Side) ==========
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

// ========== M2S Interconnect Signals ==========
// AWSELECT/ARSELECT: per-slave decode results [MST_AMT-1:0]
wire [MST_AMT-1:0]           awselect_out [0:SLV_AMT-1];
wire [MST_AMT-1:0]           arselect_out [0:SLV_AMT-1];
wire [MST_AMT-1:0]           awselect_in  [0:SLV_AMT-1];  // For default slave
wire [MST_AMT-1:0]           arselect_in  [0:SLV_AMT-1];

// Ready aggregation: M2S ready OR-reduced to master
wire                         m_awready_agg [0:MST_AMT-1];
wire                         m_wready_agg  [0:MST_AMT-1];
wire                         m_arready_agg [0:MST_AMT-1];

// ========== S2M Interconnect Signals ==========
// B/R ready aggregation: per-slave OR-reduced from all masters
wire [SLV_AMT-1:0]           s_bready_agg;
wire [SLV_AMT-1:0]           s_rready_agg;

// Read reorder grant: per-master slice [SLV_AMT-1:0]
wire [SLV_AMT-1:0]           r_order_grant_m [0:MST_AMT-1];

//=============================================================================
// Port Flattening: Master Side (Unpack)
//=============================================================================
genvar m, s;
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : UNPACK_MASTER
        // AW channel
        assign m_awid[m]     = m_AWID_i[TRANS_MST_ID_W*(m+1)-1 -: TRANS_MST_ID_W];
        assign m_awaddr[m]   = m_AWADDR_i[ADDR_WIDTH*(m+1)-1 -: ADDR_WIDTH];
        assign m_awburst[m]  = m_AWBURST_i[TRANS_BURST_W*(m+1)-1 -: TRANS_BURST_W];
        assign m_awlen[m]    = m_AWLEN_i[TRANS_DATA_LEN_W*(m+1)-1 -: TRANS_DATA_LEN_W];
        assign m_awsize[m]   = m_AWSIZE_i[TRANS_DATA_SIZE_W*(m+1)-1 -: TRANS_DATA_SIZE_W];
        assign m_awvalid[m]  = m_AWVALID_i[m];
        assign m_AWREADY_o[m] = m_awready[m];
        
        // W channel
        assign m_wdata[m]    = m_WDATA_i[DATA_WIDTH*(m+1)-1 -: DATA_WIDTH];
        assign m_wstrb[m]    = m_WSTRB_i[W_STRB*(m+1)-1 -: W_STRB];
        assign m_wlast[m]    = m_WLAST_i[m];
        assign m_wvalid[m]   = m_WVALID_i[m];
        assign m_WREADY_o[m] = m_wready[m];
        
        // B channel
        assign m_BID_o[TRANS_MST_ID_W*(m+1)-1 -: TRANS_MST_ID_W] = m_bid[m];
        assign m_BRESP_o[TRANS_WR_RESP_W*(m+1)-1 -: TRANS_WR_RESP_W] = m_bresp[m];
        assign m_BVALID_o[m] = m_bvalid[m];
        assign m_bready[m]   = m_BREADY_i[m];
        
        // AR channel
        assign m_arid[m]     = m_ARID_i[TRANS_MST_ID_W*(m+1)-1 -: TRANS_MST_ID_W];
        assign m_araddr[m]   = m_ARADDR_i[ADDR_WIDTH*(m+1)-1 -: ADDR_WIDTH];
        assign m_arburst[m]  = m_ARBURST_i[TRANS_BURST_W*(m+1)-1 -: TRANS_BURST_W];
        assign m_arlen[m]    = m_ARLEN_i[TRANS_DATA_LEN_W*(m+1)-1 -: TRANS_DATA_LEN_W];
        assign m_arsize[m]   = m_ARSIZE_i[TRANS_DATA_SIZE_W*(m+1)-1 -: TRANS_DATA_SIZE_W];
        assign m_arvalid[m]  = m_ARVALID_i[m];
        assign m_ARREADY_o[m] = m_arready[m];
        
        // R channel
        assign m_RID_o[TRANS_MST_ID_W*(m+1)-1 -: TRANS_MST_ID_W] = m_rid[m];
        assign m_RDATA_o[DATA_WIDTH*(m+1)-1 -: DATA_WIDTH] = m_rdata[m];
        assign m_RRESP_o[TRANS_WR_RESP_W*(m+1)-1 -: TRANS_WR_RESP_W] = m_rresp[m];
        assign m_RLAST_o[m] = m_rlast[m];
        assign m_RVALID_o[m] = m_rvalid[m];
        assign m_rready[m]   = m_RREADY_i[m];
    end
endgenerate

//=============================================================================
// Port Flattening: Slave Side (Pack)
//=============================================================================
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : PACK_SLAVE
        // AW channel
        assign s_AWID_o[W_SID*(s+1)-1 -: W_SID]       = s_awid[s];
        assign s_AWADDR_o[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH] = s_awaddr[s];
        assign s_AWBURST_o[TRANS_BURST_W*(s+1)-1 -: TRANS_BURST_W] = s_awburst[s];
        assign s_AWLEN_o[TRANS_DATA_LEN_W*(s+1)-1 -: TRANS_DATA_LEN_W] = s_awlen[s];
        assign s_AWSIZE_o[TRANS_DATA_SIZE_W*(s+1)-1 -: TRANS_DATA_SIZE_W] = s_awsize[s];
        assign s_AWVALID_o[s] = s_awvalid[s];
        assign s_awready[s]   = s_AWREADY_i[s];
        
        // W channel
        assign s_WDATA_o[DATA_WIDTH*(s+1)-1 -: DATA_WIDTH] = s_wdata[s];
        assign s_WSTRB_o[W_STRB*(s+1)-1 -: W_STRB] = s_wstrb[s];
        assign s_WLAST_o[s] = s_wlast[s];
        assign s_WVALID_o[s] = s_wvalid[s];
        assign s_wready[s]   = s_WREADY_i[s];
        
        // B channel
        assign s_bid[s]      = s_BID_i[W_SID*(s+1)-1 -: W_SID];
        assign s_bresp[s]    = s_BRESP_i[TRANS_WR_RESP_W*(s+1)-1 -: TRANS_WR_RESP_W];
        assign s_bvalid[s]   = s_BVALID_i[s];
        assign s_BREADY_o[s] = s_bready[s];
        
        // AR channel
        assign s_ARID_o[W_SID*(s+1)-1 -: W_SID]       = s_arid[s];
        assign s_ARADDR_o[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH] = s_araddr[s];
        assign s_ARBURST_o[TRANS_BURST_W*(s+1)-1 -: TRANS_BURST_W] = s_arburst[s];
        assign s_ARLEN_o[TRANS_DATA_LEN_W*(s+1)-1 -: TRANS_DATA_LEN_W] = s_arlen[s];
        assign s_ARSIZE_o[TRANS_DATA_SIZE_W*(s+1)-1 -: TRANS_DATA_SIZE_W] = s_arsize[s];
        assign s_ARVALID_o[s] = s_arvalid[s];
        assign s_arready[s]   = s_ARREADY_i[s];
        
        // R channel
        assign s_rid[s]      = s_RID_i[W_SID*(s+1)-1 -: W_SID];
        assign s_rdata[s]    = s_RDATA_i[DATA_WIDTH*(s+1)-1 -: DATA_WIDTH];
        assign s_rresp[s]    = s_RRESP_i[TRANS_WR_RESP_W*(s+1)-1 -: TRANS_WR_RESP_W];
        assign s_rlast[s]    = s_RLAST_i[s];
        assign s_rvalid[s]   = s_RVALID_i[s];
        assign s_RREADY_o[s] = s_rready[s];
    end
endgenerate

//=============================================================================
// Address Decode & Default Slave Logic
//=============================================================================
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : ADDR_DECODE
        // Extract slave address base/length for this slave
        wire [ADDR_WIDTH-1:0] slv_base = SLV_ADDR_BASE[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH];
        wire [7:0]          slv_len  = SLV_ADDR_LEN[8*(s+1)-1 -: 8];
        
        // Address match: compare upper bits [ADDR_WIDTH-1:slv_len]
        wire [MST_AMT-1:0] aw_match, ar_match;
        for(m = 0; m < MST_AMT; m = m + 1) begin : MATCH_LOGIC
            assign aw_match[m] = (m_awaddr[m][ADDR_WIDTH-1:slv_len] == 
                                   slv_base[ADDR_WIDTH-1:slv_len]) & slv_en_i[s];
            assign ar_match[m] = (m_araddr[m][ADDR_WIDTH-1:slv_len] == 
                                   slv_base[ADDR_WIDTH-1:slv_len]) & slv_en_i[s];
        end
        
        // AWSELECT_OUT/ARSELECT_OUT: match result for this slave
        assign awselect_out[s] = aw_match;
        assign arselect_out[s] = ar_match;
        
        // For default slave, generate AWSELECT_IN as NOT selected by any other slave
        if(DEFAULT_SLV_EN && s == DEFAULT_SLV_IDX) begin : DEFAULT_SLAVE_INPUT
            wire [MST_AMT-1:0] aw_other, ar_other;
            for(m = 0; m < MST_AMT; m = m + 1) begin : OTHER_SLV
                wire aw_any, ar_any;
                for(s_ = 0; s_ < SLV_AMT; s_ = s_ + 1) begin : ANY_SLV
                    if(s_ != DEFAULT_SLV_IDX) begin
                        assign aw_any = |{aw_any, awselect_out[s_][m]};
                        assign ar_any = |{ar_any, arselect_out[s_][m]};
                    end
                end
                assign aw_other[m] = aw_any;
                assign ar_other[m] = ar_any;
            end
            assign awselect_in[DEFAULT_SLV_IDX] = ~aw_other;
            assign arselect_in[DEFAULT_SLV_IDX] = ~ar_other;
        end
    end
endgenerate

//=============================================================================
// Ready Aggregation: Master Side (M2S OR-reduce)
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : READY_AGG_M
        wire [SLV_AMT-1:0] aw_rdy_vec, w_rdy_vec, ar_rdy_vec;
        for(s = 0; s < SLV_AMT; s = s + 1) begin : AGG_SLV
            // For simplicity, we use m_awready_agg as placeholder; actual ready from each M2S module
            assign aw_rdy_vec[s] = m_awready_agg[m]; // Will be connected in M2S instance
            assign w_rdy_vec[s]  = m_wready_agg[m];
            assign ar_rdy_vec[s] = m_arready_agg[m];
        end
        assign m_awready[m] = |aw_rdy_vec;
        assign m_wready[m]  = |w_rdy_vec;
        assign m_arready[m] = |ar_rdy_vec;
    end
endgenerate

//=============================================================================
// Ready Aggregation: Slave Side (S2M OR-reduce)
//=============================================================================
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : READY_AGG_S
        wire [MST_AMT-1:0] b_rdy_vec, r_rdy_vec;
        for(m = 0; m < MST_AMT; m = m + 1) begin : AGG_MST
            assign b_rdy_vec[m] = s_bready_agg[s];  // Will be driven by S2M module
            assign r_rdy_vec[m] = s_rready_agg[s];
        end
        assign s_bready[s] = |b_rdy_vec;
        assign s_rready[s] = |r_rdy_vec;
    end
endgenerate

//=============================================================================
// Read Reorder Grant Slicing: per-master [SLV_AMT-1:0]
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : R_ORDER_SLICE
        assign r_order_grant_m[m] = r_order_grant_i[SLV_AMT*(m+1)-1 -: SLV_AMT];
    end
endgenerate

//=============================================================================
// M2S Module Instantiation: One per Slave (axi_m2s_m_amt)
//=============================================================================
generate
    for(s = 0; s < SLV_AMT; s = s + 1) begin : INST_M2S
        // Extract slave config
        wire [ADDR_WIDTH-1:0] slv_base = SLV_ADDR_BASE[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH];
        wire [7:0]          slv_len  = SLV_ADDR_LEN[8*(s+1)-1 -: 8];
        
        // Build packed master arrays for connection
        wire [TRANS_MST_ID_W*MST_AMT-1:0] m_awid_packed;
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
        
        wire [TRANS_MST_ID_W*MST_AMT-1:0] m_arid_packed;
        wire [ADDR_WIDTH*MST_AMT-1:0]     m_araddr_packed;
        wire [TRANS_DATA_LEN_W*MST_AMT-1:0] m_arlen_packed;
        wire [TRANS_DATA_SIZE_W*MST_AMT-1:0] m_arsize_packed;
        wire [TRANS_BURST_W*MST_AMT-1:0] m_arburst_packed;
        wire [MST_AMT-1:0] m_arvalid_packed;
        wire [MST_AMT-1:0] m_arready_packed;
        
        // Pack arrays
        for(m = 0; m < MST_AMT; m = m + 1) begin : PACK_M2S
            assign m_awid_packed[TRANS_MST_ID_W*m +: TRANS_MST_ID_W] = m_awid[m];
            assign m_awaddr_packed[ADDR_WIDTH*m +: ADDR_WIDTH] = m_awaddr[m];
            assign m_awlen_packed[TRANS_DATA_LEN_W*m +: TRANS_DATA_LEN_W] = m_awlen[m];
            assign m_awsize_packed[TRANS_DATA_SIZE_W*m +: TRANS_DATA_SIZE_W] = m_awsize[m];
            assign m_awburst_packed[TRANS_BURST_W*m +: TRANS_BURST_W] = m_awburst[m];
            assign m_awvalid_packed[m] = m_awvalid[m];
            assign m_wdata_packed[DATA_WIDTH*m +: DATA_WIDTH] = m_wdata[m];
            assign m_wstrb_packed[W_STRB*m +: W_STRB] = m_wstrb[m];
            assign m_wlast_packed[m] = m_wlast[m];
            assign m_wvalid_packed[m] = m_wvalid[m];
            assign m_arid_packed[TRANS_MST_ID_W*m +: TRANS_MST_ID_W] = m_arid[m];
            assign m_araddr_packed[ADDR_WIDTH*m +: ADDR_WIDTH] = m_araddr[m];
            assign m_arlen_packed[TRANS_DATA_LEN_W*m +: TRANS_DATA_LEN_W] = m_arlen[m];
            assign m_arsize_packed[TRANS_DATA_SIZE_W*m +: TRANS_DATA_SIZE_W] = m_arsize[m];
            assign m_arburst_packed[TRANS_BURST_W*m +: TRANS_BURST_W] = m_arburst[m];
            assign m_arvalid_packed[m] = m_arvalid[m];
            
            // Ready outputs from M2S are per-master; connect to aggregated ready
            assign m_awready[m] = m_awready_packed[m];
            assign m_wready[m]  = m_wready_packed[m];
            assign m_arready[m] = m_arready_packed[m];
        end
        
        axi_m2s_m_amt #(
            .SLAVE_ID(s),
            .ADDR_BASE(slv_base),
            .ADDR_LENGTH(slv_len),
            .W_CID(SLV_ID_W),
            .W_ID(TRANS_MST_ID_W),
            .W_ADDR(ADDR_WIDTH),
            .W_DATA(DATA_WIDTH),
            .W_STRB(W_STRB),
            .W_SID(W_SID),
            .MST_AMT(MST_AMT),
            .OUTSTANDING_AMT(OUTSTANDING_AMT),
            .ALEN_W(TRANS_DATA_LEN_W),
            .ASIZE_W(TRANS_DATA_SIZE_W),
            .ABURST_W(TRANS_BURST_W),
            .SLAVE_DEFAULT((s == DEFAULT_SLV_IDX) ? DEFAULT_SLV_EN : 1'b0)
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
            
            .AWSELECT_OUT(awselect_out[s]),
            .ARSELECT_OUT(arselect_out[s]),
            .AWSELECT_IN((s == DEFAULT_SLV_IDX) ? awselect_in[s] : '0),
            .ARSELECT_IN((s == DEFAULT_SLV_IDX) ? arselect_in[s] : '0),
            .arbiter_type(arbiter_type)
        );
    end
endgenerate

//=============================================================================
// S2M Module Instantiation: One per Master (axi_s2m_s_amt)
//=============================================================================
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : INST_S2M
        // Build packed slave arrays
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
            
            assign s_bready[s] = s_bready_packed[s];
            assign s_rready[s] = s_rready_packed[s];
        end
        
        axi_s2m_s_amt #(
            .MASTER_ID(m),
            .W_CID(SLV_ID_W),
            .W_ID(TRANS_MST_ID_W),
            .W_ADDR(ADDR_WIDTH),
            .W_DATA(DATA_WIDTH),
            .W_STRB(W_STRB),
            .W_SID(W_SID),
            .SLV_AMT(SLV_AMT),
            .MST_ID_FIELD_MSB(SLV_ID_W + TRANS_MST_ID_W - 1),
            .MST_ID_FIELD_LSB(SLV_ID_W)
        ) u_axi_s2m (
            .AXI_RSTn(AXI_RSTn),
            .AXI_CLK(AXI_CLK),
            
            .M_BID(m_bid[m]),
            .M_BRESP(m_bresp[m]),
            .M_BVALID(m_bvalid[m]),
            .M_BREADY(m_bready[m]),
            
            .M_RSID(),  // Not used
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
            
            .r_order_grant(r_order_grant_m[m]),
            .arbiter_type(arbiter_type)
        );
    end
endgenerate

endmodule