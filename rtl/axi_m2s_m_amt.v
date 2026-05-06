//=============================================================================
// Module: axi_m2s_m_amt
// Desc  : Parameterized AXI Master-to-Slave routing module with W-follows-AW
//         - Supports arbitrary MST_AMT masters via flattened interface
//         - W channel strictly follows AW handshake order using FIFO+counter
//         - Retains axi_fifo_sync for W data buffering
//         - Master identity encoded implicitly via array index (no m_id port)
//=============================================================================
module axi_m2s_m_amt
#(
    parameter SLAVE_ID        = 0,              // for reference
    parameter ADDR_BASE       = 32'h0,          // Slave address base
    parameter ADDR_LENGTH     = 12,             // Effective address bits for decode
    parameter M_ID_W           = 4,              // Channel ID width
    parameter W_ID            = 4,              // Transaction ID width
    parameter W_ADDR          = 32,             // Address width
    parameter W_DATA          = 32,             // Data width
    parameter W_STRB          = (W_DATA/8),     // Data strobe width
    parameter W_SID           = M_ID_W + W_ID,   // Slave-side ID width
    parameter MST_AMT         = 3,              // Number of masters (parameterized)
    parameter OUTSTANDING_AMT = 8,              // Max outstanding AW transactions
    parameter ALEN_W          = 8,              // AWLEN/ARLEN width
    parameter ASIZE_W         = 3,              // AWSIZE/ARSIZE width
    parameter ABURST_W        = 2,              // AWBURST/ARBURST width
    parameter SLAVE_DEFAULT   = 1'b0            // Default-slave mode flag
)
(
    // Global signals
    input   wire                      AXI_RSTn,
    input   wire                      AXI_CLK,
    
    // ========== Master side ports (FLATTENED & PARAMETERIZED) ==========
    // AW channel
    input   wire  [MST_AMT*W_ID-1     : 0]        M_AWID,
    input   wire  [MST_AMT*W_ADDR-1   : 0]        M_AWADDR,
    input   wire  [MST_AMT*ALEN_W-1   : 0]        M_AWLEN,
    input   wire  [MST_AMT*ASIZE_W-1  : 0]        M_AWSIZE,
    input   wire  [MST_AMT*ABURST_W-1 : 0]        M_AWBURST,
    input   wire  [MST_AMT-1          : 0]        M_AWVALID,
    output  wire  [MST_AMT-1          : 0]        M_AWREADY,
    
    // W channel
    input   wire  [MST_AMT*W_DATA-1   : 0]        M_WDATA,
    input   wire  [MST_AMT*W_STRB-1   : 0]        M_WSTRB,
    input   wire  [MST_AMT-1          : 0]        M_WLAST,
    input   wire  [MST_AMT-1          : 0]        M_WVALID,
    output  wire  [MST_AMT-1          : 0]        M_WREADY,
    
    // AR channel
    input   wire  [MST_AMT*W_ID-1     : 0]        M_ARID,
    input   wire  [MST_AMT*W_ADDR-1   : 0]        M_ARADDR,
    input   wire  [MST_AMT*ALEN_W-1   : 0]        M_ARLEN,
    input   wire  [MST_AMT*ASIZE_W-1  : 0]        M_ARSIZE,
    input   wire  [MST_AMT*ABURST_W-1 : 0]        M_ARBURST,
    input   wire  [MST_AMT-1          : 0]        M_ARVALID,
    output  wire  [MST_AMT-1          : 0]        M_ARREADY,
    
    // ========== Slave side ports (single slave interface) ==========
    // AW channel
    output  reg    [W_SID-1     : 0]        S_AWID,
    output  reg    [W_ADDR-1   : 0]        S_AWADDR,
    output  reg    [ALEN_W-1   : 0]        S_AWLEN,
    output  reg    [ASIZE_W-1  : 0]        S_AWSIZE,
    output  reg    [ABURST_W-1 : 0]        S_AWBURST,
    output  reg                            S_AWVALID,
    input   wire                           S_AWREADY,
    
    // W channel
    output  reg    [W_DATA-1:0]            S_WDATA,
    output  reg    [W_STRB-1:0]            S_WSTRB,
    output  reg                            S_WLAST,
    output  reg                            S_WVALID,
    input   wire                           S_WREADY,
    
    // AR channel
    output  reg    [W_SID-1     : 0]        S_ARID,
    output  reg    [W_ADDR-1   : 0]        S_ARADDR,
    output  reg    [ALEN_W-1   : 0]        S_ARLEN,
    output  reg    [ASIZE_W-1  : 0]        S_ARSIZE,
    output  reg    [ABURST_W-1 : 0]        S_ARBURST,
    output  reg                            S_ARVALID,
    input   wire                           S_ARREADY,
    
    // ========== Control/Status ports ==========
    output  wire  [MST_AMT-1:0]    AWSELECT_OUT,
    output  wire  [MST_AMT-1:0]    ARSELECT_OUT,
    input   wire  [MST_AMT-1:0]    AWSELECT_IN,
    input   wire  [MST_AMT-1:0]    ARSELECT_IN,
    input   wire                   arbiter_type
    // ⚠️ channel_en REMOVED - not needed
);

//=============================================================================
// Local Parameters
//=============================================================================
localparam MST_ID_W     = $clog2(MST_AMT);                          // Width to encode master index
localparam AW_ORDER_W   = MST_ID_W + ALEN_W;                        // {mst_idx, awlen} for FIFO
localparam CNT_W        = ALEN_W + 1;                               // Beat counter: AWLEN+1

//=============================================================================
// Internal Signals - Flattened Arrays for Master Ports
//=============================================================================
wire [W_ID-1:0]        m_awid      [0:MST_AMT-1];
wire [W_ADDR-1:0]      m_awaddr    [0:MST_AMT-1];
wire [ALEN_W-1:0]      m_awlen     [0:MST_AMT-1];
wire [ASIZE_W-1:0]     m_awsize    [0:MST_AMT-1];
wire [ABURST_W-1:0]    m_awburst   [0:MST_AMT-1];
wire                   m_awvalid   [0:MST_AMT-1];
wire                   m_awready   [0:MST_AMT-1];

wire [W_DATA-1:0]      m_wdata     [0:MST_AMT-1];
wire [W_STRB-1:0]      m_wstrb     [0:MST_AMT-1];
wire                   m_wlast     [0:MST_AMT-1];
wire                   m_wvalid    [0:MST_AMT-1];
wire                   m_wready    [0:MST_AMT-1];

wire [W_ID-1:0]        m_arid      [0:MST_AMT-1];
wire [W_ADDR-1:0]      m_araddr    [0:MST_AMT-1];
wire [ALEN_W-1:0]      m_arlen     [0:MST_AMT-1];
wire [ASIZE_W-1:0]     m_arsize    [0:MST_AMT-1];
wire [ABURST_W-1:0]    m_arburst   [0:MST_AMT-1];
wire                   m_arvalid   [0:MST_AMT-1];
wire                   m_arready   [0:MST_AMT-1];

//=============================================================================
// W-Follows-AW Core: AW Order FIFO + Beat Counter Signals
//=============================================================================
wire [AW_ORDER_W-1:0]  aw_fifo_wr_data;
wire [AW_ORDER_W-1:0]  aw_fifo_rd_data;
wire                   aw_fifo_wr_en;
wire                   aw_fifo_rd_en;
wire                   aw_fifo_full;
wire                   aw_fifo_empty;

wire [CNT_W-1:0]       w_beat_cnt;                  // Remaining beats for current W transaction
wire [MST_ID_W-1:0]    cur_w_mst_id;                // Current master whose W data should be routed
wire                   w_transaction_active;        // Flag: W transaction in progress
assign w_transaction_active = (w_beat_cnt > 0);

//=============================================================================
// Address Decode & Arbitration Signals
//=============================================================================
reg  [MST_AMT-1:0] AWSELECT, ARSELECT;
wire [MST_AMT-1:0] AWGRANT, ARGRANT;
assign AWSELECT_OUT = AWSELECT;
assign ARSELECT_OUT = ARSELECT;

//=============================================================================
// Port Flattening: generate loop for unpack/pack
//=============================================================================
genvar m;
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : UNPACK_MASTER
        // AW channel unpack
        assign m_awid[m]    = M_AWID[W_ID*(m+1)-1 -: W_ID];
        assign m_awaddr[m]  = M_AWADDR[W_ADDR*(m+1)-1 -: W_ADDR];
        assign m_awlen[m]   = M_AWLEN[ALEN_W*(m+1)-1 -: ALEN_W];
        assign m_awsize[m]  = M_AWSIZE[ASIZE_W*(m+1)-1 -: ASIZE_W];
        assign m_awburst[m] = M_AWBURST[ABURST_W*(m+1)-1 -: ABURST_W];
        assign m_awvalid[m] = M_AWVALID[m];
        assign M_AWREADY[m] = m_awready[m];
        
        // W channel unpack
        assign m_wdata[m]   = M_WDATA[W_DATA*(m+1)-1 -: W_DATA];
        assign m_wstrb[m]   = M_WSTRB[W_STRB*(m+1)-1 -: W_STRB];
        assign m_wlast[m]   = M_WLAST[m];
        assign m_wvalid[m]  = M_WVALID[m];
        assign M_WREADY[m]  = m_wready[m];
        
        // AR channel unpack
        assign m_arid[m]    = M_ARID[W_ID*(m+1)-1 -: W_ID];
        assign m_araddr[m]  = M_ARADDR[W_ADDR*(m+1)-1 -: W_ADDR];
        assign m_arlen[m]   = M_ARLEN[ALEN_W*(m+1)-1 -: ALEN_W];
        assign m_arsize[m]  = M_ARSIZE[ASIZE_W*(m+1)-1 -: ASIZE_W];
        assign m_arburst[m] = M_ARBURST[ABURST_W*(m+1)-1 -: ABURST_W];
        assign m_arvalid[m] = M_ARVALID[m];
        assign M_ARREADY[m] = m_arready[m];
    end
endgenerate

//=============================================================================
// Address Decode (parameterized, NO channel_en gating)
//=============================================================================
always @(*) begin
    if (SLAVE_DEFAULT == 1'b0) begin
        for(int i = 0; i < MST_AMT; i++) begin
            // Address match: compare upper bits against ADDR_BASE
            AWSELECT[i] = (m_awaddr[i][W_ADDR-1:ADDR_LENGTH] == ADDR_BASE[W_ADDR-1:ADDR_LENGTH]);
            ARSELECT[i] = (m_araddr[i][W_ADDR-1:ADDR_LENGTH] == ADDR_BASE[W_ADDR-1:ADDR_LENGTH]);
        end
        // ⚠️ WSELECT REMOVED: W routing now follows AW order FIFO, not address decode
    end else begin
        // Default slave mode: accept any request not selected by others
        AWSELECT = ~AWSELECT_IN & {m_awvalid[MST_AMT-1:0]};
        ARSELECT = ~ARSELECT_IN & {m_arvalid[MST_AMT-1:0]};
    end
end

//=============================================================================
// AW/AR Arbitration (SINGLE instance handling both channels)
//=============================================================================
// Note: axi_arbiter_m2s_m_amt handles both AW and AR internally
axi_arbiter_m2s_m_amt #(
    .W_CID(W_CID), 
    .W_ID(W_ID), 
    .NUM(MST_AMT)
) u_axi_arbiter_aw_ar (
    .AXI_RSTn    (AXI_RSTn),
    .AXI_CLK     (AXI_CLK),
    
    // AW channel ports
    .AWSELECT    (AWSELECT),
    .AWVALID     ({m_awvalid[MST_AMT-1:0]}),
    .AWREADY     ({m_awready[MST_AMT-1:0]}),
    .AWGRANT     (AWGRANT),
    
    // AR channel ports
    .ARSELECT    (ARSELECT),
    .ARVALID     ({m_arvalid[MST_AMT-1:0]}),
    .ARREADY     ({m_arready[MST_AMT-1:0]}),
    .ARGRANT     (ARGRANT),
    
    .arbiter_type(arbiter_type)
    // ⚠️ W channel ports REMOVED - handled by AW-order FIFO logic
);
//=============================================================================
// AW Handshake Capture -> Push to Order FIFO
//=============================================================================
wire [MST_AMT-1:0] aw_handshake = AWGRANT & {S_AWREADY{MST_AMT{1'b1}}} & {m_awvalid[MST_AMT-1:0]};
wire [MST_ID_W-1:0] aw_grant_idx;

// One-hot grant to binary index encoder (priority: low index first)
always @(*) begin
    aw_grant_idx = '0;
    for(int i = 0; i < MST_AMT; i++) begin
        if(AWGRANT[i]) begin
            aw_grant_idx = i[MST_ID_W-1:0];
        end
    end
end

// AW order FIFO write data: {mst_idx, awlen}
assign aw_fifo_wr_data = {aw_grant_idx, m_awlen[aw_grant_idx]};
assign aw_fifo_wr_en   = |aw_handshake;

//=============================================================================
// AW Order FIFO Instance (modularized)
//=============================================================================
aw_order_fifo #(
    .DATA_WIDTH(AW_ORDER_W),
    .FIFO_DEPTH(OUTSTANDING_AMT)
) u_aw_order_fifo (
    .clk      (AXI_CLK),
    .rst_n    (AXI_RSTn),
    
    // Write port
    .wr_en    (aw_fifo_wr_en),
    .wr_data  (aw_fifo_wr_data),
    .wr_full  (aw_fifo_full),
    
    // Read port
    .rd_en    (aw_fifo_rd_en),
    .rd_data  (aw_fifo_rd_data),
    .rd_empty (aw_fifo_empty)
);

//=============================================================================
// W Beat Counter & Current Master Tracking
//=============================================================================
wire [MST_ID_W-1:0]  fifo_mst_idx;
wire [ALEN_W-1:0]    fifo_awlen;
assign {fifo_mst_idx, fifo_awlen} = aw_fifo_rd_data;

// FIFO read enable: when current W transaction completes (last beat + WLAST)
assign aw_fifo_rd_en = (w_beat_cnt == 1'b1 && S_WLAST && S_WREADY && S_WVALID);

// W beat counter state machine
always @(posedge AXI_CLK) begin
    if(!AXI_RSTn) begin
        w_beat_cnt   <= 0;
        cur_w_mst_id <= 0;
    end else begin
        // Case 1: AW handshake occurs -> load counter from granted master
        if(|aw_handshake) begin
            w_beat_cnt   <= m_awlen[aw_grant_idx] + 1'b1;  // beats = LEN+1
            cur_w_mst_id <= aw_grant_idx;
        end
        // Case 2: W handshake occurs -> decrement counter
        else if(w_transaction_active && S_WREADY && S_WVALID) begin
            if(w_beat_cnt == 1'b1 && S_WLAST) begin
                w_beat_cnt <= 0;  // Transaction complete, will trigger FIFO pop next cycle
            end else begin
                w_beat_cnt <= w_beat_cnt - 1'b1;
            end
        end
        // Case 3: Counter expired & FIFO not empty -> load next master from FIFO
        if(!w_transaction_active && !aw_fifo_empty) begin
            cur_w_mst_id <= fifo_mst_idx;
        end
    end
end

//=============================================================================
// W Channel: axi_fifo_sync + Dynamic Mux Routing (W follows AW)
//=============================================================================
wire [W_DATA+W_STRB+1-1:0] w_fifo_dout [0:MST_AMT-1];
wire [MST_AMT-1:0]         w_fifo_vld;

generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : W_FIFO_PER_MASTER
        axi_fifo_sync #(
            .FDW(W_DATA + W_STRB + 1),  // {WDATA, WSTRB, WLAST}
            .FAW(2)                      // depth = 2^2 = 4
        ) u_fifo_w (
            .rstn   (AXI_RSTn),
            .clr    (1'b0),
            .clk    (AXI_CLK),
            
            // Write side: from master
            .wr_rdy (m_wready[m]),
            .wr_vld (m_wvalid[m]),
            .wr_din ({m_wdata[m], m_wstrb[m], m_wlast[m]}),
            
            // Read side: to slave (gated by W-follows-AW logic)
            .rd_rdy ((cur_w_mst_id == m[MST_ID_W-1:0]) && w_transaction_active && S_WREADY),
            .rd_vld (w_fifo_vld[m]),
            .rd_dout(w_fifo_dout[m])
        );
    end
endgenerate

// W output Mux: select from current master's FIFO (priority encoder style)
always @(*) begin
    S_WDATA = '0;
    S_WSTRB = '0;
    S_WLAST = 1'b0;
    S_WVALID = 1'b0;
    
    if(w_transaction_active && !aw_fifo_empty) begin
        // Only route data from the master whose AW was granted first
        S_WDATA = w_fifo_dout[cur_w_mst_id][W_DATA+W_STRB:W_STRB+1];
        S_WSTRB = w_fifo_dout[cur_w_mst_id][W_STRB:1];
        S_WLAST = w_fifo_dout[cur_w_mst_id][0];
        S_WVALID = w_fifo_vld[cur_w_mst_id];
    end
end

//=============================================================================
// AW Ready Back-pressure: stall if FIFO full (prevent overflow)
//=============================================================================
wire aw_stall = aw_fifo_full;
assign m_awready = AWGRANT & {MST_AMT{S_AWREADY}} & ~{MST_AMT{aw_stall}};

//=============================================================================
// AW/AR Bus Packing & Routing (parameterized Mux)
//=============================================================================
localparam NUM_AW_WIDTH = W_SID + W_ADDR + ALEN_W + ASIZE_W + ABURST_W + 1;
localparam NUM_AR_WIDTH = W_SID + W_ADDR + ALEN_W + ASIZE_W + ABURST_W + 1;

wire [NUM_AW_WIDTH-1:0] bus_aw [0:MST_AMT-1];
wire [NUM_AR_WIDTH-1:0] bus_ar [0:MST_AMT-1];

generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : PACK_BUS_AW_AR
        // S_AWID/S_ARID format: {mst_idx[MST_ID_W-1:0], original_ID[W_ID-1:0]}
        // Embeds master index as ID extension for response routing in S2M module
        assign bus_aw[m] = {m[MST_ID_W-1:0], m_awid[m],
                           m_awaddr[m], m_awlen[m], m_awsize[m], m_awburst[m], m_awvalid[m]};
        assign bus_ar[m] = {m[MST_ID_W-1:0], m_arid[m],
                           m_araddr[m], m_arlen[m], m_arsize[m], m_arburst[m], m_arvalid[m]};
    end
endgenerate

`define S_AWBUS {S_AWID, S_AWADDR, S_AWLEN, S_AWSIZE, S_AWBURST, S_AWVALID}
`define S_ARBUS {S_ARID, S_ARADDR, S_ARLEN, S_ARSIZE, S_ARBURST, S_ARVALID}

// AW routing Mux (parameterized case statement)
always @(*) begin
    `S_AWBUS = '0;
    for(int i = 0; i < MST_AMT; i++) begin
        if(AWGRANT[i]) begin
            `S_AWBUS = bus_aw[i];
        end
    end
end

// AR routing Mux (parameterized case statement)
always @(*) begin
    `S_ARBUS = '0;
    for(int i = 0; i < MST_AMT; i++) begin
        if(ARGRANT[i]) begin
            `S_ARBUS = bus_ar[i];
        end
    end
end

endmodule