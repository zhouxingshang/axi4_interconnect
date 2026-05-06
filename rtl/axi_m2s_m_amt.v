//=============================================================================
// Module: axi_m2s_m_amt (Modified)
// Desc  : Parameterized AXI Master-to-Slave routing module with W-follows-AW.
//         - Supports global address decode input (aw_select_in/ar_select_in)
//         - Correct ID concatenation: {slave_idx, master_idx, orig_id}
//         - W channel strictly follows AW order via FIFO + beat counter
//         - Transparent to cross-4K splits (each sub-transaction handled individually)
//         - Provides aw_trans_done/ar_trans_done for response merging at higher level
//=============================================================================

module axi_m2s_m_amt #(
    parameter SLAVE_ID        = 0,              // Index of this slave (used for ID extension)
    parameter ADDR_BASE       = 32'h0,          // Unused when using global decode; kept for compatibility
    parameter ADDR_LENGTH     = 12,             // Unused when using global decode
    parameter M_ID_W          = 3,
    parameter W_ID            = 4,              // Original transaction ID width
    parameter W_ADDR          = 32,
    parameter W_DATA          = 32,
    parameter W_STRB          = (W_DATA/8),
    parameter W_SID           = M_ID_W + W_ID,   // Full ID width including slave and master index
    parameter MST_AMT         = 3,              // Number of masters
    parameter OUTSTANDING_AMT = 8,              // Max outstanding AW transactions
    parameter ALEN_W          = 8,
    parameter ASIZE_W         = 3,
    parameter ABURST_W        = 2,
    parameter SLAVE_DEFAULT   = 1'b0            // Unused with global decode; kept for compatibility
)(
    input  wire                      AXI_RSTn,
    input  wire                      AXI_CLK,

    // ========== Master side ports (flattened) ==========
    // AW channel
    input  wire  [MST_AMT*W_ID-1     : 0]        M_AWID,
    input  wire  [MST_AMT*W_ADDR-1   : 0]        M_AWADDR,
    input  wire  [MST_AMT*ALEN_W-1   : 0]        M_AWLEN,
    input  wire  [MST_AMT*ASIZE_W-1  : 0]        M_AWSIZE,
    input  wire  [MST_AMT*ABURST_W-1 : 0]        M_AWBURST,
    input  wire  [MST_AMT-1          : 0]        M_AWVALID,
    output wire  [MST_AMT-1          : 0]        M_AWREADY,

    // W channel
    input  wire  [MST_AMT*W_DATA-1   : 0]        M_WDATA,
    input  wire  [MST_AMT*W_STRB-1   : 0]        M_WSTRB,
    input  wire  [MST_AMT-1          : 0]        M_WLAST,
    input  wire  [MST_AMT-1          : 0]        M_WVALID,
    output wire  [MST_AMT-1          : 0]        M_WREADY,

    // AR channel
    input  wire  [MST_AMT*W_ID-1     : 0]        M_ARID,
    input  wire  [MST_AMT*W_ADDR-1   : 0]        M_ARADDR,
    input  wire  [MST_AMT*ALEN_W-1   : 0]        M_ARLEN,
    input  wire  [MST_AMT*ASIZE_W-1  : 0]        M_ARSIZE,
    input  wire  [MST_AMT*ABURST_W-1 : 0]        M_ARBURST,
    input  wire  [MST_AMT-1          : 0]        M_ARVALID,
    output wire  [MST_AMT-1          : 0]        M_ARREADY,

    // ========== Slave side ports (single slave) ==========
    // AW channel
    output reg    [W_SID-1     : 0]        S_AWID,
    output reg    [W_ADDR-1   : 0]         S_AWADDR,
    output reg    [ALEN_W-1   : 0]         S_AWLEN,
    output reg    [ASIZE_W-1  : 0]         S_AWSIZE,
    output reg    [ABURST_W-1 : 0]         S_AWBURST,
    output reg                             S_AWVALID,
    input   wire                           S_AWREADY,

    // W channel
    output reg    [W_DATA-1:0]             S_WDATA,
    output reg    [W_STRB-1:0]             S_WSTRB,
    output reg                             S_WLAST,
    output reg                             S_WVALID,
    input   wire                           S_WREADY,

    // AR channel
    output reg    [W_SID-1     : 0]        S_ARID,
    output reg    [W_ADDR-1   : 0]         S_ARADDR,
    output reg    [ALEN_W-1   : 0]         S_ARLEN,
    output reg    [ASIZE_W-1  : 0]         S_ARSIZE,
    output reg    [ABURST_W-1 : 0]         S_ARBURST,
    output reg                             S_ARVALID,
    input   wire                           S_ARREADY,

    // ========== Control/Status ports ==========
    input  wire  [MST_AMT-1:0]    AWSELECT_IN,    // Global decode select for AW (one-hot per master)
    input  wire  [MST_AMT-1:0]    ARSELECT_IN,    // Global decode select for AR
    output wire  [MST_AMT-1:0]    AWSELECT_OUT,   // Bypass input to output (for debug)
    output wire  [MST_AMT-1:0]    ARSELECT_OUT,
    input  wire                   arbiter_type,

    // Transaction completion signals (for response merging in crossbar)
    output wire                   aw_trans_done,   // Pulse when a full AW+W transaction completes
    output wire                   ar_trans_done    // Pulse when a full AR transaction completes
);

//=============================================================================
// Local Parameters
//=============================================================================
localparam MST_ID_W     = $clog2(MST_AMT);                          // Width of master index
localparam AW_ORDER_W   = MST_ID_W + ALEN_W + 1;                    // {mst_idx, awlen, sub_done_flag} (sub_done_flag used if split)
localparam CNT_W        = ALEN_W + 1;                               // Beat counter width

//=============================================================================
// Internal Signals - Flattened Master Ports
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
// W-Follows-AW Core: AW Order FIFO + Beat Counter
//=============================================================================
wire [AW_ORDER_W-1:0]  aw_fifo_wr_data;
wire [AW_ORDER_W-1:0]  aw_fifo_rd_data;
wire                   aw_fifo_wr_en;
wire                   aw_fifo_rd_en;
wire                   aw_fifo_full;
wire                   aw_fifo_empty;

reg  [CNT_W-1:0]       w_beat_cnt;                  // Remaining beats for current W transaction
reg  [MST_ID_W-1:0]    cur_w_mst_id;                // Current master whose W data should be routed
wire                   w_transaction_active;
assign w_transaction_active = (w_beat_cnt != 0);

// Transaction done detection
reg  aw_handshake_done;         // Latched when AW handshake occurs (to generate pulse)
reg  aw_done_pulse;
reg  aw_done_q;
always @(posedge AXI_CLK) begin
    if (!AXI_RSTn) begin
        aw_done_q <= 1'b0;
    end else begin
        aw_done_q <= aw_done_pulse;
    end
end
assign aw_trans_done = aw_done_pulse & ~aw_done_q;   // rising edge pulse

//=============================================================================
// Address decode bypass: use global select inputs directly
//=============================================================================
assign AWSELECT_OUT = AWSELECT_IN;
assign ARSELECT_OUT = ARSELECT_IN;

//=============================================================================
// Port Flattening: Unpack Master Interfaces
//=============================================================================
genvar m;
generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : UNPACK_MASTER
        assign m_awid[m]    = M_AWID[W_ID*(m+1)-1 -: W_ID];
        assign m_awaddr[m]  = M_AWADDR[W_ADDR*(m+1)-1 -: W_ADDR];
        assign m_awlen[m]   = M_AWLEN[ALEN_W*(m+1)-1 -: ALEN_W];
        assign m_awsize[m]  = M_AWSIZE[ASIZE_W*(m+1)-1 -: ASIZE_W];
        assign m_awburst[m] = M_AWBURST[ABURST_W*(m+1)-1 -: ABURST_W];
        assign m_awvalid[m] = M_AWVALID[m];
        assign M_AWREADY[m] = m_awready[m];

        assign m_wdata[m]   = M_WDATA[W_DATA*(m+1)-1 -: W_DATA];
        assign m_wstrb[m]   = M_WSTRB[W_STRB*(m+1)-1 -: W_STRB];
        assign m_wlast[m]   = M_WLAST[m];
        assign m_wvalid[m]  = M_WVALID[m];
        assign M_WREADY[m]  = m_wready[m];

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
// AW/AR Arbitration (single instance handling both channels)
//=============================================================================
wire [MST_AMT-1:0] AWGRANT, ARGRANT;
wire [MST_AMT-1:0] aw_ready_from_slave;
wire [MST_AMT-1:0] ar_ready_from_slave;

// The arbiter module is assumed to be correctly implemented
axi_arbiter_m2s_m_amt #(
    .W_CID(W_CID),
    .W_ID(W_ID),
    .NUM(MST_AMT)
) u_axi_arbiter_aw_ar (
    .AXI_RSTn    (AXI_RSTn),
    .AXI_CLK     (AXI_CLK),
    .AWSELECT    (AWSELECT_IN),      // Use global decode
    .AWVALID     ({m_awvalid[MST_AMT-1:0]}),
    .AWREADY     (aw_ready_from_slave),
    .AWGRANT     (AWGRANT),
    .ARSELECT    (ARSELECT_IN),
    .ARVALID     ({m_arvalid[MST_AMT-1:0]}),
    .ARREADY     (ar_ready_from_slave),
    .ARGRANT     (ARGRANT),
    .arbiter_type(arbiter_type)
);

//=============================================================================
// AW Handshake Capture -> Push to Order FIFO
//=============================================================================
wire [MST_AMT-1:0] aw_handshake = AWGRANT & {S_AWREADY{MST_AMT{1'b1}}} & {m_awvalid[MST_AMT-1:0]};
wire [MST_ID_W-1:0] aw_grant_idx;

// One-hot grant to binary index encoder (priority encoder)
always @(*) begin
    aw_grant_idx = '0;
    for(int i = 0; i < MST_AMT; i++) begin
        if(AWGRANT[i])
            aw_grant_idx = i[MST_ID_W-1:0];
    end
end

// AW order FIFO write data: {master_index, awlen}
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
    .wr_en    (aw_fifo_wr_en),
    .wr_data  (aw_fifo_wr_data),
    .wr_full  (aw_fifo_full),
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

// FIFO read enable: when current W transaction completes (last beat with WLAST)
assign aw_fifo_rd_en = (w_beat_cnt == 1'b1 && S_WLAST && S_WREADY && S_WVALID);

// W beat counter and current master tracking
always @(posedge AXI_CLK) begin
    if(!AXI_RSTn) begin
        w_beat_cnt   <= 0;
        cur_w_mst_id <= 0;
        aw_done_pulse <= 1'b0;
    end else begin
        aw_done_pulse <= 1'b0;
        // Case 1: AW handshake occurs -> load counter from granted master
        if(|aw_handshake) begin
            w_beat_cnt   <= m_awlen[aw_grant_idx] + 1'b1;  // beats = LEN+1
            cur_w_mst_id <= aw_grant_idx;
        end
        // Case 2: W data transfer occurs -> decrement counter
        else if(w_transaction_active && S_WREADY && S_WVALID) begin
            if(w_beat_cnt == 1'b1 && S_WLAST) begin
                w_beat_cnt   <= 0;                 // Transaction complete
                aw_done_pulse <= 1'b1;             // Flag completion
            end else begin
                w_beat_cnt <= w_beat_cnt - 1'b1;
            end
        end
        // Case 3: No active W transaction and FIFO not empty -> load next master from FIFO
        if(!w_transaction_active && !aw_fifo_empty && !(|aw_handshake)) begin
            cur_w_mst_id <= fifo_mst_idx;
            w_beat_cnt   <= fifo_awlen + 1'b1;
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
            .FAW(2)                     // depth = 4
        ) u_fifo_w (
            .rstn   (AXI_RSTn),
            .clr    (1'b0),
            .clk    (AXI_CLK),
            .wr_rdy (m_wready[m]),
            .wr_vld (m_wvalid[m]),
            .wr_din ({m_wdata[m], m_wstrb[m], m_wlast[m]}),
            .rd_rdy ((cur_w_mst_id == m[MST_ID_W-1:0]) && w_transaction_active && S_WREADY),
            .rd_vld (w_fifo_vld[m]),
            .rd_dout(w_fifo_dout[m])
        );
    end
endgenerate

// W output Mux: select from current master's FIFO
always @(*) begin
    S_WDATA  = '0;
    S_WSTRB  = '0;
    S_WLAST  = 1'b0;
    S_WVALID = 1'b0;
    if(w_transaction_active) begin
        S_WDATA  = w_fifo_dout[cur_w_mst_id][W_DATA+W_STRB : W_STRB+1];
        S_WSTRB  = w_fifo_dout[cur_w_mst_id][W_STRB : 1];
        S_WLAST  = w_fifo_dout[cur_w_mst_id][0];
        S_WVALID = w_fifo_vld[cur_w_mst_id];
    end
end

//=============================================================================
// AW Ready Back-pressure: stall if FIFO full
//=============================================================================
wire aw_stall = aw_fifo_full;
assign aw_ready_from_slave = AWGRANT & {S_AWREADY{MST_AMT{1'b1}}} & ~{aw_stall{MST_AMT{1'b1}}};
assign {m_awready[MST_AMT-1:0]} = aw_ready_from_slave;

//=============================================================================
// AW/AR Bus Packing & Routing with Correct ID Concatenation
//=============================================================================
// S_AWID = {SLAVE_ID[W_CID-1:0], master_index, original_AWID}
// Similarly for S_ARID
localparam BUS_AW_W = W_CID + MST_ID_W + W_ID + W_ADDR + ALEN_W + ASIZE_W + ABURST_W + 1;
localparam BUS_AR_W = W_CID + MST_ID_W + W_ID + W_ADDR + ALEN_W + ASIZE_W + ABURST_W + 1;

wire [BUS_AW_W-1:0] bus_aw [0:MST_AMT-1];
wire [BUS_AR_W-1:0] bus_ar [0:MST_AMT-1];

generate
    for(m = 0; m < MST_AMT; m = m + 1) begin : PACK_BUS_AW_AR
        // Slave ID comes from parameter (global slave index), master index from loop variable
        wire [W_CID-1:0] slave_id = SLAVE_ID[W_CID-1:0];
        wire [MST_ID_W-1:0] master_idx = m[MST_ID_W-1:0];
        assign bus_aw[m] = {slave_id, master_idx, m_awid[m],
                            m_awaddr[m], m_awlen[m], m_awsize[m], m_awburst[m], m_awvalid[m]};
        assign bus_ar[m] = {slave_id, master_idx, m_arid[m],
                            m_araddr[m], m_arlen[m], m_arsize[m], m_arburst[m], m_arvalid[m]};
    end
endgenerate

`define S_AWBUS {S_AWID, S_AWADDR, S_AWLEN, S_AWSIZE, S_AWBURST, S_AWVALID}
`define S_ARBUS {S_ARID, S_ARADDR, S_ARLEN, S_ARSIZE, S_ARBURST, S_ARVALID}

// AW routing mux (parameterized)
always @(*) begin
    `S_AWBUS = '0;
    for(int i = 0; i < MST_AMT; i++) begin
        if(AWGRANT[i]) begin
            {S_AWID, S_AWADDR, S_AWLEN, S_AWSIZE, S_AWBURST, S_AWVALID} = bus_aw[i];
        end
    end
end

// AR routing mux
always @(*) begin
    `S_ARBUS = '0;
    for(int i = 0; i < MST_AMT; i++) begin
        if(ARGRANT[i]) begin
            {S_ARID, S_ARADDR, S_ARLEN, S_ARSIZE, S_ARBURST, S_ARVALID} = bus_ar[i];
        end
    end
end

//=============================================================================
// AR transaction done detection (for read response merging)
//=============================================================================
reg ar_done_pulse;
reg ar_done_q;
wire ar_handshake = ARGRANT & {S_ARREADY{MST_AMT{1'b1}}} & {m_arvalid[MST_AMT-1:0]};
always @(posedge AXI_CLK) begin
    if(!AXI_RSTn) begin
        ar_done_pulse <= 1'b0;
        ar_done_q <= 1'b0;
    end else begin
        ar_done_pulse <= |ar_handshake;   // A single AR handshake completes the transaction (no separate data phase)
        ar_done_q <= ar_done_pulse;
    end
end
assign ar_trans_done = ar_done_pulse & ~ar_done_q;

// AR ready back-pressure: simple combination (no FIFO for AR)
assign ar_ready_from_slave = ARGRANT & {S_ARREADY{MST_AMT{1'b1}}};
assign {m_arready[MST_AMT-1:0]} = ar_ready_from_slave;

endmodule