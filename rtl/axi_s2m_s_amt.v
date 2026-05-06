//=============================================================================
// Module: axi_s2m_s_amt (Modified)
// Desc  : Parameterized AXI Slave-to-Master routing module.
//         - Supports arbitrary number of slaves (SLV_AMT)
//         - Extracts master ID from incoming BID/RID using field positions
//         - Supports read reordering via r_order_grant input (from reorder module)
//         - Provides per-master write response and read data paths
//         - Handshake hold in upstream arbiter is assumed; this module only muxes
//=============================================================================

module axi_s2m_s_amt #(
    parameter MASTER_ID       = 0,               // Index of the master this module serves
    parameter W_CID           = 4,               // Width of slave index (e.g., log2(SLV_AMT))
    parameter W_ID            = 4,               // Original transaction ID width
    parameter W_ADDR          = 32,              // Address width (unused, kept for compatibility)
    parameter W_DATA          = 32,              // Data width
    parameter W_STRB          = (W_DATA/8),      // Strobe width (unused)
    parameter W_SID           = W_CID + W_ID,    // Full ID width including slave and master index
    parameter SLV_AMT         = 4,               // Number of slaves (Requester count for B/R channels)
    parameter MST_ID_FIELD_MSB = W_ID + W_CID - 1, // MSB of master index within ID (e.g., bit position)
    parameter MST_ID_FIELD_LSB = W_ID            // LSB of master index within ID
)(
    input  wire                      AXI_RSTn,
    input  wire                      AXI_CLK,

    // ========== Master side ports (single master) ==========
    // Write response channel (B)
    output reg   [W_ID-1:0]          M_BID,
    output reg   [1:0]               M_BRESP,
    output reg                       M_BVALID,
    input  wire                      M_BREADY,

    // Read data channel (R)
    output reg   [W_DATA-1:0]        M_RDATA,
    output reg   [1:0]               M_RRESP,
    output reg                       M_RLAST,
    output reg                       M_RVALID,
    input  wire                      M_RREADY,

    // ========== Slave side ports (packed arrays) ==========
    // B channel inputs from all slaves
    input  wire  [W_SID*SLV_AMT-1:0]  S_BID,
    input  wire  [2*SLV_AMT-1:0]      S_BRESP,
    input  wire  [SLV_AMT-1:0]        S_BVALID,
    output wire  [SLV_AMT-1:0]        S_BREADY,

    // R channel inputs from all slaves
    input  wire  [W_SID*SLV_AMT-1:0]  S_RID,
    input  wire  [W_DATA*SLV_AMT-1:0] S_RDATA,
    input  wire  [2*SLV_AMT-1:0]      S_RRESP,
    input  wire  [SLV_AMT-1:0]        S_RLAST,
    input  wire  [SLV_AMT-1:0]        S_RVALID,
    output wire  [SLV_AMT-1:0]        S_RREADY,

    // ========== Control ==========
    input  wire  [SLV_AMT-1:0]        r_order_grant,   // Per-slave grant for read reordering
    input  wire                       arbiter_type     // 0: RR, 1: Fixed (unused here, passed to sub-arbiters)
);

//=============================================================================
// Local parameters
//=============================================================================
localparam MST_ID_W = $clog2(SLV_AMT);   // Actually width of master index, should equal W_CID
// Consistency check (synthesis might ignore)
initial if (MST_ID_W != W_CID) $error("MST_ID_W must equal W_CID");

//=============================================================================
// Unpack slave arrays
//=============================================================================
wire [W_SID-1:0]   s_bid      [0:SLV_AMT-1];
wire [1:0]         s_bresp    [0:SLV_AMT-1];
wire               s_bvalid   [0:SLV_AMT-1];
wire               s_bready   [0:SLV_AMT-1];

wire [W_SID-1:0]   s_rid      [0:SLV_AMT-1];
wire [W_DATA-1:0]  s_rdata    [0:SLV_AMT-1];
wire [1:0]         s_rresp    [0:SLV_AMT-1];
wire               s_rlast    [0:SLV_AMT-1];
wire               s_rvalid   [0:SLV_AMT-1];
wire               s_rready   [0:SLV_AMT-1];

genvar si;
generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : UNPACK
        assign s_bid[si]    = S_BID[W_SID*si +: W_SID];
        assign s_bresp[si]  = S_BRESP[2*si +: 2];
        assign s_bvalid[si] = S_BVALID[si];
        assign S_BREADY[si] = s_bready[si];

        assign s_rid[si]    = S_RID[W_SID*si +: W_SID];
        assign s_rdata[si]  = S_RDATA[W_DATA*si +: W_DATA];
        assign s_rresp[si]  = S_RRESP[2*si +: 2];
        assign s_rlast[si]  = S_RLAST[si];
        assign s_rvalid[si] = S_RVALID[si];
        assign S_RREADY[si] = s_rready[si];
    end
endgenerate

//=============================================================================
// Extract master index from ID (compare with MASTER_ID)
// For both B and R channels, we need to know if the incoming response is for this master.
//=============================================================================
wire [SLV_AMT-1:0] b_sel, r_sel;   // One-hot per slave: valid response for this master

generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : DECODE_ID
        // Extract the master index field from the ID (assumed to be in fixed bit positions)
        wire [MST_ID_W-1:0] bid_mst_idx = s_bid[si][MST_ID_FIELD_MSB:MST_ID_FIELD_LSB];
        wire [MST_ID_W-1:0] rid_mst_idx = s_rid[si][MST_ID_FIELD_MSB:MST_ID_FIELD_LSB];
        assign b_sel[si] = (bid_mst_idx == MASTER_ID[MST_ID_W-1:0]) && s_bvalid[si];
        assign r_sel[si] = (rid_mst_idx == MASTER_ID[MST_ID_W-1:0]) && s_rvalid[si];
    end
endgenerate

//=============================================================================
// Arbitration for B channel (write responses)
// - Use simple priority/RR arbiter (axi_arbiter_param_rr) to select one slave at a time.
// - Handshake hold is NOT required here because the crossbar's S2M module is already
//   used per master; the upstream arbiter (axi_arbiter_s2m_s_amt) may hold grant.
//   For simplicity, we just mux based on the selected grant.
//=============================================================================
wire [SLV_AMT-1:0] b_grant, r_grant;

axi_arbiter_param_rr #(
    .NUM(SLV_AMT)
) u_arb_b (
    .clk          (AXI_CLK),
    .rst_n        (AXI_RSTn),
    .arbiter_type (arbiter_type),
    .req          (b_sel),
    .grant        (b_grant)
);

//=============================================================================
// B channel output mux and ready generation
// - Drive master B outputs based on the granted slave.
// - For slaves not granted, s_bready is deasserted.
//=============================================================================
localparam B_BUS_WIDTH = W_ID + 2 + 1;   // {BID, BRESP, BVALID}
wire [B_BUS_WIDTH-1:0] b_bus [0:SLV_AMT-1];

generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : PACK_B
        // BID needs to be stripped down to original W_ID bits (remove slave and master index)
        // The original ID is in the lower W_ID bits of s_bid[si].
        assign b_bus[si] = {s_bid[si][W_ID-1:0], s_bresp[si], s_bvalid[si]};
    end
endgenerate

`define M_B_BUS {M_BID, M_BRESP, M_BVALID}
always @(*) begin
    `M_B_BUS = 0;
    for(int i = 0; i < SLV_AMT; i++) begin
        if(b_grant[i]) begin
            {M_BID, M_BRESP, M_BVALID} = b_bus[i];
        end
    end
end

// Ready generation: back-propagate master BREADY to the granted slave
generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : B_READY_GEN
        assign s_bready[si] = b_grant[si] & M_BREADY;
    end
endgenerate

//=============================================================================
// Arbitration for R channel with reordering support
// - r_order_grant input is a per-slave vector indicating which slave's response
//   is allowed to be sent to the master at this cycle (reorder buffer lookup result).
// - We logically AND the valid responses (r_sel) with the order grant to only
//   consider slaves that are both ready and permitted by the reorder logic.
// - Then we run a standard RR/Fixed arbiter on the resulting request vector.
//=============================================================================
wire [SLV_AMT-1:0] r_req_ordered = r_sel & r_order_grant;

axi_arbiter_param_rr #(
    .NUM(SLV_AMT)
) u_arb_r (
    .clk          (AXI_CLK),
    .rst_n        (AXI_RSTn),
    .arbiter_type (arbiter_type),
    .req          (r_req_ordered),
    .grant        (r_grant)
);

//=============================================================================
// R channel output mux and ready generation
// - Drive master R outputs based on the granted slave.
// - For slaves not granted, s_rready is deasserted.
//=============================================================================
localparam R_BUS_WIDTH = W_DATA + 2 + 1 + 1;   // {RDATA, RRESP, RLAST, RVALID}
wire [R_BUS_WIDTH-1:0] r_bus [0:SLV_AMT-1];

generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : PACK_R
        assign r_bus[si] = {s_rdata[si], s_rresp[si], s_rlast[si], s_rvalid[si]};
    end
endgenerate

`define M_R_BUS {M_RDATA, M_RRESP, M_RLAST, M_RVALID}
always @(*) begin
    `M_R_BUS = 0;
    for(int i = 0; i < SLV_AMT; i++) begin
        if(r_grant[i]) begin
            {M_RDATA, M_RRESP, M_RLAST, M_RVALID} = r_bus[i];
        end
    end
end

// Ready generation: back-propagate master RREADY to the granted slave
generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : R_READY_GEN
        assign s_rready[si] = r_grant[si] & M_RREADY;
    end
endgenerate

endmodule