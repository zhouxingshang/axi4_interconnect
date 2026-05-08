//=============================================================================
// Module: reorder
// Desc  : Parameterized read response reorder controller
//         - NUM input ports (one per slave), parameterized ID width
//         - Compares incoming S_RID against rob_buffer (expected order from sid_buffer)
//         - Grants only the slave whose RID matches the oldest outstanding AR
//         - Enforces in-order read response delivery to master
//=============================================================================
module reorder #(
    parameter NUM    = 3,    // Number of slave input ports (= SLV_AMT)
    parameter M_ID_W = 4,    // Master ID Width
    parameter W_ID   = 4,
    parameter W_SID  = M_ID_W + W_ID,    // ID width (= W_SID)
    parameter DEPTH  = 4     // rob_buffer depth (= sid_buffer DEPTH)
)(
    input   wire                        clk,
    input   wire                        rstn,

    // Slave-side RID inputs (packed, one per slave)
    input   wire  [W_SID*NUM-1   : 0]    s_sid,          // S_RID per slave
    input   wire  [NUM-1        : 0]    s_sid_vld,      // S_RVALID per slave

    // Expected order buffer (from sid_buffer, oldest at index 0)
    input   wire  [W_SID-1       : 0]    rob_buffer [0:DEPTH-1],

    // Grant output: one-hot per slave (only one bit set at a time)
    output  reg   [NUM-1        : 0]    order_grant
);

//=============================================================================
// Match detection: per slave, per buffer entry, high and low bits separately
// ID format: {mst_idx[M_ID_W-1:0], original_id[W_ID-1:0]}
//   rid_high: compare high M_ID_W bits (master index)
//   rid_low:  compare low  W_ID   bits (original transaction ID)
//=============================================================================
wire [DEPTH-1:0] rid_high [0:NUM-1];
wire [DEPTH-1:0] rid_low  [0:NUM-1];
wire [DEPTH-1:0] full_match [0:NUM-1];

genvar s, j;
generate
    for (s = 0; s < NUM; s = s + 1) begin : GEN_MATCH_S
        for (j = 0; j < DEPTH; j = j + 1) begin : GEN_MATCH_J
            // High bits: M_ID_W bits at top of slave s's SID slice
            assign rid_high[s][j] = s_sid_vld[s] && (s_sid[(s+1)*W_SID-1 -: M_ID_W] == rob_buffer[j][W_SID-1 -: M_ID_W]);
            // Low bits: W_ID bits at bottom of slave s's SID slice
            assign rid_low[s][j] = s_sid_vld[s] && (s_sid[s*W_SID +: W_ID] == rob_buffer[j][W_ID-1:0]);
            // Full match: both high AND low match at same position
            assign full_match[s][j] = rid_high[s][j] && rid_low[s][j];
        end
    end
endgenerate

//=============================================================================
// First-match detection per slave (rf-style priority encoding)
// first_match[s][j] = 1 if j is the earliest buffer position with full match
// Pattern: position j matches AND all positions < j have no match
//=============================================================================
wire [DEPTH-1:0] first_match [0:NUM-1];

generate
    for (s = 0; s < NUM; s = s + 1) begin : GEN_FIRST_MATCH
        assign first_match[s][0] = full_match[s][0];
        for (j = 1; j < DEPTH; j = j + 1) begin : GEN_FIRST_J
            assign first_match[s][j] = full_match[s][j] && ~(|full_match[s][j-1:0]);
        end
    end
endgenerate

//=============================================================================
// Combinational grant: slave s wins if it matches the oldest entry
// Multiple slaves cannot match the same oldest entry (unique RIDs)
//=============================================================================
wire [NUM-1:0] order_grant_comb;

generate
    for (s = 0; s < NUM; s = s + 1) begin : GEN_GRANT_COMB
        assign order_grant_comb[s] = |first_match[s];
    end
endgenerate

//=============================================================================
// Registered grant output
//=============================================================================
always @(posedge clk) begin
    if (!rstn) begin
        order_grant <= {NUM{1'b0}};
    end else begin
        order_grant <= order_grant_comb;
    end
end

endmodule
