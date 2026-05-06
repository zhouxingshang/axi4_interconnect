//=============================================================================
// Module: reorder
// Desc  : Parameterized read response reorder controller
//         - NUM input ports (one per slave), parameterized ID width
//         - Compares incoming S_RID against rob_buffer (expected order from sid_buffer)
//         - Grants only the slave whose RID matches the oldest outstanding AR
//         - Enforces in-order read response delivery to master
//=============================================================================
module reorder #(
    parameter NUM   = 3,    // Number of slave input ports (= SLV_AMT)
    parameter W_ID  = 8,    // ID width (= W_SID)
    parameter DEPTH = 4     // rob_buffer depth (= sid_buffer DEPTH)
)(
    input   wire                        clk,
    input   wire                        rstn,

    // Slave-side RID inputs (packed, one per slave)
    input   wire  [W_ID*NUM-1   : 0]    s_sid,          // S_RID per slave
    input   wire  [NUM-1        : 0]    s_sid_vld,      // S_RVALID per slave

    // Expected order buffer (from sid_buffer, oldest at index 0)
    input   wire  [W_ID-1       : 0]    rob_buffer [0:DEPTH-1],

    // Grant output: one-hot per slave (only one bit set at a time)
    output  reg   [NUM-1        : 0]    order_grant
);

//=============================================================================
// Match detection: per slave, per buffer entry
//=============================================================================
wire [DEPTH-1:0] rid_match [0:NUM-1];  // rid_match[s][j] = 1 if slave s's RID matches rob_buffer[j]

genvar s, j;
generate
    for (s = 0; s < NUM; s = s + 1) begin : GEN_MATCH_S
        for (j = 0; j < DEPTH; j = j + 1) begin : GEN_MATCH_J
            assign rid_match[s][j] = s_sid_vld[s] &&
                                     (s_sid[s*W_ID +: W_ID] == rob_buffer[j]) &&
                                     (rob_buffer[j] != {W_ID{1'b0}});  // ignore empty slots
        end
    end
endgenerate

//=============================================================================
// First-match detection per slave
// first_match[s][j] = 1 if j is the earliest buffer position matching slave s
//=============================================================================
wire [DEPTH-1:0] first_match [0:NUM-1];

generate
    for (s = 0; s < NUM; s = s + 1) begin : GEN_FIRST_MATCH
        assign first_match[s][0] = rid_match[s][0];
        for (j = 1; j < DEPTH; j = j + 1) begin : GEN_FIRST_J
            assign first_match[s][j] = rid_match[s][j] & ~(|rid_match[s][j-1:0]);
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
