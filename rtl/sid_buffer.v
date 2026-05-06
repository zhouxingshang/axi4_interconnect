//=============================================================================
// Module: sid_buffer
// Desc  : Parameterized ordered SID buffer for AXI read reorder tracking
//         - NUM write ports (one per slave), parameterized ID width
//         - Single clear port (from master-side R channel completion)
//         - Stores ARIDs in FIFO order; compacted on clear (shift-down)
//         - Empty slot detection uses all-zeros sentinel
//=============================================================================
module sid_buffer #(
    parameter NUM   = 3,    // Number of write ports (= SLV_AMT)
    parameter W_ID  = 8,    // ID width (= W_SID)
    parameter DEPTH = 4     // Buffer depth (max outstanding AR transactions)
)(
    input   wire                        clk,
    input   wire                        rstn,

    // Write ports (packed, one per slave)
    input   wire  [W_ID*NUM-1   : 0]    s_axid,
    input   wire  [NUM-1        : 0]    s_axid_vld,
    input   wire  [NUM-1        : 0]    s_fifo_rdy,
    output  wire  [NUM-1        : 0]    s_push_rdy,

    // Single clear port (from master-side R channel completion)
    input   wire                        clr_last,
    input   wire  [W_ID-1        : 0]   clr_sid,
    input   wire                        clr_sid_vld,
    output  wire                        clr_rdy,

    // Buffer output (ordered SIDs, oldest at index 0)
    output wire  [W_ID-1        : 0]    sid_buffer [0:DEPTH-1]
);

//=============================================================================
// Local parameters
//=============================================================================
localparam IDX_W = $clog2(NUM);     // Index width for write port selection
localparam BUF_IDX_W = $clog2(DEPTH);

//=============================================================================
// Write arbitration signals
//=============================================================================
wire [NUM-1:0] push_select;
wire [NUM-1:0] push_grant;
wire           full;

//=============================================================================
// Clear signals
//=============================================================================
wire           clr_active;
wire [DEPTH-1:0] clr_match;
wire [BUF_IDX_W-1:0] clr_match_idx;
wire           clr_match_valid;

//=============================================================================
// Write arbitration: priority select among write requestors
//=============================================================================
generate
    genvar pi;
    for (pi = 0; pi < NUM; pi = pi + 1) begin : GEN_PUSH_SELECT
        assign push_select[pi] = s_axid_vld[pi] & s_fifo_rdy[pi];
    end
endgenerate

assign push_grant = priority_sel(push_select);

// Write allowed only when not full and no clear in progress
assign full = (sid_buffer[DEPTH-1] != {W_ID{1'b0}});
assign clr_active = clr_sid_vld & clr_last;
wire write_allowed = ~full & ~clr_active;

// Push ready: this port won arbitration AND write is allowed
generate
    for (pi = 0; pi < NUM; pi = pi + 1) begin : GEN_PUSH_RDY
        assign s_push_rdy[pi] = push_grant[pi] & write_allowed;
    end
endgenerate

//=============================================================================
// Write logic: find first empty slot, write granted ID
//=============================================================================
wire [BUF_IDX_W-1:0] write_slot;

// Find first empty slot (all-zeros = empty)
function [BUF_IDX_W-1:0] find_empty_slot;
    input [W_ID-1:0] buf [0:DEPTH-1];
    integer ei;
    begin
        find_empty_slot = 0;
        for (ei = 0; ei < DEPTH; ei = ei + 1) begin
            if (buf[ei] == {W_ID{1'b0}}) begin
                find_empty_slot = ei[BUF_IDX_W-1:0];
                ei = DEPTH;  // break
            end
        end
    end
endfunction

assign write_slot = find_empty_slot(sid_buffer);

// Extract the winning write port's ID
wire [W_ID-1:0] wr_axid;
wire [IDX_W-1:0] push_grant_idx;

function [IDX_W-1:0] onehot_to_bin;
    input [NUM-1:0] onehot;
    integer oi;
    begin
        onehot_to_bin = 0;
        for (oi = 0; oi < NUM; oi = oi + 1) begin
            if (onehot[oi]) onehot_to_bin = oi[IDX_W-1:0];
        end
    end
endfunction

assign push_grant_idx = onehot_to_bin(push_grant);
assign wr_axid = s_axid[push_grant_idx * W_ID +: W_ID];

//=============================================================================
// Clear logic: find matching entry for clear SID
//=============================================================================
generate
    genvar ci;
    for (ci = 0; ci < DEPTH; ci = ci + 1) begin : GEN_CLR_MATCH
        assign clr_match[ci] = clr_active & (clr_sid == sid_buffer[ci]);
    end
endgenerate

// Find first matching index
function [BUF_IDX_W-1:0] find_match_idx;
    input [DEPTH-1:0] match_vec;
    integer mi;
    begin
        find_match_idx = 0;
        for (mi = 0; mi < DEPTH; mi = mi + 1) begin
            if (match_vec[mi]) begin
                find_match_idx = mi[BUF_IDX_W-1:0];
                mi = DEPTH;  // break
            end
        end
    end
endfunction

assign clr_match_idx = find_match_idx(clr_match);
assign clr_match_valid = |clr_match;
assign clr_rdy = clr_match_valid | ~clr_active;  // ready if match found or no clear requested

//=============================================================================
// Sequential: write and clear (mutually exclusive per cycle)
//=============================================================================
integer si, sj;
always @(posedge clk) begin
    if (!rstn) begin
        for (si = 0; si < DEPTH; si = si + 1) begin
            sid_buffer[si] <= {W_ID{1'b0}};
        end
    end
    else if (clr_match_valid) begin
        // Clear: shift entries above match down by one
        for (si = 0; si < DEPTH; si = si + 1) begin
            if (si < clr_match_idx) begin
                sid_buffer[si] <= sid_buffer[si];
            end else if (si < DEPTH - 1) begin
                sid_buffer[si] <= sid_buffer[si + 1];
            end else begin
                sid_buffer[si] <= {W_ID{1'b0}};
            end
        end
    end
    else if (|push_grant & write_allowed) begin
        // Write to first empty slot
        for (si = 0; si < DEPTH; si = si + 1) begin
            if (si == write_slot) begin
                sid_buffer[si] <= wr_axid;
            end
        end
    end
end

//=============================================================================
// Priority selector: fixed priority, bit 0 highest
//=============================================================================
function [NUM-1:0] priority_sel;
    input [NUM-1:0] request;
    integer pi;
    begin
        priority_sel = {NUM{1'b0}};
        for (pi = 0; pi < NUM; pi = pi + 1) begin
            if (request[pi]) begin
                priority_sel[pi] = 1'b1;
                pi = NUM;  // break: first requestor wins
            end
        end
    end
endfunction

endmodule
