//=============================================================================
// Module: sid_buffer
// Desc  : Parameterized ordered SID buffer for AXI read reorder tracking
//         - NUM_WR write ports (one per slave, = SLV_AMT)
//         - NUM_CLR clear ports (one per master, = MST_AMT)
//         - Write logic (rf-style): s_axid_vld & s_fifo_rdy → push_select
//           → priority_sel_wr → push_grant → XNOR → s_push_rdy
//         - Clear logic (rf-style): clr_last & clr_sid_vld → clr_select
//           → priority_sel_clr → clr_grant → XNOR → s_clr_rdy
//         - Low-index ports have higher priority in both write and clear arb
//         - Stores ARIDs in FIFO order; compacted on clear (shift-down)
//         - Empty slot detection uses all-zeros sentinel
//=============================================================================
module sid_buffer #(
    parameter NUM_WR = 3,    // Write ports  (= SLV_AMT)
    parameter NUM_CLR= 3,    // Clear ports  (= MST_AMT)
    parameter W_ID   = 8,    // ID width     (= W_SID)
    parameter DEPTH  = 4     // Buffer depth (max outstanding AR transactions)
)(
    input   wire                        clk,
    input   wire                        rstn,

    //----------------Write buffer-------------------
    input   wire  [W_ID*NUM_WR-1 : 0]    s_axid,
    input   wire  [NUM_WR-1      : 0]    s_axid_vld,
    input   wire  [NUM_WR-1      : 0]    s_fifo_rdy,
    output  wire  [NUM_WR-1      : 0]    s_push_rdy,

    //--------Clean buffer and ajust position--------
    input   wire  [NUM_CLR-1     : 0]    clr_last,
    input   wire  [W_ID*NUM_CLR-1: 0]    clr_sid,
    input   wire  [NUM_CLR-1     : 0]    clr_sid_vld,
    output  wire  [NUM_CLR-1     : 0]    s_clr_rdy,

    // Buffer output (ordered SIDs, oldest at index 0)
    output  reg   [W_ID-1        : 0]    sid_buffer [0:DEPTH-1]
);

//=============================================================================
// Local parameters
//=============================================================================
localparam WR_IDX_W  = $clog2(NUM_WR);   // Index width for write port selection
localparam CLR_IDX_W = $clog2(NUM_CLR);  // Index width for clear port selection
localparam BUF_IDX_W = $clog2(DEPTH);    // Index width for buffer entry

//=============================================================================
// Write buffer signals
//=============================================================================
reg  [NUM_WR-1:0]      push_select;
wire [NUM_WR-1:0]      push_grant;
wire                   full;
wire                   clr_allowed;       // rf-internal: (~|clr_grant) && (~full)

//=============================================================================
// Clear buffer signals
//=============================================================================
wire [NUM_CLR-1:0]     clr_select;
wire [NUM_CLR-1:0]     clr_grant;
wire [DEPTH-1:0]       clr_idx;

//=============================================================================
// Write buffer: push_select → priority_sel_wr → push_grant
//=============================================================================
always @(*) begin
    if (!rstn) begin
        push_select = {NUM_WR{1'b0}};
    end else begin
        for (int pi = 0; pi < NUM_WR; pi = pi + 1) begin
            push_select[pi] = s_axid_vld[pi] & s_fifo_rdy[pi];
        end
    end
end

assign push_grant = priority_sel_wr(push_select);

//=============================================================================
// Write allowed: only when no clear in progress AND buffer not full
//=============================================================================
assign full  = (sid_buffer[DEPTH-1] != {W_ID{1'b0}});
assign clr_allowed = (~|clr_grant) && (~full);

//=============================================================================
// Push ready: XNOR of push_select and push_grant, qualified by clr_allowed
//=============================================================================
generate
    genvar pi_rdy;
    for (pi_rdy = 0; pi_rdy < NUM_WR; pi_rdy = pi_rdy + 1) begin : GEN_PUSH_RDY
        assign s_push_rdy[pi_rdy] = ~(push_select[pi_rdy] ^ push_grant[pi_rdy]) && clr_allowed;
    end
endgenerate

//=============================================================================
// Write to buffer: first empty slot
//=============================================================================
always @(posedge clk) begin
    if (!rstn) begin
        for (int si = 0; si < DEPTH; si = si + 1) begin
            sid_buffer[si] <= {W_ID{1'b0}};
        end
    end
    else if (|clr_idx) begin
        // Clear has priority; handled below
    end
    else begin
        for (int pi = 0; pi < NUM_WR; pi = pi + 1) begin
            if (push_grant[pi] && s_push_rdy[pi]) begin
                for (int si = 0; si < DEPTH; si = si + 1) begin
                    if (sid_buffer[si] == {W_ID{1'b0}}) begin
                        sid_buffer[si] <= s_axid[pi * W_ID +: W_ID];
                        break;
                    end
                end
            end
        end
    end
end

//=============================================================================
// Clean buffer: clr_select → priority_sel_clr → clr_grant
//=============================================================================
generate
    genvar ci_sel;
    for (ci_sel = 0; ci_sel < NUM_CLR; ci_sel = ci_sel + 1) begin : GEN_CLR_SELECT
        assign clr_select[ci_sel] = clr_last[ci_sel] & clr_sid_vld[ci_sel];
    end
endgenerate

assign clr_grant = priority_sel_clr(clr_select);

//=============================================================================
// Clear ready: XNOR of clr_select and clr_grant per port
//=============================================================================
generate
    genvar ci_rdy;
    for (ci_rdy = 0; ci_rdy < NUM_CLR; ci_rdy = ci_rdy + 1) begin : GEN_CLR_RDY
        assign s_clr_rdy[ci_rdy] = ~(clr_select[ci_rdy] ^ clr_grant[ci_rdy]);
    end
endgenerate

//=============================================================================
// Clear index: find which buffer slot matches the granted clear port's SID
//=============================================================================
generate
    genvar cii, cp;
    for (cii = 0; cii < DEPTH; cii = cii + 1) begin : GEN_CLR_IDX
        wire [NUM_CLR-1:0] match_per_port;
        for (cp = 0; cp < NUM_CLR; cp = cp + 1) begin : GEN_MATCH_PORT
            assign match_per_port[cp] = clr_grant[cp] && (clr_sid[cp * W_ID +: W_ID] == sid_buffer[cii]);
        end
        assign clr_idx[cii] = |match_per_port;
    end
endgenerate

//=============================================================================
// Shift-down compaction on clear
//=============================================================================
always @(posedge clk) begin
    if (clr_idx[0]) begin
        sid_buffer[0] <= sid_buffer[1];
        sid_buffer[1] <= sid_buffer[2];
        sid_buffer[2] <= sid_buffer[3];
        sid_buffer[3] <= {W_ID{1'b0}};
    end
    else if (clr_idx[1]) begin
        sid_buffer[1] <= sid_buffer[2];
        sid_buffer[2] <= sid_buffer[3];
        sid_buffer[3] <= {W_ID{1'b0}};
    end
    else if (clr_idx[2]) begin
        sid_buffer[2] <= sid_buffer[3];
        sid_buffer[3] <= {W_ID{1'b0}};
    end
    else if (clr_idx[3]) begin
        sid_buffer[3] <= {W_ID{1'b0}};
    end
end

//=============================================================================
// Priority selectors: fixed priority, bit 0 highest (lowest index wins)
//=============================================================================
function [NUM_WR-1:0] priority_sel_wr;
    input [NUM_WR-1:0] request;
    integer pi;
    begin
        priority_sel_wr = {NUM_WR{1'b0}};
        for (pi = 0; pi < NUM_WR; pi = pi + 1) begin
            if (request[pi]) begin
                priority_sel_wr[pi] = 1'b1;
                pi = NUM_WR;
            end
        end
    end
endfunction

function [NUM_CLR-1:0] priority_sel_clr;
    input [NUM_CLR-1:0] request;
    integer pi;
    begin
        priority_sel_clr = {NUM_CLR{1'b0}};
        for (pi = 0; pi < NUM_CLR; pi = pi + 1) begin
            if (request[pi]) begin
                priority_sel_clr[pi] = 1'b1;
                pi = NUM_CLR;
            end
        end
    end
endfunction

endmodule
