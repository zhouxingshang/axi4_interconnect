//=============================================================================
// axi_split_b_merge — B-channel response merger for 4KB-split write transactions
//=============================================================================
// Each split write produces two sub-transactions with AWIDs {orig_id, orig_id+1}.
// Their B responses may arrive in any order (different slaves).  This module
// tracks outstanding split transactions in a register table, accumulates BRESP
// by bitwise OR, and forwards a single merged B after both sub-responses arrive.
//
// Non-split B responses (BID not in table) pass through transparently.
// AW-side decoupling: cross_4k_if pushes {orig_id} via split_info_* and
// immediately continues — no waiting for B completion.
//=============================================================================
module axi_split_b_merge #(
    parameter W_ID       = 4,             // transaction ID width
    parameter MAX_SPLIT  = 4              // max concurrent split transactions
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // ---- split info push (from cross_4k_if AW side) ----
    input  wire                 split_info_valid,
    output wire                 split_info_ready,
    input  wire [W_ID-1:0]      split_info_orig_id,

    // ---- B channel slave side (from crossbar / S2M) ----
    input  wire                 s_axi_bvalid,
    input  wire [W_ID-1:0]      s_axi_bid,
    input  wire [1:0]           s_axi_bresp,
    output wire                 s_axi_bready,

    // ---- B channel master side (to master) ----
    output reg                  m_axi_bvalid,
    output reg  [W_ID-1:0]      m_axi_bid,
    output reg  [1:0]           m_axi_bresp,
    input  wire                 m_axi_bready
);

    //=========================================================================
    // Table entry registers
    //   entry_valid[i]     — slot occupied
    //   entry_orig_id[i]   — original AWID  (sub-t1 = orig_id, sub-t2 = orig_id+1)
    //   entry_got_t1[i]    — sub-transaction 1 B received
    //   entry_got_t2[i]    — sub-transaction 2 B received
    //   entry_resp[i]      — accumulated BRESP (bitwise OR)
    //=========================================================================
    reg  [MAX_SPLIT-1:0]        entry_valid;
    reg  [MAX_SPLIT-1:0]        entry_got_t1;
    reg  [MAX_SPLIT-1:0]        entry_got_t2;
    reg  [W_ID-1:0]             entry_orig_id [0:MAX_SPLIT-1];
    reg  [1:0]                  entry_resp    [0:MAX_SPLIT-1];

    //=========================================================================
    // Combinational: BID matching against all table entries
    //=========================================================================
    wire [MAX_SPLIT-1:0] b_match_t1;   // s_axi_bid == orig_id
    wire [MAX_SPLIT-1:0] b_match_t2;   // s_axi_bid == orig_id + 1
    wire [MAX_SPLIT-1:0] b_match_any;
    wire [MAX_SPLIT-1:0] entry_done;   // got_t1 && got_t2 → ready to forward

    genvar gi;
    generate
        for (gi = 0; gi < MAX_SPLIT; gi = gi + 1) begin : GEN_MATCH
            assign b_match_t1[gi] = entry_valid[gi] &&
                                    (entry_orig_id[gi] == s_axi_bid);
            assign b_match_t2[gi] = entry_valid[gi] &&
                                    (entry_orig_id[gi] + 1'b1 == s_axi_bid);
            assign entry_done[gi] = entry_valid[gi] &&
                                    entry_got_t1[gi] && entry_got_t2[gi];
        end
    endgenerate

    assign b_match_any = b_match_t1 | b_match_t2;

    //=========================================================================
    // Combinational: index finders (priority: lowest index wins)
    //   alloc_idx    — first free slot
    //   done_idx     — first complete entry (both B received)
    //=========================================================================
    integer alloc_idx;
    integer done_idx;

    always @(*) begin
        alloc_idx = 0;
        for (int i = MAX_SPLIT-1; i >= 0; i--) begin
            if (!entry_valid[i])
                alloc_idx = i;
        end
    end

    always @(*) begin
        done_idx = 0;
        for (int i = MAX_SPLIT-1; i >= 0; i--) begin
            if (entry_done[i])
                done_idx = i;
        end
    end

    wire has_free    = ~(&entry_valid);
    wire has_pending = |entry_done;

    assign split_info_ready = has_free;

    //=========================================================================
    // B channel outputs (combinational)
    //=========================================================================
    // Priority:
    //   1. Forward merged B when any entry has both responses
    //   2. Passthrough non-split B when BID doesn't match any entry
    //   3. Absorb (m_axi_bvalid = 0) when BID matches a pending entry
    //
    // s_axi_bready:
    //   - backpressure during merged B presentation
    //   - always ready when absorbing a matching entry
    //   - passthrough of m_axi_bready otherwise
    //=========================================================================

    assign s_axi_bready = has_pending      ? 1'b0             // wait, merged B first
                        : (|b_match_any)   ? 1'b1             // absorb
                        :                    m_axi_bready;    // passthrough

    always @(*) begin
        if (has_pending) begin
            // Forward merged B from the lowest-index complete entry
            m_axi_bid    = entry_orig_id[done_idx];
            m_axi_bresp  = entry_resp[done_idx];
            m_axi_bvalid = 1'b1;
        end else if (!(|b_match_any)) begin
            // Passthrough: BID not in table → not a split transaction
            m_axi_bid    = s_axi_bid;
            m_axi_bresp  = s_axi_bresp;
            m_axi_bvalid = s_axi_bvalid;
        end else begin
            // Absorbing: BID matches, update table, hide from master
            m_axi_bid    = {W_ID{1'b0}};
            m_axi_bresp  = 2'b00;
            m_axi_bvalid = 1'b0;
        end
    end

    //=========================================================================
    // Table update (sequential)
    //=========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            entry_valid  <= {MAX_SPLIT{1'b0}};
            entry_got_t1 <= {MAX_SPLIT{1'b0}};
            entry_got_t2 <= {MAX_SPLIT{1'b0}};
        end else begin
            // —— allocation ——
            if (split_info_valid && split_info_ready) begin
                entry_valid[alloc_idx]   <= 1'b1;
                entry_orig_id[alloc_idx] <= split_info_orig_id;
                entry_got_t1[alloc_idx]  <= 1'b0;
                entry_got_t2[alloc_idx]  <= 1'b0;
                entry_resp[alloc_idx]    <= 2'b00;
            end

            // —— absorption: BID matches an active entry ——
            if (s_axi_bvalid && s_axi_bready && |b_match_any) begin
                for (int i = 0; i < MAX_SPLIT; i++) begin
                    if (b_match_t1[i]) begin
                        entry_got_t1[i] <= 1'b1;
                        entry_resp[i]   <= entry_resp[i] | s_axi_bresp;
                    end
                    if (b_match_t2[i]) begin
                        entry_got_t2[i] <= 1'b1;
                        entry_resp[i]   <= entry_resp[i] | s_axi_bresp;
                    end
                end
            end

            // —— deallocation: merged B accepted by master ——
            if (has_pending && m_axi_bvalid && m_axi_bready) begin
                entry_valid[done_idx] <= 1'b0;
            end
        end
    end

endmodule
