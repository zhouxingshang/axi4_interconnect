//=============================================================================
// axi_split_b_merge — B-channel response merger for 4KB-split write transactions
//=============================================================================
// BID = {mst_idx[M_ID_W-1:0], split_st[1:0], orig_id[W_ID-1:0]}
//   M_ID_W   — master index from crossbar, $clog2(MST_AMT)
//   split_st — 2-bit split status (set by cross_4k_if):
//     2'b00 : non-split  → passthrough (strip mst_idx+split_st, forward)
//     2'b01 : split sub-transaction 1 → absorb, accumulate BRESP
//     2'b10 : split sub-transaction 2 → absorb, accumulate BRESP
//
// Both sub-transactions share the same lower W_ID bits.  Table entries are
// allocated on first split B arrival (demand-driven, no AW-side push needed).
// Final BRESP = bitwise OR of both sub-responses:
//   either fails → whole transaction fails.
//=============================================================================
module axi_split_b_merge #(
    parameter W_ID       = 4,             // original transaction ID width
    parameter M_ID_W     = 2,             // master index width, $clog2(MST_AMT)
    parameter MAX_SPLIT  = 4              // max concurrent split transactions
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // ---- B channel slave side (from crossbar / S2M) ----
    input  wire                 s_axi_bvalid,
    input  wire [M_ID_W+W_ID+1:0] s_axi_bid,    // {mst_idx, split_st[1:0], orig_id}
    input  wire [1:0]           s_axi_bresp,
    output wire                 s_axi_bready,

    // ---- B channel master side (to master) ----
    output reg                  m_axi_bvalid,
    output reg  [W_ID-1:0]      m_axi_bid,       // mst_idx + split_st stripped
    output reg  [1:0]           m_axi_bresp,
    input  wire                 m_axi_bready
);

    //=========================================================================
    // BID field extraction
    //  split_st at [W_ID+1:W_ID], orig_id at [W_ID-1:0]
    //=========================================================================
    wire [1:0]      b_split_st  = s_axi_bid[W_ID+1:W_ID];
    wire [W_ID-1:0] b_strip_id  = s_axi_bid[W_ID-1:0];
    wire            b_is_trans1 = (b_split_st == 2'b01);
    wire            b_is_trans2 = (b_split_st == 2'b10);
    wire            b_is_split  = b_is_trans1 || b_is_trans2;

    //=========================================================================
    // Table entry registers
    //   entry_valid[i]    — slot occupied
    //   entry_orig_id[i]  — original AWID (lower W_ID bits, shared by both subs)
    //   entry_got_t1[i]   — sub-transaction 1 B received (prefix = 01)
    //   entry_got_t2[i]   — sub-transaction 2 B received (prefix = 10)
    //   entry_resp[i]     — accumulated BRESP (bitwise OR)
    //=========================================================================
    reg  [MAX_SPLIT-1:0]        entry_valid;
    reg  [MAX_SPLIT-1:0]        entry_got_t1;
    reg  [MAX_SPLIT-1:0]        entry_got_t2;
    reg  [W_ID-1:0]             entry_orig_id [0:MAX_SPLIT-1];
    reg  [1:0]                  entry_resp    [0:MAX_SPLIT-1];

    //=========================================================================
    // Combinational: key match against all table entries
    //   Both sub-transactions share the same lower W_ID bits, so a single
    //   key_match per entry covers both trans1 and trans2.
    //=========================================================================
    wire [MAX_SPLIT-1:0] key_match;    // entry_orig_id == b_strip_id
    wire [MAX_SPLIT-1:0] entry_done;   // got_t1 && got_t2 → ready to merge

    genvar gi;
    generate
        for (gi = 0; gi < MAX_SPLIT; gi = gi + 1) begin : GEN_MATCH
            assign key_match[gi]  = entry_valid[gi] &&
                                    (entry_orig_id[gi] == b_strip_id);
            assign entry_done[gi] = entry_valid[gi] &&
                                    entry_got_t1[gi] && entry_got_t2[gi];
        end
    endgenerate

    wire has_key_match = |key_match;
    wire has_pending   = |entry_done;

    //=========================================================================
    // Combinational: index finders (priority: lowest index wins)
    //   alloc_idx — first free slot (for new split B)
    //   done_idx  — first complete entry (both B received, ready to forward)
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

    wire has_free = ~(&entry_valid);

    //=========================================================================
    // B channel outputs (combinational)
    //=========================================================================
    // Priority:
    //   1. Forward merged B when any entry is complete (got_t1 && got_t2)
    //   2. Absorb split B when key matches (update table, hide from master)
    //   3. Passthrough non-split B (prefix = 00)
    //
    // s_axi_bready:
    //   - backpressure during merged B presentation
    //   - always ready when absorbing a matching split B
    //   - always ready when allocating a new split B (has_free)
    //   - passthrough of m_axi_bready for non-split B
    //=========================================================================

    wire can_accept = b_is_split && (has_key_match || has_free);

    assign s_axi_bready = has_pending      ? 1'b0             // wait: merged B first
                        : can_accept       ? 1'b1             // absorb / allocate
                        :                    m_axi_bready;    // non-split passthrough

    always @(*) begin
        if (has_pending) begin
            // Forward merged B from the lowest-index complete entry
            m_axi_bid    = entry_orig_id[done_idx];
            m_axi_bresp  = entry_resp[done_idx];
            m_axi_bvalid = 1'b1;
        end else if (!b_is_split) begin
            // Non-split B: strip prefix, passthrough
            m_axi_bid    = b_strip_id;
            m_axi_bresp  = s_axi_bresp;
            m_axi_bvalid = s_axi_bvalid;
        end else begin
            // Split B: absorbing / allocating, hide from master
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
            // —— deallocation: merged B accepted by master ——
            if (has_pending && m_axi_bready) begin
                entry_valid[done_idx] <= 1'b0;
            end

            // —— absorption / allocation: split B handshake ——
            if (s_axi_bvalid && s_axi_bready && b_is_split) begin
                if (!has_key_match && has_free) begin
                    // First split B seen for this orig_id → allocate new entry
                    entry_valid[alloc_idx]   <= 1'b1;
                    entry_orig_id[alloc_idx] <= b_strip_id;
                    entry_got_t1[alloc_idx]  <= b_is_trans1;
                    entry_got_t2[alloc_idx]  <= b_is_trans2;
                    entry_resp[alloc_idx]    <= s_axi_bresp;
                end else if (has_key_match) begin
                    // Update existing entry
                    for (int i = 0; i < MAX_SPLIT; i++) begin
                        if (key_match[i]) begin
                            if (b_is_trans1) begin
                                entry_got_t1[i] <= 1'b1;
                                entry_resp[i]   <= entry_resp[i] | s_axi_bresp;
                            end
                            if (b_is_trans2) begin
                                entry_got_t2[i] <= 1'b1;
                                entry_resp[i]   <= entry_resp[i] | s_axi_bresp;
                            end
                        end
                    end
                end
                // else: key not found & no free slot → drop (should not happen
                //       if MAX_SPLIT ≥ outstanding split transactions)
            end
        end
    end

endmodule
