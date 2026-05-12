//=============================================================================
// axi_split_r_merge — R-channel merger for 4KB-split read transactions
//=============================================================================
// RID = {prefix[1:0], orig_id[W_ID-1:0]}  (set by cross_4k_if AR split)
//   2'b00 — non-split  → passthrough (strip prefix)
//   2'b01 — split sub-transaction 1 → suppress RLAST, accumulate RRESP
//   2'b10 — split sub-transaction 2 → keep RLAST, accumulate RRESP, finalize
//
// Sub-transaction 1's last beat has RLAST=1 from the slave; this module forces
// it to 0 so the master sees a single continuous read burst.
// Sub-transaction 2's last beat keeps RLAST=1 with the accumulated RRESP
// (bitwise OR of both sub-transactions: either fails → whole fails).
//
// Reordering: the two sub-transactions go to different slaves and their R data
// may arrive out of order.  If sub2 arrives before sub1_done, it is stalled
// (s_axi_rready=0) until sub1 completes.  This requires that the S2M or
// cross_4k_if AR state machine ensures ordering, otherwise deadlock is possible.
//=============================================================================
module axi_split_r_merge #(
    parameter W_ID       = 4,             // original transaction ID width
    parameter W_DATA     = 32,            // data bus width
    parameter W_STRB     = W_DATA / 8,    // not used for R channel
    parameter MAX_SPLIT  = 4              // max concurrent split read transactions
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // ---- R channel slave side (from crossbar / S2M / post-FIFO) ----
    input  wire                 s_axi_rvalid,
    input  wire [W_ID+1:0]      s_axi_rid,        // {prefix[1:0], orig_id}
    input  wire [W_DATA-1:0]    s_axi_rdata,
    input  wire [1:0]           s_axi_rresp,
    input  wire                 s_axi_rlast,
    output wire                 s_axi_rready,

    // ---- R channel master side (to master) ----
    output reg                  m_axi_rvalid,
    output reg  [W_ID-1:0]      m_axi_rid,        // prefix stripped
    output reg  [W_DATA-1:0]    m_axi_rdata,
    output reg  [1:0]           m_axi_rresp,
    output reg                  m_axi_rlast,
    input  wire                 m_axi_rready
);

    //=========================================================================
    // RID field extraction
    //=========================================================================
    wire [1:0]      r_prefix    = s_axi_rid[W_ID+1:W_ID];
    wire [W_ID-1:0] r_strip_id  = s_axi_rid[W_ID-1:0];
    wire            r_is_trans1  = (r_prefix == 2'b01);
    wire            r_is_trans2  = (r_prefix == 2'b10);
    wire            r_is_split   = r_is_trans1 || r_is_trans2;

    //=========================================================================
    // Table entry registers
    //   entry_valid[i]    — slot occupied
    //   entry_orig_id[i]  — original ARID (lower W_ID bits, shared key)
    //   entry_sub1_done[i] — sub-transaction 1 R fully received (RLAST seen)
    //   entry_acc_resp[i] — accumulated RRESP (bitwise OR across both subs)
    //=========================================================================
    reg  [MAX_SPLIT-1:0]        entry_valid;
    reg  [MAX_SPLIT-1:0]        entry_sub1_done;
    reg  [W_ID-1:0]             entry_orig_id [0:MAX_SPLIT-1];
    reg  [1:0]                  entry_acc_resp [0:MAX_SPLIT-1];

    //=========================================================================
    // Combinational: key match against all table entries
    //=========================================================================
    wire [MAX_SPLIT-1:0] key_match;      // entry matches r_strip_id

    genvar gi;
    generate
        for (gi = 0; gi < MAX_SPLIT; gi = gi + 1) begin : GEN_MATCH
            assign key_match[gi] = entry_valid[gi] &&
                                   (entry_orig_id[gi] == r_strip_id);
        end
    endgenerate

    wire has_key_match = |key_match;

    //=========================================================================
    // Combinational: index finders
    //=========================================================================
    integer alloc_idx;
    integer match_idx;

    always @(*) begin
        alloc_idx = 0;
        for (int i = MAX_SPLIT-1; i >= 0; i--) begin
            if (!entry_valid[i])
                alloc_idx = i;
        end
    end

    // Find the matching entry index (for response accumulation)
    always @(*) begin
        match_idx = 0;
        for (int i = MAX_SPLIT-1; i >= 0; i--) begin
            if (key_match[i])
                match_idx = i;
        end
    end

    wire has_free = ~(&entry_valid);

    // Determine whether sub-transaction 2 is allowed to proceed:
    // must have sub1_done set.  If sub2 arrives before sub1 is done, stall.
    // Guard with has_key_match: without a matching entry, sub2 is unrecognized.
    wire sub2_allowed = has_key_match && entry_sub1_done[match_idx];

    //=========================================================================
    // Forwarding checks (per-beat combinational)
    //=========================================================================
    // can_pass: this beat can be forwarded to master
    //   - non-split: always
    //   - sub1: always (but RLAST forced to 0)
    //   - sub2: only if sub1_done (otherwise stall)
    //=========================================================================
    wire can_pass = !r_is_split ||                          // non-split
                    r_is_trans1 ||                          // sub1 always passes
                    (r_is_trans2 && sub2_allowed);          // sub2 only after sub1

    // s_axi_rready: accept when we can forward and master is ready
    assign s_axi_rready = can_pass && m_axi_rready;

    //=========================================================================
    // R channel outputs (combinational, per-beat)
    //=========================================================================
    // RLAST handling:
    //   sub1 last beat (RLAST=1) → forced to 0 (master sees continuous burst)
    //   sub2 last beat (RLAST=1) → passed through
    // RRESP:
    //   accumulated per orig_id across all beats (OR)
    //   on sub2 last beat: output final accumulated value
    //=========================================================================
    wire [1:0] merged_resp = has_key_match
                             ? (entry_acc_resp[match_idx] | s_axi_rresp)
                             : s_axi_rresp;

    always @(*) begin
        if (s_axi_rvalid && can_pass) begin
            m_axi_rid    = r_strip_id;                       // always strip prefix
            m_axi_rdata  = s_axi_rdata;
            m_axi_rvalid = 1'b1;

            if (r_is_trans1) begin
                // Sub-transaction 1: suppress RLAST
                m_axi_rlast = 1'b0;
                m_axi_rresp = s_axi_rresp;                   // per-beat passthrough
            end else if (r_is_trans2 && sub2_allowed) begin
                // Sub-transaction 2: keep RLAST, output merged RRESP on last beat
                m_axi_rlast = s_axi_rlast;
                m_axi_rresp = s_axi_rlast ? merged_resp : s_axi_rresp;
            end else begin
                // Non-split: full passthrough
                m_axi_rlast = s_axi_rlast;
                m_axi_rresp = s_axi_rresp;
            end
        end else begin
            m_axi_rid    = {W_ID{1'b0}};
            m_axi_rdata  = {W_DATA{1'b0}};
            m_axi_rlast  = 1'b0;
            m_axi_rresp  = 2'b00;
            m_axi_rvalid = 1'b0;
        end
    end

    //=========================================================================
    // Table update (sequential)
    //=========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            entry_valid     <= {MAX_SPLIT{1'b0}};
            entry_sub1_done <= {MAX_SPLIT{1'b0}};
        end else begin
            // —— deallocation: sub2 last beat forwarded ——
            if (s_axi_rvalid && s_axi_rready && r_is_trans2 &&
                sub2_allowed && s_axi_rlast && m_axi_rready) begin
                entry_valid[match_idx]     <= 1'b0;
                entry_sub1_done[match_idx] <= 1'b0;
            end

            // —— absorption / allocation: split R beat ——
            if (s_axi_rvalid && s_axi_rready && r_is_split) begin
                if (!has_key_match && has_free) begin
                    // First split R beat for this orig_id → allocate
                    entry_valid[alloc_idx]     <= 1'b1;
                    entry_orig_id[alloc_idx]   <= r_strip_id;
                    entry_sub1_done[alloc_idx] <= r_is_trans1 && s_axi_rlast;
                    entry_acc_resp[alloc_idx]  <= s_axi_rresp;
                end else if (has_key_match) begin
                    // Update existing entry: accumulate RRESP, track sub1 done
                    if (r_is_trans1 && s_axi_rlast)
                        entry_sub1_done[match_idx] <= 1'b1;
                    entry_acc_resp[match_idx] <= entry_acc_resp[match_idx] | s_axi_rresp;
                end
            end
        end
    end

endmodule
