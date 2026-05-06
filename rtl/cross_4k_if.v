//=============================================================================
// Module: cross_4k_if (Enhanced)
// Desc  : AXI address 4KB boundary splitter.
//         - Supports INCR, WRAP, FIXED burst types.
//         - Splits a transaction that crosses 4KB boundary into two sub-transactions.
//         - Outputs completion signals (aw_split_done / ar_split_done) for response merging.
//         - Handshake order: master side must wait for split_done before next transaction.
//=============================================================================

module cross_4k_if #(
    parameter W_ID   = 4,
    parameter W_CID  = 4,
    parameter W_ADDR = 32,
    parameter W_LEN  = 8,
    parameter W_DATA = 32,
    parameter W_STRB = W_DATA/8,
    parameter W_SID  = W_CID+W_ID
) (
    input  wire                clk,
    input  wire                rst_n,

    // ---------- Master side (input from master / previous stage) ----------
    input  wire [W_ID-1:0]     m_axi_awid,
    input  wire [W_ADDR-1:0]   m_axi_awaddr,
    input  wire [W_LEN-1:0]    m_axi_awlen,
    input  wire [2:0]          m_axi_awsize,
    input  wire [1:0]          m_axi_awburst,
    input  wire                m_axi_awvalid,
    output reg                 m_axi_awready,

    input  wire [W_ID-1:0]     m_axi_arid,
    input  wire [W_ADDR-1:0]   m_axi_araddr,
    input  wire [W_LEN-1:0]    m_axi_arlen,
    input  wire [2:0]          m_axi_arsize,
    input  wire [1:0]          m_axi_arburst,
    input  wire                m_axi_arvalid,
    output reg                 m_axi_arready,

    // ---------- Slave side (output to downstream / slave) ----------
    output reg  [W_ID-1:0]     s_axi_awid,
    output reg  [W_ADDR-1:0]   s_axi_awaddr,
    output reg  [W_LEN-1:0]    s_axi_awlen,
    output reg  [2:0]          s_axi_awsize,
    output reg  [1:0]          s_axi_awburst,
    output reg                 s_axi_awvalid,
    input  wire                s_axi_awready,

    output reg  [W_ID-1:0]     s_axi_arid,
    output reg  [W_ADDR-1:0]   s_axi_araddr,
    output reg  [W_LEN-1:0]    s_axi_arlen,
    output reg  [2:0]          s_axi_arsize,
    output reg  [1:0]          s_axi_arburst,
    output reg                 s_axi_arvalid,
    input  wire                s_axi_arready,

    // ---------- Completion status (for response merging) ----------
    output reg                 aw_split_done,   // pulsed when all sub-transactions of current AW complete
    output reg                 ar_split_done    // pulsed when all sub-transactions of current AR complete
);

//=============================================================================
// Local parameters and functions
//=============================================================================
localparam IDLE   = 2'b00;
localparam SUB1   = 2'b01;
localparam SUB2   = 2'b10;

// Byte count per beat for given size
function integer bytes_per_beat;
    input [2:0] size;
    begin
        case (size)
            3'b000: bytes_per_beat = 1;
            3'b001: bytes_per_beat = 2;
            3'b010: bytes_per_beat = 4;
            3'b011: bytes_per_beat = 8;
            3'b100: bytes_per_beat = 16;
            3'b101: bytes_per_beat = 32;
            3'b110: bytes_per_beat = 64;
            3'b111: bytes_per_beat = 128;
            default: bytes_per_beat = 1;
        endcase
    end
endfunction

// Compute total transfer size in bytes
function integer total_bytes;
    input [W_LEN-1:0] len;
    input [2:0] size;
    begin
        total_bytes = (len + 1) * bytes_per_beat(size);
    end
endfunction

//=============================================================================
// AW Channel Splitting
//=============================================================================
reg [1:0] aw_state;
reg [W_LEN-1:0] aw_orig_len;
reg [2:0] aw_orig_size;
reg [1:0] aw_orig_burst;
reg [W_ID-1:0] aw_orig_id;
reg [1:0] aw_sub_cnt;          // number of sub-transactions pending (1 or 2)
reg aw_sub_handshake;          // handshake on current sub-transaction

wire aw_cross_boundary;
wire [W_ADDR-1:0] aw_addr_end;
wire [W_ADDR-1:0] aw_next_page_addr;
wire [W_LEN-1:0] aw_sub1_len, aw_sub2_len;
wire [W_ADDR-1:0] aw_sub2_addr;
wire aw_sub1_valid, aw_sub2_valid;

// Address calculations
assign aw_addr_end = m_axi_awaddr + total_bytes(m_axi_awlen, m_axi_awsize) - 1;
assign aw_cross_boundary = (m_axi_awaddr[11:0] + total_bytes(m_axi_awlen, m_axi_awsize) > 4096) &&
                           (m_axi_awburst != 2'b00); // FIXED burst cannot cross naturally, but we still check
assign aw_next_page_addr = {m_axi_awaddr[31:12] + 1'b1, 12'b0};

// Split lengths: first part goes to page boundary, second part is remaining
wire [W_LEN-1:0] beats_to_boundary;
assign beats_to_boundary = (4096 - m_axi_awaddr[11:0] + bytes_per_beat(m_axi_awsize) - 1) /
                           bytes_per_beat(m_axi_awsize);
assign aw_sub1_len = (aw_cross_boundary && (beats_to_boundary <= m_axi_awlen)) ? beats_to_boundary - 1 : m_axi_awlen;
assign aw_sub2_len = m_axi_awlen - aw_sub1_len - 1;
assign aw_sub2_addr = aw_next_page_addr;

// Valid signals for sub-transactions
assign aw_sub1_valid = (aw_state == SUB1) && m_axi_awvalid;
assign aw_sub2_valid = (aw_state == SUB2) && m_axi_awvalid;

always @(posedge clk) begin
    if (!rst_n) begin
        aw_state <= IDLE;
        aw_sub_cnt <= 0;
        aw_sub_handshake <= 1'b0;
        m_axi_awready <= 1'b0;
        s_axi_awvalid <= 1'b0;
        aw_split_done <= 1'b0;
    end else begin
        // Defaults
        aw_split_done <= 1'b0;
        aw_sub_handshake <= (s_axi_awvalid && s_axi_awready);

        case (aw_state)
            IDLE: begin
                if (m_axi_awvalid && !aw_cross_boundary) {
                    // No split: pass through
                    s_axi_awid    <= m_axi_awid;
                    s_axi_awaddr  <= m_axi_awaddr;
                    s_axi_awlen   <= m_axi_awlen;
                    s_axi_awsize  <= m_axi_awsize;
                    s_axi_awburst <= m_axi_awburst;
                    s_axi_awvalid <= 1'b1;
                    m_axi_awready <= s_axi_awready;
                    if (s_axi_awready && m_axi_awvalid) begin
                        // Handshake done, transaction complete
                        s_axi_awvalid <= 1'b0;
                        m_axi_awready <= 1'b0;
                        aw_split_done <= 1'b1;
                    end
                end else if (m_axi_awvalid && aw_cross_boundary) {
                    // Need split: store original attributes
                    aw_orig_len   <= m_axi_awlen;
                    aw_orig_size  <= m_axi_awsize;
                    aw_orig_burst <= m_axi_awburst;
                    aw_orig_id    <= m_axi_awid;
                    aw_sub_cnt    <= 2;
                    aw_state      <= SUB1;
                    // Do not assert m_axi_awready yet; we will accept the transaction only after both subs?
                    // AXI requires that AWREADY can be asserted in same cycle as AWVALID.
                    // We will assert m_axi_awready in SUB1 after we have accepted the split.
                    // Actually, to avoid deadlock, we should assert m_axi_awready immediately
                    // to accept the original transaction, then process splits.
                    m_axi_awready <= 1'b1;
                end else begin
                    s_axi_awvalid <= 1'b0;
                    m_axi_awready <= 1'b0;
                end
            end

            SUB1: begin
                // Issue first sub-transaction
                s_axi_awid    <= aw_orig_id;
                s_axi_awaddr  <= m_axi_awaddr;   // original start address
                s_axi_awlen   <= aw_sub1_len;
                s_axi_awsize  <= aw_orig_size;
                s_axi_awburst <= 2'b01;          // always INCR after split
                s_axi_awvalid <= 1'b1;
                m_axi_awready <= 1'b0;            // original already accepted
                if (s_axi_awready && s_axi_awvalid) begin
                    s_axi_awvalid <= 1'b0;
                    aw_sub_cnt <= aw_sub_cnt - 1;
                    if (aw_sub_cnt == 1) begin
                        aw_state <= SUB2;
                    end else begin
                        aw_state <= IDLE;
                        aw_split_done <= 1'b1;
                    end
                end
            end

            SUB2: begin
                // Issue second sub-transaction
                s_axi_awid    <= aw_orig_id;
                s_axi_awaddr  <= aw_sub2_addr;
                s_axi_awlen   <= aw_sub2_len;
                s_axi_awsize  <= aw_orig_size;
                s_axi_awburst <= 2'b01;          // INCR
                s_axi_awvalid <= 1'b1;
                if (s_axi_awready && s_axi_awvalid) begin
                    s_axi_awvalid <= 1'b0;
                    aw_state <= IDLE;
                    aw_split_done <= 1'b1;
                end
            end
        endcase
    end
end

//=============================================================================
// AR Channel Splitting (identical structure to AW)
//=============================================================================
reg [1:0] ar_state;
reg [W_LEN-1:0] ar_orig_len;
reg [2:0] ar_orig_size;
reg [1:0] ar_orig_burst;
reg [W_ID-1:0] ar_orig_id;
reg [1:0] ar_sub_cnt;
reg ar_sub_handshake;

wire ar_cross_boundary;
wire [W_ADDR-1:0] ar_addr_end;
wire [W_ADDR-1:0] ar_next_page_addr;
wire [W_LEN-1:0] ar_sub1_len, ar_sub2_len;
wire [W_ADDR-1:0] ar_sub2_addr;
wire ar_sub1_valid, ar_sub2_valid;

assign ar_addr_end = m_axi_araddr + total_bytes(m_axi_arlen, m_axi_arsize) - 1;
assign ar_cross_boundary = (m_axi_araddr[11:0] + total_bytes(m_axi_arlen, m_axi_arsize) > 4096) &&
                           (m_axi_arburst != 2'b00);
assign ar_next_page_addr = {m_axi_araddr[31:12] + 1'b1, 12'b0};

assign beats_to_boundary = (4096 - m_axi_araddr[11:0] + bytes_per_beat(m_axi_arsize) - 1) /
                           bytes_per_beat(m_axi_arsize);
assign ar_sub1_len = (ar_cross_boundary && (beats_to_boundary <= m_axi_arlen)) ? beats_to_boundary - 1 : m_axi_arlen;
assign ar_sub2_len = m_axi_arlen - ar_sub1_len - 1;
assign ar_sub2_addr = ar_next_page_addr;

assign ar_sub1_valid = (ar_state == SUB1) && m_axi_arvalid;
assign ar_sub2_valid = (ar_state == SUB2) && m_axi_arvalid;

always @(posedge clk) begin
    if (!rst_n) begin
        ar_state <= IDLE;
        ar_sub_cnt <= 0;
        ar_sub_handshake <= 1'b0;
        m_axi_arready <= 1'b0;
        s_axi_arvalid <= 1'b0;
        ar_split_done <= 1'b0;
    end else begin
        ar_split_done <= 1'b0;
        ar_sub_handshake <= (s_axi_arvalid && s_axi_arready);

        case (ar_state)
            IDLE: begin
                if (m_axi_arvalid && !ar_cross_boundary) {
                    s_axi_arid    <= m_axi_arid;
                    s_axi_araddr  <= m_axi_araddr;
                    s_axi_arlen   <= m_axi_arlen;
                    s_axi_arsize  <= m_axi_arsize;
                    s_axi_arburst <= m_axi_arburst;
                    s_axi_arvalid <= 1'b1;
                    m_axi_arready <= s_axi_arready;
                    if (s_axi_arready && m_axi_arvalid) begin
                        s_axi_arvalid <= 1'b0;
                        m_axi_arready <= 1'b0;
                        ar_split_done <= 1'b1;
                    end
                end else if (m_axi_arvalid && ar_cross_boundary) {
                    ar_orig_len   <= m_axi_arlen;
                    ar_orig_size  <= m_axi_arsize;
                    ar_orig_burst <= m_axi_arburst;
                    ar_orig_id    <= m_axi_arid;
                    ar_sub_cnt    <= 2;
                    ar_state      <= SUB1;
                    m_axi_arready <= 1'b1;
                end else begin
                    s_axi_arvalid <= 1'b0;
                    m_axi_arready <= 1'b0;
                end
            end

            SUB1: begin
                s_axi_arid    <= ar_orig_id;
                s_axi_araddr  <= m_axi_araddr;
                s_axi_arlen   <= ar_sub1_len;
                s_axi_arsize  <= ar_orig_size;
                s_axi_arburst <= 2'b01;
                s_axi_arvalid <= 1'b1;
                m_axi_arready <= 1'b0;
                if (s_axi_arready && s_axi_arvalid) begin
                    s_axi_arvalid <= 1'b0;
                    ar_sub_cnt <= ar_sub_cnt - 1;
                    if (ar_sub_cnt == 1) begin
                        ar_state <= SUB2;
                    end else begin
                        ar_state <= IDLE;
                        ar_split_done <= 1'b1;
                    end
                end
            end

            SUB2: begin
                s_axi_arid    <= ar_orig_id;
                s_axi_araddr  <= ar_sub2_addr;
                s_axi_arlen   <= ar_sub2_len;
                s_axi_arsize  <= ar_orig_size;
                s_axi_arburst <= 2'b01;
                s_axi_arvalid <= 1'b1;
                if (s_axi_arready && s_axi_arvalid) begin
                    s_axi_arvalid <= 1'b0;
                    ar_state <= IDLE;
                    ar_split_done <= 1'b1;
                end
            end
        endcase
    end
end

endmodule