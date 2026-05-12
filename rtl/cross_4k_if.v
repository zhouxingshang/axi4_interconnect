// INCR BURST ONLY
// 4KB boundary splitter: splits AW/AR when transaction crosses 4KB
// W channel: forces WLAST=1 on last beat of sub-transaction 1
// B channel: merges two B responses back into one for the master
module cross_4k_if #(
      parameter W_ID   = 4           // ID width
              , W_CID  = 4
              , W_ADDR = 32          // address width
              , W_LEN  = 8
              , W_DATA = 32          // data width
              , W_STRB = (W_DATA/8)  // data strobe width
              , W_SID  = (W_CID+W_ID)// slave ID
) (
      input                          clk
    , input                          rst_n

    // ---- AR channel ----
    , input  wire    [W_ID-1 :0]     m_axi_arid
    , input  wire    [W_ADDR-1:0]    m_axi_araddr
    , input  wire    [W_LEN-1 :0]    m_axi_arlen
    , input  wire    [2 :0]          m_axi_arsize
    , input  wire    [1 :0]          m_axi_arburst
    , input  wire                    m_axi_arvalid
    , output reg                     m_axi_arready

    // ---- AW channel ----
    , input  wire    [W_ID-1 :0]     m_axi_awid
    , input  wire    [W_ADDR-1:0]    m_axi_awaddr
    , input  wire    [W_LEN-1 :0]    m_axi_awlen
    , input  wire    [2 :0]          m_axi_awsize
    , input  wire    [1 :0]          m_axi_awburst
    , input  wire                    m_axi_awvalid
    , output reg                     m_axi_awready

    // ---- W channel (passthrough + WLAST insertion) ----
    , input  wire    [W_DATA-1:0]    m_axi_wdata
    , input  wire    [W_STRB-1:0]    m_axi_wstrb
    , input  wire                    m_axi_wlast
    , input  wire                    m_axi_wvalid
    , output wire                    m_axi_wready

    , output wire    [W_DATA-1:0]    s_axi_wdata
    , output wire    [W_STRB-1:0]    s_axi_wstrb
    , output wire                    s_axi_wlast
    , output wire                    s_axi_wvalid
    , input  wire                    s_axi_wready

    // ---- B channel (response merging for split writes) ----
    , input  wire    [W_ID-1:0]      s_axi_bid
    , input  wire    [1:0]           s_axi_bresp
    , input  wire                    s_axi_bvalid
    , output wire                    s_axi_bready

    , output reg     [W_ID-1:0]      m_axi_bid
    , output reg     [1:0]           m_axi_bresp
    , output reg                     m_axi_bvalid
    , input  wire                    m_axi_bready

    // ---- AR slave side ----
    , output  reg    [W_ID-1:0]      s_axi_arid
    , output  reg    [W_ADDR-1:0]    s_axi_araddr
    , output  reg    [W_LEN-1:0]     s_axi_arlen
    , output  reg    [2:0]           s_axi_arsize
    , output  reg    [1:0]           s_axi_arburst
    , output  reg                    s_axi_arvalid
    , input   wire                   s_axi_arready

    // ---- AW slave side ----
    , output  reg    [W_ID-1:0]      s_axi_awid
    , output  reg    [W_ADDR-1:0]    s_axi_awaddr
    , output  reg    [W_LEN-1:0]     s_axi_awlen
    , output  reg    [2:0]           s_axi_awsize
    , output  reg    [1:0]           s_axi_awburst
    , output  reg                    s_axi_awvalid
    , input   wire                   s_axi_awready

);

// ---- address calculation helpers ----
// Total bytes = (LEN + 1) << SIZE; end_addr = start + total_bytes - 1
wire [W_ADDR-1:0]    m_araddr_end;
wire [W_ADDR-1:0]    m_awaddr_end;

wire [W_ADDR:0] ar_total_bytes = ({1'b0, m_axi_arlen} + 1'b1) << m_axi_arsize;
wire [W_ADDR:0] aw_total_bytes = ({1'b0, m_axi_awlen} + 1'b1) << m_axi_awsize;

assign m_araddr_end = m_axi_araddr + ar_total_bytes[W_ADDR-1:0] - 1'b1;
assign m_awaddr_end = m_axi_awaddr + aw_total_bytes[W_ADDR-1:0] - 1'b1;

// 4KB boundary: bit 12 differs → transaction crosses a 4KB page
assign ar_cross4k_flag = m_axi_araddr[12] ^ m_araddr_end[12];
assign aw_cross4k_flag = m_axi_awaddr[12] ^ m_awaddr_end[12];

// bytes per beat: 1 << SIZE
wire [7:0] ar_bpb = 8'd1 << m_axi_arsize;   // AR bytes per beat
wire [7:0] aw_bpb = 8'd1 << m_axi_awsize;   // AW bytes per beat

// total beats in the original transaction
wire [W_LEN:0] ar_total_beats = m_axi_arlen + 1'b1;
wire [W_LEN:0] aw_total_beats = m_axi_awlen + 1'b1;

// bytes from start to next 4KB boundary
wire [12:0] ar_bytes_to_bnd = 13'h1000 - {1'b0, m_axi_araddr[11:0]};
wire [12:0] aw_bytes_to_bnd = 13'h1000 - {1'b0, m_axi_awaddr[11:0]};

// beats that fit entirely within the first 4KB page (ceil division)
wire [W_LEN:0] ar_beats_page1 = (ar_bytes_to_bnd + {5'b0, ar_bpb} - 1'b1) >> m_axi_arsize;
wire [W_LEN:0] aw_beats_page1 = (aw_bytes_to_bnd + {5'b0, aw_bpb} - 1'b1) >> m_axi_awsize;

//signal for split to 2-transactions
reg [W_ID-1:0]      trans_arid    ; 
reg [2:0]           trans_arsize  ; 
reg [1:0]           trans_arburst ; 
reg [W_ADDR-1:0]    trans1_araddr ; 
reg [W_LEN-1:0]     trans1_arlen  ; 
reg [W_ADDR-1:0]    trans2_araddr ; 
reg [W_LEN-1:0]     trans2_arlen  ; 
reg                 trans_arvalid ; 

reg [W_ID-1:0]      trans_awid    ; 
reg [2:0]           trans_awsize  ; 
reg [1:0]           trans_awburst ; 
reg [W_ADDR-1:0]    trans1_awaddr ; 
reg [W_LEN-1:0]     trans1_awlen  ; 
reg [W_ADDR-1:0]    trans2_awaddr ; 
reg [W_LEN-1:0]     trans2_awlen  ; 
reg                 trans_awvalid ; 
reg                 aw_trans_stall;

always @(*) begin
    if (m_axi_arvalid & ar_cross4k_flag) begin
        trans_arid    = m_axi_arid   ;
        trans_arsize  = m_axi_arsize ;
        trans_arburst = m_axi_arburst;
        trans1_araddr = m_axi_araddr ;
        trans1_arlen  = ar_beats_page1 - 1'b1;      // AWLEN = beats_in_page1 - 1
        trans2_araddr = {m_axi_araddr[W_ADDR-1:12] + 1'b1, 12'h0};
        trans2_arlen  = ar_total_beats - ar_beats_page1 - 1'b1;
        trans_arvalid = m_axi_arvalid;
    end
    if (m_axi_awvalid & aw_cross4k_flag) begin
        trans_awid    = m_axi_awid   ;
        trans_awsize  = m_axi_awsize ;
        trans_awburst = m_axi_awburst;
        trans1_awaddr = m_axi_awaddr ;
        trans1_awlen  = aw_beats_page1 - 1'b1;      // AWLEN = beats_in_page1 - 1
        trans2_awaddr = {m_axi_awaddr[W_ADDR-1:12] + 1'b1, 12'h0};
        trans2_awlen  = aw_total_beats - aw_beats_page1 - 1'b1;
        trans_awvalid = m_axi_awvalid;
    end
end

reg [2:0] ST_AR_C4K, ST_AW_C4K;
parameter IDLE   = 3'h0,
          TRANS1 = 3'h1,
          TRANS2 = 3'h2;

//ARADDR
always @(posedge clk) begin
    if (!rst_n) begin
        ST_AR_C4K <= 0;
    end else begin
        case (ST_AR_C4K)
            IDLE: begin 
                if (ar_cross4k_flag) begin
                    ST_AR_C4K  <= TRANS1;
                end
            end
            TRANS1: begin
                if (s_axi_arready) begin
                    ST_AR_C4K  <= TRANS2;
                end
            end
            TRANS2: if (s_axi_arready) begin
                ST_AR_C4K  <= IDLE;
            end
        endcase
    end
end

always @(*) begin
    if (ST_AR_C4K==TRANS1) begin
        s_axi_arid    = trans_arid   ;
        s_axi_araddr  = trans1_araddr ;
        s_axi_arlen   = trans1_arlen  ;
        s_axi_arsize  = trans_arsize ;
        s_axi_arburst = trans_arburst;
        s_axi_arvalid = trans_arvalid;
        m_axi_arready = 0;
    end
    else if (ST_AR_C4K==TRANS2) begin
        s_axi_arid    = trans_arid   ;
        s_axi_araddr  = trans2_araddr ;
        s_axi_arlen   = trans2_arlen  ;
        s_axi_arsize  = trans_arsize ;
        s_axi_arburst = trans_arburst;
        s_axi_arvalid = trans_arvalid;
        m_axi_arready = 1;
    end
    else if(ST_AR_C4K==IDLE) begin
        if(!ar_cross4k_flag && s_axi_arready) begin
            s_axi_arid    = m_axi_arid   ;
            s_axi_araddr  = m_axi_araddr ;
            s_axi_arlen   = m_axi_arlen  ;
            s_axi_arsize  = m_axi_arsize ;
            s_axi_arburst = m_axi_arburst;
            s_axi_arvalid = m_axi_arvalid;
            m_axi_arready = 1;
        end
        else begin
            s_axi_arid    = 0;
            s_axi_araddr  = 0;
            s_axi_arlen   = 0;
            s_axi_arsize  = 0;
            s_axi_arburst = 0;
            s_axi_arvalid = 0;
            m_axi_arready = 0;
        end
    end
end

//AWADDR
always @(posedge clk) begin
    if (!rst_n) begin
        ST_AW_C4K <= 0;
    end else begin
        case (ST_AW_C4K)
            IDLE: begin 
                if (aw_cross4k_flag) begin
                    ST_AW_C4K  <= TRANS1;
                end
            end
            TRANS1: begin
                if (s_axi_awready) begin
                    ST_AW_C4K  <= TRANS2;
                end
            end
            TRANS2: if (s_axi_awready) begin
                ST_AW_C4K  <= IDLE;
            end
        endcase
    end
end

always @(*) begin
    if (ST_AW_C4K == TRANS1) begin
        s_axi_awid    = trans_awid   ;
        s_axi_awaddr  = trans1_awaddr ;
        s_axi_awlen   = trans1_awlen  ;
        s_axi_awsize  = trans_awsize ;
        s_axi_awburst = trans_awburst;
        s_axi_awvalid = trans_awvalid;
        m_axi_awready = 0;
    end
    else if (ST_AW_C4K == TRANS2) begin
        s_axi_awid    = trans_awid + 1'b1;   // increment ID for sub-transaction 2
        s_axi_awaddr  = trans2_awaddr ;
        s_axi_awlen   = trans2_awlen  ;
        s_axi_awsize  = trans_awsize ;
        s_axi_awburst = trans_awburst;
        s_axi_awvalid = trans_awvalid;
        m_axi_awready = 1;
    end
    else if(ST_AW_C4K == IDLE) begin
        if(!aw_cross4k_flag && s_axi_awready) begin
            s_axi_awid    = m_axi_awid   ;
            s_axi_awaddr  = m_axi_awaddr ;
            s_axi_awlen   = m_axi_awlen  ;
            s_axi_awsize  = m_axi_awsize ;
            s_axi_awburst = m_axi_awburst;
            s_axi_awvalid = m_axi_awvalid;
            m_axi_awready = 1;
        end
        else begin
            s_axi_awid    = 0;
            s_axi_awaddr  = 0;
            s_axi_awlen   = 0;
            s_axi_awsize  = 0;
            s_axi_awburst = 0;
            s_axi_awvalid = 0;
            m_axi_awready = 0;
        end
    end
end

//=============================================================================
// W Channel: WLAST insertion for split sub-transaction 1
//=============================================================================
// WLAST is forced to 1 on the last beat of sub-transaction 1 (beat trans1_awlen).
// After that, the counter continues and WLAST passes through from the master
// on the last beat of the original transaction (which is sub-transaction 2's last).
//=============================================================================
reg [W_LEN-1:0] w_beat_cnt;        // beat index within the original transaction
reg             w_trans1_done;      // sub-transaction 1 W phase completed
reg             w_aw_split;         // current write transaction was split
reg [W_LEN-1:0] orig_awlen_reg;    // original AWLEN (from master, before split)

// Capture original AWLEN when AW handshake completes on master side
always @(posedge clk) begin
    if (!rst_n) begin
        orig_awlen_reg <= 0;
        w_aw_split     <= 0;
    end else begin
        if (m_axi_awvalid && m_axi_awready) begin
            orig_awlen_reg <= m_axi_awlen;
            w_aw_split     <= aw_cross4k_flag;
        end
    end
end

// W beat counter: starts counting when sub-transaction 1 AW is accepted
always @(posedge clk) begin
    if (!rst_n) begin
        w_beat_cnt    <= 0;
        w_trans1_done <= 0;
    end else begin
        // Load / start on TRANS1 AW handshake
        if (ST_AW_C4K == TRANS1 && s_axi_awvalid && s_axi_awready && aw_cross4k_flag) begin
            w_beat_cnt    <= 0;
            w_trans1_done <= 0;
        end

        // W handshake: increment counter
        if (m_axi_wvalid && m_axi_wready) begin
            if (!w_trans1_done && w_aw_split && w_beat_cnt == trans1_awlen) begin
                // Last beat of sub-transaction 1 → force WLAST, move to sub-transaction 2
                w_trans1_done <= 1;
            end
            w_beat_cnt <= w_beat_cnt + 1'b1;
        end
    end
end

// W channel routing
// Stalling: between TRANS1 W done and TRANS2 AW accepted, stall W to prevent
// sub-transaction 2 data from reaching the slave before its AW is sent.
wire w_stall = w_trans1_done && w_aw_split &&
               !(ST_AW_C4K == TRANS2 && s_axi_awvalid && s_axi_awready);

assign s_axi_wdata  = m_axi_wdata;
assign s_axi_wstrb  = m_axi_wstrb;
assign s_axi_wvalid = m_axi_wvalid && !w_stall;
assign s_axi_wlast  = (!w_trans1_done && w_aw_split && w_beat_cnt == trans1_awlen)
                      ? 1'b1              // force WLAST on last beat of sub-transaction 1
                      : m_axi_wlast;      // passthrough otherwise
assign m_axi_wready = s_axi_wready && !w_stall;

//=============================================================================
// B Channel: merge two B responses → one for split write transactions
//=============================================================================
// Sub-transaction 1 uses original AWID; sub-transaction 2 uses AWID+1.
// Both B responses must be received before forwarding a single merged B to
// the master.  Final BRESP = bitwise OR of both sub-transactions' BRESP:
// if either fails the whole transaction fails.
// BID-keyed tracking handles either arrival order.
//=============================================================================
reg             b_split_active;     // armed: split write B merging in progress
reg             b_got_trans1;       // sub-transaction 1's B received
reg             b_got_trans2;       // sub-transaction 2's B received
reg [1:0]       b_resp_merged;      // accumulated BRESP = BRESP_trans1 | BRESP_trans2
reg [W_ID-1:0]  b_orig_awid;       // original AWID (before +1 for sub-transaction 2)

always @(posedge clk) begin
    if (!rst_n) begin
        b_split_active <= 0;
        b_got_trans1   <= 0;
        b_got_trans2   <= 0;
        b_resp_merged  <= 2'b00;
        b_orig_awid    <= 0;
    end else begin
        // Arm on split AW acceptance (master side)
        if (m_axi_awvalid && m_axi_awready && aw_cross4k_flag) begin
            b_split_active <= 1;
            b_got_trans1   <= 0;
            b_got_trans2   <= 0;
            b_resp_merged  <= 2'b00;
            b_orig_awid    <= m_axi_awid;
        end

        // B handshake: accumulate BRESP by OR, track by BID
        if (b_split_active && s_axi_bvalid && s_axi_bready) begin
            if (s_axi_bid == b_orig_awid) begin
                b_got_trans1  <= 1;
                b_resp_merged <= b_resp_merged | s_axi_bresp;
            end else if (s_axi_bid == b_orig_awid + 1'b1) begin
                b_got_trans2  <= 1;
                b_resp_merged <= b_resp_merged | s_axi_bresp;
            end
        end

        // Both received and merged B accepted by master → done
        if (b_split_active && b_got_trans1 && b_got_trans2 && m_axi_bready) begin
            b_split_active <= 0;
            b_got_trans1   <= 0;
            b_got_trans2   <= 0;
        end
    end
end

// B ready to slave: accept until both sub-transaction responses received
assign s_axi_bready = (b_split_active && !(b_got_trans1 && b_got_trans2))
                      ? 1'b1 : m_axi_bready;

// B response to master
always @(*) begin
    if (b_split_active && b_got_trans1 && b_got_trans2) begin
        // Both sub-transactions done → forward merged B
        m_axi_bid    = b_orig_awid;
        m_axi_bresp  = b_resp_merged;
        m_axi_bvalid = 1'b1;
    end else if (!b_split_active) begin
        // Passthrough mode (no split, or merge complete)
        m_axi_bid    = s_axi_bid;
        m_axi_bresp  = s_axi_bresp;
        m_axi_bvalid = s_axi_bvalid;
    end else begin
        // Still waiting for both B responses
        m_axi_bid    = 0;
        m_axi_bresp  = 0;
        m_axi_bvalid = 0;
    end
end

endmodule