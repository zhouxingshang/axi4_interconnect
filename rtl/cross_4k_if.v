// INCR BURST ONLY
// 4KB boundary splitter: splits AW/AR when transaction crosses 4KB
// W channel: forces WLAST=1 on last beat of sub-transaction 1
// B channel: handled externally by axi_split_b_merge
module cross_4k_if #(
      parameter W_ID   = 4           // ID width
              , W_ADDR = 32          // address width
              , W_LEN  = 8
              , W_DATA = 32          // data width
              , W_STRB = (W_DATA/8)  // data strobe width
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

    // ---- AR slave side ----
    , output  reg    [W_ID+1:0]      s_axi_arid          // prefix[1:0] + orig_id
    , output  reg    [W_ADDR-1:0]    s_axi_araddr
    , output  reg    [W_LEN-1:0]     s_axi_arlen
    , output  reg    [2:0]           s_axi_arsize
    , output  reg    [1:0]           s_axi_arburst
    , output  reg                    s_axi_arvalid
    , input   wire                   s_axi_arready

    // ---- AW slave side ----
    , output  reg    [W_ID+1:0]      s_axi_awid          // prefix[1:0] + orig_id
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
reg [W_LEN-1:0]     trans1_awlen_lat ;
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
        trans1_awlen_lat = aw_beats_page1 - 1'b1;      // AWLEN = beats_in_page1 - 1
        trans2_awaddr    = {m_axi_awaddr[W_ADDR-1:12] + 1'b1, 12'h0};
        trans2_awlen  = aw_total_beats - aw_beats_page1 - 1'b1;
        trans_awvalid = m_axi_awvalid;
    end
end

reg [1:0] ST_AR_C4K, ST_AW_C4K;
reg [1:0] next_ST_AR_C4K, next_ST_AW_C4K;
parameter IDLE   = 2'h0,
          TRANS1 = 2'h1,
          TRANS2 = 2'h2;

//=============================================================================
// AR FSM: two-stage (next=comb, ST=reg) to eliminate 1-cycle transition lag
//=============================================================================

// Combinational next-state logic
always @(*) begin
    next_ST_AR_C4K = ST_AR_C4K;
    case (ST_AR_C4K)
        IDLE: begin
            if (ar_cross4k_flag && m_axi_arvalid)
                next_ST_AR_C4K = TRANS1;
        end
        TRANS1: begin
            if (s_axi_arready)
                next_ST_AR_C4K = TRANS2;
        end
        TRANS2: begin
            if (s_axi_arready)
                next_ST_AR_C4K = IDLE;
        end
    endcase
end

// Registered state
always @(posedge clk) begin
    if (!rst_n)
        ST_AR_C4K <= IDLE;
    else
        ST_AR_C4K <= next_ST_AR_C4K;
end

// Look-ahead: TRANS1 phase active on the same cycle the trigger fires
wire in_trans1_r_phase;
assign in_trans1_r_phase = (ST_AR_C4K == TRANS1)
                        || (ST_AR_C4K == IDLE && ar_cross4k_flag && m_axi_arvalid);

// AR output mux (uses in_trans1_r_phase for look-ahead)
always @(*) begin
    if (in_trans1_r_phase) begin
        s_axi_arid    = {2'b01, trans_arid};
        s_axi_araddr  = trans1_araddr;
        s_axi_arlen   = trans1_arlen;
        s_axi_arsize  = trans_arsize;
        s_axi_arburst = trans_arburst;
        s_axi_arvalid = trans_arvalid;
        m_axi_arready = 0;
    end
    else if (ST_AR_C4K == TRANS2) begin
        s_axi_arid    = {2'b10, trans_arid};
        s_axi_araddr  = trans2_araddr;
        s_axi_arlen   = trans2_arlen;
        s_axi_arsize  = trans_arsize;
        s_axi_arburst = trans_arburst;
        s_axi_arvalid = trans_arvalid;
        m_axi_arready = 1;
    end
    else if (!ar_cross4k_flag && s_axi_arready) begin
        // IDLE, non-split passthrough
        s_axi_arid    = {2'b00, m_axi_arid};
        s_axi_araddr  = m_axi_araddr;
        s_axi_arlen   = m_axi_arlen;
        s_axi_arsize  = m_axi_arsize;
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

//=============================================================================
// AW FSM: two-stage (next=comb, ST=reg) to eliminate 1-cycle transition lag
//=============================================================================

// Combinational next-state logic
always @(*) begin
    next_ST_AW_C4K = ST_AW_C4K;
    case (ST_AW_C4K)
        IDLE: begin
            if (aw_cross4k_flag && m_axi_awvalid)
                next_ST_AW_C4K = TRANS1;
        end
        TRANS1: begin
            if (s_axi_awready)
                next_ST_AW_C4K = TRANS2;
        end
        TRANS2: begin
            if (s_axi_awready)
                next_ST_AW_C4K = IDLE;
        end
    endcase
end

// Registered state
always @(posedge clk) begin
    if (!rst_n)
        ST_AW_C4K <= IDLE;
    else
        ST_AW_C4K <= next_ST_AW_C4K;
end

// Look-ahead: TRANS1 phase active on the same cycle the trigger fires
wire in_trans1_w_phase;
assign in_trans1_w_phase = (ST_AW_C4K == TRANS1)
                        || (ST_AW_C4K == IDLE && aw_cross4k_flag && m_axi_awvalid);

// AW output mux (uses in_trans1_w_phase for look-ahead)
always @(*) begin
    if (in_trans1_w_phase) begin
        s_axi_awid    = {2'b01, trans_awid};
        s_axi_awaddr  = trans1_awaddr;
        s_axi_awlen   = trans1_awlen_lat;
        s_axi_awsize  = trans_awsize;
        s_axi_awburst = trans_awburst;
        s_axi_awvalid = trans_awvalid;
        m_axi_awready = 0;
    end
    else if (ST_AW_C4K == TRANS2) begin
        s_axi_awid    = {2'b10, trans_awid};
        s_axi_awaddr  = trans2_awaddr;
        s_axi_awlen   = trans2_awlen;
        s_axi_awsize  = trans_awsize;
        s_axi_awburst = trans_awburst;
        s_axi_awvalid = trans_awvalid;
        m_axi_awready = 1;
    end
    else if (!aw_cross4k_flag && s_axi_awready) begin
        // IDLE, non-split passthrough
        s_axi_awid    = {2'b00, m_axi_awid};
        s_axi_awaddr  = m_axi_awaddr;
        s_axi_awlen   = m_axi_awlen;
        s_axi_awsize  = m_axi_awsize;
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

//=============================================================================
// W Channel: WLAST insertion for split sub-transaction 1
//=============================================================================
// WLAST is forced to 1 on the last beat of sub-transaction 1 (beat t1_awlen_capt).
// After that, the counter continues and WLAST passes through from the master
// on the last beat of the original transaction (which is sub-transaction 2's last).
//=============================================================================
reg [W_LEN-1:0] w_beat_cnt;        // beat index within the original transaction
reg             w_trans1_done;      // sub-transaction 1 W phase completed
reg             w_aw_split;         // current write transaction was split
reg [W_LEN-1:0] orig_awlen_reg;    // original AWLEN (from master, before split)
reg [W_LEN-1:0] t1_awlen_capt;  // registered t1_awlen_capt (captured early)
reg             w_stall_rel;        // latch: once sub-AW2 accepted, permanently release W stall

//=============================================================================
// AW info capture: register split flag + lengths as soon as AWVALID is seen
// (before AW handshake), so W beats that arrive early still see correct split.
//=============================================================================
always @(posedge clk) begin
    if (!rst_n) begin
        orig_awlen_reg   <= 0;
        t1_awlen_capt <= 0;
        w_aw_split       <= 0;
        w_stall_rel      <= 0;
    end else begin
        // ---- Early capture: as soon as AWVALID with cross4k is first asserted ----
        if (m_axi_awvalid && aw_cross4k_flag && ST_AW_C4K == IDLE) begin
            orig_awlen_reg   <= m_axi_awlen;
            t1_awlen_capt <= aw_beats_page1 - 1'b1;
            w_aw_split       <= 1'b1;
            w_stall_rel      <= 1'b0;
        end
        // ---- Update at master handshake: clear for next non-split transaction ----
        if (m_axi_awvalid && m_axi_awready) begin
            orig_awlen_reg   <= m_axi_awlen;
            w_aw_split       <= aw_cross4k_flag;
            w_stall_rel      <= 1'b0;
        end
        // Release W stall once sub-AW2 is accepted by slave
        if (ST_AW_C4K == TRANS2 && s_axi_awvalid && s_axi_awready)
            w_stall_rel <= 1;
    end
end

// W beat counter: starts counting when sub-transaction 1 AW is accepted
always @(posedge clk) begin
    if (!rst_n) begin
        w_beat_cnt    <= 0;
        w_trans1_done <= 0;
    end else begin
        // Load / start on TRANS1 AW handshake (reset for each new split transaction)
        if (in_trans1_w_phase && s_axi_awvalid && s_axi_awready && w_aw_split) begin
            w_beat_cnt    <= 0;
            w_trans1_done <= 0;
        end

        // W handshake: increment counter
        if (m_axi_wvalid && m_axi_wready) begin
            if (!w_trans1_done && w_aw_split && w_beat_cnt == t1_awlen_capt) begin
                // Last beat of sub-transaction 1 → force WLAST, move to sub-transaction 2
                w_trans1_done <= 1;
            end
            w_beat_cnt <= w_beat_cnt + 1'b1;
        end
    end
end

// W channel routing
// Stalling: between sub-W1 WLAST and sub-AW2 acceptance, stall W.
// Once sub-AW2 is accepted (w_stall_rel=1), stall is permanently released.
wire w_stall = w_trans1_done && w_aw_split && !w_stall_rel;

// Combinational split indicator: covers both registered split state AND
// the current cycle's sub-AW1 handshake.  Eliminates the 1-cycle lag of
// w_aw_split so that a single-beat sub-transaction 1 still gets forced
// WLAST even when W data arrives concurrently with the AW handshake.
wire w_aw_split_comb;
assign w_aw_split_comb = w_aw_split || in_trans1_w_phase;

assign s_axi_wdata  = m_axi_wdata;
assign s_axi_wstrb  = m_axi_wstrb;
assign s_axi_wvalid = m_axi_wvalid && !w_stall;
assign s_axi_wlast  = (!w_trans1_done && w_aw_split_comb && w_beat_cnt == t1_awlen_capt)
                      ? 1'b1              // force WLAST on last beat of sub-transaction 1
                      : m_axi_wlast;      // passthrough otherwise
assign m_axi_wready = s_axi_wready && !w_stall;

// TRACE: 4KB split W channel detailed debug
wire wlast_src_forced = !w_trans1_done && w_aw_split_comb && w_beat_cnt == t1_awlen_capt;
wire wlast_src_master = !wlast_src_forced;

// Combined status trace every cycle when split is active
// Shows: AW FSM state, split flags, stall condition, beat counter
always @(posedge clk) begin
    if (rst_n) begin
        if (w_aw_split || w_trans1_done)
            $display("[TRACE_C4K] %0t AW_ST=%0d split=%b t1done=%b stall=%b stall_rel=%b beat=%0d/%0d",
                     $time, ST_AW_C4K, w_aw_split, w_trans1_done, w_stall, w_stall_rel,
                     w_beat_cnt, orig_awlen_reg);
    end
end

// W beat trace: shows master→slave W handshake with WLAST source
// m_hs = master handshake (WVALID & WREADY)
// s_hs = slave  handshake (WVALID & WREADY)
always @(posedge clk) begin
    if (rst_n) begin
        if (m_axi_wvalid && m_axi_wready)
            $display("[TRACE_C4K_W] %0t M_W_HS beat=%0d m_wlast=%b -> s_wlast=%b (src=%s)",
                     $time, w_beat_cnt, m_axi_wlast, s_axi_wlast,
                     wlast_src_forced ? "FORCED" : "PASSTHRU");
        else if (s_axi_wvalid && s_axi_wready)
            $display("[TRACE_C4K_W] %0t S_W_HS beat=%0d m_wlast=%b -> s_wlast=%b (src=%s)",
                     $time, w_beat_cnt, m_axi_wlast, s_axi_wlast,
                     wlast_src_forced ? "FORCED" : "PASSTHRU");
        else if (m_axi_wvalid && !m_axi_wready)
            $display("[TRACE_C4K_W] %0t M_W_STALL beat=%0d m_vld=1 m_rdy=0 stall=%b",
                     $time, w_beat_cnt, w_stall);
        else if (s_axi_wvalid && !s_axi_wready)
            $display("[TRACE_C4K_W] %0t S_W_STALL beat=%0d s_vld=1 s_rdy=0",
                     $time, w_beat_cnt);
    end
end

// Split boundary trace: marks transitions
always @(posedge clk) begin
    if (rst_n) begin
        if (w_aw_split && ST_AW_C4K == TRANS1)
            $display("[TRACE_C4K] %0t SPLIT_T1 awlen=%0d beats_page1=%0d addr_end_bit12=%b",
                     $time, t1_awlen_capt, aw_beats_page1, m_awaddr_end[12]);
        if (w_aw_split && ST_AW_C4K == TRANS2)
            $display("[TRACE_C4K] %0t SPLIT_T2 awlen=%0d beats_page2=%0d",
                     $time, trans2_awlen, aw_total_beats - aw_beats_page1);
        if (w_trans1_done && w_aw_split && !w_stall_rel)
            $display("[TRACE_C4K] %0t W_STALL_START waiting for sub-AW2 accept", $time);
    end
end

endmodule