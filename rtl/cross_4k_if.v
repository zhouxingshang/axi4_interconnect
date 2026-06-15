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

reg              w_buf_valid;
reg [W_DATA-1:0] w_buf_data;
reg [W_STRB-1:0] w_buf_strb;
reg              w_buf_last;
wire w_buf_ready = !w_buf_valid || s_axi_wready;

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
                if (ar_cross4k_flag && m_axi_arvalid) begin
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
        s_axi_arid    = {2'b01, trans_arid};     // split sub-transaction 1
        s_axi_araddr  = trans1_araddr ;
        s_axi_arlen   = trans1_arlen  ;
        s_axi_arsize  = trans_arsize ;
        s_axi_arburst = trans_arburst;
        s_axi_arvalid = trans_arvalid;
        m_axi_arready = 0;
    end
    else if (ST_AR_C4K==TRANS2) begin
        s_axi_arid    = {2'b10, trans_arid};     // split sub-transaction 2
        s_axi_araddr  = trans2_araddr ;
        s_axi_arlen   = trans2_arlen  ;
        s_axi_arsize  = trans_arsize ;
        s_axi_arburst = trans_arburst;
        s_axi_arvalid = trans_arvalid;
        m_axi_arready = 1;
    end
    else if(ST_AR_C4K==IDLE) begin
        if(!ar_cross4k_flag && s_axi_arready) begin
            s_axi_arid    = {2'b00, m_axi_arid};  // non-split passthrough
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
                if (aw_cross4k_flag && m_axi_awvalid) begin
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
        s_axi_awid    = {2'b01, trans_awid};     // split sub-transaction 1
        s_axi_awaddr  = trans1_awaddr ;
        s_axi_awlen   = trans1_awlen  ;
        s_axi_awsize  = trans_awsize ;
        s_axi_awburst = trans_awburst;
        s_axi_awvalid = trans_awvalid;
        m_axi_awready = 0;
    end
    else if (ST_AW_C4K == TRANS2) begin
        s_axi_awid    = {2'b10, trans_awid};     // split sub-transaction 2
        s_axi_awaddr  = trans2_awaddr ;
        s_axi_awlen   = trans2_awlen  ;
        s_axi_awsize  = trans_awsize ;
        s_axi_awburst = trans_awburst;
        s_axi_awvalid = trans_awvalid;
        m_axi_awready = 1;
    end
    else if(ST_AW_C4K == IDLE) begin
        if(!aw_cross4k_flag && s_axi_awready) begin
            s_axi_awid    = {2'b00, m_axi_awid};  // non-split passthrough
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
reg             w_stall_rel;        // latch: once sub-AW2 accepted, permanently release W stall
reg [W_LEN-1:0] trans1_awlen_reg;
// Capture original AWLEN when AW handshake completes on master side
always @(posedge clk) begin
    if (!rst_n) begin
        w_aw_split       <= 1'b0;
        trans1_awlen_reg <= 0;
        w_stall_rel      <= 1'b0;
    end else begin
        // 当状态机在 IDLE 侦测到跨 4K 写请求时，立刻在下一个时钟周期锁存状态
        if (ST_AW_C4K == IDLE && m_axi_awvalid && aw_cross4k_flag) begin
            w_aw_split       <= 1'b1;
            trans1_awlen_reg <= aw_beats_page1 - 1'b1;
            w_stall_rel      <= 1'b0;   // 新拆分传输开始，复位释放信号
        end 
        // 当整笔 Master Burst 的最后一拍 W 完毕后，清空拆分标记
        else if (w_buf_valid && w_buf_ready && w_buf_last) begin
            w_aw_split       <= 1'b0;
        end

        // 维持原样：当第二笔子传输的 AW 被 Slave 接受后，永久释放 W 管道的 Stall
        if (ST_AW_C4K == TRANS2 && s_axi_awvalid && s_axi_awready) begin
            w_stall_rel <= 1'b1;
        end
    end
end

// W beat counter: starts counting when sub-transaction 1 AW is accepted
always @(posedge clk) begin
    if (!rst_n) begin
        w_beat_cnt    <= 0;
        w_trans1_done <= 1'b0;
    end else begin
        if (m_axi_wvalid && m_axi_wready) begin
            if (m_axi_wlast) begin
                w_beat_cnt    <= 0;
                //w_trans1_done <= 1'b0; // 整笔大传输结束，复位
            end else begin
                w_beat_cnt <= w_beat_cnt + 1'b1;
                // 当处于拆分状态，且当前拍刚好是第一笔子传输的最后一拍时，标记第一部分结束
                if (w_aw_split && ((w_beat_cnt - 1)== trans1_awlen_reg)) begin
                    w_trans1_done <= 1'b1;
                end
            end
        end else if(w_buf_valid && w_buf_ready) begin
            if(w_buf_last)
                w_trans1_done <= 1'b0;
        end
    end
end

// W channel routing
// Stalling: between sub-W1 WLAST and sub-AW2 acceptance, stall W.
// Once sub-AW2 is accepted (w_stall_rel=1), stall is permanently released.
wire w_stall = w_trans1_done && w_aw_split && !w_stall_rel;

// Combinational split indicator: true even before w_aw_split is registered
wire w_split_comb = w_aw_split || (m_axi_awvalid && aw_cross4k_flag);

// ---- W Skid Buffer (active only during split transactions) ----



always @(posedge clk) begin
    if (!rst_n) begin
        w_buf_valid <= 0;
        w_buf_data  <= 0;
        w_buf_strb  <= 0;
        w_buf_last  <= 0;
    end else if (w_split_comb && w_buf_ready) begin
        w_buf_valid <= m_axi_wvalid;
        w_buf_data  <= m_axi_wdata;
        w_buf_strb  <= m_axi_wstrb;
        w_buf_last  <= m_axi_wlast;
    end else if (!w_split_comb) begin
        w_buf_valid <= 0;
    end
end

// Output mux: buffered (split) vs direct (non-split)
assign s_axi_wdata  = w_split_comb ? w_buf_data  : m_axi_wdata;
assign s_axi_wstrb  = w_split_comb ? w_buf_strb  : m_axi_wstrb;
assign s_axi_wvalid = w_split_comb ? (w_buf_valid && !w_stall) : (m_axi_wvalid && !w_stall);
assign m_axi_wready = w_split_comb ? (w_buf_ready && !w_stall) : (s_axi_wready && !w_stall);

// WLAST computation (works on buffered data during split, direct during non-split)
wire w_wlast_int;
assign w_wlast_int = (!w_trans1_done && w_aw_split && (w_beat_cnt - 1) == trans1_awlen)
                     ? 1'b1
                     : (w_split_comb ? w_buf_last : m_axi_wlast);
assign s_axi_wlast = w_wlast_int;



// TRACE: 4KB split debug
always @(posedge clk) begin
    if (rst_n) begin
        if (w_aw_split || w_trans1_done)
            $display("[TRACE_C4K] %0t ST_AW=%0d split=%b t1done=%b stall=%b beat=%0d",
                     $time, ST_AW_C4K, w_aw_split, w_trans1_done, w_stall, w_beat_cnt);
    end
end
always @(posedge clk) begin
    if (rst_n) begin
        if (m_axi_wvalid || s_axi_wvalid)
            $display("[TRACE_C4K] %0t W m_vld=%b m_rdy=%b s_vld=%b s_rdy=%b slast=%b",
                     $time, m_axi_wvalid, m_axi_wready, s_axi_wvalid, s_axi_wready, s_axi_wlast);
    end
end

endmodule
