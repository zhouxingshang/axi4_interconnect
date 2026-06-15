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
wire [W_ADDR-1:0]    m_araddr_end;
wire [W_ADDR-1:0]    m_awaddr_end;
wire                 ar_cross4k_flag;
wire                 aw_cross4k_flag;

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

// signal for split to 2-transactions
reg [W_ID-1:0]       trans_arid    ; 
reg [2:0]            trans_arsize  ; 
reg [1:0]            trans_arburst ; 
reg [W_ADDR-1:0]     trans1_araddr ; 
reg [W_LEN-1:0]      trans1_arlen  ; 
reg [W_ADDR-1:0]     trans2_araddr ; 
reg [W_LEN-1:0]      trans2_arlen  ; 
reg                  trans_arvalid ; 

reg [W_ID-1:0]       trans_awid    ; 
reg [2:0]            trans_awsize  ; 
reg [1:0]            trans_awburst ; 
reg [W_ADDR-1:0]     trans1_awaddr ; 
reg [W_LEN-1:0]      trans1_awlen_lat ;
reg [W_ADDR-1:0]     trans2_awaddr ;
reg [W_LEN-1:0]      trans2_awlen  ;
reg                  trans_awvalid ;

reg [1:0] ST_AR_C4K, ST_AW_C4K;
localparam IDLE   = 2'h0,
           TRANS1 = 2'h1,
           TRANS2 = 2'h2;

// ---- W Skid Buffer Registers ----
reg                  w_buf_valid;
reg [W_DATA-1:0]     w_buf_data;
reg [W_STRB-1:0]     w_buf_strb;
reg                  w_buf_last;

//=============================================================================
// Transaction data registers: Captures parameters precisely at IDLE handshake
//=============================================================================
always @(posedge clk) begin
    if (!rst_n) begin
        trans_arid    <= 0;
        trans_arsize  <= 0;
        trans_arburst <= 0;
        trans1_araddr <= 0;
        trans1_arlen  <= 0;
        trans2_araddr <= 0;
        trans2_arlen  <= 0;
        trans_arvalid <= 0;
    end else if (ST_AR_C4K == IDLE && m_axi_arvalid && ar_cross4k_flag) begin
        trans_arid    <= m_axi_arid   ;
        trans_arsize  <= m_axi_arsize ;
        trans_arburst <= m_axi_arburst;
        trans1_araddr <= m_axi_araddr ;
        trans1_arlen  <= ar_beats_page1 - 1'b1;
        trans2_araddr <= {m_axi_araddr[W_ADDR-1:12] + 1'b1, 12'h0};
        trans2_arlen  <= ar_total_beats - ar_beats_page1 - 1'b1;
        trans_arvalid <= 1'b1;
    end else if (ST_AR_C4K == TRANS2 && s_axi_arvalid && s_axi_arready) begin
        trans_arvalid <= 1'b0;
    end
end

always @(posedge clk) begin
    if (!rst_n) begin
        trans_awid       <= 0;
        trans_awsize     <= 0;
        trans_awburst    <= 0;
        trans1_awaddr    <= 0;
        trans1_awlen_lat <= 0;
        trans2_awaddr    <= 0;
        trans2_awlen     <= 0;
        trans_awvalid    <= 0;
    end else if (ST_AW_C4K == IDLE && m_axi_awvalid && aw_cross4k_flag) begin
        trans_awid       <= m_axi_awid   ;
        trans_awsize     <= m_axi_awsize ;
        trans_awburst    <= m_axi_awburst;
        trans1_awaddr    <= m_axi_awaddr ;
        trans1_awlen_lat <= aw_beats_page1 - 1'b1;
        trans2_awaddr    <= {m_axi_awaddr[W_ADDR-1:12] + 1'b1, 12'h0};
        trans2_awlen     <= aw_total_beats - aw_beats_page1 - 1'b1;
        trans_awvalid    <= 1'b1;
    end else if (ST_AW_C4K == TRANS2 && s_axi_awvalid && s_axi_awready) begin
        trans_awvalid    <= 1'b0;
    end
end

//=============================================================================
// AR FSM & Output Control Mux
//=============================================================================
always @(posedge clk) begin
    if (!rst_n) begin
        ST_AR_C4K <= IDLE;
    end else begin
        case (ST_AR_C4K)
            IDLE: begin
                if (ar_cross4k_flag && m_axi_arvalid)
                    ST_AR_C4K <= TRANS1;
            end
            TRANS1: begin
                if (s_axi_arvalid && s_axi_arready)
                    ST_AR_C4K <= TRANS2;
            end
            TRANS2: begin
                if (s_axi_arvalid && s_axi_arready)
                    ST_AR_C4K <= IDLE;
            end
            default: ST_AR_C4K <= IDLE;
        endcase
    end
end

always @(*) begin
    // Default assignments to avoid Latches
    s_axi_arid    = 0;
    s_axi_araddr  = 0;
    s_axi_arlen   = 0;
    s_axi_arsize  = 0;
    s_axi_arburst = 0;
    s_axi_arvalid = 0;
    m_axi_arready = 0;

    case (ST_AR_C4K)
        IDLE: begin
            if (!ar_cross4k_flag) begin
                s_axi_arid    = {2'b00, m_axi_arid};
                s_axi_araddr  = m_axi_araddr;
                s_axi_arlen   = m_axi_arlen;
                s_axi_arsize  = m_axi_arsize;
                s_axi_arburst = m_axi_arburst;
                s_axi_arvalid = m_axi_arvalid;
                m_axi_arready = s_axi_arready;
            end else begin
                m_axi_arready = 1'b1; // Absorb crossing trans immediately
            end
        end
        TRANS1: begin
            s_axi_arid    = {2'b01, trans_arid};
            s_axi_araddr  = trans1_araddr;
            s_axi_arlen   = trans1_arlen;
            s_axi_arsize  = trans_arsize;
            s_axi_arburst = trans_arburst;
            s_axi_arvalid = trans_arvalid;
            m_axi_arready = 1'b0;
        end
        TRANS2: begin
            s_axi_arid    = {2'b10, trans_arid};
            s_axi_araddr  = trans2_araddr;
            s_axi_arlen   = trans2_arlen;
            s_axi_arsize  = trans_arsize;
            s_axi_arburst = trans_arburst;
            s_axi_arvalid = trans_arvalid;
            m_axi_arready = 1'b0;
        end
    endcase
end

//=============================================================================
// AW FSM & Output Control Mux
//=============================================================================
always @(posedge clk) begin
    if (!rst_n) begin
        ST_AW_C4K <= IDLE;
    end else begin
        case (ST_AW_C4K)
            IDLE: begin
                if (aw_cross4k_flag && m_axi_awvalid)
                    ST_AW_C4K <= TRANS1;
            end
            TRANS1: begin
                if (s_axi_awvalid && s_axi_awready)
                    ST_AW_C4K <= TRANS2;
            end
            TRANS2: begin
                if (s_axi_awvalid && s_axi_awready)
                    ST_AW_C4K <= IDLE;
            end
            default: ST_AW_C4K <= IDLE;
        endcase
    end
end

always @(*) begin
    // Default assignments to avoid Latches
    s_axi_awid    = 0;
    s_axi_awaddr  = 0;
    s_axi_awlen   = 0;
    s_axi_awsize  = 0;
    s_axi_awburst = 0;
    s_axi_awvalid = 0;
    m_axi_awready = 0;

    case (ST_AW_C4K)
        IDLE: begin
            if (!aw_cross4k_flag) begin
                s_axi_awid    = {2'b00, m_axi_awid};
                s_axi_awaddr  = m_axi_awaddr;
                s_axi_awlen   = m_axi_awlen;
                s_axi_awsize  = m_axi_awsize;
                s_axi_awburst = m_axi_awburst;
                s_axi_awvalid = m_axi_awvalid;
                m_axi_awready = s_axi_awready;
            end else begin
                m_axi_awready = 1'b1; // Absorb crossing trans immediately
            end
        end
        TRANS1: begin
            s_axi_awid    = {2'b01, trans_awid};
            s_axi_awaddr  = trans1_awaddr;
            s_axi_awlen   = trans1_awlen_lat;
            s_axi_awsize  = trans_awsize;
            s_axi_awburst = trans_awburst;
            s_axi_awvalid = trans_awvalid;
            m_axi_awready = 1'b0;
        end
        TRANS2: begin
            s_axi_awid    = {2'b10, trans_awid};
            s_axi_awaddr  = trans2_awaddr;
            s_axi_awlen   = trans2_awlen;
            s_axi_awsize  = trans_awsize;
            s_axi_awburst = trans_awburst;
            s_axi_awvalid = trans_awvalid;
            m_axi_awready = 1'b0;
        end
    endcase
end

//=============================================================================
// W Channel Control & Handshake Tracking Logic
//=============================================================================
reg [W_LEN-1:0] w_beat_cnt;        // beat index within the original transaction
reg             w_trans1_done;      // sub-transaction 1 W phase completed
reg             w_aw_split;         // current write transaction was split
reg [W_LEN-1:0] orig_awlen_reg;    // original AWLEN (from master, before split)
reg [W_LEN-1:0] t1_awlen_capt;     // registered trans1_awlen (captured early)
reg             w_stall_rel;        // latch: once sub-AW2 accepted, permanently release W stall

// AW split state tracking
always @(posedge clk) begin
    if (!rst_n) begin
        orig_awlen_reg <= 0;
        t1_awlen_capt  <= 0;
        w_aw_split     <= 0;
        w_stall_rel    <= 0;
    end else begin
        // Capture split write parameters upon IDLE acceptance
        if (ST_AW_C4K == IDLE && m_axi_awvalid && m_axi_awready) begin
            orig_awlen_reg <= m_axi_awlen;
            t1_awlen_capt  <= aw_beats_page1 - 1'b1;
            w_aw_split     <= aw_cross4k_flag;
            w_stall_rel    <= 1'b0;
        end
        
        // Release W stall once slave accepts the TRANS2 AW address
        if (ST_AW_C4K == TRANS2 && s_axi_awvalid && s_axi_awready) begin
            w_stall_rel <= 1'b1;
        end
        
        // Reset tracking state after the last data beat of the write stream
        if (s_axi_wvalid && s_axi_wready && s_axi_wlast) begin
            w_aw_split  <= 1'b0;
            w_stall_rel <= 1'b0;
        end
    end
end

// W Channel Counter & sub-WLAST Generation Control
always @(posedge clk) begin
    if (!rst_n) begin
        w_beat_cnt    <= 0;
        w_trans1_done <= 0;
    end else begin
        if (s_axi_wvalid && s_axi_wready) begin
            if (w_aw_split && !w_trans1_done && (w_beat_cnt == t1_awlen_capt)) begin
                w_trans1_done <= 1'b1;
            end

            if (s_axi_wlast) begin
                w_beat_cnt    <= 0;
                w_trans1_done <= 1'b0;
            end else begin
                w_beat_cnt <= w_beat_cnt + 1'b1;
            end
        end
    end
end

// W Channel Stall Condition Logic
wire w_stall = w_trans1_done && w_aw_split && !w_stall_rel;

// ---- Standard W Skid Buffer Implementation ----
assign m_axi_wready = (!w_buf_valid || (s_axi_wready && !w_stall));

always @(posedge clk) begin
    if (!rst_n) begin
        w_buf_valid <= 1'b0;
        w_buf_data  <= 0;
        w_buf_strb  <= 0;
        w_buf_last  <= 1'b0;
    end else if (m_axi_wready) begin
        w_buf_valid <= m_axi_wvalid;
        w_buf_data  <= m_axi_wdata;
        w_buf_strb  <= m_axi_wstrb;
        w_buf_last  <= m_axi_wlast;
    end
end

// Downstream Slave Routing
assign s_axi_wdata  = w_buf_data;
assign s_axi_wstrb  = w_buf_strb;
assign s_axi_wvalid = w_buf_valid && !w_stall;

// Force WLAST on the final data beat of sub-transaction 1
assign s_axi_wlast  = (w_aw_split && !w_trans1_done && (w_beat_cnt == t1_awlen_capt)) 
                      ? 1'b1 
                      : w_buf_last;

//=============================================================================
// TRACE / SIMULATION MONITORING
//=============================================================================
wire wlast_src_forced = !w_trans1_done && w_aw_split && (w_beat_cnt == t1_awlen_capt);

always @(posedge clk) begin
    if (rst_n) begin
        if (w_aw_split || w_trans1_done)
            $display("[TRACE_C4K] %0t AW_ST=%0d split=%b t1done=%b stall=%b stall_rel=%b beat=%0d/%0d",
                     $time, ST_AW_C4K, w_aw_split, w_trans1_done, w_stall, w_stall_rel,
                     w_beat_cnt, orig_awlen_reg);
    end
end

always @(posedge clk) begin
    if (rst_n) begin
        if (m_axi_wvalid && m_axi_wready)
            $display("[TRACE_C4K_W] %0t M_W_HS m_last=%b (buf_empty=%b)",
                     $time, m_axi_wlast, !w_buf_valid);
                     
        if (w_buf_valid && s_axi_wready && !w_stall)
            $display("[TRACE_C4K_W] %0t S_W_HS beat=%0d buf_last=%b -> s_wlast=%b (src=%s)",
                     $time, w_beat_cnt, w_buf_last, s_axi_wlast,
                     wlast_src_forced ? "FORCED" : "PASSTHRU");
        else if (w_buf_valid && !s_axi_wready)
            $display("[TRACE_C4K_W] %0t S_W_STALL beat=%0d s_vld=1 s_rdy=0",
                     $time, w_beat_cnt);
        else if (!w_buf_valid && m_axi_wvalid)
            $display("[TRACE_C4K_W] %0t BUF_LOAD m_last=%b", $time, m_axi_wlast);
    end
end

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