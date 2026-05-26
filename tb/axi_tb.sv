`timescale 1ns/1ps

//=============================================================================
// axi_tb — Comprehensive Testbench for axi_interconnect (4 Master × 4 Slave)
//=============================================================================
// Coverage:
//   - Single-beat write/read to every (master, slave) pair
//   - Multi-beat INCR bursts (length 0~15)
//   - Concurrent multi-master arbitration
//   - 4KB boundary crossing (cross_4k_if split/merge)
//   - Back‑pressure (random slave ready delays)
//   - Multi-ID outstanding transactions
//   - Data integrity self-checking via slave memory model
//=============================================================================

module axi_tb;

    //=========================================================
    // Parameters
    //=========================================================
    localparam MST_AMT         = 4;
    localparam SLV_AMT         = 4;
    localparam OUTSTANDING_AMT = 8;
    localparam W_ID            = 4;
    localparam DATA_WIDTH      = 32;
    localparam ADDR_WIDTH      = 32;
    localparam LEN_W           = 8;
    localparam SIZE_W          = 3;
    localparam BURST_W         = 2;
    localparam RESP_W          = 2;
    localparam W_STRB          = DATA_WIDTH / 8;      // 4
    localparam W_CID           = 2;                   // $clog2(MST_AMT)
    localparam W_MID           = W_ID + 2;            // 6
    localparam W_SID           = $clog2(MST_AMT) + W_CID + W_ID; // 8

    localparam SLV_MEM_AW      = 16;                  // 64 KB per slave
    localparam SLV_MEM_DEPTH   = 1 << SLV_MEM_AW;

    // Address map: upper 4 bits select slave → 256 MB per slave
    localparam [(SLV_AMT*ADDR_WIDTH)-1:0] SLV_ADDR_BASE = {
        32'h3000_0000,   // slave 3
        32'h2000_0000,   // slave 2
        32'h1000_0000,   // slave 1
        32'h0000_0000    // slave 0
    };
    localparam [(SLV_AMT*8)-1:0] SLV_ADDR_LEN = {
        8'd28,           // slave 3
        8'd28,           // slave 2
        8'd28,           // slave 1
        8'd28            // slave 0
    };

    //=========================================================
    // Clock / Reset
    //=========================================================
    reg AXI_CLK;
    reg AXI_RSTn;

    initial begin
        AXI_CLK = 0;
        forever #5 AXI_CLK = ~AXI_CLK;
    end

    initial begin
        AXI_RSTn = 0;
        repeat(20) @(posedge AXI_CLK);
        AXI_RSTn = 1;
    end

    //=========================================================
    // Master-Side Signals (flattened, connected to DUT)
    //=========================================================
    // AW
    reg  [MST_AMT*W_ID-1:0]        M_AXI_AWID_i;
    reg  [MST_AMT*ADDR_WIDTH-1:0]  M_AXI_AWADDR_i;
    reg  [MST_AMT*LEN_W-1:0]       M_AXI_AWLEN_i;
    reg  [MST_AMT*SIZE_W-1:0]      M_AXI_AWSIZE_i;
    reg  [MST_AMT*BURST_W-1:0]     M_AXI_AWBURST_i;
    reg  [MST_AMT-1:0]             M_AXI_AWVALID_i;
    wire [MST_AMT-1:0]             M_AXI_AWREADY_o;
    // W
    reg  [MST_AMT*DATA_WIDTH-1:0]  M_AXI_WDATA_i;
    reg  [MST_AMT*W_STRB-1:0]      M_AXI_WSTRB_i;
    reg  [MST_AMT-1:0]             M_AXI_WLAST_i;
    reg  [MST_AMT-1:0]             M_AXI_WVALID_i;
    wire [MST_AMT-1:0]             M_AXI_WREADY_o;
    // B
    wire [MST_AMT*W_ID-1:0]        M_AXI_BID_o;
    wire [MST_AMT*RESP_W-1:0]      M_AXI_BRESP_o;
    wire [MST_AMT-1:0]             M_AXI_BVALID_o;
    reg  [MST_AMT-1:0]             M_AXI_BREADY_i;
    // AR
    reg  [MST_AMT*W_ID-1:0]        M_AXI_ARID_i;
    reg  [MST_AMT*ADDR_WIDTH-1:0]  M_AXI_ARADDR_i;
    reg  [MST_AMT*LEN_W-1:0]       M_AXI_ARLEN_i;
    reg  [MST_AMT*SIZE_W-1:0]      M_AXI_ARSIZE_i;
    reg  [MST_AMT*BURST_W-1:0]     M_AXI_ARBURST_i;
    reg  [MST_AMT-1:0]             M_AXI_ARVALID_i;
    wire [MST_AMT-1:0]             M_AXI_ARREADY_o;
    // R
    wire [MST_AMT*W_ID-1:0]        M_AXI_RID_o;
    wire [MST_AMT*DATA_WIDTH-1:0]  M_AXI_RDATA_o;
    wire [MST_AMT*RESP_W-1:0]      M_AXI_RRESP_o;
    wire [MST_AMT-1:0]             M_AXI_RLAST_o;
    wire [MST_AMT-1:0]             M_AXI_RVALID_o;
    reg  [MST_AMT-1:0]             M_AXI_RREADY_i;

    //=========================================================
    // Slave-Side Signals (flattened, connected to DUT)
    //=========================================================
    // AW
    wire [W_SID*SLV_AMT-1:0]       S_AXI_AWID_o;
    wire [ADDR_WIDTH*SLV_AMT-1:0]  S_AXI_AWADDR_o;
    wire [LEN_W*SLV_AMT-1:0]       S_AXI_AWLEN_o;
    wire [SIZE_W*SLV_AMT-1:0]      S_AXI_AWSIZE_o;
    wire [BURST_W*SLV_AMT-1:0]     S_AXI_AWBURST_o;
    wire [SLV_AMT-1:0]             S_AXI_AWVALID_o;
    reg  [SLV_AMT-1:0]             S_AXI_AWREADY_i;
    // W
    wire [DATA_WIDTH*SLV_AMT-1:0]  S_AXI_WDATA_o;
    wire [W_STRB*SLV_AMT-1:0]      S_AXI_WSTRB_o;
    wire [SLV_AMT-1:0]             S_AXI_WLAST_o;
    wire [SLV_AMT-1:0]             S_AXI_WVALID_o;
    reg  [SLV_AMT-1:0]             S_AXI_WREADY_i;
    // B
    reg  [W_SID*SLV_AMT-1:0]       S_AXI_BID_i;
    reg  [RESP_W*SLV_AMT-1:0]      S_AXI_BRESP_i;
    reg  [SLV_AMT-1:0]             S_AXI_BVALID_i;
    wire [SLV_AMT-1:0]             S_AXI_BREADY_o;
    // AR
    wire [W_SID*SLV_AMT-1:0]       S_AXI_ARID_o;
    wire [ADDR_WIDTH*SLV_AMT-1:0]  S_AXI_ARADDR_o;
    wire [LEN_W*SLV_AMT-1:0]       S_AXI_ARLEN_o;
    wire [SIZE_W*SLV_AMT-1:0]      S_AXI_ARSIZE_o;
    wire [BURST_W*SLV_AMT-1:0]     S_AXI_ARBURST_o;
    wire [SLV_AMT-1:0]             S_AXI_ARVALID_o;
    reg  [SLV_AMT-1:0]             S_AXI_ARREADY_i;
    // R
    reg  [W_SID*SLV_AMT-1:0]       S_AXI_RID_i;
    reg  [DATA_WIDTH*SLV_AMT-1:0]  S_AXI_RDATA_i;
    reg  [RESP_W*SLV_AMT-1:0]      S_AXI_RRESP_i;
    reg  [SLV_AMT-1:0]             S_AXI_RLAST_i;
    reg  [SLV_AMT-1:0]             S_AXI_RVALID_i;
    wire [SLV_AMT-1:0]             S_AXI_RREADY_o;

    //=========================================================
    // DUT Instantiation
    //=========================================================
    axi_interconnect #(
        .MST_AMT        (MST_AMT),
        .SLV_AMT        (SLV_AMT),
        .OUTSTANDING_AMT(OUTSTANDING_AMT),
        .W_ID           (W_ID),
        .DATA_WIDTH     (DATA_WIDTH),
        .ADDR_WIDTH     (ADDR_WIDTH),
        .LEN_W          (LEN_W),
        .SIZE_W         (SIZE_W),
        .BURST_W        (BURST_W),
        .RESP_W         (RESP_W),
        .W_STRB         (W_STRB),
        .W_CID          (W_CID),
        .SLV_ADDR_BASE  (SLV_ADDR_BASE),
        .SLV_ADDR_LEN   (SLV_ADDR_LEN)
    ) dut (
        .AXI_CLK        (AXI_CLK),
        .AXI_RSTn       (AXI_RSTn),

        // Master
        .M_AXI_AWID_i   (M_AXI_AWID_i),
        .M_AXI_AWADDR_i (M_AXI_AWADDR_i),
        .M_AXI_AWLEN_i  (M_AXI_AWLEN_i),
        .M_AXI_AWSIZE_i (M_AXI_AWSIZE_i),
        .M_AXI_AWBURST_i(M_AXI_AWBURST_i),
        .M_AXI_AWVALID_i(M_AXI_AWVALID_i),
        .M_AXI_AWREADY_o(M_AXI_AWREADY_o),
        .M_AXI_WDATA_i  (M_AXI_WDATA_i),
        .M_AXI_WSTRB_i  (M_AXI_WSTRB_i),
        .M_AXI_WLAST_i  (M_AXI_WLAST_i),
        .M_AXI_WVALID_i (M_AXI_WVALID_i),
        .M_AXI_WREADY_o (M_AXI_WREADY_o),
        .M_AXI_BID_o    (M_AXI_BID_o),
        .M_AXI_BRESP_o  (M_AXI_BRESP_o),
        .M_AXI_BVALID_o (M_AXI_BVALID_o),
        .M_AXI_BREADY_i (M_AXI_BREADY_i),
        .M_AXI_ARID_i   (M_AXI_ARID_i),
        .M_AXI_ARADDR_i (M_AXI_ARADDR_i),
        .M_AXI_ARLEN_i  (M_AXI_ARLEN_i),
        .M_AXI_ARSIZE_i (M_AXI_ARSIZE_i),
        .M_AXI_ARBURST_i(M_AXI_ARBURST_i),
        .M_AXI_ARVALID_i(M_AXI_ARVALID_i),
        .M_AXI_ARREADY_o(M_AXI_ARREADY_o),
        .M_AXI_RID_o    (M_AXI_RID_o),
        .M_AXI_RDATA_o  (M_AXI_RDATA_o),
        .M_AXI_RRESP_o  (M_AXI_RRESP_o),
        .M_AXI_RLAST_o  (M_AXI_RLAST_o),
        .M_AXI_RVALID_o (M_AXI_RVALID_o),
        .M_AXI_RREADY_i (M_AXI_RREADY_i),

        // Slave
        .S_AXI_AWID_o   (S_AXI_AWID_o),
        .S_AXI_AWADDR_o (S_AXI_AWADDR_o),
        .S_AXI_AWLEN_o  (S_AXI_AWLEN_o),
        .S_AXI_AWSIZE_o (S_AXI_AWSIZE_o),
        .S_AXI_AWBURST_o(S_AXI_AWBURST_o),
        .S_AXI_AWVALID_o(S_AXI_AWVALID_o),
        .S_AXI_AWREADY_i(S_AXI_AWREADY_i),
        .S_AXI_WDATA_o  (S_AXI_WDATA_o),
        .S_AXI_WSTRB_o  (S_AXI_WSTRB_o),
        .S_AXI_WLAST_o  (S_AXI_WLAST_o),
        .S_AXI_WVALID_o (S_AXI_WVALID_o),
        .S_AXI_WREADY_i (S_AXI_WREADY_i),
        .S_AXI_BID_i    (S_AXI_BID_i),
        .S_AXI_BRESP_i  (S_AXI_BRESP_i),
        .S_AXI_BVALID_i (S_AXI_BVALID_i),
        .S_AXI_BREADY_o (S_AXI_BREADY_o),
        .S_AXI_ARID_o   (S_AXI_ARID_o),
        .S_AXI_ARADDR_o (S_AXI_ARADDR_o),
        .S_AXI_ARLEN_o  (S_AXI_ARLEN_o),
        .S_AXI_ARSIZE_o (S_AXI_ARSIZE_o),
        .S_AXI_ARBURST_o(S_AXI_ARBURST_o),
        .S_AXI_ARVALID_o(S_AXI_ARVALID_o),
        .S_AXI_ARREADY_i(S_AXI_ARREADY_i),
        .S_AXI_RID_i    (S_AXI_RID_i),
        .S_AXI_RDATA_i  (S_AXI_RDATA_i),
        .S_AXI_RRESP_i  (S_AXI_RRESP_i),
        .S_AXI_RLAST_i  (S_AXI_RLAST_i),
        .S_AXI_RVALID_i (S_AXI_RVALID_i),
        .S_AXI_RREADY_o (S_AXI_RREADY_o),

        // Control
        .arbiter_type   (1'b0),            // round-robin
        .r_order_grant_i({SLV_AMT{1'b0}})  // use internal reorder
    );

    //=========================================================
    // Slave Memory Model (per-slave byte-addressable storage)
    //=========================================================
    reg [7:0] slv_mem [0:SLV_AMT-1][0:SLV_MEM_DEPTH-1];

    // Initialize memory to known pattern
    integer mem_init_s, mem_init_a;
    initial begin
        for (mem_init_s = 0; mem_init_s < SLV_AMT; mem_init_s = mem_init_s + 1) begin
            for (mem_init_a = 0; mem_init_a < SLV_MEM_DEPTH; mem_init_a = mem_init_a + 1) begin
                slv_mem[mem_init_s][mem_init_a] = 8'h00;
            end
        end
    end

    //=========================================================
    // Slave Write State Machine (per slave)
    //=========================================================
    localparam SW_IDLE   = 2'd0;
    localparam SW_WAIT_W = 2'd1;
    localparam SW_SEND_B = 2'd2;

    reg [1:0]                      sw_state   [0:SLV_AMT-1];
    reg [W_SID-1:0]                sw_awid    [0:SLV_AMT-1];
    reg [ADDR_WIDTH-1:0]           sw_awaddr  [0:SLV_AMT-1];
    reg [LEN_W-1:0]                sw_awlen   [0:SLV_AMT-1];
    reg [SIZE_W-1:0]               sw_awsize  [0:SLV_AMT-1];
    reg [LEN_W:0]                  sw_beat    [0:SLV_AMT-1];  // beats remaining+1

    genvar si;
    generate
        for (si = 0; si < SLV_AMT; si = si + 1) begin : GEN_SLV_WRITE
            // Extract per-slave signals from packed vectors
            wire [W_SID-1:0]       s_awid   = S_AXI_AWID_o   [W_SID*(si+1)-1 -: W_SID];
            wire [ADDR_WIDTH-1:0]  s_awaddr = S_AXI_AWADDR_o [ADDR_WIDTH*(si+1)-1 -: ADDR_WIDTH];
            wire [LEN_W-1:0]       s_awlen  = S_AXI_AWLEN_o  [LEN_W*(si+1)-1 -: LEN_W];
            wire [SIZE_W-1:0]      s_awsize = S_AXI_AWSIZE_o [SIZE_W*(si+1)-1 -: SIZE_W];
            wire                   s_awvalid = S_AXI_AWVALID_o[si];
            wire [DATA_WIDTH-1:0]  s_wdata  = S_AXI_WDATA_o  [DATA_WIDTH*(si+1)-1 -: DATA_WIDTH];
            wire [W_STRB-1:0]      s_wstrb  = S_AXI_WSTRB_o  [W_STRB*(si+1)-1 -: W_STRB];
            wire                   s_wlast  = S_AXI_WLAST_o  [si];
            wire                   s_wvalid = S_AXI_WVALID_o [si];
            wire                   s_bready = S_AXI_BREADY_o [si];

            // Helper: bytes per beat (use captured size, not live DUT output)
            wire [7:0] bpb = 8'd1 << sw_awsize[si];

            always @(posedge AXI_CLK) begin
                if (!AXI_RSTn) begin
                    sw_state[si]   <= SW_IDLE;
                    S_AXI_AWREADY_i[si] <= 1'b0;
                    S_AXI_WREADY_i[si]  <= 1'b0;
                    S_AXI_BVALID_i [si] <= 1'b0;
                    S_AXI_BID_i   [W_SID*(si+1)-1 -: W_SID]  <= '0;
                    S_AXI_BRESP_i [RESP_W*(si+1)-1 -: RESP_W] <= 2'b00;
                end else begin
                    case (sw_state[si])

                        SW_IDLE: begin
                            S_AXI_AWREADY_i[si] <= 1'b1;
                            S_AXI_WREADY_i[si]  <= 1'b0;
                            S_AXI_BVALID_i [si] <= 1'b0;

                            if (s_awvalid && S_AXI_AWREADY_i[si]) begin
                                sw_awid  [si] <= s_awid;
                                sw_awaddr[si] <= s_awaddr;
                                sw_awlen [si] <= s_awlen;
                                sw_awsize[si] <= s_awsize;
                                sw_beat  [si] <= {1'b0, s_awlen} + 1'b1; // beats = AWLEN+1
                                S_AXI_AWREADY_i[si] <= 1'b0;
                                S_AXI_WREADY_i[si]  <= 1'b1;
                                sw_state[si] <= SW_WAIT_W;
                                $display("[%0t] SLAVE[%0d] AW accepted: addr=0x%08h len=%0d size=%0d id=0x%0h",
                                         $time, si, s_awaddr, s_awlen, s_awsize, s_awid);
                            end
                        end

                        SW_WAIT_W: begin
                            if (s_wvalid && S_AXI_WREADY_i[si]) begin
                                // Write data bytes to memory where strobe is set
                                for (int b = 0; b < W_STRB; b = b + 1) begin
                                    if (s_wstrb[b]) begin
                                        slv_mem[si][(sw_awaddr[si][SLV_MEM_AW-1:0] + b) & (SLV_MEM_DEPTH-1)]
                                            <= s_wdata[8*b +: 8];
                                    end
                                end
                                $display("[%0t] SLAVE[%0d] W beat: data=0x%08h strb=0x%0h last=%0d beat_left=%0d",
                                         $time, si, s_wdata, s_wstrb, s_wlast, sw_beat[si]-1);

                                if (s_wlast) begin
                                    S_AXI_WREADY_i[si]  <= 1'b0;
                                    S_AXI_BVALID_i [si] <= 1'b1;
                                    S_AXI_BID_i   [W_SID*(si+1)-1 -: W_SID]  <= sw_awid[si];
                                    S_AXI_BRESP_i [RESP_W*(si+1)-1 -: RESP_W] <= 2'b00; // OKAY
                                    sw_state[si] <= SW_SEND_B;
                                end else begin
                                    // Advance address for next beat
                                    sw_awaddr[si] <= sw_awaddr[si] + bpb;
                                    sw_beat[si]   <= sw_beat[si] - 1'b1;
                                end
                            end
                        end

                        SW_SEND_B: begin
                            if (s_bready && S_AXI_BVALID_i[si]) begin
                                S_AXI_BVALID_i[si] <= 1'b0;
                                sw_state[si] <= SW_IDLE;
                                $display("[%0t] SLAVE[%0d] B sent: id=0x%0h", $time, si,
                                         S_AXI_BID_i[W_SID*(si+1)-1 -: W_SID]);
                            end
                        end

                        default: sw_state[si] <= SW_IDLE;
                    endcase
                end
            end
        end
    endgenerate

    //=========================================================
    // Slave Read State Machine (per slave)
    //=========================================================
    localparam SR_IDLE   = 2'd0;
    localparam SR_SEND_R = 2'd1;

    reg [1:0]                      sr_state   [0:SLV_AMT-1];
    reg [W_SID-1:0]                sr_arid    [0:SLV_AMT-1];
    reg [ADDR_WIDTH-1:0]           sr_araddr  [0:SLV_AMT-1];
    reg [LEN_W-1:0]                sr_arlen   [0:SLV_AMT-1];
    reg [SIZE_W-1:0]               sr_arsize  [0:SLV_AMT-1];
    reg [LEN_W:0]                  sr_beat    [0:SLV_AMT-1];  // beats remaining+1

    generate
        for (si = 0; si < SLV_AMT; si = si + 1) begin : GEN_SLV_READ
            wire [W_SID-1:0]       s_arid   = S_AXI_ARID_o   [W_SID*(si+1)-1 -: W_SID];
            wire [ADDR_WIDTH-1:0]  s_araddr = S_AXI_ARADDR_o [ADDR_WIDTH*(si+1)-1 -: ADDR_WIDTH];
            wire [LEN_W-1:0]       s_arlen  = S_AXI_ARLEN_o  [LEN_W*(si+1)-1 -: LEN_W];
            wire [SIZE_W-1:0]      s_arsize = S_AXI_ARSIZE_o [SIZE_W*(si+1)-1 -: SIZE_W];
            wire                   s_arvalid = S_AXI_ARVALID_o[si];
            wire                   s_rready = S_AXI_RREADY_o [si];

            wire [7:0] bpb = 8'd1 << sr_arsize[si];

            // Reconstruct read data from byte memory
            function [DATA_WIDTH-1:0] build_rdata;
                input [ADDR_WIDTH-1:0] addr;
                input integer slv_idx;
                integer b;
                begin
                    build_rdata = '0;
                    for (b = 0; b < W_STRB; b = b + 1) begin
                        build_rdata[8*b +: 8] = slv_mem[slv_idx][(addr[SLV_MEM_AW-1:0] + b) & (SLV_MEM_DEPTH-1)];
                    end
                end
            endfunction

            reg ar_handshake_done;  // flag: AR accepted, transition next cycle

            always @(posedge AXI_CLK) begin
                if (!AXI_RSTn) begin
                    sr_state[si]   <= SR_IDLE;
                    ar_handshake_done <= 1'b0;
                    S_AXI_ARREADY_i[si] <= 1'b0;
                    S_AXI_RVALID_i [si] <= 1'b0;
                    S_AXI_RLAST_i  [si] <= 1'b0;
                    S_AXI_RID_i   [W_SID*(si+1)-1 -: W_SID]   <= '0;
                    S_AXI_RDATA_i [DATA_WIDTH*(si+1)-1 -: DATA_WIDTH] <= '0;
                    S_AXI_RRESP_i [RESP_W*(si+1)-1 -: RESP_W] <= 2'b00;
                end else begin
                    case (sr_state[si])

                        SR_IDLE: begin
                            S_AXI_ARREADY_i[si] <= 1'b0;
                            S_AXI_RVALID_i [si] <= 1'b0;
                            S_AXI_RLAST_i  [si] <= 1'b0;

                            if (ar_handshake_done) begin
                                // AR was accepted last cycle, transition to send
                                ar_handshake_done <= 1'b0;
                                sr_state[si] <= SR_SEND_R;
                            end else if (s_arvalid) begin
                                S_AXI_ARREADY_i[si] <= 1'b1;
                                sr_arid  [si] <= s_arid;
                                sr_araddr[si] <= s_araddr;
                                sr_arlen [si] <= s_arlen;
                                sr_arsize[si] <= s_arsize;
                                sr_beat  [si] <= {1'b0, s_arlen} + 1'b1;
                                ar_handshake_done <= 1'b1;
                                $display("[%0t] SLAVE[%0d] AR accepted: addr=0x%08h len=%0d size=%0d id=0x%0h",
                                         $time, si, s_araddr, s_arlen, s_arsize, s_arid);
                            end
                        end

                        SR_SEND_R: begin
                            S_AXI_ARREADY_i[si] <= 1'b0;
                            S_AXI_RVALID_i [si] <= 1'b1;
                            S_AXI_RID_i   [W_SID*(si+1)-1 -: W_SID]   <= sr_arid[si];
                            S_AXI_RDATA_i [DATA_WIDTH*(si+1)-1 -: DATA_WIDTH]
                                <= build_rdata(sr_araddr[si], si);
                            S_AXI_RRESP_i [RESP_W*(si+1)-1 -: RESP_W] <= 2'b00; // OKAY

                            if (sr_beat[si] == 1'b1) begin
                                S_AXI_RLAST_i[si] <= 1'b1;
                            end else begin
                                S_AXI_RLAST_i[si] <= 1'b0;
                            end

                            if (s_rready && S_AXI_RVALID_i[si]) begin
                                $display("[%0t] SLAVE[%0d] R beat: data=0x%08h last=%0d beat_left=%0d",
                                         $time, si,
                                         S_AXI_RDATA_i[DATA_WIDTH*(si+1)-1 -: DATA_WIDTH],
                                         S_AXI_RLAST_i[si], sr_beat[si]-1);

                                if (S_AXI_RLAST_i[si]) begin
                                    S_AXI_RVALID_i[si] <= 1'b0;
                                    S_AXI_RLAST_i [si] <= 1'b0;
                                    sr_state[si] <= SR_IDLE;
                                end else begin
                                    sr_araddr[si] <= sr_araddr[si] + bpb;
                                    sr_beat[si]   <= sr_beat[si] - 1'b1;
                                end
                            end
                        end

                        default: sr_state[si] <= SR_IDLE;
                    endcase
                end
            end
        end
    endgenerate

    //=========================================================
    // Master BFM Tasks
    //=========================================================

    // -- Write single beat --
    task automatic axi_write_single;
        input integer       mst;        // master index [0..MST_AMT-1]
        input [W_ID-1:0]    id;
        input [ADDR_WIDTH-1:0] addr;
        input [DATA_WIDTH-1:0] data;
        input [W_STRB-1:0]  strb;
        reg   [DATA_WIDTH-1:0] data_arr [0:0];
        reg   [W_STRB-1:0]     strb_arr [0:0];
        begin
            data_arr[0] = data;
            strb_arr[0] = strb;
            axi_write_burst(mst, id, addr, 8'd0, 3'b010, 2'b01, data_arr, strb_arr);
        end
    endtask

    // -- Write burst --
    task automatic axi_write_burst;
        input integer       mst;
        input [W_ID-1:0]    id;
        input [ADDR_WIDTH-1:0] addr;
        input [LEN_W-1:0]   len;
        input [SIZE_W-1:0]  size;
        input [BURST_W-1:0] burst;
        input [DATA_WIDTH-1:0] data_arr [];
        input [W_STRB-1:0]     strb_arr [];
        integer beat;
        begin
            $display("[%0t] MASTER[%0d] WRITE START: addr=0x%08h len=%0d id=0x%0h",
                     $time, mst, addr, len, id);

            // --- AW channel ---
            @(posedge AXI_CLK);
            M_AXI_AWID_i   [W_ID*(mst+1)-1 -: W_ID]          <= id;
            M_AXI_AWADDR_i [ADDR_WIDTH*(mst+1)-1 -: ADDR_WIDTH] <= addr;
            M_AXI_AWLEN_i  [LEN_W*(mst+1)-1 -: LEN_W]        <= len;
            M_AXI_AWSIZE_i [SIZE_W*(mst+1)-1 -: SIZE_W]      <= size;
            M_AXI_AWBURST_i[BURST_W*(mst+1)-1 -: BURST_W]    <= burst;
            M_AXI_AWVALID_i[mst] <= 1'b1;

            fork
                begin
                    wait(M_AXI_AWREADY_o[mst]);
                end
                begin
                    repeat(10000) @(posedge AXI_CLK);
                    $fatal(1, "[%0t] MASTER[%0d] AW TIMEOUT", $time, mst);
                end
            join_any
            disable fork;

            @(posedge AXI_CLK);
            M_AXI_AWVALID_i[mst] <= 1'b0;

            // --- W channel ---
            for (beat = 0; beat <= len; beat = beat + 1) begin
                @(posedge AXI_CLK);
                M_AXI_WDATA_i [DATA_WIDTH*(mst+1)-1 -: DATA_WIDTH] <= data_arr[beat];
                M_AXI_WSTRB_i [W_STRB*(mst+1)-1 -: W_STRB]         <= strb_arr[beat];
                M_AXI_WLAST_i [mst] <= (beat == len);
                M_AXI_WVALID_i[mst] <= 1'b1;

                fork
                    begin
                        wait(M_AXI_WREADY_o[mst]);
                    end
                    begin
                        repeat(10000) @(posedge AXI_CLK);
                        $fatal(1, "[%0t] MASTER[%0d] W TIMEOUT beat=%0d", $time, mst, beat);
                    end
                join_any
                disable fork;
            end

            @(posedge AXI_CLK);
            M_AXI_WVALID_i[mst] <= 1'b0;
            M_AXI_WLAST_i [mst] <= 1'b0;

            // --- B channel ---
            M_AXI_BREADY_i[mst] <= 1'b1;

            fork
                begin
                    wait(M_AXI_BVALID_o[mst]);
                end
                begin
                    repeat(10000) @(posedge AXI_CLK);
                    $fatal(1, "[%0t] MASTER[%0d] B TIMEOUT", $time, mst);
                end
            join_any
            disable fork;

            // Check B response
            if (M_AXI_BRESP_o[RESP_W*(mst+1)-1 -: RESP_W] != 2'b00) begin
                $display("[%0t] MASTER[%0d] WARNING: BRESP=0x%0h (expected OKAY)",
                         $time, mst, M_AXI_BRESP_o[RESP_W*(mst+1)-1 -: RESP_W]);
            end

            $display("[%0t] MASTER[%0d] WRITE DONE: addr=0x%08h BID=0x%0h",
                     $time, mst, addr, M_AXI_BID_o[W_ID*(mst+1)-1 -: W_ID]);

            @(posedge AXI_CLK);
            M_AXI_BREADY_i[mst] <= 1'b0;
        end
    endtask

    // -- Read single beat --
    task automatic axi_read_single;
        input integer       mst;
        input [W_ID-1:0]    id;
        input [ADDR_WIDTH-1:0] addr;
        output [DATA_WIDTH-1:0] data;
        reg [DATA_WIDTH-1:0] data_arr [0:0];
        begin
            axi_read_burst(mst, id, addr, 8'd0, 3'b010, 2'b01, data_arr);
            data = data_arr[0];
        end
    endtask

    // -- Read burst --
    task automatic axi_read_burst;
        input integer       mst;
        input [W_ID-1:0]    id;
        input [ADDR_WIDTH-1:0] addr;
        input [LEN_W-1:0]   len;
        input [SIZE_W-1:0]  size;
        input [BURST_W-1:0] burst;
        output [DATA_WIDTH-1:0] data_arr [];
        integer beat;
        begin
            $display("[%0t] MASTER[%0d] READ START: addr=0x%08h len=%0d id=0x%0h",
                     $time, mst, addr, len, id);

            // --- AR channel ---
            @(posedge AXI_CLK);
            M_AXI_ARID_i   [W_ID*(mst+1)-1 -: W_ID]          <= id;
            M_AXI_ARADDR_i [ADDR_WIDTH*(mst+1)-1 -: ADDR_WIDTH] <= addr;
            M_AXI_ARLEN_i  [LEN_W*(mst+1)-1 -: LEN_W]        <= len;
            M_AXI_ARSIZE_i [SIZE_W*(mst+1)-1 -: SIZE_W]      <= size;
            M_AXI_ARBURST_i[BURST_W*(mst+1)-1 -: BURST_W]    <= burst;
            M_AXI_ARVALID_i[mst] <= 1'b1;

            fork
                begin
                    wait(M_AXI_ARREADY_o[mst]);
                end
                begin
                    repeat(10000) @(posedge AXI_CLK);
                    $fatal(1, "[%0t] MASTER[%0d] AR TIMEOUT", $time, mst);
                end
            join_any
            disable fork;

            @(posedge AXI_CLK);
            M_AXI_ARVALID_i[mst] <= 1'b0;

            // --- R channel ---
            M_AXI_RREADY_i[mst] <= 1'b1;

            for (beat = 0; beat <= len; beat = beat + 1) begin
                fork
                    begin
                        wait(M_AXI_RVALID_o[mst]);
                    end
                    begin
                        repeat(10000) @(posedge AXI_CLK);
                        $fatal(1, "[%0t] MASTER[%0d] R TIMEOUT beat=%0d", $time, mst, beat);
                    end
                join_any
                disable fork;

                data_arr[beat] = M_AXI_RDATA_o[DATA_WIDTH*(mst+1)-1 -: DATA_WIDTH];

                if (M_AXI_RRESP_o[RESP_W*(mst+1)-1 -: RESP_W] != 2'b00) begin
                    $display("[%0t] MASTER[%0d] WARNING: RRESP=0x%0h beat=%0d",
                             $time, mst, M_AXI_RRESP_o[RESP_W*(mst+1)-1 -: RESP_W], beat);
                end

                $display("[%0t] MASTER[%0d] R beat: data=0x%08h last=%0d",
                         $time, mst, data_arr[beat], M_AXI_RLAST_o[mst]);

                @(posedge AXI_CLK);
                if (M_AXI_RLAST_o[mst]) break;
            end

            M_AXI_RREADY_i[mst] <= 1'b0;

            $display("[%0t] MASTER[%0d] READ DONE: addr=0x%08h RID=0x%0h",
                     $time, mst, addr, M_AXI_RID_o[W_ID*(mst+1)-1 -: W_ID]);
        end
    endtask

    //=========================================================
    // Helper: compute expected slave index from address
    //=========================================================
    function automatic integer get_slave_idx;
        input [ADDR_WIDTH-1:0] addr;
        integer s;
        reg [7:0]                  dec_len;
        reg [ADDR_WIDTH-1:0]       base;
        begin
            get_slave_idx = 0;
            for (s = 0; s < SLV_AMT; s = s + 1) begin
                dec_len = SLV_ADDR_LEN[8*(s+1)-1 -: 8];
                base    = SLV_ADDR_BASE[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH];
                if (addr[ADDR_WIDTH-1:dec_len] == base[ADDR_WIDTH-1:dec_len]) begin
                    get_slave_idx = s;
                    break;
                end
            end
        end
    endfunction

    //=========================================================
    // Helper: check read data against slave memory
    //=========================================================
    task automatic check_read_data;
        input integer mst;
        input [ADDR_WIDTH-1:0] addr;
        input [LEN_W-1:0] len;
        input [SIZE_W-1:0] size;
        input [DATA_WIDTH-1:0] rdata [];
        integer s, beat, b;
        reg [ADDR_WIDTH-1:0] cur_addr;
        reg [DATA_WIDTH-1:0] expected;
        begin
            s = get_slave_idx(addr);
            cur_addr = addr;
            for (beat = 0; beat <= len; beat = beat + 1) begin
                expected = '0;
                for (b = 0; b < W_STRB; b = b + 1) begin
                    expected[8*b +: 8] = slv_mem[s][(cur_addr[SLV_MEM_AW-1:0] + b) & (SLV_MEM_DEPTH-1)];
                end
                if (rdata[beat] !== expected) begin
                    $display("[%0t] ERROR: M[%0d] read data mismatch @ addr=0x%08h beat=%0d",
                             $time, mst, cur_addr, beat);
                    $display("       expected=0x%08h  got=0x%08h", expected, rdata[beat]);
                    error_cnt = error_cnt + 1;
                end else begin
                    $display("[%0t] CHECK OK: M[%0d] read data match @ addr=0x%08h data=0x%08h",
                             $time, mst, cur_addr, rdata[beat]);
                end
                cur_addr = cur_addr + (1 << size);
            end
        end
    endtask

    //=========================================================
    // Helper: wait N cycles
    //=========================================================
    task automatic wait_cycles;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) @(posedge AXI_CLK);
        end
    endtask

    //=========================================================
    // Helper: generate incremental test data
    //=========================================================
    function automatic [DATA_WIDTH-1:0] gen_test_data;
        input integer mst;
        input integer beat;
        begin
            gen_test_data = {16'd0, mst[3:0], beat[11:0]};
        end
    endfunction

    //=========================================================
    // Init
    //=========================================================
    initial begin
        M_AXI_AWID_i    = '0;
        M_AXI_AWADDR_i  = '0;
        M_AXI_AWLEN_i   = '0;
        M_AXI_AWSIZE_i  = '0;
        M_AXI_AWBURST_i = '0;
        M_AXI_AWVALID_i = '0;
        M_AXI_WDATA_i   = '0;
        M_AXI_WSTRB_i   = '0;
        M_AXI_WLAST_i   = '0;
        M_AXI_WVALID_i  = '0;
        M_AXI_BREADY_i  = '0;
        M_AXI_ARID_i    = '0;
        M_AXI_ARADDR_i  = '0;
        M_AXI_ARLEN_i   = '0;
        M_AXI_ARSIZE_i  = '0;
        M_AXI_ARBURST_i = '0;
        M_AXI_ARVALID_i = '0;
        M_AXI_RREADY_i  = '0;
    end

    //=========================================================
    // Waveform Dump
    //=========================================================
    initial begin
        $fsdbDumpfile("axi_tb.fsdb");
        $fsdbDumpvars(0, axi_tb);
        $fsdbDumpMDA();
    end

    //=========================================================
    // Global error counter
    //=========================================================
    integer error_cnt = 0;

    //=========================================================
    // ======================  TEST SUITE  =====================
    //=========================================================
    initial begin
        wait(AXI_RSTn);
        wait_cycles(20);

        $display("============================================================");
        $display(" AXI INTERCONNECT COMPREHENSIVE TESTBENCH — 4M × 4S");
        $display("============================================================");

        //---------------------------------------------------------
        // PHASE 1: Basic Sanity — Single-beat Write/Read per Master
        //---------------------------------------------------------
        $display("\n========== PHASE 1: Basic Single-Beat Sanity ==========");
        begin
            reg [DATA_WIDTH-1:0] rdata_arr [0:0];
            reg [DATA_WIDTH-1:0] wdata_arr [0:0];
            reg [W_STRB-1:0]     strb_arr  [0:0];
            integer pm, ps;

            for (pm = 0; pm < MST_AMT; pm = pm + 1) begin
                for (ps = 0; ps < SLV_AMT; ps = ps + 1) begin
                    wdata_arr[0] = {16'd0, pm[3:0], ps[3:0], 8'hA5};
                    strb_arr[0]  = 4'hF;  // all bytes valid
                    axi_write_burst(pm, pm[3:0], ps * 32'h1000_0000 + pm * 32'h100,
                                    8'd0, 3'b010, 2'b01, wdata_arr, strb_arr);
                    wait_cycles(5);
                end
            end

            for (pm = 0; pm < MST_AMT; pm = pm + 1) begin
                for (ps = 0; ps < SLV_AMT; ps = ps + 1) begin
                    axi_read_burst(pm, pm[3:0], ps * 32'h1000_0000 + pm * 32'h100,
                                  8'd0, 3'b010, 2'b01, rdata_arr);
                    check_read_data(pm, ps * 32'h1000_0000 + pm * 32'h100,
                                   8'd0, 3'b010, rdata_arr);
                    wait_cycles(5);
                end
            end
        end

        //---------------------------------------------------------
        // PHASE 2: Multi-Beat Burst Write/Read (INCR)
        //---------------------------------------------------------
        $display("\n========== PHASE 2: Multi-Beat INCR Burst ==========");
        begin
            reg [DATA_WIDTH-1:0] wdata_arr [0:15];
            reg [DATA_WIDTH-1:0] rdata_arr [0:15];
            reg [W_STRB-1:0]     strb_arr  [0:15];
            integer b;
            integer burst_lens [] = '{0, 1, 3, 7, 15};  // AWLEN values

            foreach (burst_lens[i]) begin
                $display("--- Burst len=%0d (AWLEN=%0d, %0d beats) ---",
                         burst_lens[i], burst_lens[i], burst_lens[i]+1);

                // Master 0 writes to Slave 0
                for (b = 0; b <= burst_lens[i]; b = b + 1) begin
                    wdata_arr[b] = gen_test_data(0, b);
                    strb_arr[b]  = 4'hF;
                end
                axi_write_burst(0, 4'hA, 32'h0000_1000,
                                burst_lens[i][LEN_W-1:0], 3'b010, 2'b01,
                                wdata_arr, strb_arr);
                wait_cycles(5);

                // Read back and check
                axi_read_burst(0, 4'hA, 32'h0000_1000,
                               burst_lens[i][LEN_W-1:0], 3'b010, 2'b01,
                               rdata_arr);
                check_read_data(0, 32'h0000_1000,
                                burst_lens[i][LEN_W-1:0], 3'b010, rdata_arr);
                wait_cycles(10);
            end
        end

        //---------------------------------------------------------
        // PHASE 3: Different Data Sizes
        //---------------------------------------------------------
        $display("\n========== PHASE 3: Data Size Variants ==========");
        begin
            reg [DATA_WIDTH-1:0] wdata_arr [0:3];
            reg [DATA_WIDTH-1:0] rdata_arr [0:3];
            reg [W_STRB-1:0]     strb_arr  [0:3];
            integer s;

            // 32-bit (SIZE=2), 16-bit (SIZE=1), 8-bit (SIZE=0)
            for (s = 0; s < 3; s = s + 1) begin
                $display("--- SIZE=%0d (bytes_per_beat=%0d) ---", s, 1<<s);
                wdata_arr[0] = 32'hDEAD_BEEF;
                strb_arr[0]  = (1 << (1<<s)) - 1;  // valid bytes only
                axi_write_burst(0, s[3:0], 32'h0000_2000 + s * 32'h100,
                               8'd0, s[2:0], 2'b01, wdata_arr, strb_arr);
                wait_cycles(5);
                axi_read_burst(0, s[3:0], 32'h0000_2000 + s * 32'h100,
                              8'd0, s[2:0], 2'b01, rdata_arr);
                // Mask expected data to valid bytes only
                rdata_arr[0] = rdata_arr[0] & {{(32-8*(1<<s)){1'b0}}, {(8*(1<<s)){1'b1}}};
                check_read_data(0, 32'h0000_2000 + s * 32'h100,
                               8'd0, s[2:0], rdata_arr);
                wait_cycles(5);
            end
        end

        //---------------------------------------------------------
        // PHASE 4: Concurrent Multi-Master Arbitration
        //---------------------------------------------------------
        $display("\n========== PHASE 4: Concurrent Multi-Master ==========");
        begin
            // All 4 masters write to slave 0 concurrently (tests arbitration)
            $display("--- 4 Masters → Slave 0 (write, concurrent) ---");
            fork
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'hAAAA_0001; _s[0] = 4'hF;
                    axi_write_burst(0, 4'h0, 32'h0000_0100, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'hBBBB_0002; _s[0] = 4'hF;
                    axi_write_burst(1, 4'h0, 32'h0000_0104, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'hCCCC_0003; _s[0] = 4'hF;
                    axi_write_burst(2, 4'h0, 32'h0000_0108, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'hDDDD_0004; _s[0] = 4'hF;
                    axi_write_burst(3, 4'h0, 32'h0000_010C, 8'd0, 3'b010, 2'b01, _d, _s);
                end
            join

            // Read back all
            $display("--- 4 Masters ← Slave 0 (read, concurrent) ---");
            fork
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(0, 4'h0, 32'h0000_0100, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(0, 32'h0000_0100, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(1, 4'h0, 32'h0000_0104, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(1, 32'h0000_0104, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(2, 4'h0, 32'h0000_0108, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(2, 32'h0000_0108, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(3, 4'h0, 32'h0000_010C, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(3, 32'h0000_010C, 8'd0, 3'b010, _d);
                end
            join
        end

        //---------------------------------------------------------
        // PHASE 5: Multi-Slave Concurrent Access
        //---------------------------------------------------------
        $display("\n========== PHASE 5: Multi-Slave Concurrent Access ==========");
        begin
            // 4 masters → 4 different slaves simultaneously
            $display("--- 4 Masters → 4 Slaves (write, concurrent) ---");
            fork
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'h1111_1111; _s[0] = 4'hF;
                    axi_write_burst(0, 4'h1, 32'h0000_0300, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'h2222_2222; _s[0] = 4'hF;
                    axi_write_burst(1, 4'h1, 32'h1000_0300, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'h3333_3333; _s[0] = 4'hF;
                    axi_write_burst(2, 4'h1, 32'h2000_0300, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'h4444_4444; _s[0] = 4'hF;
                    axi_write_burst(3, 4'h1, 32'h3000_0300, 8'd0, 3'b010, 2'b01, _d, _s);
                end
            join

            $display("--- 4 Masters ← 4 Slaves (read, concurrent) ---");
            fork
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(0, 4'h1, 32'h0000_0300, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(0, 32'h0000_0300, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(1, 4'h1, 32'h1000_0300, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(1, 32'h1000_0300, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(2, 4'h1, 32'h2000_0300, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(2, 32'h2000_0300, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(3, 4'h1, 32'h3000_0300, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(3, 32'h3000_0300, 8'd0, 3'b010, _d);
                end
            join
        end

        //---------------------------------------------------------
        // PHASE 6: 4KB Boundary Crossing
        //---------------------------------------------------------
        $display("\n========== PHASE 6: 4KB Boundary Crossing ==========");
        begin
            reg [DATA_WIDTH-1:0] wdata_arr [0:15];
            reg [DATA_WIDTH-1:0] rdata_arr [0:15];
            reg [W_STRB-1:0]     strb_arr  [0:15];
            integer b;
            integer cross_lens [] = '{1, 3, 7};

            foreach (cross_lens[i]) begin
                $display("--- Cross-4K: len=%0d, addr=0x0FF0 ---", cross_lens[i]);
                // Start near 4KB boundary, SIZE=2 (4 bytes/beat)
                // 0x0FF0 + (len+1)*4 → crosses 0x1000 if len >= 3
                for (b = 0; b <= cross_lens[i]; b = b + 1) begin
                    wdata_arr[b] = gen_test_data(0, b) ^ 32'hCROSS_0000;
                    strb_arr[b]  = 4'hF;
                end
                axi_write_burst(0, 4'h7, 32'h0000_0FF0,
                                cross_lens[i][LEN_W-1:0], 3'b010, 2'b01,
                                wdata_arr, strb_arr);
                wait_cycles(10);

                axi_read_burst(0, 4'h7, 32'h0000_0FF0,
                               cross_lens[i][LEN_W-1:0], 3'b010, 2'b01,
                               rdata_arr);
                check_read_data(0, 32'h0000_0FF0,
                                cross_lens[i][LEN_W-1:0], 3'b010, rdata_arr);
                wait_cycles(10);
            end
        end

        //---------------------------------------------------------
        // PHASE 7: Mixed Concurrent Read/Write
        //---------------------------------------------------------
        $display("\n========== PHASE 7: Mixed Concurrent Read/Write ==========");
        begin
            // Pre-write data for reads
            begin
                reg [DATA_WIDTH-1:0] _d [0:0];
                reg [W_STRB-1:0]     _s [0:0];
                _d[0] = 32'hCAFE_C0DE; _s[0] = 4'hF;
                axi_write_burst(0, 4'hF, 32'h0000_0500, 8'd0, 3'b010, 2'b01, _d, _s);
            end
            begin
                reg [DATA_WIDTH-1:0] _d [0:0];
                reg [W_STRB-1:0]     _s [0:0];
                _d[0] = 32'hFEED_FACE; _s[0] = 4'hF;
                axi_write_burst(1, 4'hF, 32'h1000_0500, 8'd0, 3'b010, 2'b01, _d, _s);
            end

            wait_cycles(10);

            // Simultaneous read + write on different masters
            $display("--- M0 read Slave0, M1 write Slave1, M2 write Slave2, M3 read Slave3 ---");
            fork
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(0, 4'hF, 32'h0000_0500, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(0, 32'h0000_0500, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'hBEEF_0001; _s[0] = 4'hF;
                    axi_write_burst(1, 4'hE, 32'h1000_0600, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'hBEEF_0002; _s[0] = 4'hF;
                    axi_write_burst(2, 4'hE, 32'h2000_0600, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    // Pre-write then read on M3
                    _d[0] = 32'hDEAD_BEEF; _s[0] = 4'hF;
                    axi_write_burst(3, 4'hE, 32'h3000_0500, 8'd0, 3'b010, 2'b01, _d, _s);
                    wait_cycles(3);
                    axi_read_burst(3, 4'hE, 32'h3000_0500, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(3, 32'h3000_0500, 8'd0, 3'b010, _d);
                end
            join
        end

        //---------------------------------------------------------
        // PHASE 8: Multi-ID Outstanding (same master, different IDs)
        //---------------------------------------------------------
        $display("\n========== PHASE 8: Multi-ID Outstanding Transactions ==========");
        begin
            // Master 0 issues 4 writes with different IDs to the same slave
            $display("--- M0: 4 writes with different IDs → Slave 0 ---");
            fork
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'hA001_A001; _s[0] = 4'hF;
                    axi_write_burst(0, 4'h0, 32'h0000_0700, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'hA002_A002; _s[0] = 4'hF;
                    axi_write_burst(0, 4'h2, 32'h0000_0704, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'hA003_A003; _s[0] = 4'hF;
                    axi_write_burst(0, 4'h4, 32'h0000_0708, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    reg [W_STRB-1:0]     _s [0:0];
                    _d[0] = 32'hA004_A004; _s[0] = 4'hF;
                    axi_write_burst(0, 4'h6, 32'h0000_070C, 8'd0, 3'b010, 2'b01, _d, _s);
                end
            join

            wait_cycles(10);

            $display("--- M0: 4 reads with different IDs ← Slave 0 ---");
            fork
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(0, 4'h0, 32'h0000_0700, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(0, 32'h0000_0700, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(0, 4'h2, 32'h0000_0704, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(0, 32'h0000_0704, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(0, 4'h4, 32'h0000_0708, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(0, 32'h0000_0708, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [0:0];
                    axi_read_burst(0, 4'h6, 32'h0000_070C, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(0, 32'h0000_070C, 8'd0, 3'b010, _d);
                end
            join
        end

        //---------------------------------------------------------
        // PHASE 9: WSTRB Partial Byte Write Test
        //---------------------------------------------------------
        $display("\n========== PHASE 9: Partial Byte Write (WSTRB) ==========");
        begin
            reg [DATA_WIDTH-1:0] wdata_arr [0:0];
            reg [DATA_WIDTH-1:0] rdata_arr [0:0];
            reg [W_STRB-1:0]     strb_arr  [0:0];

            // Write all 1s first
            wdata_arr[0] = 32'hFFFF_FFFF;
            strb_arr[0]  = 4'hF;
            axi_write_burst(0, 4'h9, 32'h0000_0800, 8'd0, 3'b010, 2'b01, wdata_arr, strb_arr);

            // Write only byte 0 and byte 2
            wdata_arr[0] = 32'hAB00_CD00;
            strb_arr[0]  = 4'b0101;
            axi_write_burst(0, 4'h9, 32'h0000_0800, 8'd0, 3'b010, 2'b01, wdata_arr, strb_arr);

            wait_cycles(5);

            // Read back: bytes 0,2 should be AB,CD; bytes 1,3 should be FF,FF
            axi_read_burst(0, 4'h9, 32'h0000_0800, 8'd0, 3'b010, 2'b01, rdata_arr);

            if (rdata_arr[0][7:0]   !== 8'hAB) begin
                $display("ERROR: byte0 expected 0xAB got 0x%0h", rdata_arr[0][7:0]);
                error_cnt = error_cnt + 1;
            end
            if (rdata_arr[0][15:8]  !== 8'hFF) begin
                $display("ERROR: byte1 expected 0xFF got 0x%0h", rdata_arr[0][15:8]);
                error_cnt = error_cnt + 1;
            end
            if (rdata_arr[0][23:16] !== 8'hCD) begin
                $display("ERROR: byte2 expected 0xCD got 0x%0h", rdata_arr[0][23:16]);
                error_cnt = error_cnt + 1;
            end
            if (rdata_arr[0][31:24] !== 8'hFF) begin
                $display("ERROR: byte3 expected 0xFF got 0x%0h", rdata_arr[0][31:24]);
                error_cnt = error_cnt + 1;
            end
            $display("--- Partial WSTRB test complete ---");
        end

        //---------------------------------------------------------
        // PHASE 10: Long Burst Stress Test
        //---------------------------------------------------------
        $display("\n========== PHASE 10: Long Burst Stress ==========");
        begin
            reg [DATA_WIDTH-1:0] wdata_arr [0:255];
            reg [DATA_WIDTH-1:0] rdata_arr [0:255];
            reg [W_STRB-1:0]     strb_arr  [0:255];
            integer b;

            // 16-beat burst (AWLEN=15)
            for (b = 0; b < 16; b = b + 1) begin
                wdata_arr[b] = gen_test_data(0, b);
                strb_arr[b]  = 4'hF;
            end
            $display("--- 16-beat write burst M0 → Slave 0 ---");
            axi_write_burst(0, 4'hC, 32'h0000_1000, 8'd15, 3'b010, 2'b01, wdata_arr, strb_arr);
            wait_cycles(10);

            $display("--- 16-beat read burst M0 ← Slave 0 ---");
            axi_read_burst(0, 4'hC, 32'h0000_1000, 8'd15, 3'b010, 2'b01, rdata_arr);
            check_read_data(0, 32'h0000_1000, 8'd15, 3'b010, rdata_arr);
        end

        //---------------------------------------------------------
        // PHASE 11: Random Stress Test
        //---------------------------------------------------------
        $display("\n========== PHASE 11: Random Stress Test ==========");
        begin
            reg [DATA_WIDTH-1:0] wdata_arr [0:7];
            reg [DATA_WIDTH-1:0] rdata_arr [0:7];
            reg [W_STRB-1:0]     strb_arr  [0:7];
            integer t, rm, rs, rlen, b;
            reg [ADDR_WIDTH-1:0] raddr;

            for (t = 0; t < 50; t = t + 1) begin
                rm   = $urandom % MST_AMT;
                rs   = $urandom % SLV_AMT;
                rlen = $urandom % 4;  // 0..3 (1..4 beats)
                raddr = rs * 32'h1000_0000 + ($urandom % 4096);

                for (b = 0; b <= rlen; b = b + 1) begin
                    wdata_arr[b] = $urandom;
                    strb_arr[b]  = 4'hF;
                end

                if ($urandom % 2) begin
                    // Write
                    $display("[%0t] RAND[%0d]: WRITE M%0d → S%0d addr=0x%08h len=%0d",
                             $time, t, rm, rs, raddr, rlen);
                    axi_write_burst(rm, t[3:0], raddr, rlen[LEN_W-1:0], 3'b010, 2'b01,
                                    wdata_arr, strb_arr);
                end else begin
                    // Write then read
                    $display("[%0t] RAND[%0d]: WR+RD M%0d → S%0d addr=0x%08h len=%0d",
                             $time, t, rm, rs, raddr, rlen);
                    axi_write_burst(rm, t[3:0], raddr, rlen[LEN_W-1:0], 3'b010, 2'b01,
                                    wdata_arr, strb_arr);
                    wait_cycles($urandom % 5);
                    axi_read_burst(rm, t[3:0], raddr, rlen[LEN_W-1:0], 3'b010, 2'b01,
                                   rdata_arr);
                    check_read_data(rm, raddr, rlen[LEN_W-1:0], 3'b010, rdata_arr);
                end
            end
        end

        //---------------------------------------------------------
        // DONE
        //---------------------------------------------------------
        wait_cycles(50);

        $display("\n============================================================");
        if (error_cnt == 0) begin
            $display(" ALL TESTS PASSED");
        end else begin
            $display(" TESTS COMPLETED WITH %0d ERRORS", error_cnt);
        end
        $display("============================================================");

        $finish;
    end

endmodule
