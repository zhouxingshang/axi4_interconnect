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

    localparam SLV_MEM_AW      = 13;                  // 8 KB per slave (byte addr)
    localparam SLV_MEM_WORD     = 2048;                // 2048 × 32-bit = 8 KB
    localparam SLV_MEM_DEPTH   = SLV_MEM_WORD;

    // Address map: bits [31:13] select slave → 8 KB per slave
    localparam [(SLV_AMT*ADDR_WIDTH)-1:0] SLV_ADDR_BASE = {
        32'h0000_6000,   // slave 3  (addr[31:13] = 3)
        32'h0000_4000,   // slave 2  (addr[31:13] = 2)
        32'h0000_2000,   // slave 1  (addr[31:13] = 1)
        32'h0000_0000    // slave 0  (addr[31:13] = 0)
    };
    localparam [(SLV_AMT*8)-1:0] SLV_ADDR_LEN = {
        8'd13,           // slave 3
        8'd13,           // slave 2
        8'd13,           // slave 1
        8'd13            // slave 0
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
        .arbiter_type   (1'b0)             // round-robin
    );

    //=========================================================
    // Global error counter
    //=========================================================
    integer error_cnt;
    integer phase_cnt  = 0;
    integer monitor_cnt = 0;

    //=========================================================
    // Slave Memory Model (per-slave 32-bit word storage)
    //=========================================================
    reg [DATA_WIDTH-1:0] slv_mem [0:SLV_AMT-1][0:SLV_MEM_DEPTH-1];

    // Initialize memory to known pattern
    integer mem_init_s, mem_init_a;
    initial begin
        for (mem_init_s = 0; mem_init_s < SLV_AMT; mem_init_s = mem_init_s + 1) begin
            for (mem_init_a = 0; mem_init_a < SLV_MEM_DEPTH; mem_init_a = mem_init_a + 1) begin
                slv_mem[mem_init_s][mem_init_a] = 32'h0000_0000;
            end
        end
    end

    //=========================================================
    // Slave Channel-Handler Queue Data Structures
    //=========================================================

    // -- B-done queue (write handler → B handler, per slave, depth 4) --
    reg [W_SID-1:0] b_done_awid    [0:SLV_AMT-1][0:3];
    reg [1:0]       b_done_wr_ptr  [0:SLV_AMT-1];
    reg [1:0]       b_done_rd_ptr  [0:SLV_AMT-1];
    integer         b_done_cnt     [0:SLV_AMT-1];

    // -- AR queue (AR handler → R handler, per slave, depth 4) --
    reg [W_SID-1:0]      ar_q_arid   [0:SLV_AMT-1][0:3];
    reg [ADDR_WIDTH-1:0] ar_q_araddr [0:SLV_AMT-1][0:3];
    reg [LEN_W:0]        ar_q_total  [0:SLV_AMT-1][0:3];  // beats = AWLEN+1
    reg [SIZE_W-1:0]     ar_q_arsize [0:SLV_AMT-1][0:3];
    reg [1:0]            ar_q_wr_ptr [0:SLV_AMT-1];
    reg [1:0]            ar_q_rd_ptr [0:SLV_AMT-1];
    integer              ar_q_cnt    [0:SLV_AMT-1];

    integer q_init_s;
    initial begin
        for (q_init_s = 0; q_init_s < SLV_AMT; q_init_s = q_init_s + 1) begin
            b_done_wr_ptr[q_init_s] = 2'b00;
            b_done_rd_ptr[q_init_s] = 2'b00;
            b_done_cnt[q_init_s]    = 0;
            ar_q_wr_ptr[q_init_s]   = 2'b00;
            ar_q_rd_ptr[q_init_s]   = 2'b00;
            ar_q_cnt[q_init_s]      = 0;
        end
    end

    //=========================================================
    // Helper: reconstruct read data from word memory
    //=========================================================
    function [DATA_WIDTH-1:0] build_rdata;
        input [ADDR_WIDTH-1:0] addr;
        input integer slv_idx;
        begin
            build_rdata = slv_mem[slv_idx][addr[SLV_MEM_AW-1:2] & (SLV_MEM_DEPTH-1)];
        end
    endfunction

    //=========================================================
    // Slave Write Handler (AW → W queue → memory → B queue)
    //=========================================================
    task automatic slv_write_handler(int slv_id);
        // Pending AW transaction queue (depth 4)
        reg [W_SID-1:0]      q_awid    [0:3];
        reg [ADDR_WIDTH-1:0] q_awaddr  [0:3];
        reg [LEN_W:0]        q_total   [0:3];  // beats = AWLEN+1
        reg [SIZE_W-1:0]     q_awsize  [0:3];
        integer              q_wr_ptr, q_rd_ptr, q_cnt;

        // Current active transaction (being written)
        reg [W_SID-1:0]      cur_awid;
        reg [ADDR_WIDTH-1:0] cur_awaddr;
        reg [SIZE_W-1:0]     cur_awsize;
        reg [LEN_W:0]        cur_total;
        reg [LEN_W:0]        cur_beat_cnt;
        reg                  cur_active;

        reg [DATA_WIDTH-1:0] wdata;
        reg                  wlast;
        begin
            q_wr_ptr   = 0;
            q_rd_ptr   = 0;
            q_cnt      = 0;
            cur_active = 1'b0;

            forever begin
                @(posedge AXI_CLK);

                // Backpressure: deassert AWREADY when queue full
                S_AXI_AWREADY_i[slv_id] <= (q_cnt < 4);

                // AW handshake → enqueue transaction
                if (S_AXI_AWVALID_o[slv_id] && S_AXI_AWREADY_i[slv_id]) begin
                    q_awid  [q_wr_ptr] = S_AXI_AWID_o  [W_SID*(slv_id+1)-1 -: W_SID];
                    q_awaddr[q_wr_ptr] = S_AXI_AWADDR_o[ADDR_WIDTH*(slv_id+1)-1 -: ADDR_WIDTH];
                    q_awsize[q_wr_ptr] = S_AXI_AWSIZE_o[SIZE_W*(slv_id+1)-1 -: SIZE_W];
                    q_total [q_wr_ptr] = {1'b0, S_AXI_AWLEN_o[LEN_W*(slv_id+1)-1 -: LEN_W]} + 1'b1;
                    q_wr_ptr = (q_wr_ptr + 1) & 3;
                    q_cnt    = q_cnt + 1;
                    $display("[%0t] SLAVE[%0d] AW accepted: addr=0x%08h len=%0d size=%0d id=0x%0h",
                             $time, slv_id, q_awaddr[(q_wr_ptr-1)&3], S_AXI_AWLEN_o[LEN_W*(slv_id+1)-1 -: LEN_W],
                             q_awsize[(q_wr_ptr-1)&3], q_awid[(q_wr_ptr-1)&3]);
                end

                // Dequeue next transaction if idle
                if (!cur_active && q_cnt > 0) begin
                    cur_awid     = q_awid  [q_rd_ptr];
                    cur_awaddr   = q_awaddr[q_rd_ptr];
                    cur_awsize   = q_awsize[q_rd_ptr];
                    cur_total    = q_total [q_rd_ptr];
                    cur_beat_cnt = 0;
                    cur_active   = 1'b1;
                    q_rd_ptr = (q_rd_ptr + 1) & 3;
                    q_cnt    = q_cnt - 1;
                end

                // W data handshake
                if (cur_active && S_AXI_WVALID_o[slv_id] && S_AXI_WREADY_i[slv_id]) begin
                    wdata = S_AXI_WDATA_o[DATA_WIDTH*(slv_id+1)-1 -: DATA_WIDTH];
                    wlast = S_AXI_WLAST_o[slv_id];

                    // Write full word to memory
                    slv_mem[slv_id][cur_awaddr[SLV_MEM_AW-1:2] & (SLV_MEM_DEPTH-1)]
                        <= wdata;

                    $display("[%0t] SLAVE[%0d] W beat: data=0x%08h last=%0d beat_left=%0d",
                             $time, slv_id, wdata, wlast, cur_total - cur_beat_cnt - 1);

                    if (wlast) begin
                        cur_active = 1'b0;
                        // Push to shared B-done queue
                        b_done_awid[slv_id][b_done_wr_ptr[slv_id]] = cur_awid;
                        b_done_wr_ptr[slv_id] = (b_done_wr_ptr[slv_id] + 1) & 3;
                        b_done_cnt[slv_id]    = b_done_cnt[slv_id] + 1;
                    end else begin
                        // Advance address for next beat
                        cur_awaddr   = cur_awaddr + (8'd1 << cur_awsize);
                        cur_beat_cnt = cur_beat_cnt + 1;
                    end
                end

                // Gate WREADY: only accept W data when actively processing a transaction
                S_AXI_WREADY_i[slv_id] <= cur_active;
            end
        end
    endtask

    //=========================================================
    // Slave B Response Handler (pops from B-done queue)
    //=========================================================
    task automatic slv_b_handler(int slv_id);
        forever begin
            @(posedge AXI_CLK);

            // Drive B response if queue has pending
            if (!S_AXI_BVALID_i[slv_id] && b_done_cnt[slv_id] > 0) begin
                S_AXI_BID_i  [W_SID*(slv_id+1)-1 -: W_SID] <= b_done_awid[slv_id][b_done_rd_ptr[slv_id]];
                S_AXI_BRESP_i[RESP_W*(slv_id+1)-1 -: RESP_W] <= 2'b00;
                S_AXI_BVALID_i[slv_id] <= 1'b1;
                b_done_rd_ptr[slv_id] = (b_done_rd_ptr[slv_id] + 1) & 3;
                b_done_cnt[slv_id]    = b_done_cnt[slv_id] - 1;
            end

            // B handshake complete
            if (S_AXI_BVALID_i[slv_id] && S_AXI_BREADY_o[slv_id]) begin
                S_AXI_BVALID_i[slv_id] <= 1'b0;
                $display("[%0t] SLAVE[%0d] B sent: id=0x%0h", $time, slv_id,
                         S_AXI_BID_i[W_SID*(slv_id+1)-1 -: W_SID]);
            end
        end
    endtask

    //=========================================================
    // Slave AR Handler (enqueues AR transactions)
    //=========================================================
    task automatic slv_ar_handler(int slv_id);
        forever begin
            @(posedge AXI_CLK);

            // Backpressure: deassert ARREADY when queue full
            S_AXI_ARREADY_i[slv_id] <= (ar_q_cnt[slv_id] < 4);

            if (S_AXI_ARVALID_o[slv_id] && S_AXI_ARREADY_i[slv_id]) begin
                ar_q_arid  [slv_id][ar_q_wr_ptr[slv_id]] = S_AXI_ARID_o  [W_SID*(slv_id+1)-1 -: W_SID];
                ar_q_araddr[slv_id][ar_q_wr_ptr[slv_id]] = S_AXI_ARADDR_o[ADDR_WIDTH*(slv_id+1)-1 -: ADDR_WIDTH];
                ar_q_arsize[slv_id][ar_q_wr_ptr[slv_id]] = S_AXI_ARSIZE_o[SIZE_W*(slv_id+1)-1 -: SIZE_W];
                ar_q_total [slv_id][ar_q_wr_ptr[slv_id]] = {1'b0, S_AXI_ARLEN_o[LEN_W*(slv_id+1)-1 -: LEN_W]} + 1'b1;
                ar_q_wr_ptr[slv_id] = (ar_q_wr_ptr[slv_id] + 1) & 3;
                ar_q_cnt[slv_id]    = ar_q_cnt[slv_id] + 1;
                $display("[%0t] SLAVE[%0d] AR accepted: addr=0x%08h len=%0d size=%0d id=0x%0h",
                         $time, slv_id,
                         S_AXI_ARADDR_o[ADDR_WIDTH*(slv_id+1)-1 -: ADDR_WIDTH],
                         S_AXI_ARLEN_o[LEN_W*(slv_id+1)-1 -: LEN_W],
                         S_AXI_ARSIZE_o[SIZE_W*(slv_id+1)-1 -: SIZE_W],
                         S_AXI_ARID_o[W_SID*(slv_id+1)-1 -: W_SID]);
            end
        end
    endtask

    //=========================================================
    // Slave R Data Handler (pops AR queue → drives R beats)
    //=========================================================
    task automatic slv_r_handler(int slv_id);
        reg [W_SID-1:0]      cur_arid;
        reg [ADDR_WIDTH-1:0] cur_araddr;
        reg [SIZE_W-1:0]     cur_arsize;
        reg [LEN_W:0]        cur_total;
        reg [LEN_W:0]        cur_beat_cnt;
        reg                  cur_active;

        begin
            cur_active = 1'b0;

            forever begin
                @(posedge AXI_CLK);

                // Dequeue next transaction if idle
                if (!cur_active && ar_q_cnt[slv_id] > 0) begin
                    cur_arid     = ar_q_arid  [slv_id][ar_q_rd_ptr[slv_id]];
                    cur_araddr   = ar_q_araddr[slv_id][ar_q_rd_ptr[slv_id]];
                    cur_arsize   = ar_q_arsize[slv_id][ar_q_rd_ptr[slv_id]];
                    cur_total    = ar_q_total [slv_id][ar_q_rd_ptr[slv_id]];
                    cur_beat_cnt = 0;
                    cur_active   = 1'b1;
                    ar_q_rd_ptr[slv_id] = (ar_q_rd_ptr[slv_id] + 1) & 3;
                    ar_q_cnt[slv_id]    = ar_q_cnt[slv_id] - 1;

                    // Drive first R beat
                    S_AXI_RVALID_i[slv_id] <= 1'b1;
                    S_AXI_RID_i  [W_SID*(slv_id+1)-1 -: W_SID] <= cur_arid;
                    S_AXI_RDATA_i[DATA_WIDTH*(slv_id+1)-1 -: DATA_WIDTH]
                        <= build_rdata(cur_araddr, slv_id);
                    S_AXI_RRESP_i[RESP_W*(slv_id+1)-1 -: RESP_W] <= 2'b00;
                    S_AXI_RLAST_i[slv_id] <= (cur_total == 1'b1);
                end

                // R handshake: advance or finish
                if (cur_active && S_AXI_RVALID_i[slv_id] && S_AXI_RREADY_o[slv_id]) begin
                    if (S_AXI_RLAST_i[slv_id]) begin
                        S_AXI_RVALID_i[slv_id] <= 1'b0;
                        S_AXI_RLAST_i [slv_id] <= 1'b0;
                        cur_active = 1'b0;
                        $display("[%0t] SLAVE[%0d] R last beat done: id=0x%0h", $time, slv_id, cur_arid);
                    end else begin
                        cur_beat_cnt = cur_beat_cnt + 1;
                        cur_araddr   = cur_araddr + (8'd1 << cur_arsize);
                        S_AXI_RDATA_i[DATA_WIDTH*(slv_id+1)-1 -: DATA_WIDTH]
                            <= build_rdata(cur_araddr, slv_id);
                        S_AXI_RLAST_i[slv_id] <= (cur_beat_cnt == cur_total - 1);
                        $display("[%0t] SLAVE[%0d] R beat: data=0x%08h last=%0d beat_left=%0d",
                                 $time, slv_id,
                                 build_rdata(cur_araddr, slv_id),
                                 (cur_beat_cnt == cur_total - 1), cur_total - cur_beat_cnt - 1);
                    end
                end
            end
        end
    endtask

    //=========================================================
    // Fork per-slave handler tasks
    //=========================================================
    integer slv_fork_idx;
    initial begin
        for (slv_fork_idx = 0; slv_fork_idx < SLV_AMT; slv_fork_idx = slv_fork_idx + 1) begin
            automatic int sid = slv_fork_idx;
            fork
                slv_write_handler(sid);
                slv_b_handler(sid);
                slv_ar_handler(sid);
                slv_r_handler(sid);
            join_none
        end
    end

    //=========================================================
    // Master-Side Background W/B Infrastructure
    //=========================================================

    // -- Per-master W data queue (depth 4, circular buffer) --
    reg [DATA_WIDTH-1:0] m_wr_data_q  [0:MST_AMT-1][0:3][0:255];
    reg [LEN_W-1:0]      m_wr_len_q   [0:MST_AMT-1][0:3];
    reg [1:0]            m_wr_q_wr_ptr [0:MST_AMT-1];
    reg [1:0]            m_wr_q_rd_ptr [0:MST_AMT-1];
    integer              m_wr_q_cnt    [0:MST_AMT-1];

    // -- Per-master B response queue (depth 4, circular buffer) --
    reg [RESP_W-1:0]     m_b_resp_q   [0:MST_AMT-1][0:3];
    reg [1:0]            m_b_q_wr_ptr [0:MST_AMT-1];
    reg [1:0]            m_b_q_rd_ptr [0:MST_AMT-1];
    integer              m_b_q_cnt    [0:MST_AMT-1];

    // -- Per-master R data queue (depth 4, each entry = one burst) --
    reg [DATA_WIDTH-1:0] m_rd_data_q  [0:MST_AMT-1][0:3][0:255];
    reg [RESP_W-1:0]     m_rd_resp_q  [0:MST_AMT-1][0:3][0:255];
    reg [7:0]            m_rd_len_q   [0:MST_AMT-1][0:3];  // total beats stored
    reg [1:0]            m_rd_q_wr_ptr [0:MST_AMT-1];
    reg [1:0]            m_rd_q_rd_ptr [0:MST_AMT-1];
    integer              m_rd_q_cnt    [0:MST_AMT-1];

    integer mst_q_init;
    initial begin
        for (mst_q_init = 0; mst_q_init < MST_AMT; mst_q_init = mst_q_init + 1) begin
            m_wr_q_wr_ptr[mst_q_init] = 2'b00;
            m_wr_q_rd_ptr[mst_q_init] = 2'b00;
            m_wr_q_cnt[mst_q_init]    = 0;
            m_b_q_wr_ptr[mst_q_init]  = 2'b00;
            m_b_q_rd_ptr[mst_q_init]  = 2'b00;
            m_b_q_cnt[mst_q_init]     = 0;
            m_rd_q_wr_ptr[mst_q_init] = 2'b00;
            m_rd_q_rd_ptr[mst_q_init] = 2'b00;
            m_rd_q_cnt[mst_q_init]    = 0;
        end
    end

    // -- Per-master W driver (background, drains W queue, WSTRB always 4'hF) --
    task automatic mst_w_driver(int mst);
        reg [DATA_WIDTH-1:0] wdata_arr [0:255];
        reg [LEN_W-1:0]      wlen;
        integer beat;
        begin
            forever begin
                @(posedge AXI_CLK);
                if (!M_AXI_WVALID_i[mst] && m_wr_q_cnt[mst] > 0) begin
                    wlen = m_wr_len_q[mst][m_wr_q_rd_ptr[mst]];
                    for (beat = 0; beat <= wlen; beat = beat + 1) begin
                        wdata_arr[beat] = m_wr_data_q[mst][m_wr_q_rd_ptr[mst]][beat];
                    end
                    m_wr_q_rd_ptr[mst] = (m_wr_q_rd_ptr[mst] + 1) & 3;
                    m_wr_q_cnt[mst]    = m_wr_q_cnt[mst] - 1;

                    for (beat = 0; beat <= wlen; beat = beat + 1) begin
                        @(posedge AXI_CLK);
                        M_AXI_WDATA_i [DATA_WIDTH*(mst+1)-1 -: DATA_WIDTH] <= wdata_arr[beat];
                        M_AXI_WSTRB_i [W_STRB*(mst+1)-1 -: W_STRB]         <= {W_STRB{1'b1}};
                        M_AXI_WLAST_i [mst] <= (beat == wlen);
                        M_AXI_WVALID_i[mst] <= 1'b1;
                        fork
                            begin wait(M_AXI_WREADY_o[mst]); end
                            begin repeat(10000) @(posedge AXI_CLK);
                                  $fatal(1, "[%0t] W_DRIVER[%0d] TIMEOUT beat=%0d", $time, mst, beat); end
                        join_any
                        disable fork;
                    end
                    @(posedge AXI_CLK);
                    M_AXI_WVALID_i[mst] <= 1'b0;
                    M_AXI_WLAST_i [mst] <= 1'b0;
                end
            end
        end
    endtask

    // -- Per-master B listener (background, BREADY=1, captures B to queue) --
    task automatic mst_b_listener(int mst);
        forever begin
            @(posedge AXI_CLK);
            M_AXI_BREADY_i[mst] <= 1'b1;
            if (M_AXI_BVALID_o[mst] && M_AXI_BREADY_i[mst]) begin
                m_b_resp_q  [mst][m_b_q_wr_ptr[mst]]
                    = M_AXI_BRESP_o[RESP_W*(mst+1)-1 -: RESP_W];
                m_b_q_wr_ptr[mst] = (m_b_q_wr_ptr[mst] + 1) & 3;
                m_b_q_cnt[mst]    = m_b_q_cnt[mst] + 1;
                $display("[%0t] B_LISTENER[%0d]: B resp=0x%0h BID=0x%0h",
                         $time, mst, M_AXI_BRESP_o[RESP_W*(mst+1)-1 -: RESP_W],
                         M_AXI_BID_o[W_ID*(mst+1)-1 -: W_ID]);
            end
        end
    endtask

    // -- Per-master R listener (background, RREADY=1, captures R beats to queue) --
    task automatic mst_r_listener(int mst);
        reg [DATA_WIDTH-1:0] cur_data [0:255];
        reg [RESP_W-1:0]     cur_resp [0:255];
        reg [7:0]            cur_len;
        reg                  cur_active;
        integer              beat;
        begin
            cur_active = 0;
            forever begin
                @(posedge AXI_CLK);
                M_AXI_RREADY_i[mst] <= 1'b1;

                if (M_AXI_RVALID_o[mst] && M_AXI_RREADY_i[mst]) begin
                    if (!cur_active) begin
                        cur_len    = 0;
                        cur_active = 1;
                    end
                    cur_data[cur_len] = M_AXI_RDATA_o[DATA_WIDTH*(mst+1)-1 -: DATA_WIDTH];
                    cur_resp[cur_len] = M_AXI_RRESP_o[RESP_W*(mst+1)-1 -: RESP_W];
                    cur_len = cur_len + 1;

                    $display("[%0t] R_LISTENER[%0d]: beat=%0d data=0x%08h rlast=%0d",
                             $time, mst, cur_len-1, cur_data[cur_len-1], M_AXI_RLAST_o[mst]);

                    if (M_AXI_RLAST_o[mst]) begin
                        // Push completed burst to shared queue
                        m_rd_len_q[mst][m_rd_q_wr_ptr[mst]] = cur_len;
                        for (beat = 0; beat < cur_len; beat = beat + 1) begin
                            m_rd_data_q[mst][m_rd_q_wr_ptr[mst]][beat] = cur_data[beat];
                            m_rd_resp_q[mst][m_rd_q_wr_ptr[mst]][beat] = cur_resp[beat];
                        end
                        m_rd_q_wr_ptr[mst] = (m_rd_q_wr_ptr[mst] + 1) & 3;
                        m_rd_q_cnt[mst]    = m_rd_q_cnt[mst] + 1;
                        cur_active = 0;
                    end
                end
            end
        end
    endtask

    // Fork per-master background tasks
    integer mst_bg_idx;
    initial begin
        for (mst_bg_idx = 0; mst_bg_idx < MST_AMT; mst_bg_idx = mst_bg_idx + 1) begin
            automatic int mid = mst_bg_idx;
            fork
                mst_w_driver(mid);
                mst_b_listener(mid);
                mst_r_listener(mid);
            join_none
        end
    end

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
        reg   [DATA_WIDTH-1:0] data_arr [];
        reg   [W_STRB-1:0]     strb_arr [];
        begin
            data_arr = new[1];
            strb_arr = new[1];
            data_arr[0] = data;
            strb_arr[0] = strb;
            axi_write_burst(mst, id, addr, 8'd0, 3'b010, 2'b01, data_arr, strb_arr);
        end
    endtask

    // -- Write burst (AW/W separated: AW sent immediately, W queued to background driver) --
    task automatic axi_write_burst;
        input integer       mst;
        input [W_ID-1:0]    id;
        input [ADDR_WIDTH-1:0] addr;
        input [LEN_W-1:0]   len;
        input [SIZE_W-1:0]  size;
        input [BURST_W-1:0] burst;
        input [DATA_WIDTH-1:0] data_arr [];
        input [W_STRB-1:0]     strb_arr [];
        reg [RESP_W-1:0] bresp;
        integer beat;
        begin
            $display("[%0t] MASTER[%0d] WRITE START: addr=0x%08h len=%0d id=0x%0h",
                     $time, mst, addr, len, id);

            // Send AW first (blocks until cross_4k_if accepts into FIFO)
            axi_aw_send(mst, id, addr, len, size, burst);

            // Wait for AW to propagate through pipeline FIFO → crossbar M2S,
            // so pending_aw_cnt is updated before W data arrives
            repeat(3) @(posedge AXI_CLK);

            // Queue W data after pipeline delay
            while (m_wr_q_cnt[mst] >= 4) @(posedge AXI_CLK);
            m_wr_len_q[mst][m_wr_q_wr_ptr[mst]] = len;
            for (beat = 0; beat <= len; beat = beat + 1) begin
                m_wr_data_q[mst][m_wr_q_wr_ptr[mst]][beat] = data_arr[beat];
            end
            m_wr_q_wr_ptr[mst] = (m_wr_q_wr_ptr[mst] + 1) & 3;
            m_wr_q_cnt[mst]    = m_wr_q_cnt[mst] + 1;

            // Wait for B response from shared queue (filled by background mst_b_listener)
            while (m_b_q_cnt[mst] == 0) @(posedge AXI_CLK);
            bresp = m_b_resp_q[mst][m_b_q_rd_ptr[mst]];
            m_b_q_rd_ptr[mst] = (m_b_q_rd_ptr[mst] + 1) & 3;
            m_b_q_cnt[mst]    = m_b_q_cnt[mst] - 1;

            if (bresp != 2'b00) begin
                $display("[%0t] MASTER[%0d] WARNING: BRESP=0x%0h (expected OKAY)",
                         $time, mst, bresp);
            end

            $display("[%0t] MASTER[%0d] WRITE DONE: addr=0x%08h BID=0x%0h",
                     $time, mst, addr, M_AXI_BID_o[W_ID*(mst+1)-1 -: W_ID]);
        end
    endtask

    // -- Read single beat --
    task automatic axi_read_single;
        input integer       mst;
        input [W_ID-1:0]    id;
        input [ADDR_WIDTH-1:0] addr;
        output [DATA_WIDTH-1:0] data;
        reg [DATA_WIDTH-1:0] data_arr [];
        begin
            data_arr = new[1];
            axi_read_burst(mst, id, addr, 8'd0, 3'b010, 2'b01, data_arr);
            data = data_arr[0];
        end
    endtask

    // -- Read burst (AR/R separated: AR sent immediately, R captured by background listener) --
    task automatic axi_read_burst;
        input integer       mst;
        input [W_ID-1:0]    id;
        input [ADDR_WIDTH-1:0] addr;
        input [LEN_W-1:0]   len;
        input [SIZE_W-1:0]  size;
        input [BURST_W-1:0] burst;
        ref [DATA_WIDTH-1:0] data_arr [];
        reg [7:0] rlen;
        integer beat;
        begin
            $display("[%0t] MASTER[%0d] READ START: addr=0x%08h len=%0d id=0x%0h",
                     $time, mst, addr, len, id);

            // Send AR immediately (non-blocking)
            axi_ar_send(mst, id, addr, len, size, burst);

            // Wait for R data from shared queue (filled by background mst_r_listener)
            while (m_rd_q_cnt[mst] == 0) @(posedge AXI_CLK);
            rlen = m_rd_len_q[mst][m_rd_q_rd_ptr[mst]];

            for (beat = 0; beat < rlen; beat = beat + 1) begin
                data_arr[beat] = m_rd_data_q[mst][m_rd_q_rd_ptr[mst]][beat];
                if (m_rd_resp_q[mst][m_rd_q_rd_ptr[mst]][beat] != 2'b00) begin
                    $display("[%0t] MASTER[%0d] WARNING: RRESP=0x%0h beat=%0d",
                             $time, mst, m_rd_resp_q[mst][m_rd_q_rd_ptr[mst]][beat], beat);
                end
                $display("[%0t] MASTER[%0d] R beat: data=0x%08h last=%0d",
                         $time, mst, data_arr[beat], (beat == rlen - 1));
            end

            m_rd_q_rd_ptr[mst] = (m_rd_q_rd_ptr[mst] + 1) & 3;
            m_rd_q_cnt[mst]    = m_rd_q_cnt[mst] - 1;

            $display("[%0t] MASTER[%0d] READ DONE: addr=0x%08h RID=0x%0h",
                     $time, mst, addr, M_AXI_RID_o[W_ID*(mst+1)-1 -: W_ID]);
        end
    endtask

    //=========================================================
    // Split Master BFM Tasks (non-blocking per-channel operations)
    //=========================================================

    // -- Send AW channel only, return after handshake --
    task automatic axi_aw_send;
        input integer       mst;
        input [W_ID-1:0]    id;
        input [ADDR_WIDTH-1:0] addr;
        input [LEN_W-1:0]   len;
        input [SIZE_W-1:0]  size;
        input [BURST_W-1:0] burst;
        begin
            @(posedge AXI_CLK);
            M_AXI_AWID_i   [W_ID*(mst+1)-1 -: W_ID]          <= id;
            M_AXI_AWADDR_i [ADDR_WIDTH*(mst+1)-1 -: ADDR_WIDTH] <= addr;
            M_AXI_AWLEN_i  [LEN_W*(mst+1)-1 -: LEN_W]        <= len;
            M_AXI_AWSIZE_i [SIZE_W*(mst+1)-1 -: SIZE_W]      <= size;
            M_AXI_AWBURST_i[BURST_W*(mst+1)-1 -: BURST_W]    <= burst;
            M_AXI_AWVALID_i[mst] <= 1'b1;

            // Wait one cycle for AWVALID to propagate, then poll AWREADY at posedge
            @(posedge AXI_CLK);
            while (!M_AXI_AWREADY_o[mst]) @(posedge AXI_CLK);

            // Deassert immediately (arbiter uses S_AWREADY, no race)
            M_AXI_AWVALID_i[mst] <= 1'b0;
        end
    endtask

    // -- Wait for B response, return BRESP (reads from shared B queue) --
    task automatic axi_b_recv;
        input integer       mst;
        output [RESP_W-1:0] bresp;
        begin
            // B listener (background) always has BREADY=1 and fills the queue
            while (m_b_q_cnt[mst] == 0) @(posedge AXI_CLK);
            bresp = m_b_resp_q[mst][m_b_q_rd_ptr[mst]];
            m_b_q_rd_ptr[mst] = (m_b_q_rd_ptr[mst] + 1) & 3;
            m_b_q_cnt[mst]    = m_b_q_cnt[mst] - 1;
        end
    endtask

    // -- Combined pipelined write (AW → queued W → B, same architecture as axi_write_burst) --
    task automatic axi_write_pipelined;
        input integer       mst;
        input [W_ID-1:0]    id;
        input [ADDR_WIDTH-1:0] addr;
        input [LEN_W-1:0]   len;
        input [SIZE_W-1:0]  size;
        input [BURST_W-1:0] burst;
        input [DATA_WIDTH-1:0] data_arr [];
        input [W_STRB-1:0]     strb_arr [];
        reg [RESP_W-1:0] bresp;
        integer beat;
        begin
            $display("[%0t] MASTER[%0d] WRITE START: addr=0x%08h len=%0d id=0x%0h",
                     $time, mst, addr, len, id);

            // Queue W data → background mst_w_driver sends it
            while (m_wr_q_cnt[mst] >= 4) @(posedge AXI_CLK);
            m_wr_len_q[mst][m_wr_q_wr_ptr[mst]] = len;
            for (beat = 0; beat <= len; beat = beat + 1) begin
                m_wr_data_q[mst][m_wr_q_wr_ptr[mst]][beat] = data_arr[beat];
            end
            m_wr_q_wr_ptr[mst] = (m_wr_q_wr_ptr[mst] + 1) & 3;
            m_wr_q_cnt[mst]    = m_wr_q_cnt[mst] + 1;

            // Send AW immediately (non-blocking)
            axi_aw_send(mst, id, addr, len, size, burst);

            // Wait for B from shared queue
            while (m_b_q_cnt[mst] == 0) @(posedge AXI_CLK);
            bresp = m_b_resp_q[mst][m_b_q_rd_ptr[mst]];
            m_b_q_rd_ptr[mst] = (m_b_q_rd_ptr[mst] + 1) & 3;
            m_b_q_cnt[mst]    = m_b_q_cnt[mst] - 1;

            if (bresp != 2'b00) begin
                $display("[%0t] MASTER[%0d] WARNING: BRESP=0x%0h (expected OKAY)",
                         $time, mst, bresp);
            end
            $display("[%0t] MASTER[%0d] WRITE DONE: addr=0x%08h BID=0x%0h",
                     $time, mst, addr, M_AXI_BID_o[W_ID*(mst+1)-1 -: W_ID]);
        end
    endtask

    // -- Send AR channel only, return after handshake --
    task automatic axi_ar_send;
        input integer       mst;
        input [W_ID-1:0]    id;
        input [ADDR_WIDTH-1:0] addr;
        input [LEN_W-1:0]   len;
        input [SIZE_W-1:0]  size;
        input [BURST_W-1:0] burst;
        begin
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
        end
    endtask

    // -- Receive R beats, fill data_arr (reads from shared R queue) --
    task automatic axi_r_recv;
        input integer       mst;
        ref [DATA_WIDTH-1:0] data_arr [];
        input [LEN_W-1:0]   len;
        reg [7:0] rlen;
        integer beat;
        begin
            // R listener (background) always has RREADY=1 and fills the queue
            while (m_rd_q_cnt[mst] == 0) @(posedge AXI_CLK);
            rlen = m_rd_len_q[mst][m_rd_q_rd_ptr[mst]];
            data_arr = new[rlen];

            for (beat = 0; beat < rlen; beat = beat + 1) begin
                data_arr[beat] = m_rd_data_q[mst][m_rd_q_rd_ptr[mst]][beat];
                if (m_rd_resp_q[mst][m_rd_q_rd_ptr[mst]][beat] != 2'b00) begin
                    $display("[%0t] MASTER[%0d] WARNING: RRESP=0x%0h beat=%0d",
                             $time, mst, m_rd_resp_q[mst][m_rd_q_rd_ptr[mst]][beat], beat);
                end
                $display("[%0t] MASTER[%0d] R beat: data=0x%08h last=%0d",
                         $time, mst, data_arr[beat], (beat == rlen - 1));
            end

            m_rd_q_rd_ptr[mst] = (m_rd_q_rd_ptr[mst] + 1) & 3;
            m_rd_q_cnt[mst]    = m_rd_q_cnt[mst] - 1;
        end
    endtask

    // -- Combined pipelined read (AR → R using split tasks) --
    task automatic axi_read_pipelined;
        input integer       mst;
        input [W_ID-1:0]    id;
        input [ADDR_WIDTH-1:0] addr;
        input [LEN_W-1:0]   len;
        input [SIZE_W-1:0]  size;
        input [BURST_W-1:0] burst;
        ref [DATA_WIDTH-1:0] data_arr [];
        begin
            $display("[%0t] MASTER[%0d] READ START: addr=0x%08h len=%0d id=0x%0h",
                     $time, mst, addr, len, id);
            axi_ar_send(mst, id, addr, len, size, burst);
            axi_r_recv(mst, data_arr, len);
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
                // Use shift instead of variable part-select
                if ((addr >> dec_len) == (base >> dec_len)) begin
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
        integer s, beat;
        reg [ADDR_WIDTH-1:0] cur_addr;
        reg [DATA_WIDTH-1:0] expected;
        begin
            s = get_slave_idx(addr);
            cur_addr = addr;
            for (beat = 0; beat <= len; beat = beat + 1) begin
                expected = slv_mem[s][cur_addr[SLV_MEM_AW-1:2] & (SLV_MEM_DEPTH-1)];
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
        M_AXI_BREADY_i  = {MST_AMT{1'b1}};  // B listener maintains this
        M_AXI_ARID_i    = '0;
        M_AXI_ARADDR_i  = '0;
        M_AXI_ARLEN_i   = '0;
        M_AXI_ARSIZE_i  = '0;
        M_AXI_ARBURST_i = '0;
        M_AXI_ARVALID_i = '0;
        // BREADY driven by mst_b_listener; AWREADY/ARREADY driven by slave handler tasks
        M_AXI_RREADY_i  = {MST_AMT{1'b1}};  // R listener maintains this

        // Slave-side signals
        S_AXI_WREADY_i  = {SLV_AMT{1'b1}};
        S_AXI_BVALID_i  = '0;
        S_AXI_BID_i     = '0;
        S_AXI_BRESP_i   = '0;
        S_AXI_RVALID_i  = '0;
        S_AXI_RLAST_i   = '0;
        S_AXI_RID_i     = '0;
        S_AXI_RDATA_i   = '0;
        S_AXI_RRESP_i   = '0;
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
    // Deadlock Watchdog: finish if no AXI activity for 50000 cycles
    //=========================================================
    integer watchdog_cnt;
    reg     watchdog_active;
    initial begin
        watchdog_cnt    = 0;
        watchdog_active = 0;
        wait(AXI_RSTn);
        wait_cycles(20);
        watchdog_active = 1;
    end

    always @(posedge AXI_CLK) begin
        if (watchdog_active) begin
            // Any handshake on any channel resets the watchdog
            if (|(M_AXI_AWVALID_i & M_AXI_AWREADY_o) ||
                |(M_AXI_WVALID_i & M_AXI_WREADY_o) ||
                |(M_AXI_BVALID_o & M_AXI_BREADY_i) ||
                |(M_AXI_ARVALID_i & M_AXI_ARREADY_o) ||
                |(M_AXI_RVALID_o & M_AXI_RREADY_i)) begin
                watchdog_cnt = 0;
            end else begin
                watchdog_cnt = watchdog_cnt + 1;
                if (watchdog_cnt >= 50000) begin
                    $display("\n[%0t] DEADLOCK: no AXI activity for 50000 cycles", $time);
                    $display("  m_wr_q_cnt = {%0d,%0d,%0d,%0d}",
                             m_wr_q_cnt[0], m_wr_q_cnt[1], m_wr_q_cnt[2], m_wr_q_cnt[3]);
                    $display("  m_b_q_cnt  = {%0d,%0d,%0d,%0d}",
                             m_b_q_cnt[0], m_b_q_cnt[1], m_b_q_cnt[2], m_b_q_cnt[3]);
                    $display("  m_rd_q_cnt = {%0d,%0d,%0d,%0d}",
                             m_rd_q_cnt[0], m_rd_q_cnt[1], m_rd_q_cnt[2], m_rd_q_cnt[3]);
                    $finish;
                end
            end
        end
    end

    always @(posedge AXI_CLK) begin
        if (phase_cnt >= 7 && monitor_cnt < 30) begin
            $display("[%0t] MONITOR [phase=%0d][cnt=%0d]", $time, phase_cnt, monitor_cnt);
            monitor_cnt <= monitor_cnt + 1;
            // ---- Master Side (M[0..3]) ----
            for (int m = 0; m < MST_AMT; m = m + 1) begin
                $display("  M[%0d] AW: addr=0x%08h valid=%0d ready=%0d",
                         m, M_AXI_AWADDR_i[ADDR_WIDTH*(m+1)-1 -: ADDR_WIDTH],
                         M_AXI_AWVALID_i[m], M_AXI_AWREADY_o[m]);
                $display("  M[%0d] W : data=0x%08h last=%0d valid=%0d ready=%0d",
                         m, M_AXI_WDATA_i[DATA_WIDTH*(m+1)-1 -: DATA_WIDTH],
                         M_AXI_WLAST_i[m], M_AXI_WVALID_i[m], M_AXI_WREADY_o[m]);
                $display("  M[%0d] B : id=0x%0h resp=0x%0h valid=%0d ready=%0d",
                         m, M_AXI_BID_o[W_ID*(m+1)-1 -: W_ID],
                         M_AXI_BRESP_o[RESP_W*(m+1)-1 -: RESP_W],
                         M_AXI_BVALID_o[m], M_AXI_BREADY_i[m]);
                $display("  M[%0d] AR: addr=0x%08h valid=%0d ready=%0d",
                         m, M_AXI_ARADDR_i[ADDR_WIDTH*(m+1)-1 -: ADDR_WIDTH],
                         M_AXI_ARVALID_i[m], M_AXI_ARREADY_o[m]);
                $display("  M[%0d] R : data=0x%08h last=%0d valid=%0d ready=%0d",
                         m, M_AXI_RDATA_o[DATA_WIDTH*(m+1)-1 -: DATA_WIDTH],
                         M_AXI_RLAST_o[m], M_AXI_RVALID_o[m], M_AXI_RREADY_i[m]);
            end
            // ---- Slave Side (S[0..3]) ----
            for (int s = 0; s < SLV_AMT; s = s + 1) begin
                $display("  S[%0d] AW: addr=0x%08h valid=%0d ready=%0d",
                         s, S_AXI_AWADDR_o[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH],
                         S_AXI_AWVALID_o[s], S_AXI_AWREADY_i[s]);
                $display("  S[%0d] W : data=0x%08h last=%0d valid=%0d ready=%0d",
                         s, S_AXI_WDATA_o[DATA_WIDTH*(s+1)-1 -: DATA_WIDTH],
                         S_AXI_WLAST_o[s], S_AXI_WVALID_o[s], S_AXI_WREADY_i[s]);
                $display("  S[%0d] B : id=0x%0h resp=0x%0h valid=%0d ready=%0d",
                         s, S_AXI_BID_i[W_SID*(s+1)-1 -: W_SID],
                         S_AXI_BRESP_i[RESP_W*(s+1)-1 -: RESP_W],
                         S_AXI_BVALID_i[s], S_AXI_BREADY_o[s]);
                $display("  S[%0d] AR: addr=0x%08h valid=%0d ready=%0d",
                         s, S_AXI_ARADDR_o[ADDR_WIDTH*(s+1)-1 -: ADDR_WIDTH],
                         S_AXI_ARVALID_o[s], S_AXI_ARREADY_i[s]);
                $display("  S[%0d] R : data=0x%08h last=%0d valid=%0d ready=%0d",
                         s, S_AXI_RDATA_i[DATA_WIDTH*(s+1)-1 -: DATA_WIDTH],
                         S_AXI_RLAST_i[s], S_AXI_RVALID_i[s], S_AXI_RREADY_o[s]);
            end
        end
    end

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
        phase_cnt = phase_cnt + 1; $display("\n[PHASE %0d] ========== PHASE 1: Basic Single-Beat Sanity ==========", phase_cnt);
        begin
            reg [DATA_WIDTH-1:0] rdata_arr [];
            reg [DATA_WIDTH-1:0] wdata_arr [];
            reg [W_STRB-1:0]     strb_arr  [];
            integer pm, ps;
            rdata_arr = new[1];
            wdata_arr = new[1];
            strb_arr  = new[1];

            for (pm = 0; pm < MST_AMT; pm = pm + 1) begin
                for (ps = 0; ps < SLV_AMT; ps = ps + 1) begin
                    wdata_arr[0] = {16'd0, pm[3:0], ps[3:0], 8'hA5};
                    strb_arr[0]  = 4'hF;  // all bytes valid
                    axi_write_burst(pm, pm[3:0], ps * 32'h0000_2000 + pm * 32'h100,
                                    8'd0, 3'b010, 2'b01, wdata_arr, strb_arr);
                    wait_cycles(5);
                end
            end

            for (pm = 0; pm < MST_AMT; pm = pm + 1) begin
                for (ps = 0; ps < SLV_AMT; ps = ps + 1) begin
                    axi_read_burst(pm, pm[3:0], ps * 32'h0000_2000 + pm * 32'h100,
                                  8'd0, 3'b010, 2'b01, rdata_arr);
                    check_read_data(pm, ps * 32'h0000_2000 + pm * 32'h100,
                                   8'd0, 3'b010, rdata_arr);
                    wait_cycles(5);
                end
            end
        end

        //---------------------------------------------------------
        // PHASE 2: Multi-Beat Burst Write/Read (INCR)
        //---------------------------------------------------------
        phase_cnt = phase_cnt + 1; $display("\n[PHASE %0d] ========== PHASE 2: Multi-Beat INCR Burst ==========", phase_cnt);
        begin
            reg [DATA_WIDTH-1:0] wdata_arr [];
            reg [DATA_WIDTH-1:0] rdata_arr [];
            reg [W_STRB-1:0]     strb_arr  [];
            integer bi, b, blen;
            wdata_arr = new[16];
            rdata_arr = new[16];
            strb_arr  = new[16];

            for (bi = 0; bi < 5; bi = bi + 1) begin
                blen = (bi == 0) ? 0 : (bi == 1) ? 1 : (bi == 2) ? 3 : (bi == 3) ? 7 : 15;
                $display("--- Burst len=%0d (AWLEN=%0d, %0d beats) ---",
                         blen, blen, blen+1);

                // Master 0 writes to Slave 0
                for (b = 0; b <= blen; b = b + 1) begin
                    wdata_arr[b] = gen_test_data(0, b);
                    strb_arr[b]  = 4'hF;
                end
                axi_write_burst(0, 4'hA, 32'h0000_1000,
                                blen[LEN_W-1:0], 3'b010, 2'b01,
                                wdata_arr, strb_arr);
                wait_cycles(5);

                // Read back and check
                axi_read_burst(0, 4'hA, 32'h0000_1000,
                               blen[LEN_W-1:0], 3'b010, 2'b01,
                               rdata_arr);
                check_read_data(0, 32'h0000_1000,
                                blen[LEN_W-1:0], 3'b010, rdata_arr);
                wait_cycles(10);
            end
        end

        //---------------------------------------------------------
        // PHASE 3: Full-Word Writes (WSTRB = 4'hF, single-beat)
        //---------------------------------------------------------
        phase_cnt = phase_cnt + 1; $display("\n[PHASE %0d] ========== PHASE 3: Full-Word Single-Beat Writes ==========", phase_cnt);
        begin
            reg [DATA_WIDTH-1:0] wdata_arr [];
            reg [DATA_WIDTH-1:0] rdata_arr [];
            reg [W_STRB-1:0]     strb_arr  [];
            integer p;
            wdata_arr = new[1];
            rdata_arr = new[1];
            strb_arr  = new[1];
            strb_arr[0] = 4'hF;

            // Write known patterns to 4 consecutive words, read back and check
            wdata_arr[0] = 32'h1234_5678;
            axi_write_burst(0, 4'h3, 32'h0000_2000, 8'd0, 3'b010, 2'b01, wdata_arr, strb_arr);
            wait_cycles(2);
            axi_read_burst(0, 4'h3, 32'h0000_2000, 8'd0, 3'b010, 2'b01, rdata_arr);
            check_read_data(0, 32'h0000_2000, 8'd0, 3'b010, rdata_arr);

            wdata_arr[0] = 32'hAAAA_BBBB;
            axi_write_burst(1, 4'h3, 32'h0000_2004, 8'd0, 3'b010, 2'b01, wdata_arr, strb_arr);
            wait_cycles(2);
            axi_read_burst(1, 4'h3, 32'h0000_2004, 8'd0, 3'b010, 2'b01, rdata_arr);
            check_read_data(1, 32'h0000_2004, 8'd0, 3'b010, rdata_arr);

            wdata_arr[0] = 32'hFFFF_0000;
            axi_write_burst(2, 4'h3, 32'h0000_2008, 8'd0, 3'b010, 2'b01, wdata_arr, strb_arr);
            wait_cycles(2);
            axi_read_burst(2, 4'h3, 32'h0000_2008, 8'd0, 3'b010, 2'b01, rdata_arr);
            check_read_data(2, 32'h0000_2008, 8'd0, 3'b010, rdata_arr);

            wdata_arr[0] = 32'h0000_FFFF;
            axi_write_burst(3, 4'h3, 32'h0000_200C, 8'd0, 3'b010, 2'b01, wdata_arr, strb_arr);
            wait_cycles(2);
            axi_read_burst(3, 4'h3, 32'h0000_200C, 8'd0, 3'b010, 2'b01, rdata_arr);
            check_read_data(3, 32'h0000_200C, 8'd0, 3'b010, rdata_arr);
        end

        //---------------------------------------------------------
        // PHASE 4: Concurrent Multi-Master Arbitration
        //---------------------------------------------------------
        phase_cnt = phase_cnt + 1; $display("\n[PHASE %0d] ========== PHASE 4: Concurrent Multi-Master ==========", phase_cnt);
        begin
            // All 4 masters write to slave 0 concurrently (tests arbitration)
            $display("--- 4 Masters → Slave 0 (write, concurrent) ---");
            fork
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    reg [W_STRB-1:0]     _s [];
                    _d = new[1]; _s = new[1];
                    _d[0] = 32'hAAAA_0001; _s[0] = 4'hF;
                    axi_write_burst(0, 4'h0, 32'h0000_0100, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    reg [W_STRB-1:0]     _s [];
                    _d = new[1]; _s = new[1];
                    _d[0] = 32'hBBBB_0002; _s[0] = 4'hF;
                    axi_write_burst(1, 4'h0, 32'h0000_0104, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    reg [W_STRB-1:0]     _s [];
                    _d = new[1]; _s = new[1];
                    _d[0] = 32'hCCCC_0003; _s[0] = 4'hF;
                    axi_write_burst(2, 4'h0, 32'h0000_0108, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    reg [W_STRB-1:0]     _s [];
                    _d = new[1]; _s = new[1];
                    _d[0] = 32'hDDDD_0004; _s[0] = 4'hF;
                    axi_write_burst(3, 4'h0, 32'h0000_010C, 8'd0, 3'b010, 2'b01, _d, _s);
                end
            join

            // Read back all
            $display("--- 4 Masters ← Slave 0 (read, concurrent) ---");
            fork
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    _d = new[1];
                    axi_read_burst(0, 4'h0, 32'h0000_0100, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(0, 32'h0000_0100, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    _d = new[1];
                    axi_read_burst(1, 4'h0, 32'h0000_0104, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(1, 32'h0000_0104, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    _d = new[1];
                    axi_read_burst(2, 4'h0, 32'h0000_0108, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(2, 32'h0000_0108, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    _d = new[1];
                    axi_read_burst(3, 4'h0, 32'h0000_010C, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(3, 32'h0000_010C, 8'd0, 3'b010, _d);
                end
            join
        end

        //---------------------------------------------------------
        // PHASE 5: Multi-Slave Concurrent Access
        //---------------------------------------------------------
        phase_cnt = phase_cnt + 1; $display("\n[PHASE %0d] ========== PHASE 5: Multi-Slave Concurrent Access ==========", phase_cnt);
        begin
            // 4 masters → 4 different slaves simultaneously
            $display("--- 4 Masters → 4 Slaves (write, concurrent) ---");
            fork
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    reg [W_STRB-1:0]     _s [];
                    _d = new[1]; _s = new[1];
                    _d[0] = 32'h1111_1111; _s[0] = 4'hF;
                    axi_write_burst(0, 4'h1, 32'h0000_0300, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    reg [W_STRB-1:0]     _s [];
                    _d = new[1]; _s = new[1];
                    _d[0] = 32'h2222_2222; _s[0] = 4'hF;
                    axi_write_burst(1, 4'h1, 32'h0000_2300, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    reg [W_STRB-1:0]     _s [];
                    _d = new[1]; _s = new[1];
                    _d[0] = 32'h3333_3333; _s[0] = 4'hF;
                    axi_write_burst(2, 4'h1, 32'h0000_4300, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    reg [W_STRB-1:0]     _s [];
                    _d = new[1]; _s = new[1];
                    _d[0] = 32'h4444_4444; _s[0] = 4'hF;
                    axi_write_burst(3, 4'h1, 32'h0000_6300, 8'd0, 3'b010, 2'b01, _d, _s);
                end
            join

            $display("--- 4 Masters ← 4 Slaves (read, concurrent) ---");
            fork
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    _d = new[1];
                    axi_read_burst(0, 4'h1, 32'h0000_0300, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(0, 32'h0000_0300, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    _d = new[1];
                    axi_read_burst(1, 4'h1, 32'h0000_2300, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(1, 32'h0000_2300, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    _d = new[1];
                    axi_read_burst(2, 4'h1, 32'h0000_4300, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(2, 32'h0000_4300, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    _d = new[1];
                    axi_read_burst(3, 4'h1, 32'h0000_6300, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(3, 32'h0000_6300, 8'd0, 3'b010, _d);
                end
            join
        end

        //---------------------------------------------------------
        //---------------------------------------------------------
        // PHASE 6: Mixed Concurrent Read/Write
        //---------------------------------------------------------
        phase_cnt = phase_cnt + 1; $display("\n[PHASE %0d] ========== PHASE 6: Mixed Concurrent Read/Write ==========", phase_cnt);
        begin
            // Pre-write data for reads
            begin
                reg [DATA_WIDTH-1:0] _d [];
                reg [W_STRB-1:0]     _s [];
                _d = new[1]; _s = new[1];
                _d[0] = 32'hCAFE_C0DE; _s[0] = 4'hF;
                axi_write_burst(0, 4'hF, 32'h0000_0500, 8'd0, 3'b010, 2'b01, _d, _s);
            end
            begin
                reg [DATA_WIDTH-1:0] _d [];
                reg [W_STRB-1:0]     _s [];
                _d = new[1]; _s = new[1];
                _d[0] = 32'hFEED_FACE; _s[0] = 4'hF;
                axi_write_burst(1, 4'hF, 32'h0000_2500, 8'd0, 3'b010, 2'b01, _d, _s);
            end

            wait_cycles(10);

            // Simultaneous read + write on different masters
            $display("--- M0 read Slave0, M1 write Slave1, M2 write Slave2, M3 read Slave3 ---");
            fork
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    _d = new[1];
                    axi_read_burst(0, 4'hF, 32'h0000_0500, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(0, 32'h0000_0500, 8'd0, 3'b010, _d);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    reg [W_STRB-1:0]     _s [];
                    _d = new[1]; _s = new[1];
                    _d[0] = 32'hBEEF_0001; _s[0] = 4'hF;
                    axi_write_burst(1, 4'hE, 32'h0000_2600, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    reg [W_STRB-1:0]     _s [];
                    _d = new[1]; _s = new[1];
                    _d[0] = 32'hBEEF_0002; _s[0] = 4'hF;
                    axi_write_burst(2, 4'hE, 32'h0000_4600, 8'd0, 3'b010, 2'b01, _d, _s);
                end
                begin
                    reg [DATA_WIDTH-1:0] _d [];
                    reg [W_STRB-1:0]     _s [];
                    _d = new[1]; _s = new[1];
                    // Pre-write then read on M3
                    _d[0] = 32'hDEAD_BEEF; _s[0] = 4'hF;
                    axi_write_burst(3, 4'hE, 32'h0000_6500, 8'd0, 3'b010, 2'b01, _d, _s);
                    wait_cycles(3);
                    axi_read_burst(3, 4'hE, 32'h0000_6500, 8'd0, 3'b010, 2'b01, _d);
                    check_read_data(3, 32'h0000_6500, 8'd0, 3'b010, _d);
                end
            join
        end

        //---------------------------------------------------------
        // PHASE 7: Multi-ID Outstanding (same master, different IDs)
        //---------------------------------------------------------
        phase_cnt = phase_cnt + 1; $display("\n[PHASE %0d] ========== PHASE 7: Multi-ID Outstanding Transactions ==========", phase_cnt);
        begin
            // Master 0 issues 4 writes with different IDs to the same slave
            // AW sent sequentially (single master driver), W/B can be pipelined
            $display("--- M0: 4 writes with different IDs → Slave 0 ---");
            begin
                reg [DATA_WIDTH-1:0] _d [];
                reg [W_STRB-1:0]     _s [];
                _d = new[1]; _s = new[1];
                _d[0] = 32'hA001_A001; _s[0] = 4'hF;
                axi_write_burst(0, 4'h0, 32'h0000_0700, 8'd0, 3'b010, 2'b01, _d, _s);
            end
            begin
                reg [DATA_WIDTH-1:0] _d [];
                reg [W_STRB-1:0]     _s [];
                _d = new[1]; _s = new[1];
                _d[0] = 32'hA002_A002; _s[0] = 4'hF;
                axi_write_burst(0, 4'h2, 32'h0000_0704, 8'd0, 3'b010, 2'b01, _d, _s);
            end
            begin
                reg [DATA_WIDTH-1:0] _d [];
                reg [W_STRB-1:0]     _s [];
                _d = new[1]; _s = new[1];
                _d[0] = 32'hA003_A003; _s[0] = 4'hF;
                axi_write_burst(0, 4'h4, 32'h0000_0708, 8'd0, 3'b010, 2'b01, _d, _s);
            end
            begin
                reg [DATA_WIDTH-1:0] _d [];
                reg [W_STRB-1:0]     _s [];
                _d = new[1]; _s = new[1];
                _d[0] = 32'hA004_A004; _s[0] = 4'hF;
                axi_write_burst(0, 4'h6, 32'h0000_070C, 8'd0, 3'b010, 2'b01, _d, _s);
            end

            wait_cycles(10);

            $display("--- M0: 4 reads with different IDs ← Slave 0 ---");
            begin
                reg [DATA_WIDTH-1:0] _d [];
                _d = new[1];
                axi_read_burst(0, 4'h0, 32'h0000_0700, 8'd0, 3'b010, 2'b01, _d);
                check_read_data(0, 32'h0000_0700, 8'd0, 3'b010, _d);
            end
            begin
                reg [DATA_WIDTH-1:0] _d [];
                _d = new[1];
                axi_read_burst(0, 4'h2, 32'h0000_0704, 8'd0, 3'b010, 2'b01, _d);
                check_read_data(0, 32'h0000_0704, 8'd0, 3'b010, _d);
            end
            begin
                reg [DATA_WIDTH-1:0] _d [];
                _d = new[1];
                axi_read_burst(0, 4'h4, 32'h0000_0708, 8'd0, 3'b010, 2'b01, _d);
                check_read_data(0, 32'h0000_0708, 8'd0, 3'b010, _d);
            end
            begin
                reg [DATA_WIDTH-1:0] _d [];
                _d = new[1];
                axi_read_burst(0, 4'h6, 32'h0000_070C, 8'd0, 3'b010, 2'b01, _d);
                check_read_data(0, 32'h0000_070C, 8'd0, 3'b010, _d);
            end
        end

        //---------------------------------------------------------
        // PHASE 8: Long Burst Stress Test
        //---------------------------------------------------------
        phase_cnt = phase_cnt + 1; $display("\n[PHASE %0d] ========== PHASE 8: Long Burst Stress ==========", phase_cnt);
        begin
            reg [DATA_WIDTH-1:0] wdata_arr [];
            reg [DATA_WIDTH-1:0] rdata_arr [];
            reg [W_STRB-1:0]     strb_arr  [];
            integer b;
            wdata_arr = new[256];
            rdata_arr = new[256];
            strb_arr  = new[256];

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
        // PHASE 9: Random Stress Test
        //---------------------------------------------------------
        phase_cnt = phase_cnt + 1; $display("\n[PHASE %0d] ========== PHASE 9: Random Stress Test ==========", phase_cnt);
        begin
            reg [DATA_WIDTH-1:0] wdata_arr [];
            reg [DATA_WIDTH-1:0] rdata_arr [];
            reg [W_STRB-1:0]     strb_arr  [];
            integer t, rm, rs, rlen, b;
            reg [ADDR_WIDTH-1:0] raddr;
            wdata_arr = new[8];
            rdata_arr = new[8];
            strb_arr  = new[8];

            for (t = 0; t < 50; t = t + 1) begin
                rm   = $urandom % MST_AMT;
                rs   = $urandom % SLV_AMT;
                rlen = $urandom % 4;  // 0..3 (1..4 beats)
                raddr = rs * 32'h0000_2000 + ($urandom % 4096);

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
        // PHASE 10: 4KB Boundary Crossing (moved to last)
        //---------------------------------------------------------
        phase_cnt = phase_cnt + 1; $display("\n[PHASE %0d] ========== PHASE 10: 4KB Boundary Crossing ==========", phase_cnt);
        begin
            reg [DATA_WIDTH-1:0] wdata_arr [];
            reg [DATA_WIDTH-1:0] rdata_arr [];
            reg [W_STRB-1:0]     strb_arr  [];
            integer b, bi, blen;
            wdata_arr = new[16];
            rdata_arr = new[16];
            strb_arr  = new[16];
            for (bi = 0; bi < 3; bi = bi + 1) begin
                blen = (bi == 0) ? 1 : (bi == 1) ? 3 : 7;
                $display("--- Cross-4K: len=%0d, addr=0x0FF0 ---", blen);
                // Start near 4KB boundary, SIZE=2 (4 bytes/beat)
                // 0x0FF0 + (len+1)*4 → crosses 0x1000 if len >= 3
                for (b = 0; b <= blen; b = b + 1) begin
                    wdata_arr[b] = gen_test_data(0, b) ^ 32'hC000_0000;
                    strb_arr[b]  = 4'hF;
                end
                axi_write_burst(0, 4'h7, 32'h0000_0FF0,
                                blen[LEN_W-1:0], 3'b010, 2'b01,
                                wdata_arr, strb_arr);
                wait_cycles(10);

                axi_read_burst(0, 4'h7, 32'h0000_0FF0,
                               blen[LEN_W-1:0], 3'b010, 2'b01,
                               rdata_arr);
                check_read_data(0, 32'h0000_0FF0,
                                blen[LEN_W-1:0], 3'b010, rdata_arr);
                wait_cycles(10);
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
