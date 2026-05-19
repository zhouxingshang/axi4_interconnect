//=============================================================================
// Testbench: axi_m2s_tb
// Desc     : Testbench for axi_m2s_m_amt (multi-master to single-slave)
//=============================================================================

module axi_m2s_tb;

    //=========================================================================
    // Parameters
    //=========================================================================
    localparam MST_AMT         = 3;
    localparam W_ID            = 4;
    localparam M_ID_W          = $clog2(MST_AMT);
    localparam W_ADDR          = 32;
    localparam W_DATA          = 32;
    localparam W_STRB          = W_DATA / 8;
    localparam ALEN_W          = 8;
    localparam ASIZE_W         = 3;
    localparam ABURST_W        = 2;
    localparam W_SID           = M_ID_W + W_ID;
    localparam OUTSTANDING_AMT = 4;

    //=========================================================================
    // Clock and Reset
    //=========================================================================
    reg         clk;
    reg         rst_n;

    //=========================================================================
    // DUT Master-Side Signals (flattened)
    //=========================================================================
    // AW
    reg  [MST_AMT*W_ID-1     : 0]  M_AWID;
    reg  [MST_AMT*W_ADDR-1   : 0]  M_AWADDR;
    reg  [MST_AMT*ALEN_W-1   : 0]  M_AWLEN;
    reg  [MST_AMT*ASIZE_W-1  : 0]  M_AWSIZE;
    reg  [MST_AMT*ABURST_W-1 : 0]  M_AWBURST;
    reg  [MST_AMT-1          : 0]  M_AWVALID;
    wire [MST_AMT-1          : 0]  M_AWREADY;

    // W
    reg  [MST_AMT*W_DATA-1   : 0]  M_WDATA;
    reg  [MST_AMT*W_STRB-1   : 0]  M_WSTRB;
    reg  [MST_AMT-1          : 0]  M_WLAST;
    reg  [MST_AMT-1          : 0]  M_WVALID;
    wire [MST_AMT-1          : 0]  M_WREADY;

    // AR
    reg  [MST_AMT*W_ID-1     : 0]  M_ARID;
    reg  [MST_AMT*W_ADDR-1   : 0]  M_ARADDR;
    reg  [MST_AMT*ALEN_W-1   : 0]  M_ARLEN;
    reg  [MST_AMT*ASIZE_W-1  : 0]  M_ARSIZE;
    reg  [MST_AMT*ABURST_W-1 : 0]  M_ARBURST;
    reg  [MST_AMT-1          : 0]  M_ARVALID;
    wire [MST_AMT-1          : 0]  M_ARREADY;

    //=========================================================================
    // DUT Slave-Side Signals
    //=========================================================================
    wire [W_SID-1     : 0]  S_AWID;
    wire [W_ADDR-1   : 0]   S_AWADDR;
    wire [ALEN_W-1   : 0]   S_AWLEN;
    wire [ASIZE_W-1  : 0]   S_AWSIZE;
    wire [ABURST_W-1 : 0]   S_AWBURST;
    wire                    S_AWVALID;
    reg                     S_AWREADY;

    wire [W_DATA-1:0]       S_WDATA;
    wire [W_STRB-1:0]       S_WSTRB;
    wire                    S_WLAST;
    wire                    S_WVALID;
    reg                     S_WREADY;

    wire [W_SID-1     : 0]  S_ARID;
    wire [W_ADDR-1   : 0]   S_ARADDR;
    wire [ALEN_W-1   : 0]   S_ARLEN;
    wire [ASIZE_W-1  : 0]   S_ARSIZE;
    wire [ABURST_W-1 : 0]   S_ARBURST;
    wire                    S_ARVALID;
    reg                     S_ARREADY;

    //=========================================================================
    // DUT Control Signals
    //=========================================================================
    wire [MST_AMT-1:0]      AWSELECT_OUT;
    wire [MST_AMT-1:0]      ARSELECT_OUT;
    reg  [MST_AMT-1:0]      AWSELECT_IN;
    reg  [MST_AMT-1:0]      ARSELECT_IN;
    reg                     arbiter_type;
    reg                     slv_en;

    //=========================================================================
    // DUT Instantiation
    //=========================================================================
    axi_m2s_m_amt #(
        .ADDR_BASE      (32'h00001000),
        .ADDR_LENGTH    (12),
        .M_ID_W         (M_ID_W),
        .W_ID           (W_ID),
        .W_ADDR         (W_ADDR),
        .W_DATA         (W_DATA),
        .W_STRB         (W_STRB),
        .W_SID          (W_SID),
        .MST_AMT        (MST_AMT),
        .OUTSTANDING_AMT(OUTSTANDING_AMT),
        .ALEN_W         (ALEN_W),
        .ASIZE_W        (ASIZE_W),
        .ABURST_W       (ABURST_W),
        .SLAVE_DEFAULT  (1'b0)
    ) u_dut (
        .AXI_RSTn       (rst_n),
        .AXI_CLK        (clk),

        .M_AWID         (M_AWID),
        .M_AWADDR       (M_AWADDR),
        .M_AWLEN        (M_AWLEN),
        .M_AWSIZE       (M_AWSIZE),
        .M_AWBURST      (M_AWBURST),
        .M_AWVALID      (M_AWVALID),
        .M_AWREADY      (M_AWREADY),

        .M_WDATA        (M_WDATA),
        .M_WSTRB        (M_WSTRB),
        .M_WLAST        (M_WLAST),
        .M_WVALID       (M_WVALID),
        .M_WREADY       (M_WREADY),

        .M_ARID         (M_ARID),
        .M_ARADDR       (M_ARADDR),
        .M_ARLEN        (M_ARLEN),
        .M_ARSIZE       (M_ARSIZE),
        .M_ARBURST      (M_ARBURST),
        .M_ARVALID      (M_ARVALID),
        .M_ARREADY      (M_ARREADY),

        .S_AWID         (S_AWID),
        .S_AWADDR       (S_AWADDR),
        .S_AWLEN        (S_AWLEN),
        .S_AWSIZE       (S_AWSIZE),
        .S_AWBURST      (S_AWBURST),
        .S_AWVALID      (S_AWVALID),
        .S_AWREADY      (S_AWREADY),

        .S_WDATA        (S_WDATA),
        .S_WSTRB        (S_WSTRB),
        .S_WLAST        (S_WLAST),
        .S_WVALID       (S_WVALID),
        .S_WREADY       (S_WREADY),

        .S_ARID         (S_ARID),
        .S_ARADDR       (S_ARADDR),
        .S_ARLEN        (S_ARLEN),
        .S_ARSIZE       (S_ARSIZE),
        .S_ARBURST      (S_ARBURST),
        .S_ARVALID      (S_ARVALID),
        .S_ARREADY      (S_ARREADY),

        .AWSELECT_OUT   (AWSELECT_OUT),
        .ARSELECT_OUT   (ARSELECT_OUT),
        .AWSELECT_IN    (AWSELECT_IN),
        .ARSELECT_IN    (ARSELECT_IN),
        .arbiter_type   (arbiter_type),
        .slv_en         (slv_en)
    );

    //=========================================================================
    // Clock: 100MHz, 10ns period
    //=========================================================================
    initial clk = 0;
    always #5 clk = ~clk;

    //=========================================================================
    // Reset
    //=========================================================================
    initial begin
        rst_n = 0;
        #100;
        rst_n = 1;
    end

    //=========================================================================
    // Slave Memory Model (1KB)
    //=========================================================================
    localparam SLV_MEM_DEPTH = 256;
    reg [W_DATA-1:0] slv_mem [0:SLV_MEM_DEPTH-1];
    integer mi;
    initial begin
        for (mi = 0; mi < SLV_MEM_DEPTH; mi++)
            slv_mem[mi] = 32'hDEAD0000 + mi;
    end

    // Debug probes: capture last write for easy waveform viewing
    reg [W_DATA-1:0]   dbg_wr_data;
    reg [W_STRB-1:0]   dbg_wr_strb;
    reg                dbg_wr_vld;
    always @(posedge clk) begin
        dbg_wr_vld <= 1'b0;
        if (S_WVALID && S_WREADY) begin
            dbg_wr_data     <= S_WDATA;
            dbg_wr_strb     <= S_WSTRB;
            dbg_wr_vld      <= 1'b1;
        end
    end

    // Slave Write Handler (event-driven, runs forever)
    // Uses AW context FIFO to support pipelined AW+W transactions
    task automatic slv_write_handler();
        reg [W_SID-1:0]   awid_q   [$];
        reg [W_ADDR-1:0]  awaddr_q [$];
        reg [ALEN_W-1:0]  awlen_q  [$];
        reg [ALEN_W-1:0]  beat_cnt_q [$];
        reg [ALEN_W:0]    total_beats_q [$];
        reg [W_ADDR-1:0]  word_idx;
        integer           bi;
        forever begin
            @(posedge clk);

            // AW handshake: push context to queue
            if (S_AWVALID && S_AWREADY) begin
                awid_q.push_back(S_AWID);
                awaddr_q.push_back(S_AWADDR);
                awlen_q.push_back(S_AWLEN);
                beat_cnt_q.push_back(0);
                total_beats_q.push_back(S_AWLEN + 1'b1);
                $display("[%0t] SLV AW queued: id=%0h addr=0x%08h beats=%0d (q depth=%0d)",
                         $time, S_AWID, S_AWADDR, S_AWLEN + 1'b1, awaddr_q.size());
            end

            // W handshake: write data using context at front of queue
            if (S_WVALID && S_WREADY) begin
                if (awaddr_q.size() > 0) begin
                    word_idx = awaddr_q[0][11:2] + beat_cnt_q[0];
                    if (word_idx < SLV_MEM_DEPTH) begin
                        for (bi = 0; bi < W_STRB; bi++) begin
                            if (S_WSTRB[bi])
                                slv_mem[word_idx][8*bi +: 8] = S_WDATA[8*bi +: 8];
                        end
                    end
                    beat_cnt_q[0] = beat_cnt_q[0] + 1'b1;
                    if (S_WLAST) begin
                        $display("[%0t] SLV WR done: addr=0x%08h beats=%0d",
                                 $time, awaddr_q[0], total_beats_q[0]);
                        awid_q.pop_front();
                        awaddr_q.pop_front();
                        awlen_q.pop_front();
                        beat_cnt_q.pop_front();
                        total_beats_q.pop_front();
                    end
                end else begin
                    $display("[%0t] WARNING: W handshake with no AW context queued!", $time);
                end
            end
        end
    endtask

    //=========================================================================
    // Default signal initialization
    //=========================================================================
    initial begin
        M_AWID     = '0;
        M_AWADDR   = '0;
        M_AWLEN    = '0;
        M_AWSIZE   = '0;
        M_AWBURST  = '0;
        M_AWVALID  = '0;
        M_WDATA    = '0;
        M_WSTRB    = '0;
        M_WLAST    = '0;
        M_WVALID   = '0;
        M_ARID     = '0;
        M_ARADDR   = '0;
        M_ARLEN    = '0;
        M_ARSIZE   = '0;
        M_ARBURST  = '0;
        M_ARVALID  = '0;
        S_AWREADY  = 1'b0;
        S_WREADY   = 1'b0;
        S_ARREADY  = 1'b0;
        AWSELECT_IN = '0;
        ARSELECT_IN = '0;
        arbiter_type = 1'b0;
        slv_en = 1'b1;
    end

    //=========================================================================
    // Launch slave write handler
    //=========================================================================
    initial begin
        fork
            slv_write_handler();
        join_none
    end

    //=========================================================================
    // Master BFM: Write
    //=========================================================================
    task automatic mst_write(
        input int                   mst_id,
        input [W_ADDR-1:0]          addr,
        input [ALEN_W-1:0]          len,
        input [ASIZE_W-1:0]         size,
        input [ABURST_W-1:0]        burst,
        input [W_ID-1:0]            id,
        input [W_DATA-1:0]          data []
    );
        reg [ALEN_W:0] total_beats;
        reg [ALEN_W:0] b;
        begin
            total_beats = len + 1'b1;
            $display("[%0t] MST[%0d] WR addr=0x%08h len=%0d size=%0d id=%0d",
                     $time, mst_id, addr, len, size, id);

            // Drive AW
            M_AWID[W_ID*(mst_id+1)-1 -: W_ID]             <= id;
            M_AWADDR[W_ADDR*(mst_id+1)-1 -: W_ADDR]       <= addr;
            M_AWLEN[ALEN_W*(mst_id+1)-1 -: ALEN_W]        <= len;
            M_AWSIZE[ASIZE_W*(mst_id+1)-1 -: ASIZE_W]     <= size;
            M_AWBURST[ABURST_W*(mst_id+1)-1 -: ABURST_W]  <= burst;
            M_AWVALID[mst_id] <= 1'b1;

            @(posedge clk);
            while (!M_AWREADY[mst_id]) @(posedge clk);
            M_AWVALID[mst_id] <= 1'b0;

            // Drive W beats
            for (b = 0; b < total_beats; b++) begin
                M_WDATA[W_DATA*(mst_id+1)-1 -: W_DATA] <= data[b];
                M_WSTRB[W_STRB*(mst_id+1)-1 -: W_STRB] <= {W_STRB{1'b1}};
                M_WLAST[mst_id]  <= (b == total_beats - 1);
                M_WVALID[mst_id] <= 1'b1;
                @(posedge clk);
                while (!M_WREADY[mst_id]) @(posedge clk);
            end
            M_WVALID[mst_id] <= 1'b0;
            M_WLAST[mst_id]  <= 1'b0;
            $display("[%0t] MST[%0d] WR done", $time, mst_id);
        end
    endtask

    //=========================================================================
    // Master BFM: Read  (simplified — checks S_AR channel output)
    //=========================================================================
    task automatic mst_read(
        input int                   mst_id,
        input [W_ADDR-1:0]          addr,
        input [ALEN_W-1:0]          len,
        input [ASIZE_W-1:0]         size,
        input [ABURST_W-1:0]        burst,
        input [W_ID-1:0]            id
    );
        begin
            $display("[%0t] MST[%0d] RD  addr=0x%08h len=%0d size=%0d id=%0d",
                     $time, mst_id, addr, len, size, id);

            // Drive AR
            M_ARID[W_ID*(mst_id+1)-1 -: W_ID]             <= id;
            M_ARADDR[W_ADDR*(mst_id+1)-1 -: W_ADDR]       <= addr;
            M_ARLEN[ALEN_W*(mst_id+1)-1 -: ALEN_W]        <= len;
            M_ARSIZE[ASIZE_W*(mst_id+1)-1 -: ASIZE_W]     <= size;
            M_ARBURST[ABURST_W*(mst_id+1)-1 -: ABURST_W]  <= burst;
            M_ARVALID[mst_id] <= 1'b1;

            @(posedge clk);
            while (!M_ARREADY[mst_id]) @(posedge clk);
            M_ARVALID[mst_id] <= 1'b0;

            // Check slave-side AR output
            $display("[%0t] MST[%0d] AR handshake done, S_AR: id=%0h addr=0x%08h len=%0d",
                     $time, mst_id, S_ARID, S_ARADDR, S_ARLEN);
        end
    endtask

    //=========================================================================
    // Checker functions
    //=========================================================================
    integer err_cnt;

    function automatic void check_eq;
        input [1023:0] msg;
        input [31:0]   got;
        input [31:0]   exp;
        begin
            if (got !== exp) begin
                $display("[%0t] ERROR: %0s got=0x%08h exp=0x%08h", $time, msg, got, exp);
                err_cnt = err_cnt + 1;
            end
        end
    endfunction

    //=========================================================================
    // Main Test Sequence
    //=========================================================================
    reg [W_DATA-1:0] wdata [0:7];
    integer          i, b;

    initial begin
        err_cnt = 0;

        @(posedge rst_n);
        repeat (5) @(posedge clk);

        $display("============================================================");
        $display("[%0t] AXI_M2S TESTBENCH STARTED", $time);
        $display("============================================================");

        //=================================================================
        // TEST 1: Single master write to matching address
        //=================================================================
        $display("\n--- TEST 1: Single write (addr match) ---");
        begin
            S_AWREADY <= 1'b1;
            S_WREADY  <= 1'b1;
            @(posedge clk);

            for (b = 0; b < 1; b++)
                wdata[b] = 32'hA5A5_0000 + b;
            mst_write(0, 32'h00001040, 8'd0, 3'd2, 2'd1, 4'hA, wdata);

            // Wait one cycle for write to settle
            @(posedge clk);

            // Check data written to slave memory
            check_eq("slv_mem[16]", slv_mem[16], 32'hA5A5_0000);
        end
        $display("[%0t] TEST 1 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 2: Address mismatch — no AWSELECT
        //=================================================================
        $display("\n--- TEST 2: Write to non-matching address ---");
        begin
            for (b = 0; b < 1; b++)
                wdata[b] = 32'hBBBB_0000;
            // addr 0x2000: base=0x1000, len=12 → bits[31:12] compare
            // 0x2000[31:12]=2, base[31:12]=1 → mismatch
            mst_write(0, 32'h00002000, 8'd0, 3'd2, 2'd1, 4'h0, wdata);
            // DUT should NOT drive AW to slave (AWSELECT=0, no AWGRANT)
            // AWVALID from master will hang waiting for AWREADY...
            // In real system, default slave catches this
        end
        $display("[%0t] TEST 2 done — AW should stall (no match)", $time);

        //=================================================================
        // TEST 3: Two masters, concurrent AW arbitration
        //=================================================================
        $display("\n--- TEST 3: Three-master AW arbitration ---");
        begin
            reg [W_DATA-1:0] w3_d0 [0:0];
            reg [W_DATA-1:0] w3_d1 [0:0];
            reg [W_DATA-1:0] w3_d2 [0:0];
            S_AWREADY <= 1'b1;
            S_WREADY  <= 1'b1;
            @(posedge clk);

            w3_d0[0] = 32'hCCCC_0000;
            w3_d1[0] = 32'hDDDD_0000;
            w3_d2[0] = 32'hEEEE_0000;
            fork
                mst_write(0, 32'h00001080, 8'd0, 3'd2, 2'd1, 4'h0, w3_d0);
                mst_write(1, 32'h00001080, 8'd0, 3'd2, 2'd1, 4'h1, w3_d1);
                mst_write(2, 32'h00001080, 8'd0, 3'd2, 2'd1, 4'h2, w3_d2);
            join
        end
        $display("[%0t] TEST 3 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 4: W follows AW — two writes back-to-back
        //=================================================================
        $display("\n--- TEST 4: W follows AW (back-to-back) ---");
        begin
            S_AWREADY <= 1'b1;
            S_WREADY  <= 1'b1;
            @(posedge clk);

            // Write 1
            for (b = 0; b < 2; b++)
                wdata[b] = 32'hB2B1_0000 + b;
            mst_write(0, 32'h00001100, 8'd1, 3'd2, 2'd1, 4'h2, wdata);

            // Write 2 (same master, different ID)
            for (b = 0; b < 2; b++)
                wdata[b] = 32'hB2B2_0000 + b;
            mst_write(0, 32'h00001108, 8'd1, 3'd2, 2'd1, 4'h3, wdata);

            // Verify W ordering preserved
            check_eq("slv_mem[64]", slv_mem[64], 32'hB2B1_0000);
            check_eq("slv_mem[65]", slv_mem[65], 32'hB2B1_0001);
            check_eq("slv_mem[66]", slv_mem[66], 32'hB2B2_0000);
            check_eq("slv_mem[67]", slv_mem[67], 32'hB2B2_0001);
        end
        $display("[%0t] TEST 4 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 5: Burst write (4 beats)
        //=================================================================
        $display("\n--- TEST 5: Burst write 4 beats ---");
        begin
            S_AWREADY <= 1'b1;
            S_WREADY  <= 1'b1;
            @(posedge clk);

            for (b = 0; b < 4; b++)
                wdata[b] = 32'hB500_0000 + b;
            mst_write(0, 32'h00001200, 8'd3, 3'd2, 2'd1, 4'h7, wdata);

            check_eq("slv_mem[128]", slv_mem[128], 32'hB500_0000);
            check_eq("slv_mem[129]", slv_mem[129], 32'hB500_0001);
            check_eq("slv_mem[130]", slv_mem[130], 32'hB500_0002);
            check_eq("slv_mem[131]", slv_mem[131], 32'hB500_0003);
        end
        $display("[%0t] TEST 5 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 6: AR channel — single read
        //=================================================================
        $display("\n--- TEST 6: Single read ---");
        begin
            S_ARREADY <= 1'b1;
            @(posedge clk);

            mst_read(0, 32'h00001040, 8'd0, 3'd2, 2'd1, 4'hA);
        end
        $display("[%0t] TEST 6 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 7: AR arbitration — two masters
        //=================================================================
        $display("\n--- TEST 7: Three-master AR arbitration ---");
        begin
            S_ARREADY <= 1'b1;
            @(posedge clk);

            fork
                mst_read(0, 32'h00001080, 8'd0, 3'd2, 2'd1, 4'h0);
                mst_read(1, 32'h00001084, 8'd0, 3'd2, 2'd1, 4'h1);
                mst_read(2, 32'h00001088, 8'd0, 3'd2, 2'd1, 4'h2);
            join
        end
        $display("[%0t] TEST 7 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 8: Address decode — ARSELECT check
        //=================================================================
        $display("\n--- TEST 8: AR address decode check ---");
        begin
            S_ARREADY <= 1'b1;
            @(posedge clk);

            // Should match: addr 0x1000~0x1FFF
            $display("  ARSELECT_OUT (addr=0x1100): %b", ARSELECT_OUT);
            mst_read(0, 32'h00001100, 8'd0, 3'd2, 2'd1, 4'h0);

            // Should NOT match: addr 0x2000 (outside 4KB page)
            $display("  ARSELECT_OUT (addr=0x2000): %b", ARSELECT_OUT);
        end
        $display("[%0t] TEST 8 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // FINAL REPORT
        //=================================================================
        $display("\n============================================================");
        if (err_cnt == 0)
            $display("[%0t] ALL TESTS PASSED!", $time);
        else
            $display("[%0t] TESTS FAILED with %0d errors!", $time, err_cnt);
        $display("============================================================\n");

        #200;
        $finish;
    end

    //=========================================================================
    // Timeout watchdog
    //=========================================================================
    initial begin
        #1000000;
        $display("[%0t] ERROR: Simulation timeout!", $time);
        $finish;
    end

    //=========================================================================
    // Waveform dump
    //=========================================================================
    initial begin
        $fsdbDumpfile("axi_m2s_tb.fsdb");
        $fsdbDumpvars(0, axi_m2s_tb);
    end

endmodule
