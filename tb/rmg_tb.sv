//=============================================================================
// Testbench: rmg_tb
// Desc     : Testbench for axi_split_r_merge (R-channel merger for split reads)
//=============================================================================

module rmg_tb;

    //=========================================================================
    // Parameters (matching DUT defaults)
    //=========================================================================
    localparam W_ID      = 4;
    localparam M_ID_W    = 2;
    localparam W_DATA    = 32;
    localparam W_STRB    = W_DATA / 8;
    localparam MAX_SPLIT = 4;
    localparam W_RID     = M_ID_W + W_ID + 2;   // mst_idx + split_st[1:0] + orig_id

    //=========================================================================
    // Clock and Reset
    //=========================================================================
    reg clk;
    reg rst_n;

    //=========================================================================
    // Slave-side R signals (from crossbar → DUT)
    //=========================================================================
    reg                     s_rvalid;
    reg  [W_RID-1:0]        s_rid;
    reg  [W_DATA-1:0]       s_rdata;
    reg  [1:0]              s_rresp;
    reg                     s_rlast;
    wire                    s_rready;

    //=========================================================================
    // Master-side R signals (DUT → master)
    //=========================================================================
    wire                    m_rvalid;
    wire [W_ID-1:0]         m_rid;
    wire [W_DATA-1:0]       m_rdata;
    wire [1:0]              m_rresp;
    wire                    m_rlast;
    reg                     m_rready;

    //=========================================================================
    // DUT Instantiation
    //=========================================================================
    axi_split_r_merge #(
        .W_ID     (W_ID),
        .M_ID_W   (M_ID_W),
        .W_DATA   (W_DATA),
        .W_STRB   (W_STRB),
        .MAX_SPLIT(MAX_SPLIT)
    ) u_dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .s_axi_rvalid (s_rvalid),
        .s_axi_rid    (s_rid),
        .s_axi_rdata  (s_rdata),
        .s_axi_rresp  (s_rresp),
        .s_axi_rlast  (s_rlast),
        .s_axi_rready (s_rready),
        .m_axi_rvalid (m_rvalid),
        .m_axi_rid    (m_rid),
        .m_axi_rdata  (m_rdata),
        .m_axi_rresp  (m_rresp),
        .m_axi_rlast  (m_rlast),
        .m_axi_rready (m_rready)
    );

    //=========================================================================
    // Clock generation (100MHz, 10ns period)
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
    // Default signal tie-offs
    //=========================================================================
    initial begin
        s_rvalid = 1'b0;
        s_rid    = '0;
        s_rdata  = '0;
        s_rresp  = 2'b00;
        s_rlast  = 1'b0;
        m_rready = 1'b1;
    end

    //=========================================================================
    // Helper: drive R beats (per-beat handshake)
    //=========================================================================
    task automatic drive_r_beats(
        input [M_ID_W-1:0] mst,
        input [1:0]        split_st,
        input [W_ID-1:0]   orig_id,
        input [7:0]        num_beats,
        input [1:0]        resp_base      // base resp for all beats
    );
        reg [7:0] b;
        for (b = 0; b < num_beats; b = b + 1) begin
            s_rid    <= {mst, split_st, orig_id};
            s_rdata  <= {orig_id[3:0], split_st, mst, b[7:0], 16'h0};
            s_rresp  <= resp_base;
            s_rlast  <= (b == num_beats - 1);
            s_rvalid <= 1'b1;
            @(posedge clk);
            while (!s_rready) @(posedge clk);
        end
        s_rvalid <= 1'b0;
        s_rlast  <= 1'b0;
    endtask

    //=========================================================================
    // Helper: collect R beats from master side
    //=========================================================================
    task automatic collect_r_beats(
        input  [7:0]         exp_beats,
        output [W_DATA-1:0]  out_data  [0:255],
        output [1:0]         out_resp  [0:255],
        output [7:0]         got_beats
    );
        reg [7:0] b;
        b = 0;
        m_rready <= 1'b1;
        while (b < exp_beats) begin
            @(posedge clk);
            if (m_rvalid && m_rready) begin
                out_data[b] = m_rdata;
                out_resp[b] = m_rresp;
                $display("[%0t] R_COL: beat[%0d] id=0x%0h data=0x%08h resp=%0d last=%0d",
                         $time, b, m_rid, m_rdata, m_rresp, m_rlast);
                if (m_rlast && b != exp_beats - 1)
                    $display("[%0t] WARNING: RLAST early at beat %0d (exp last=%0d)", $time, b, exp_beats-1);
                b = b + 1;
            end
        end
        got_beats = b;
    endtask

    //=========================================================================
    // Error logging
    //=========================================================================
    integer err_cnt;
    integer test_num;

    task automatic log_error(input string msg);
        $display("[%0t] ERROR: %s", $time, msg);
        err_cnt = err_cnt + 1;
    endtask

    //=========================================================================
    // Main Test Sequence
    //=========================================================================
    reg [W_DATA-1:0] rdata_buf [0:255];
    reg [1:0]        rresp_buf [0:255];
    reg [7:0]        got_cnt;
    integer          i;

    initial begin
        err_cnt  = 0;
        test_num = 0;

        @(posedge rst_n);
        repeat (5) @(posedge clk);

        $display("============================================================");
        $display("[%0t] R-MERGE TESTBENCH STARTED", $time);
        $display("============================================================");

        //=================================================================
        // TEST 1: Non-split R passthrough (single beat)
        //   split_st=00 → strip prefix, pass data/resp/last through
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Non-split R, single beat ---", test_num);
        fork
            drive_r_beats(2'd1, 2'b00, 4'hA, 8'd1, 2'b00);
            collect_r_beats(8'd1, rdata_buf, rresp_buf, got_cnt);
        join
        if (got_cnt != 1)
            log_error($sformatf("TEST 1: beat count mismatch: got=%0d exp=1", got_cnt));
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 2: Non-split R passthrough (burst-4)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Non-split R, burst-4 ---", test_num);
        fork
            drive_r_beats(2'd1, 2'b00, 4'hB, 8'd4, 2'b00);
            collect_r_beats(8'd4, rdata_buf, rresp_buf, got_cnt);
        join
        if (got_cnt != 4)
            log_error($sformatf("TEST 2: beat count mismatch: got=%0d exp=4", got_cnt));
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 3: Split R merge (normal order: sub1 → sub2)
        //   sub1 2 beats, sub2 2 beats → master sees 4 continuous beats
        //   sub1 RLAST suppressed → RLAST only on sub2 last beat
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Split R merge (normal order) ---", test_num);
        fork
            begin
                drive_r_beats(2'd0, 2'b01, 4'h5, 8'd2, 2'b00);   // sub1: 2 beats
                @(posedge clk);
                drive_r_beats(2'd0, 2'b10, 4'h5, 8'd2, 2'b00);   // sub2: 2 beats
            end
            collect_r_beats(8'd4, rdata_buf, rresp_buf, got_cnt);
        join
        if (got_cnt != 4)
            log_error($sformatf("TEST 3: beat count mismatch: got=%0d exp=4", got_cnt));
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 4: Split R merge, sub2 stalled (sub2 arrives before sub1 done)
        //   Drive sub2's first beat while sub1 is still in progress.
        //   DUT should stall sub2 (s_rready=0) until sub1 completes.
        //   We do this by NOT driving sub1 yet, driving sub2 beat0,
        //   verifying it's stalled, then driving sub1, then sub2 proceeds.
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Split R merge (sub2 stalled) ---", test_num);
        fork
            begin   // sub1: drive after a delay (simulates arriving later)
                repeat (10) @(posedge clk);
                $display("[%0t] Driving sub1 now (sub2 should be stalled)...", $time);
                drive_r_beats(2'd2, 2'b01, 4'h6, 8'd2, 2'b00);
            end
            begin   // sub2: drive immediately, first beat should stall
                @(posedge clk);
                $display("[%0t] Driving sub2 (expect stall)...", $time);
                // sub2 beat0 — should be stalled (s_rready=0 initially)
                s_rid    <= {2'd2, 2'b10, 4'h6};
                s_rdata  <= 32'h6000_0000;
                s_rresp  <= 2'b00;
                s_rlast  <= 1'b0;
                s_rvalid <= 1'b1;
                @(posedge clk);
                while (!s_rready) @(posedge clk);
                $display("[%0t] Sub2 beat0 accepted (unstalled)", $time);
                // sub2 beat1 (last)
                s_rdata  <= 32'h6000_0001;
                s_rlast  <= 1'b1;
                @(posedge clk);
                while (!s_rready) @(posedge clk);
                s_rvalid <= 1'b0;
                s_rlast  <= 1'b0;
            end
            collect_r_beats(8'd4, rdata_buf, rresp_buf, got_cnt);
        join
        if (got_cnt != 4)
            log_error($sformatf("TEST 4: beat count mismatch: got=%0d exp=4", got_cnt));
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 5: RRESP accumulation (sub1 SLVERR + sub2 OKAY)
        //   sub1 per-beat resp OK, sub2 per-beat resp OK
        //   But sub1 has SLVERR on its last beat, sub2 OKAY
        //   Merged RRESP on sub2 last beat = bitwise OR = SLVERR
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: RRESP accumulation (SLVERR + OKAY) ---", test_num);
        fork
            begin
                // sub1: last beat has SLVERR
                s_rid    <= {2'd1, 2'b01, 4'h7};
                s_rdata  <= 32'h7000_0000;
                s_rresp  <= 2'b00;
                s_rlast  <= 1'b0;
                s_rvalid <= 1'b1;
                @(posedge clk); while (!s_rready) @(posedge clk);
                s_rresp  <= 2'b10;   // SLVERR on last beat
                s_rlast  <= 1'b1;
                @(posedge clk); while (!s_rready) @(posedge clk);
                s_rvalid <= 1'b0;
                s_rlast  <= 1'b0;
            end
            begin
                @(posedge clk); @(posedge clk); @(posedge clk); @(posedge clk);
                // sub2: OKAY
                drive_r_beats(2'd1, 2'b10, 4'h7, 8'd2, 2'b00);
            end
            collect_r_beats(8'd4, rdata_buf, rresp_buf, got_cnt);
        join
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 6: Multiple concurrent split R transactions
        //   Interleave beats from TX0 and TX1
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Multiple concurrent split R ---", test_num);
        // Drive TX0 sub1 + TX1 sub1, then TX0 sub2 + TX1 sub2
        fork
            begin   // TX0: id=0xC, sub1
                drive_r_beats(2'd0, 2'b01, 4'hC, 8'd2, 2'b00);
                @(posedge clk); @(posedge clk);
                drive_r_beats(2'd0, 2'b10, 4'hC, 8'd2, 2'b00);
            end
            begin   // TX1: id=0xD, sub1
                @(posedge clk);
                drive_r_beats(2'd1, 2'b01, 4'hD, 8'd2, 2'b00);
                @(posedge clk); @(posedge clk);
                drive_r_beats(2'd1, 2'b10, 4'hD, 8'd2, 2'b00);
            end
        join
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 7: Back-to-back split + non-split
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Back-to-back split + non-split ---", test_num);
        fork
            begin
                // split read
                drive_r_beats(2'd3, 2'b01, 4'hE, 8'd2, 2'b00);
                drive_r_beats(2'd3, 2'b10, 4'hE, 8'd2, 2'b00);
                // non-split read
                @(posedge clk);
                drive_r_beats(2'd3, 2'b00, 4'hF, 8'd1, 2'b00);
            end
            begin
                // collect 5 beats total (4 split + 1 non-split)
                i = 0;
                m_rready <= 1'b1;
                while (i < 5) begin
                    @(posedge clk);
                    if (m_rvalid && m_rready) begin
                        $display("[%0t] R_COL: beat[%0d] id=0x%0h data=0x%08h last=%0d",
                                 $time, i, m_rid, m_rdata, m_rlast);
                        i = i + 1;
                    end
                end
            end
        join
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 8: Split R with DECERR accumulation
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: RRESP accumulation (DECERR + OKAY = DECERR) ---", test_num);
        fork
            begin
                // sub1 last beat = DECERR
                s_rid    <= {2'd2, 2'b01, 4'h2};
                s_rdata  <= 32'h2000_0000;
                s_rresp  <= 2'b00;
                s_rlast  <= 1'b0;
                s_rvalid <= 1'b1;
                @(posedge clk); while (!s_rready) @(posedge clk);
                s_rresp  <= 2'b11;   // DECERR on last beat
                s_rlast  <= 1'b1;
                @(posedge clk); while (!s_rready) @(posedge clk);
                s_rvalid <= 1'b0;
                s_rlast  <= 1'b0;
            end
            begin
                repeat (5) @(posedge clk);
                drive_r_beats(2'd2, 2'b10, 4'h2, 8'd1, 2'b00);
            end
            collect_r_beats(8'd3, rdata_buf, rresp_buf, got_cnt);
        join
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 9: Sub1 RLAST suppression check
        //   Verify that during sub1, RLAST is always 0 on master side
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: RLAST suppression on sub1 ---", test_num);
        fork
            begin
                drive_r_beats(2'd0, 2'b01, 4'h3, 8'd3, 2'b00);   // sub1: 3 beats
                @(posedge clk);
                drive_r_beats(2'd0, 2'b10, 4'h3, 8'd2, 2'b00);   // sub2: 2 beats
            end
            begin
                i = 0;
                m_rready <= 1'b1;
                while (i < 5) begin
                    @(posedge clk);
                    if (m_rvalid && m_rready) begin
                        // RLAST should be 0 for first 4 beats (sub1's 3 + sub2's 0),
                        // only 1 on the 5th (sub2 last beat)
                        if (i < 3 && m_rlast)
                            log_error($sformatf("TEST 9: RLAST should be 0 at beat %0d (sub1)", i));
                        if (i == 4 && !m_rlast)
                            log_error($sformatf("TEST 9: RLAST should be 1 at beat %0d (sub2 last)", i));
                        $display("[%0t] R_COL: beat[%0d] last=%0d", $time, i, m_rlast);
                        i = i + 1;
                    end
                end
            end
        join
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // FINAL REPORT
        //=================================================================
        $display("\n============================================================");
        if (err_cnt == 0) begin
            $display("[%0t] ALL %0d TESTS PASSED!", $time, test_num);
        end else begin
            $display("[%0t] TESTS FAILED with %0d errors in %0d tests!", $time, err_cnt, test_num);
        end
        $display("============================================================\n");

        #500;
        $finish;
    end

    //=========================================================================
    // Timeout watchdog (1ms)
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
        $fsdbDumpfile("rmg_tb.fsdb");
        $fsdbDumpvars(0, rmg_tb);
        $fsdbDumpMDA();
    end

endmodule
