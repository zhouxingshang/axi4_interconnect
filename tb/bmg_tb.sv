//=============================================================================
// Testbench: bmg_tb
// Desc     : Testbench for axi_split_b_merge (B-channel response merger)
//=============================================================================

module bmg_tb;

    //=========================================================================
    // Parameters (matching DUT defaults)
    //=========================================================================
    localparam W_ID      = 4;
    localparam M_ID_W    = 2;
    localparam MAX_SPLIT = 4;
    localparam W_BID     = M_ID_W + W_ID + 2;   // mst_idx + split_st[1:0] + orig_id

    //=========================================================================
    // Clock and Reset
    //=========================================================================
    reg clk;
    reg rst_n;

    //=========================================================================
    // Slave-side B signals (from crossbar → DUT)
    //=========================================================================
    reg                     s_bvalid;
    reg  [W_BID-1:0]        s_bid;
    reg  [1:0]              s_bresp;
    wire                    s_bready;

    //=========================================================================
    // Master-side B signals (DUT → master)
    //=========================================================================
    wire                    m_bvalid;
    wire [W_ID-1:0]         m_bid;
    wire [1:0]              m_bresp;
    reg                     m_bready;

    //=========================================================================
    // DUT Instantiation
    //=========================================================================
    axi_split_b_merge #(
        .W_ID     (W_ID),
        .M_ID_W   (M_ID_W),
        .MAX_SPLIT(MAX_SPLIT)
    ) u_dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .s_axi_bvalid (s_bvalid),
        .s_axi_bid    (s_bid),
        .s_axi_bresp  (s_bresp),
        .s_axi_bready (s_bready),
        .m_axi_bvalid (m_bvalid),
        .m_axi_bid    (m_bid),
        .m_axi_bresp  (m_bresp),
        .m_axi_bready (m_bready)
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
        s_bvalid = 1'b0;
        s_bid    = '0;
        s_bresp  = 2'b00;
        m_bready = 1'b1;
    end

    //=========================================================================
    // Helper: drive a single B response (non-blocking, 1-cycle pulse)
    //=========================================================================
    task automatic drive_b(
        input [M_ID_W-1:0] mst,
        input [1:0]        split_st,
        input [W_ID-1:0]   orig_id,
        input [1:0]        resp
    );
        s_bid    <= {mst, split_st, orig_id};
        s_bresp  <= resp;
        s_bvalid <= 1'b1;
        @(posedge clk);
        while (!s_bready) @(posedge clk);
        s_bvalid <= 1'b0;
        $display("[%0t] B_DRV: mst=%0d split=%0d id=0x%0h resp=%0d → BREADY",
                 $time, mst, split_st, orig_id, resp);
    endtask

    //=========================================================================
    // Helper: collect B response from master side
    //=========================================================================
    task automatic collect_b(
        output [W_ID-1:0] out_id,
        output [1:0]      out_resp
    );
        m_bready <= 1'b1;
        @(posedge clk);
        while (!m_bvalid) @(posedge clk);
        out_id   = m_bid;
        out_resp = m_bresp;
        $display("[%0t] B_COL: id=0x%0h resp=%0d", $time, out_id, out_resp);
        @(posedge clk);
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
    reg [W_ID-1:0] col_id;
    reg [1:0]      col_resp;

    initial begin
        err_cnt  = 0;
        test_num = 0;

        @(posedge rst_n);
        repeat (5) @(posedge clk);

        $display("============================================================");
        $display("[%0t] B-MERGE TESTBENCH STARTED", $time);
        $display("============================================================");

        //=================================================================
        // TEST 1: Non-split B passthrough
        //   split_st=00 → strip {mst, split_st}, forward orig_id + resp
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Non-split B passthrough ---", test_num);
        fork
            drive_b(2'd1, 2'b00, 4'hA, 2'b00);
            collect_b(col_id, col_resp);
        join
        if (col_id !== 4'hA)
            log_error($sformatf("TEST 1: BID mismatch: got=0x%0h exp=0xA", col_id));
        if (col_resp !== 2'b00)
            log_error($sformatf("TEST 1: BRESP mismatch: got=%0d exp=0", col_resp));
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 2: Split B merge (trans1 then trans2)
        //   sub1 arrives (stored), sub2 arrives → merge & forward
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Split B merge (normal order) ---", test_num);
        fork
            begin
                drive_b(2'd0, 2'b01, 4'h5, 2'b00);   // sub-transaction 1
                drive_b(2'd0, 2'b10, 4'h5, 2'b00);   // sub-transaction 2 → triggers merge
            end
            begin
                // Should see exactly 1 merged B forwarded
                @(posedge clk);
                while (!m_bvalid) @(posedge clk);
                col_id   = m_bid;
                col_resp = m_bresp;
                $display("[%0t] B_MERGED: id=0x%0h resp=%0d", $time, col_id, col_resp);
            end
        join
        if (col_id !== 4'h5)
            log_error($sformatf("TEST 2: merged BID mismatch: got=0x%0h exp=0x5", col_id));
        if (col_resp !== 2'b00)
            log_error($sformatf("TEST 2: merged BRESP mismatch: got=%0d exp=0", col_resp));
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 3: Split B merge (trans2 then trans1 — reverse order)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Split B merge (reverse order) ---", test_num);
        fork
            begin
                drive_b(2'd0, 2'b10, 4'h6, 2'b00);   // sub2 first
                drive_b(2'd0, 2'b01, 4'h6, 2'b00);   // sub1 second → merge
            end
            begin
                @(posedge clk);
                while (!m_bvalid) @(posedge clk);
                col_id   = m_bid;
                col_resp = m_bresp;
                $display("[%0t] B_MERGED: id=0x%0h resp=%0d", $time, col_id, col_resp);
            end
        join
        if (col_id !== 4'h6)
            log_error($sformatf("TEST 3: merged BID mismatch: got=0x%0h exp=0x6", col_id));
        if (col_resp !== 2'b00)
            log_error($sformatf("TEST 3: merged BRESP mismatch: got=%0d exp=0", col_resp));
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 4: BRESP accumulation (sub1 SLVERR + sub2 OKAY → SLVERR)
        //   BRESP bitwise OR: 2'b10 | 2'b00 = 2'b10 (SLVERR)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: BRESP accumulation (SLVERR + OKAY = SLVERR) ---", test_num);
        fork
            begin
                drive_b(2'd1, 2'b01, 4'h7, 2'b10);   // sub1: SLVERR
                drive_b(2'd1, 2'b10, 4'h7, 2'b00);   // sub2: OKAY
            end
            begin
                @(posedge clk);
                while (!m_bvalid) @(posedge clk);
                col_id   = m_bid;
                col_resp = m_bresp;
                $display("[%0t] B_MERGED: id=0x%0h resp=%0d", $time, col_id, col_resp);
            end
        join
        if (col_id !== 4'h7)
            log_error($sformatf("TEST 4: merged BID mismatch: got=0x%0h exp=0x7", col_id));
        if (col_resp !== 2'b10)
            log_error($sformatf("TEST 4: merged BRESP mismatch: got=%0d exp=2 (SLVERR)", col_resp));
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 5: Both sub-transactions SLVERR → merged SLVERR
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: BRESP accumulation (SLVERR + SLVERR = SLVERR) ---", test_num);
        fork
            begin
                drive_b(2'd0, 2'b01, 4'h8, 2'b10);   // sub1: SLVERR
                drive_b(2'd0, 2'b10, 4'h8, 2'b10);   // sub2: SLVERR
            end
            begin
                @(posedge clk);
                while (!m_bvalid) @(posedge clk);
                col_id   = m_bid;
                col_resp = m_bresp;
                $display("[%0t] B_MERGED: id=0x%0h resp=%0d", $time, col_id, col_resp);
            end
        join
        if (col_resp !== 2'b10)
            log_error($sformatf("TEST 5: merged BRESP mismatch: got=%0d exp=2 (SLVERR)", col_resp));
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 6: DECERR accumulation (sub1 DECERR + sub2 OKAY → DECERR)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: BRESP accumulation (DECERR + OKAY = DECERR) ---", test_num);
        fork
            begin
                drive_b(2'd3, 2'b01, 4'h9, 2'b11);   // sub1: DECERR
                drive_b(2'd3, 2'b10, 4'h9, 2'b00);   // sub2: OKAY
            end
            begin
                @(posedge clk);
                while (!m_bvalid) @(posedge clk);
                col_id   = m_bid;
                col_resp = m_bresp;
                $display("[%0t] B_MERGED: id=0x%0h resp=%0d", $time, col_id, col_resp);
            end
        join
        if (col_resp !== 2'b11)
            log_error($sformatf("TEST 6: merged BRESP mismatch: got=%0d exp=3 (DECERR)", col_resp));
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 7: Multiple concurrent split transactions (different IDs)
        //   TX0: id=0xC, TX1: id=0xD, interleave sub1/sub2
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Multiple concurrent split B ---", test_num);
        fork
            begin   // TX0 sub1
                drive_b(2'd0, 2'b01, 4'hC, 2'b00);
            end
            begin   // TX1 sub1
                @(posedge clk);
                drive_b(2'd1, 2'b01, 4'hD, 2'b00);
            end
            begin   // TX0 sub2 → merge TX0
                @(posedge clk);
                @(posedge clk);
                drive_b(2'd0, 2'b10, 4'hC, 2'b00);
            end
            begin   // TX1 sub2 → merge TX1
                @(posedge clk);
                @(posedge clk);
                @(posedge clk);
                drive_b(2'd1, 2'b10, 4'hD, 2'b00);
            end
            begin   // Collector: runs in parallel to catch both merges
                repeat (2) begin
                    @(posedge clk);
                    while (!m_bvalid) @(posedge clk);
                    $display("[%0t] B_MERGED: id=0x%0h resp=%0d", $time, m_bid, m_bresp);
                    if (m_bid !== 4'hC && m_bid !== 4'hD)
                        log_error($sformatf("TEST 7: unexpected merged BID=0x%0h", m_bid));
                    @(posedge clk);  // wait 1 cycle for deallocation
                end
            end
        join
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 8: Back-to-back non-split + split
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Back-to-back non-split + split ---", test_num);
        fork
            begin
                drive_b(2'd2, 2'b00, 4'hE, 2'b00);    // non-split
                drive_b(2'd2, 2'b01, 4'hF, 2'b00);    // split sub1
                drive_b(2'd2, 2'b10, 4'hF, 2'b00);    // split sub2 → merge
            end
            begin
                repeat (2) begin
                    @(posedge clk);
                    while (!m_bvalid) @(posedge clk);
                    $display("[%0t] B_COL: id=0x%0h resp=%0d", $time, m_bid, m_bresp);
                end
            end
        join
        $display("[%0t] TEST %0d DONE (errors=%0d)", $time, test_num, err_cnt);

        //=================================================================
        // TEST 9: Non-split B with non-zero resp
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Non-split B with SLVERR ---", test_num);
        fork
            drive_b(2'd1, 2'b00, 4'h3, 2'b10);
            collect_b(col_id, col_resp);
        join
        if (col_resp !== 2'b10)
            log_error($sformatf("TEST 9: non-split BRESP mismatch: got=%0d exp=2", col_resp));
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
        $fsdbDumpfile("bmg_tb.fsdb");
        $fsdbDumpvars(0, bmg_tb);
        $fsdbDumpMDA();
    end

endmodule
