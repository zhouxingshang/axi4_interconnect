//=============================================================================
// Testbench: axi_s2m_s_amt_tb
// Desc     : Testbench for axi_s2m_s_amt (slave-to-master response mux)
//=============================================================================

module axi_s2m_tb;

    //=========================================================================
    // Parameters (matching crossbar config)
    //=========================================================================
    localparam MST_AMT         = 3;
    localparam SLV_AMT         = 3;
    localparam W_CID           = 2;
    localparam W_ID            = 6;
    localparam W_ADDR          = 32;
    localparam W_DATA          = 32;
    localparam W_STRB          = W_DATA / 8;
    localparam W_SID           = W_CID + W_ID;           // 10
    localparam MST_ID_W        = $clog2(MST_AMT);       // 2
    localparam MST_ID_FIELD_MSB = W_SID - 1;             // 9
    localparam MST_ID_FIELD_LSB = W_ID;                  // 6
    localparam NUM_B_WIDTH     = W_SID + 2 + 1;          // 13
    localparam NUM_R_WIDTH     = W_SID + W_DATA + 2 + 1 + 1; // 46

    //=========================================================================
    // Clock and Reset
    //=========================================================================
    reg         clk;
    reg         rst_n;

    //=========================================================================
    // DUT Master-Side Signals
    //=========================================================================
    wire [W_SID-1:0]   M_BID;
    wire [1:0]         M_BRESP;
    wire               M_BVALID;
    reg                M_BREADY;

    wire [W_SID-1:0]   M_RSID;
    wire [W_DATA-1:0]  M_RDATA;
    wire [1:0]         M_RRESP;
    wire               M_RLAST;
    wire               M_RVALID;
    reg                M_RREADY;

    //=========================================================================
    // DUT Slave-Side Signals (flattened)
    //=========================================================================
    reg  [W_SID*SLV_AMT-1:0]   S_BID;
    reg  [2*SLV_AMT-1:0]       S_BRESP;
    reg  [SLV_AMT-1:0]         S_BVALID;
    wire [SLV_AMT-1:0]         S_BREADY;

    reg  [W_SID*SLV_AMT-1:0]   S_RID;
    reg  [W_DATA*SLV_AMT-1:0]  S_RDATA;
    reg  [2*SLV_AMT-1:0]       S_RRESP;
    reg  [SLV_AMT-1:0]         S_RLAST;
    reg  [SLV_AMT-1:0]         S_RVALID;
    wire [SLV_AMT-1:0]         S_RREADY;

    //=========================================================================
    // DUT Control Signals
    //=========================================================================
    reg  [SLV_AMT-1:0]   r_order_grant;
    reg                  arbiter_type;

    //=========================================================================
    // Instantiate DUT (MASTER_ID=0: routes responses destined for master 0)
    //=========================================================================
    axi_s2m_s_amt #(
        .MASTER_ID        (0),
        .W_CID            (W_CID),
        .W_ID             (W_ID),
        .W_ADDR           (W_ADDR),
        .W_DATA           (W_DATA),
        .W_STRB           (W_STRB),
        .W_SID            (W_SID),
        .MST_AMT          (MST_AMT),
        .SLV_AMT          (SLV_AMT),
        .MST_ID_FIELD_MSB (MST_ID_FIELD_MSB),
        .MST_ID_FIELD_LSB (MST_ID_FIELD_LSB)
    ) u_dut (
        .AXI_RSTn       (rst_n),
        .AXI_CLK        (clk),

        .M_BID          (M_BID),
        .M_BRESP        (M_BRESP),
        .M_BVALID       (M_BVALID),
        .M_BREADY       (M_BREADY),

        .M_RSID         (M_RSID),
        .M_RDATA        (M_RDATA),
        .M_RRESP        (M_RRESP),
        .M_RLAST        (M_RLAST),
        .M_RVALID       (M_RVALID),
        .M_RREADY       (M_RREADY),

        .S_BID          (S_BID),
        .S_BRESP        (S_BRESP),
        .S_BVALID       (S_BVALID),
        .S_BREADY       (S_BREADY),

        .S_RID          (S_RID),
        .S_RDATA        (S_RDATA),
        .S_RRESP        (S_RRESP),
        .S_RLAST        (S_RLAST),
        .S_RVALID       (S_RVALID),
        .S_RREADY       (S_RREADY),

        .r_order_grant  (r_order_grant),
        .arbiter_type   (arbiter_type)
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
    // Default signal initialization
    //=========================================================================
    initial begin
        M_BREADY     = 1'b0;
        M_RREADY     = 1'b0;
        S_BID        = '0;
        S_BRESP      = '0;
        S_BVALID     = '0;
        S_RID        = '0;
        S_RDATA      = '0;
        S_RRESP      = '0;
        S_RLAST      = '0;
        S_RVALID     = '0;
        r_order_grant = {SLV_AMT{1'b1}};  // All slaves allowed
        arbiter_type  = 1'b0;             // Round-robin
    end

    //=========================================================================
    // Helper: Build SID for a given master
    // SID format: {mst_idx[MST_ID_W-1:0], original_ID[W_ID-1:0]}
    // SID[9:6] = mst_idx, SID[5:0] = original_ID
    //=========================================================================
    function automatic [W_SID-1:0] make_sid;
        input [MST_ID_W-1:0] mst_id;
        input [W_ID-1:0]     orig_id;
        begin
            make_sid = {mst_id, orig_id};
        end
    endfunction

    //=========================================================================
    // Slave BFM: Send B response from a slave
    //=========================================================================
    task automatic slv_send_b(
        input int               slv_id,
        input [W_ID-1:0]        orig_id,
        input [MST_ID_W-1:0]    dst_mst,   // destination master
        input [1:0]             bresp
    );
        reg [W_SID-1:0] sid;
        begin
            sid = make_sid(dst_mst, orig_id);
            $display("[%0t] SLV[%0d] B: sid=0x%03x dst_mst=%0d id=%0d resp=%0d",
                     $time, slv_id, sid, dst_mst, orig_id, bresp);

            S_BID[W_SID*(slv_id+1)-1 -: W_SID] <= sid;
            S_BRESP[2*(slv_id+1)-1 -: 2]      <= bresp;
            S_BVALID[slv_id] <= 1'b1;

            @(posedge clk);
            while (!S_BREADY[slv_id]) @(posedge clk);
            S_BVALID[slv_id] <= 1'b0;
            $display("[%0t] SLV[%0d] B handshake done", $time, slv_id);
        end
    endtask

    //=========================================================================
    // Slave BFM: Send R response (single beat) from a slave
    //=========================================================================
    task automatic slv_send_r(
        input int               slv_id,
        input [W_ID-1:0]        orig_id,
        input [MST_ID_W-1:0]    dst_mst,
        input [W_DATA-1:0]      rdata,
        input [1:0]             rresp,
        input                   rlast
    );
        reg [W_SID-1:0] sid;
        begin
            sid = make_sid(dst_mst, orig_id);
            $display("[%0t] SLV[%0d] R: sid=0x%03x dst_mst=%0d data=0x%08h last=%0d",
                     $time, slv_id, sid, dst_mst, rdata, rlast);

            S_RID[W_SID*(slv_id+1)-1 -: W_SID]       <= sid;
            S_RDATA[W_DATA*(slv_id+1)-1 -: W_DATA]   <= rdata;
            S_RRESP[2*(slv_id+1)-1 -: 2]             <= rresp;
            S_RLAST[slv_id]                          <= rlast;
            S_RVALID[slv_id] <= 1'b1;

            @(posedge clk);
            while (!S_RREADY[slv_id]) @(posedge clk);
            S_RVALID[slv_id] <= 1'b0;
            $display("[%0t] SLV[%0d] R handshake done", $time, slv_id);
        end
    endtask

    //=========================================================================
    // Slave BFM: Send R burst from a slave
    //=========================================================================
    task automatic slv_send_r_burst(
        input int               slv_id,
        input [W_ID-1:0]        orig_id,
        input [MST_ID_W-1:0]    dst_mst,
        input [W_DATA-1:0]      rdata    [],
        input [1:0]             rresp
    );
        integer b;
        begin
            for (b = 0; b < rdata.size(); b++) begin
                slv_send_r(slv_id, orig_id, dst_mst, rdata[b], rresp,
                           (b == rdata.size() - 1));
            end
        end
    endtask

    //=========================================================================
    // Master BFM: Receive B response on master side
    //=========================================================================
    task automatic mst_recv_b(
        output [W_SID-1:0]  bid,
        output [1:0]        bresp
    );
        begin
            @(posedge clk);
            while (!M_BVALID) @(posedge clk);
            M_BREADY <= 1'b1;
            bid   = M_BID;
            bresp = M_BRESP;
            @(posedge clk);
            M_BREADY <= 1'b0;
            while (M_BVALID && M_BREADY) @(posedge clk); // in case of back-to-back
        end
    endtask

    //=========================================================================
    // Master BFM: Receive R beat on master side
    //=========================================================================
    task automatic mst_recv_r(
        output [W_SID-1:0]  rsid,
        output [W_DATA-1:0] rdata,
        output [1:0]        rresp,
        output              rlast
    );
        begin
            @(posedge clk);
            while (!M_RVALID) @(posedge clk);
            M_RREADY <= 1'b1;
            rsid  = M_RSID;
            rdata = M_RDATA;
            rresp = M_RRESP;
            rlast = M_RLAST;
            @(posedge clk);
            M_RREADY <= 1'b0;
        end
    endtask

    //=========================================================================
    // Master BFM: Receive full R burst on master side
    //=========================================================================
    task automatic mst_recv_r_burst(
        output [W_SID-1:0]   rsid,
        ref   [W_DATA-1:0]   rdata [0:7],
        output [1:0]         rresp,
        output int           beat_cnt
    );
        reg  rlast;
        int  b;
        begin
            b = 0;
            @(posedge clk);
            while (!M_RVALID) @(posedge clk);
            M_RREADY <= 1'b1;
            do begin
                rdata[b] = M_RDATA;
                rlast = M_RLAST;
                if (b == 0) begin rsid = M_RSID; rresp = M_RRESP; end
                b = b + 1;
                @(posedge clk);
                while (!M_RVALID) @(posedge clk);
            end while (!rlast);
            beat_cnt = b;
            @(posedge clk);
            M_RREADY <= 1'b0;
        end
    endtask

    //=========================================================================
    // Checker
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
    reg [W_SID-1:0]   cap_sid;
    reg [1:0]         cap_resp;
    reg [W_SID-1:0]   cap_sid1, cap_sid2;
    reg [1:0]         cap_resp1, cap_resp2;
    reg               cap_rlast;
    reg [W_DATA-1:0]  cap_rdata [0:7];
    integer           cap_rbeat_cnt;
    reg [W_DATA-1:0]  rburst_data [0:3];
    integer           bi;

    initial begin
        err_cnt = 0;

        @(posedge rst_n);
        repeat (5) @(posedge clk);

        $display("============================================================");
        $display("[%0t] AXI_S2M_S_AMT TESTBENCH STARTED (MASTER_ID=0)", $time);
        $display("============================================================");

        //=================================================================
        // TEST 1: Single B response — matching master ID
        //=================================================================
        $display("\n--- TEST 1: Single B (master match) ---");
        begin
            fork
                begin
                    slv_send_b(0, 6'h0A, 2'd0, 2'b00);       // slv0 → mst0, OK
                end
                begin
                    mst_recv_b(cap_sid, cap_resp);
                end
            join

            check_eq("M_BID",    cap_sid,     make_sid(2'd0, 6'h0A));
            check_eq("M_BRESP",  cap_resp,    2'b00);
        end
        $display("[%0t] TEST 1 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 2: B response — NON-matching master ID (filtered out)
        //=================================================================
        $display("\n--- TEST 2: B with non-matching master (should NOT see on M) ---");
        begin
            reg [W_SID-1:0] sid;
            // Drive B response from slv1 targeting master 1 (NOT master 0)
            // Use raw signal drive instead of slv_send_b to avoid deadlock
            sid = make_sid(2'd1, 6'h0B);
            S_BID[W_SID*2-1 -: W_SID]    <= sid;     // slv1 slot
            S_BRESP[2*2-1 -: 2]          <= 2'b00;
            S_BVALID[1] <= 1'b1;

            repeat (3) @(posedge clk);

            // Verify: M_BVALID stays 0 (response filtered out)
            check_eq("M_BVALID (should be 0)", M_BVALID, 1'b0);
            // Verify: S_BREADY stays 0 (no grant for wrong master)
            check_eq("S_BREADY[1] (should be 0)", S_BREADY[1], 1'b0);

            // Clean up
            S_BVALID[1] <= 1'b0;
        end
        $display("[%0t] TEST 2 done — B from wrong master filtered (errors=%0d)",
                 $time, err_cnt);

        //=================================================================
        // TEST 3: Two slaves B arbitration (matching masters)
        //=================================================================
        $display("\n--- TEST 3: Two-slave B arbitration ---");
        begin
            // Both sends and receives in one fork
            // Receives are sequential to avoid both sampling the same M_BVALID
            fork
                slv_send_b(0, 6'h10, 2'd0, 2'b00);
                slv_send_b(2, 6'h11, 2'd0, 2'b01);
                begin
                    mst_recv_b(cap_sid1, cap_resp1);
                    mst_recv_b(cap_sid2, cap_resp2);
                end
            join
            $display("[%0t] TEST 3: recv1 sid=0x%03x resp=%0d  recv2 sid=0x%03x resp=%0d",
                     $time, cap_sid1, cap_resp1, cap_sid2, cap_resp2);
        end
        $display("[%0t] TEST 3 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 4: Single R beat
        //=================================================================
        $display("\n--- TEST 4: Single R beat ---");
        begin
            fork
                begin
                    slv_send_r(0, 6'h20, 2'd0, 32'hDEAD_BEEF, 2'b00, 1'b1);
                end
                begin
                    mst_recv_r(cap_sid, cap_rdata[0], cap_resp, cap_rlast);
                    $display("[%0t] MST recv R: sid=0x%03x data=0x%08h last=%0d",
                             $time, cap_sid, cap_rdata[0], cap_rlast);
                end
            join

            check_eq("M_RSID",    cap_sid,        make_sid(2'd0, 6'h20));
            check_eq("M_RDATA",   cap_rdata[0],   32'hDEAD_BEEF);
            check_eq("M_RRESP",   cap_resp,       2'b00);
        end
        $display("[%0t] TEST 4 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 5: R burst (multi-beat)
        //=================================================================
        $display("\n--- TEST 5: R burst (4 beats) ---");
        begin
            rburst_data[0] = 32'hA000_0000;
            rburst_data[1] = 32'hA000_0001;
            rburst_data[2] = 32'hA000_0002;
            rburst_data[3] = 32'hA000_0003;

            fork
                begin
                    slv_send_r_burst(0, 6'h30, 2'd0, rburst_data, 2'b00);
                end
                begin
                    mst_recv_r_burst(cap_sid, cap_rdata, cap_resp, cap_rbeat_cnt);
                    $display("[%0t] MST recv R burst: %0d beats", $time, cap_rbeat_cnt);
                end
            join

            check_eq("R burst beat0", cap_rdata[0], 32'hA000_0000);
            check_eq("R burst beat1", cap_rdata[1], 32'hA000_0001);
            check_eq("R burst beat2", cap_rdata[2], 32'hA000_0002);
            check_eq("R burst beat3", cap_rdata[3], 32'hA000_0003);
        end
        $display("[%0t] TEST 5 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 6: R arbitration — two slaves respond simultaneously
        //=================================================================
        $display("\n--- TEST 6: Two-slave R arbitration ---");
        begin
            fork
                begin
                    slv_send_r(0, 6'h40, 2'd0, 32'hCAFE_0000, 2'b00, 1'b1);
                end
                begin
                    slv_send_r(1, 6'h41, 2'd0, 32'hCAFE_0001, 2'b00, 1'b1);
                end
            join
            $display("[%0t] TEST 6: both R sends done", $time);

            fork
                begin
                    mst_recv_r(cap_sid, cap_rdata[0], cap_resp, cap_rlast);
                    $display("[%0t] MST recv R1: sid=0x%03x data=0x%08h",
                             $time, cap_sid, cap_rdata[0]);
                end
                begin
                    mst_recv_r(cap_sid, cap_rdata[0], cap_resp, cap_rlast);
                    $display("[%0t] MST recv R2: sid=0x%03x data=0x%08h",
                             $time, cap_sid, cap_rdata[0]);
                end
            join
        end
        $display("[%0t] TEST 6 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 7: R-order grant gating
        //=================================================================
        $display("\n--- TEST 7: r_order_grant gating ---");
        begin
            // Block slave 0 via r_order_grant
            r_order_grant <= 3'b010;  // Only slave 1 allowed
            @(posedge clk);

            // Send R from both slaves targeting master 0
            // slv0 should be blocked (r_order_grant=0), slv1 should pass
            fork
                begin
                    slv_send_r(0, 6'h50, 2'd0, 32'hBBBB_0000, 2'b00, 1'b1);
                end
                begin
                    slv_send_r(1, 6'h51, 2'd0, 32'hBBBB_0001, 2'b00, 1'b1);
                end
            join
            $display("[%0t] TEST 7: both R sends done (slv0 should have been blocked)", $time);

            // Only one response should arrive (from slv1)
            mst_recv_r(cap_sid, cap_rdata[0], cap_resp, cap_rlast);
            check_eq("R from slv1 (gated ok)", cap_rdata[0], 32'hBBBB_0001);

            // Restore r_order_grant
            r_order_grant <= {SLV_AMT{1'b1}};
        end
        $display("[%0t] TEST 7 done (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 8: Back-to-back R + B interleaved
        //=================================================================
        $display("\n--- TEST 8: B+R interleaved ---");
        begin
            fork
                begin
                    // R from slv2
                    slv_send_r(2, 6'h60, 2'd0, 32'hF00D_0000, 2'b00, 1'b1);
                end
                begin
                    // B from slv0
                    slv_send_b(0, 6'h61, 2'd0, 2'b11);
                end
            join

            // Receive both (order may vary due to independent channels)
            fork
                begin
                    mst_recv_r(cap_sid, cap_rdata[0], cap_resp, cap_rlast);
                    $display("[%0t] MST recv R: data=0x%08h", $time, cap_rdata[0]);
                end
                begin
                    mst_recv_b(cap_sid, cap_resp);
                    $display("[%0t] MST recv B: sid=0x%03x resp=%0d", $time, cap_sid, cap_resp);
                end
            join
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
    // Waveform dump (FSDB)
    //=========================================================================
    initial begin
        $fsdbDumpfile("axi_s2m_tb.fsdb");
        $fsdbDumpvars(0, axi_s2m_tb);
        $fsdbDumpMDA;
    end

endmodule
