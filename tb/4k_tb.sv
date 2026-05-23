//=============================================================================
// Testbench: cross_4k_if_tb
// Desc     : Testbench for cross_4k_if (AXI 4KB boundary splitter)
//=============================================================================

module 4k_tb;

    //=========================================================================
    // Parameters (matching DUT defaults)
    //=========================================================================
    localparam W_ID   = 4;
    localparam W_ADDR = 32;
    localparam W_LEN  = 8;
    localparam W_DATA = 32;
    localparam W_STRB = W_DATA / 8;

    //=========================================================================
    // Clock and Reset
    //=========================================================================
    reg clk;
    reg rst_n;

    //=========================================================================
    // Master-side signals
    //=========================================================================
    // AR
    reg  [W_ID-1:0]     m_arid;
    reg  [W_ADDR-1:0]   m_araddr;
    reg  [W_LEN-1:0]    m_arlen;
    reg  [2:0]          m_arsize;
    reg  [1:0]          m_arburst;
    reg                 m_arvalid;
    wire                m_arready;

    // AW
    reg  [W_ID-1:0]     m_awid;
    reg  [W_ADDR-1:0]   m_awaddr;
    reg  [W_LEN-1:0]    m_awlen;
    reg  [2:0]          m_awsize;
    reg  [1:0]          m_awburst;
    reg                 m_awvalid;
    wire                m_awready;

    // W
    reg  [W_DATA-1:0]   m_wdata;
    reg  [W_STRB-1:0]   m_wstrb;
    reg                 m_wlast;
    reg                 m_wvalid;
    wire                m_wready;

    // Slave-side signals
    wire [W_ID+1:0]     s_arid;
    wire [W_ADDR-1:0]   s_araddr;
    wire [W_LEN-1:0]    s_arlen;
    wire [2:0]          s_arsize;
    wire [1:0]          s_arburst;
    wire                s_arvalid;
    reg                 s_arready;

    wire [W_ID+1:0]     s_awid;
    wire [W_ADDR-1:0]   s_awaddr;
    wire [W_LEN-1:0]    s_awlen;
    wire [2:0]          s_awsize;
    wire [1:0]          s_awburst;
    wire                s_awvalid;
    reg                 s_awready;

    wire [W_DATA-1:0]   s_wdata;
    wire [W_STRB-1:0]   s_wstrb;
    wire                s_wlast;
    wire                s_wvalid;
    reg                 s_wready;

    //=========================================================================
    // DUT Instantiation
    //=========================================================================
    cross_4k_if #(
        .W_ID  (W_ID),
        .W_ADDR(W_ADDR),
        .W_LEN (W_LEN),
        .W_DATA(W_DATA),
        .W_STRB(W_STRB)
    ) u_dut (
        .clk            (clk),
        .rst_n          (rst_n),
        // AR
        .m_axi_arid     (m_arid),
        .m_axi_araddr   (m_araddr),
        .m_axi_arlen    (m_arlen),
        .m_axi_arsize   (m_arsize),
        .m_axi_arburst  (m_arburst),
        .m_axi_arvalid  (m_arvalid),
        .m_axi_arready  (m_arready),
        // AW
        .m_axi_awid     (m_awid),
        .m_axi_awaddr   (m_awaddr),
        .m_axi_awlen    (m_awlen),
        .m_axi_awsize   (m_awsize),
        .m_axi_awburst  (m_awburst),
        .m_axi_awvalid  (m_awvalid),
        .m_axi_awready  (m_awready),
        // W
        .m_axi_wdata    (m_wdata),
        .m_axi_wstrb    (m_wstrb),
        .m_axi_wlast    (m_wlast),
        .m_axi_wvalid   (m_wvalid),
        .m_axi_wready   (m_wready),
        // Slave-side W
        .s_axi_wdata    (s_wdata),
        .s_axi_wstrb    (s_wstrb),
        .s_axi_wlast    (s_wlast),
        .s_axi_wvalid   (s_wvalid),
        .s_axi_wready   (s_wready),
        // Slave-side AR
        .s_axi_arid     (s_arid),
        .s_axi_araddr   (s_araddr),
        .s_axi_arlen    (s_arlen),
        .s_axi_arsize   (s_arsize),
        .s_axi_arburst  (s_arburst),
        .s_axi_arvalid  (s_arvalid),
        .s_axi_arready  (s_arready),
        // Slave-side AW
        .s_axi_awid     (s_awid),
        .s_axi_awaddr   (s_awaddr),
        .s_axi_awlen    (s_awlen),
        .s_axi_awsize   (s_awsize),
        .s_axi_awburst  (s_awburst),
        .s_axi_awvalid  (s_awvalid),
        .s_axi_awready  (s_awready)
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
        m_arid    = '0;
        m_araddr  = '0;
        m_arlen   = '0;
        m_arsize  = 3'd2;   // 4 bytes per beat
        m_arburst = 2'd1;   // INCR
        m_arvalid = 1'b0;

        m_awid    = '0;
        m_awaddr  = '0;
        m_awlen   = '0;
        m_awsize  = 3'd2;
        m_awburst = 2'd1;
        m_awvalid = 1'b0;

        m_wdata   = '0;
        m_wstrb   = '0;
        m_wlast   = 1'b0;
        m_wvalid  = 1'b0;

        s_arready = 1'b1;
        s_awready = 1'b1;
        s_wready  = 1'b1;
    end

    //=========================================================================
    // Helper: drive AR transaction
    //=========================================================================
    task automatic drive_ar(
        input [W_ID-1:0]   id,
        input [W_ADDR-1:0] addr,
        input [W_LEN-1:0]  len,
        input [2:0]        size,
        input [1:0]        burst
    );
        m_arid    <= id;
        m_araddr  <= addr;
        m_arlen   <= len;
        m_arsize  <= size;
        m_arburst <= burst;
        m_arvalid <= 1'b1;
        @(posedge clk);
        while (!m_arready) @(posedge clk);
        m_arvalid <= 1'b0;
        $display("[%0t] AR: id=0x%0h addr=0x%08h len=%0d size=%0d → ARREADY",
                 $time, id, addr, len, size);
    endtask

    //=========================================================================
    // Helper: drive AW + W transaction
    //=========================================================================
    task automatic drive_aw_w(
        input [W_ID-1:0]   id,
        input [W_ADDR-1:0] addr,
        input [W_LEN-1:0]  len,
        input [2:0]        size,
        input [1:0]        burst
    );
        reg [7:0] b;
        reg [7:0] total;
        total = len + 8'd1;

        // AW
        m_awid    <= id;
        m_awaddr  <= addr;
        m_awlen   <= len;
        m_awsize  <= size;
        m_awburst <= burst;
        m_awvalid <= 1'b1;
        @(posedge clk);
        while (!m_awready) @(posedge clk);
        m_awvalid <= 1'b0;
        $display("[%0t] AW: id=0x%0h addr=0x%08h len=%0d size=%0d → AWREADY",
                 $time, id, addr, len, size);

        // W beats
        for (b = 0; b < total; b = b + 1) begin
            m_wdata  <= {addr[15:0], b[7:0], 8'h0};
            m_wstrb  <= {W_STRB{1'b1}};
            m_wlast  <= (b == total - 1);
            m_wvalid <= 1'b1;
            @(posedge clk);
            while (!m_wready) @(posedge clk);
        end
        m_wvalid <= 1'b0;
        m_wlast  <= 1'b0;
    endtask

    //=========================================================================
    // Slave-side AR monitor
    //=========================================================================
    task automatic mon_s_ar();
        forever begin
            @(posedge clk);
            if (s_arvalid && s_arready) begin
                $display("[%0t] S_AR: id=0x%0h (pref=%0d) addr=0x%08h len=%0d size=%0d",
                         $time, s_arid, s_arid[W_ID+1-:2], s_araddr, s_arlen, s_arsize);
            end
        end
    endtask

    //=========================================================================
    // Slave-side AW monitor
    //=========================================================================
    task automatic mon_s_aw();
        forever begin
            @(posedge clk);
            if (s_awvalid && s_awready) begin
                $display("[%0t] S_AW: id=0x%0h (pref=%0d) addr=0x%08h len=%0d size=%0d",
                         $time, s_awid, s_awid[W_ID+1-:2], s_awaddr, s_awlen, s_awsize);
            end
        end
    endtask

    //=========================================================================
    // Slave-side W monitor
    //=========================================================================
    task automatic mon_s_w();
        reg [7:0] w_cnt;
        w_cnt = 0;
        forever begin
            @(posedge clk);
            if (s_wvalid && s_wready) begin
                $display("[%0t] S_W: beat[%0d] data=0x%08h last=%0d",
                         $time, w_cnt, s_wdata, s_wlast);
                w_cnt = w_cnt + 1;
                if (s_wlast) w_cnt = 0;
            end
        end
    endtask

    //=========================================================================
    // Main Test Sequence
    //=========================================================================
    integer err_cnt;
    integer test_num;

    initial begin
        err_cnt  = 0;
        test_num = 0;

        @(posedge rst_n);
        repeat (5) @(posedge clk);

        // Fork slave monitors
        fork
            mon_s_ar();
            mon_s_aw();
            mon_s_w();
        join_none

        $display("============================================================");
        $display("[%0t] CROSS_4K_IF TESTBENCH STARTED", $time);
        $display("============================================================");

        //=================================================================
        // TEST 1: Non-split AR (single beat, does not cross 4KB)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Non-split AR, single beat ---", test_num);
        drive_ar(4'hA, 32'h0000_0100, 8'd0, 3'd2, 2'd1);
        repeat (3) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 2: Non-split AR (burst within 4KB)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Non-split AR, burst within 4KB ---", test_num);
        drive_ar(4'hB, 32'h0000_0100, 8'd7, 3'd2, 2'd1);
        repeat (3) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 3: Split AR (crosses 4KB)
        //   addr=0xFFC, size=2 (4B/beat), len=1 (2 beats, 8 bytes)
        //   beats: 0xFFC-0xFFF, 0x1000-0x1003 → crosses 4KB at 0x1000
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Split AR, crosses 4KB ---", test_num);
        drive_ar(4'hC, 32'h0000_0FFC, 8'd1, 3'd2, 2'd1);
        repeat (5) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 4: Split AR with more beats
        //   addr=0xFF0, size=2 (4B/beat), len=7 (8 beats, 32 bytes)
        //   crosses: 0xFF0-0xFFF (4 beats), 0x1000-0x100F (4 beats)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Split AR, many beats ---", test_num);
        drive_ar(4'hD, 32'h0000_0FF0, 8'd7, 3'd2, 2'd1);
        repeat (5) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 5: Non-split AW+W (single beat)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Non-split AW+W, single beat ---", test_num);
        drive_aw_w(4'h1, 32'h0000_0200, 8'd0, 3'd2, 2'd1);
        repeat (3) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 6: Non-split AW+W (burst within 4KB)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Non-split AW+W, burst within 4KB ---", test_num);
        drive_aw_w(4'h2, 32'h0000_0200, 8'd3, 3'd2, 2'd1);
        repeat (3) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 7: Split AW+W (crosses 4KB)
        //   addr=0xFF8, size=2 (4B/beat), len=3 (4 beats, 16 bytes)
        //   beats: 0xFF8-0xFFB, 0xFFC-0xFFF, 0x1000-0x1003, 0x1004-0x1007
        //   page1: bytes 0xFF8..0xFFF = 8 bytes → 2 beats (0xFF8, 0xFFC)
        //   page2: bytes 0x1000..0x1007 = 8 bytes → 2 beats (0x1000, 0x1004)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Split AW+W, crosses 4KB ---", test_num);
        drive_aw_w(4'h3, 32'h0000_0FF8, 8'd3, 3'd2, 2'd1);
        repeat (5) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 8: Split AW+W, long burst
        //   addr=0xFE0, size=2 (4B/beat), len=15 (16 beats, 64 bytes)
        //   page1: 0xFE0..0xFFF = 32 bytes → 8 beats
        //   page2: 0x1000..0x101F = 32 bytes → 8 beats
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Split AW+W, long burst ---", test_num);
        drive_aw_w(4'h4, 32'h0000_0FE0, 8'd15, 3'd2, 2'd1);
        repeat (5) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 9: Back-to-back AR (non-split then split)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Back-to-back AR ---", test_num);
        drive_ar(4'hE, 32'h0000_0500, 8'd3, 3'd2, 2'd1);
        @(posedge clk);
        drive_ar(4'hF, 32'h0000_0FF0, 8'd7, 3'd2, 2'd1);
        repeat (5) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 10: Back-to-back AW+W (split then non-split)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Back-to-back AW+W ---", test_num);
        drive_aw_w(4'h5, 32'h0000_0F00, 8'd63, 3'd2, 2'd1);  // crosses 4KB
        @(posedge clk);
        drive_aw_w(4'h6, 32'h0000_0800, 8'd3, 3'd2, 2'd1);   // within 4KB
        repeat (5) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 11: AR addr exactly at 4KB boundary
        //   addr=0x1000, size=2, len=1 (2 beats: 0x1000-0x1007)
        //   page1: 0x1000..0x1007 → 2 beats, no split
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: AR at 4KB boundary (no split) ---", test_num);
        drive_ar(4'hA, 32'h0000_1000, 8'd1, 3'd2, 2'd1);
        repeat (3) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 12: AW+W ending right before boundary
        //   addr=0xFFC, size=2, len=0 (1 beat, 4 bytes: 0xFFC-0xFFF)
        //   end = 0xFFF, bit12=0 → no split
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: AW+W ending at 0xFFF (no split) ---", test_num);
        drive_aw_w(4'h7, 32'h0000_0FFC, 8'd0, 3'd2, 2'd1);
        repeat (3) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 13: Back-to-back split ARs (stress test)
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: Back-to-back split ARs ---", test_num);
        drive_ar(4'hA, 32'h0000_0FF0, 8'd7, 3'd2, 2'd1);   // crosses 4KB
        @(posedge clk);
        drive_ar(4'hB, 32'h0000_1FF0, 8'd7, 3'd2, 2'd1);   // crosses 4KB
        repeat (8) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // TEST 14: AR single beat at high address (addr=0xFFFFF000)
        //   size=2, len=0, 4 bytes at 0xFFFFF000 → within 4KB, no split
        //=================================================================
        test_num = test_num + 1;
        $display("\n--- TEST %0d: AR high addr, no split ---", test_num);
        drive_ar(4'hC, 32'hFFFF_F000, 8'd0, 3'd2, 2'd1);
        repeat (3) @(posedge clk);
        $display("[%0t] TEST %0d DONE", $time, test_num);

        //=================================================================
        // FINAL REPORT
        //=================================================================
        $display("\n============================================================");
        $display("[%0t] ALL %0d TESTS COMPLETED", $time, test_num);
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
        $fsdbDumpfile("4k_tb.fsdb");
        $fsdbDumpvars(0, 4k_tb);
        $fsdbDumpMDA();
    end

endmodule
