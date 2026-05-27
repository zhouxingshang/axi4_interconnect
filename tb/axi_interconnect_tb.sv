//=============================================================================
// Testbench: axi_interconnect_tb
//=============================================================================

module axi_interconnect_tb;

    //=========================================================================
    // Parameters (match DUT defaults)
    //=========================================================================
    localparam MST_AMT         = 4;
    localparam SLV_AMT         = 4;
    localparam W_ID            = 4;
    localparam DATA_WIDTH      = 32;
    localparam ADDR_WIDTH      = 32;
    localparam LEN_W           = 8;
    localparam SIZE_W          = 3;
    localparam BURST_W         = 2;
    localparam RESP_W          = 2;
    localparam W_STRB          = DATA_WIDTH / 8;
    localparam W_CID           = 2;
    localparam W_MID           = W_CID + W_ID;
    localparam W_SID           = $clog2(MST_AMT) + W_CID + W_ID;
    typedef logic [DATA_WIDTH-1:0] axi_data_buf_t [0:255];

    // 8KB address space per slave (decode bits [31:13])
    localparam SLV_ADDR_SPAN   = 13;   // 8KB = 2^13
    localparam [0:SLV_AMT*ADDR_WIDTH-1] SLV_ADDR_BASE = {
        32'h00006000,   // Slave 3: 0x6000-0x7FFF
        32'h00004000,   // Slave 2: 0x4000-0x5FFF
        32'h00002000,   // Slave 1: 0x2000-0x3FFF
        32'h00000000    // Slave 0: 0x0000-0x1FFF
    };
    localparam [0:SLV_AMT*8-1] SLV_ADDR_LEN = {
        8'd13, 8'd13, 8'd13, 8'd13
    };

    //=========================================================================
    // Clock and Reset
    //=========================================================================
    reg         clk;
    reg         rst_n;

    //=========================================================================
    // DUT Signals: Master Side (flattened)
    //=========================================================================
    // AW channel (driven by TB)
    reg  [W_ID*MST_AMT-1        : 0]  M_AXI_AWID;
    reg  [ADDR_WIDTH*MST_AMT-1  : 0]  M_AXI_AWADDR;
    reg  [LEN_W*MST_AMT-1       : 0]  M_AXI_AWLEN;
    reg  [SIZE_W*MST_AMT-1      : 0]  M_AXI_AWSIZE;
    reg  [BURST_W*MST_AMT-1     : 0]  M_AXI_AWBURST;
    reg  [MST_AMT-1             : 0]  M_AXI_AWVALID;
    wire [MST_AMT-1             : 0]  M_AXI_AWREADY;

    // W channel (driven by TB)
    reg  [DATA_WIDTH*MST_AMT-1  : 0]  M_AXI_WDATA;
    reg  [W_STRB*MST_AMT-1      : 0]  M_AXI_WSTRB;
    reg  [MST_AMT-1             : 0]  M_AXI_WLAST;
    reg  [MST_AMT-1             : 0]  M_AXI_WVALID;
    wire [MST_AMT-1             : 0]  M_AXI_WREADY;

    // B channel (DUT output)
    wire [W_ID*MST_AMT-1        : 0]  M_AXI_BID;
    wire [RESP_W*MST_AMT-1      : 0]  M_AXI_BRESP;
    wire [MST_AMT-1             : 0]  M_AXI_BVALID;
    reg  [MST_AMT-1             : 0]  M_AXI_BREADY;

    // AR channel (driven by TB)
    reg  [W_ID*MST_AMT-1        : 0]  M_AXI_ARID;
    reg  [ADDR_WIDTH*MST_AMT-1  : 0]  M_AXI_ARADDR;
    reg  [LEN_W*MST_AMT-1       : 0]  M_AXI_ARLEN;
    reg  [SIZE_W*MST_AMT-1      : 0]  M_AXI_ARSIZE;
    reg  [BURST_W*MST_AMT-1     : 0]  M_AXI_ARBURST;
    reg  [MST_AMT-1             : 0]  M_AXI_ARVALID;
    wire [MST_AMT-1             : 0]  M_AXI_ARREADY;

    // R channel (DUT output)
    wire [W_ID*MST_AMT-1        : 0]  M_AXI_RID;
    wire [DATA_WIDTH*MST_AMT-1  : 0]  M_AXI_RDATA;
    wire [RESP_W*MST_AMT-1      : 0]  M_AXI_RRESP;
    wire [MST_AMT-1             : 0]  M_AXI_RLAST;
    wire [MST_AMT-1             : 0]  M_AXI_RVALID;
    reg  [MST_AMT-1             : 0]  M_AXI_RREADY;

    //=========================================================================
    // DUT Signals: Slave Side (flattened)
    //=========================================================================
    // AW channel
    wire [W_SID*SLV_AMT-1       : 0]  S_AXI_AWID;
    wire [ADDR_WIDTH*SLV_AMT-1  : 0]  S_AXI_AWADDR;
    wire [LEN_W*SLV_AMT-1       : 0]  S_AXI_AWLEN;
    wire [SIZE_W*SLV_AMT-1      : 0]  S_AXI_AWSIZE;
    wire [BURST_W*SLV_AMT-1     : 0]  S_AXI_AWBURST;
    wire [SLV_AMT-1             : 0]  S_AXI_AWVALID;
    reg  [SLV_AMT-1             : 0]  S_AXI_AWREADY;

    // W channel
    wire [DATA_WIDTH*SLV_AMT-1  : 0]  S_AXI_WDATA;
    wire [W_STRB*SLV_AMT-1      : 0]  S_AXI_WSTRB;
    wire [SLV_AMT-1             : 0]  S_AXI_WLAST;
    wire [SLV_AMT-1             : 0]  S_AXI_WVALID;
    reg  [SLV_AMT-1             : 0]  S_AXI_WREADY;

    // B channel
    reg  [W_SID*SLV_AMT-1       : 0]  S_AXI_BID;
    reg  [RESP_W*SLV_AMT-1      : 0]  S_AXI_BRESP;
    reg  [SLV_AMT-1             : 0]  S_AXI_BVALID;
    wire [SLV_AMT-1             : 0]  S_AXI_BREADY;

    // AR channel
    wire [W_SID*SLV_AMT-1       : 0]  S_AXI_ARID;
    wire [ADDR_WIDTH*SLV_AMT-1  : 0]  S_AXI_ARADDR;
    wire [LEN_W*SLV_AMT-1       : 0]  S_AXI_ARLEN;
    wire [SIZE_W*SLV_AMT-1      : 0]  S_AXI_ARSIZE;
    wire [BURST_W*SLV_AMT-1     : 0]  S_AXI_ARBURST;
    wire [SLV_AMT-1             : 0]  S_AXI_ARVALID;
    reg  [SLV_AMT-1             : 0]  S_AXI_ARREADY;

    // R channel
    reg  [W_SID*SLV_AMT-1       : 0]  S_AXI_RID;
    reg  [DATA_WIDTH*SLV_AMT-1  : 0]  S_AXI_RDATA;
    reg  [RESP_W*SLV_AMT-1      : 0]  S_AXI_RRESP;
    reg  [SLV_AMT-1             : 0]  S_AXI_RLAST;
    reg  [SLV_AMT-1             : 0]  S_AXI_RVALID;
    wire [SLV_AMT-1             : 0]  S_AXI_RREADY;

    //=========================================================================
    // Control Signals
    //=========================================================================
    reg                             arbiter_type;

    //=========================================================================
    // DUT Instantiation
    //=========================================================================
    axi_interconnect #(
        .MST_AMT        (MST_AMT),
        .SLV_AMT        (SLV_AMT),
        .OUTSTANDING_AMT(8),
        .W_ID           (W_ID),
        .DATA_WIDTH     (DATA_WIDTH),
        .ADDR_WIDTH     (ADDR_WIDTH),
        .LEN_W          (LEN_W),
        .SIZE_W         (SIZE_W),
        .BURST_W        (BURST_W),
        .RESP_W         (RESP_W),
        .SLV_ADDR_BASE  (SLV_ADDR_BASE),
        .SLV_ADDR_LEN   (SLV_ADDR_LEN)
    ) u_dut (
        .AXI_CLK        (clk),
        .AXI_RSTn       (rst_n),

        // Master ports
        .M_AXI_AWID_i   (M_AXI_AWID),
        .M_AXI_AWADDR_i (M_AXI_AWADDR),
        .M_AXI_AWLEN_i  (M_AXI_AWLEN),
        .M_AXI_AWSIZE_i (M_AXI_AWSIZE),
        .M_AXI_AWBURST_i(M_AXI_AWBURST),
        .M_AXI_AWVALID_i(M_AXI_AWVALID),
        .M_AXI_AWREADY_o(M_AXI_AWREADY),

        .M_AXI_WDATA_i  (M_AXI_WDATA),
        .M_AXI_WSTRB_i  (M_AXI_WSTRB),
        .M_AXI_WLAST_i  (M_AXI_WLAST),
        .M_AXI_WVALID_i (M_AXI_WVALID),
        .M_AXI_WREADY_o (M_AXI_WREADY),

        .M_AXI_BID_o    (M_AXI_BID),
        .M_AXI_BRESP_o  (M_AXI_BRESP),
        .M_AXI_BVALID_o (M_AXI_BVALID),
        .M_AXI_BREADY_i (M_AXI_BREADY),

        .M_AXI_ARID_i   (M_AXI_ARID),
        .M_AXI_ARADDR_i (M_AXI_ARADDR),
        .M_AXI_ARLEN_i  (M_AXI_ARLEN),
        .M_AXI_ARSIZE_i (M_AXI_ARSIZE),
        .M_AXI_ARBURST_i(M_AXI_ARBURST),
        .M_AXI_ARVALID_i(M_AXI_ARVALID),
        .M_AXI_ARREADY_o(M_AXI_ARREADY),

        .M_AXI_RID_o    (M_AXI_RID),
        .M_AXI_RDATA_o  (M_AXI_RDATA),
        .M_AXI_RRESP_o  (M_AXI_RRESP),
        .M_AXI_RLAST_o  (M_AXI_RLAST),
        .M_AXI_RVALID_o (M_AXI_RVALID),
        .M_AXI_RREADY_i (M_AXI_RREADY),

        // Slave ports
        .S_AXI_AWID_o   (S_AXI_AWID),
        .S_AXI_AWADDR_o (S_AXI_AWADDR),
        .S_AXI_AWLEN_o  (S_AXI_AWLEN),
        .S_AXI_AWSIZE_o (S_AXI_AWSIZE),
        .S_AXI_AWBURST_o(S_AXI_AWBURST),
        .S_AXI_AWVALID_o(S_AXI_AWVALID),
        .S_AXI_AWREADY_i(S_AXI_AWREADY),

        .S_AXI_WDATA_o  (S_AXI_WDATA),
        .S_AXI_WSTRB_o  (S_AXI_WSTRB),
        .S_AXI_WLAST_o  (S_AXI_WLAST),
        .S_AXI_WVALID_o (S_AXI_WVALID),
        .S_AXI_WREADY_i (S_AXI_WREADY),

        .S_AXI_BID_i    (S_AXI_BID),
        .S_AXI_BRESP_i  (S_AXI_BRESP),
        .S_AXI_BVALID_i (S_AXI_BVALID),
        .S_AXI_BREADY_o (S_AXI_BREADY),

        .S_AXI_ARID_o   (S_AXI_ARID),
        .S_AXI_ARADDR_o (S_AXI_ARADDR),
        .S_AXI_ARLEN_o  (S_AXI_ARLEN),
        .S_AXI_ARSIZE_o (S_AXI_ARSIZE),
        .S_AXI_ARBURST_o(S_AXI_ARBURST),
        .S_AXI_ARVALID_o(S_AXI_ARVALID),
        .S_AXI_ARREADY_i(S_AXI_ARREADY),

        .S_AXI_RID_i    (S_AXI_RID),
        .S_AXI_RDATA_i  (S_AXI_RDATA),
        .S_AXI_RRESP_i  (S_AXI_RRESP),
        .S_AXI_RLAST_i  (S_AXI_RLAST),
        .S_AXI_RVALID_i (S_AXI_RVALID),
        .S_AXI_RREADY_o (S_AXI_RREADY),

        // Control
        .arbiter_type   (arbiter_type)
    );

    //=========================================================================
    // Clock Generation: 100MHz, 10ns period
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
    // Slave Memory Model
    //=========================================================================
    // Simple memory model per slave: 4KB of storage
    localparam SLV_MEM_DEPTH = 2048;  // 2048 x 32-bit = 8KB
    reg [DATA_WIDTH-1:0] slv_mem [0:SLV_AMT-1][0:SLV_MEM_DEPTH-1];

    // Initialize slave memory with known pattern (address-dependent)
    integer init_s, init_i;
    initial begin
        for (init_s = 0; init_s < SLV_AMT; init_s = init_s + 1) begin
            for (init_i = 0; init_i < SLV_MEM_DEPTH; init_i = init_i + 1) begin
                slv_mem[init_s][init_i] = {init_s[1:0], 14'h0, init_i[15:0]};
            end
        end
    end

    //=========================================================================
    // Slave Response Tasks
    //=========================================================================
    // Each slave handles its own AW/W/B and AR/R channels
    // Since the interconnect output is flattened, we slice out per-slave signals

    // Slave write handling
    task automatic slv_write_handler(int slv_id);
        reg [W_SID-1:0]     awid;
        reg [ADDR_WIDTH-1:0] awaddr;
        reg [LEN_W-1:0]      awlen;
        reg [2:0]            awsize;
        reg [1:0]            awburst;
        reg [DATA_WIDTH-1:0] wdata;
        reg [W_STRB-1:0]     wstrb;
        reg                  wlast;
        reg [W_SID-1:0]      bid_resp;
        reg [1:0]            bresp_val;
        reg [ADDR_WIDTH-1:0] word_idx;
        reg [7:0]            beat_cnt;
        reg [7:0]            total_beats;
        reg                  aw_done;
        integer              bi;
        forever begin
            @(posedge clk);
            // AW handshake
            if (S_AXI_AWVALID[slv_id] && S_AXI_AWREADY[slv_id]) begin
                awid    = S_AXI_AWID[W_SID*(slv_id+1)-1 -: W_SID];
                awaddr  = S_AXI_AWADDR[ADDR_WIDTH*(slv_id+1)-1 -: ADDR_WIDTH];
                awlen   = S_AXI_AWLEN[LEN_W*(slv_id+1)-1 -: LEN_W];
                awsize  = S_AXI_AWSIZE[SIZE_W*(slv_id+1)-1 -: SIZE_W];
                awburst = S_AXI_AWBURST[BURST_W*(slv_id+1)-1 -: BURST_W];
                aw_done = 1'b1;
                beat_cnt = 0;
                total_beats = awlen + 8'd1;
                word_idx = awaddr[SLV_ADDR_SPAN-1:2]; // word-aligned
            end

            // W data handshake
            if (aw_done && S_AXI_WVALID[slv_id] && S_AXI_WREADY[slv_id]) begin
                wdata = S_AXI_WDATA[DATA_WIDTH*(slv_id+1)-1 -: DATA_WIDTH];
                wstrb = S_AXI_WSTRB[W_STRB*(slv_id+1)-1 -: W_STRB];
                wlast = S_AXI_WLAST[slv_id];
                // Write to memory with byte strobes
                word_idx = awaddr[SLV_ADDR_SPAN-1:2] + beat_cnt;
                if (word_idx < SLV_MEM_DEPTH) begin
                    for (bi = 0; bi < W_STRB; bi = bi + 1) begin
                        if (wstrb[bi])
                            slv_mem[slv_id][word_idx][8*bi +: 8] = wdata[8*bi +: 8];
                    end
                end
                beat_cnt = beat_cnt + 8'd1;
                if (wlast) begin
                    aw_done = 1'b0;
                    bid_resp = awid;
                    bresp_val = 2'b00; // OKAY
                    // Drive B response
                    S_AXI_BID[W_SID*(slv_id+1)-1 -: W_SID] <= bid_resp;
                    S_AXI_BRESP[RESP_W*(slv_id+1)-1 -: RESP_W] <= bresp_val;
                    S_AXI_BVALID[slv_id] <= 1'b1;
                end
            end

            // B handshake complete
            if (S_AXI_BVALID[slv_id] && S_AXI_BREADY[slv_id]) begin
                S_AXI_BVALID[slv_id] <= 1'b0;
            end
        end
    endtask

    // Slave read handling
    task automatic slv_read_handler(int slv_id);
        reg [W_SID-1:0]      arid;
        reg [ADDR_WIDTH-1:0] araddr;
        reg [LEN_W-1:0]      arlen;
        reg [2:0]            arsize;
        reg [1:0]            arburst;
        reg [7:0]            beat_cnt;
        reg [7:0]            total_beats;
        reg [ADDR_WIDTH-1:0] word_idx;
        reg                  ar_done;
        forever begin
            @(posedge clk);
            // AR handshake
            if (S_AXI_ARVALID[slv_id] && S_AXI_ARREADY[slv_id]) begin
                arid    = S_AXI_ARID[W_SID*(slv_id+1)-1 -: W_SID];
                araddr  = S_AXI_ARADDR[ADDR_WIDTH*(slv_id+1)-1 -: ADDR_WIDTH];
                arlen   = S_AXI_ARLEN[LEN_W*(slv_id+1)-1 -: LEN_W];
                arsize  = S_AXI_ARSIZE[SIZE_W*(slv_id+1)-1 -: SIZE_W];
                arburst = S_AXI_ARBURST[BURST_W*(slv_id+1)-1 -: BURST_W];
                ar_done = 1'b1;
                beat_cnt = 0;
                total_beats = arlen + 8'd1;
                word_idx = araddr[SLV_ADDR_SPAN-1:2];
                // Drive first beat
                S_AXI_RID[W_SID*(slv_id+1)-1 -: W_SID] <= arid;
                if (word_idx < SLV_MEM_DEPTH)
                    S_AXI_RDATA[DATA_WIDTH*(slv_id+1)-1 -: DATA_WIDTH] <= slv_mem[slv_id][word_idx];
                else
                    S_AXI_RDATA[DATA_WIDTH*(slv_id+1)-1 -: DATA_WIDTH] <= 32'hDEAD_BEEF;
                S_AXI_RRESP[RESP_W*(slv_id+1)-1 -: RESP_W] <= 2'b00;
                S_AXI_RLAST[slv_id] <= (total_beats == 8'd1);
                S_AXI_RVALID[slv_id] <= 1'b1;
            end

            // R handshake: advance to next beat
            if (ar_done && S_AXI_RVALID[slv_id] && S_AXI_RREADY[slv_id]) begin

                if (beat_cnt == total_beats - 1) begin
                    S_AXI_RVALID[slv_id] <= 1'b0;
                    S_AXI_RLAST[slv_id]  <= 1'b0;
                    ar_done = 1'b0;
                end
                else begin
                    beat_cnt = beat_cnt + 1;

                    word_idx = araddr[SLV_ADDR_SPAN-1:2] + beat_cnt;

                    if (word_idx < SLV_MEM_DEPTH)
                        S_AXI_RDATA[DATA_WIDTH*(slv_id+1)-1 -: DATA_WIDTH]
                            <= slv_mem[slv_id][word_idx];
                    else
                        S_AXI_RDATA[DATA_WIDTH*(slv_id+1)-1 -: DATA_WIDTH]
                            <= 32'hDEAD_BEEF;

                    S_AXI_RLAST[slv_id] <= (beat_cnt + 1 == total_beats - 1);
                end
            end
        end
    endtask

    //=========================================================================
    // Master BFM: Write
    //=========================================================================
    // Drive a write transaction from master 'mst_id'
    // data[] must have (len+1) entries
    task automatic axi_write(
        input int               mst_id,
        input [ADDR_WIDTH-1:0]  addr,
        input [LEN_W-1:0]       len,
        input [2:0]             size,
        input [1:0]             burst,
        input [W_ID-1:0]        id,
        input axi_data_buf_t    data
    );
        reg [LEN_W:0] total_beats;
        reg [LEN_W:0] b;

        begin
            total_beats = len + 1'b1;

            $display("[%0t] MASTER[%0d] WRITE: addr=0x%08h, len=%0d, size=%0d, id=%0d",
                    $time, mst_id, addr, len, size, id);

            // Drive AW
            M_AXI_AWID[W_ID*(mst_id+1)-1 -: W_ID] <= id;
            M_AXI_AWADDR[ADDR_WIDTH*(mst_id+1)-1 -: ADDR_WIDTH] <= addr;
            M_AXI_AWLEN[LEN_W*(mst_id+1)-1 -: LEN_W] <= len;
            M_AXI_AWSIZE[SIZE_W*(mst_id+1)-1 -: SIZE_W] <= size;
            M_AXI_AWBURST[BURST_W*(mst_id+1)-1 -: BURST_W] <= burst;
            M_AXI_AWVALID[mst_id] <= 1'b1;

            // Wait AW handshake
            @(posedge clk);

            while (!M_AXI_AWREADY[mst_id])
                @(posedge clk);

            M_AXI_AWVALID[mst_id] <= 1'b0;

            // Drive W channel
            for (b = 0; b < total_beats; b = b + 1) begin

                M_AXI_WDATA[DATA_WIDTH*(mst_id+1)-1 -: DATA_WIDTH] <= data[b];
                M_AXI_WSTRB[W_STRB*(mst_id+1)-1 -: W_STRB] <= {W_STRB{1'b1}};
                M_AXI_WLAST[mst_id] <= (b == total_beats - 1);
                M_AXI_WVALID[mst_id] <= 1'b1;

                @(posedge clk);

                while (!M_AXI_WREADY[mst_id])
                    @(posedge clk);
            end

            M_AXI_WVALID[mst_id] <= 1'b0;
            M_AXI_WLAST[mst_id]  <= 1'b0;

            // Wait B response
            M_AXI_BREADY[mst_id] <= 1'b1;

            @(posedge clk);

            while (!M_AXI_BVALID[mst_id])
                @(posedge clk);

            $display("[%0t] MASTER[%0d] WRITE BRESP: id=0x%0h, resp=%0d",
                    $time,
                    mst_id,
                    M_AXI_BID[W_ID*(mst_id+1)-1 -: W_ID],
                    M_AXI_BRESP[RESP_W*(mst_id+1)-1 -: RESP_W]);

            M_AXI_BREADY[mst_id] <= 1'b0;
        end
    endtask

    //=========================================================================
    // Master BFM: Read
    //=========================================================================
    task automatic axi_read(
        input int               mst_id,
        input [ADDR_WIDTH-1:0]  addr,
        input [LEN_W-1:0]       len,
        input [2:0]             size,
        input [1:0]             burst,
        input [W_ID-1:0]        id,
        output axi_data_buf_t   rdata
    );
        reg [LEN_W:0] total_beats;
        reg [LEN_W:0] b;
        begin
            total_beats = len + 1'b1;

            $display("[%0t] MASTER[%0d] READ: addr=0x%08h, len=%0d, size=%0d, id=%0d",
                    $time, mst_id, addr, len, size, id);

            // Drive AR
            M_AXI_ARID[W_ID*(mst_id+1)-1 -: W_ID] <= id;
            M_AXI_ARADDR[ADDR_WIDTH*(mst_id+1)-1 -: ADDR_WIDTH] <= addr;
            M_AXI_ARLEN[LEN_W*(mst_id+1)-1 -: LEN_W] <= len;
            M_AXI_ARSIZE[SIZE_W*(mst_id+1)-1 -: SIZE_W] <= size;
            M_AXI_ARBURST[BURST_W*(mst_id+1)-1 -: BURST_W] <= burst;
            M_AXI_ARVALID[mst_id] <= 1'b1;

            // Wait AR handshake
            @(posedge clk);
            while (!M_AXI_ARREADY[mst_id])
                @(posedge clk);

            M_AXI_ARVALID[mst_id] <= 1'b0;

            // Collect R data
            M_AXI_RREADY[mst_id] <= 1'b1;

            for (b = 0; b < total_beats; b = b + 1) begin
                @(posedge clk);

                while (!M_AXI_RVALID[mst_id])
                    @(posedge clk);

                rdata[b] = M_AXI_RDATA[DATA_WIDTH*(mst_id+1)-1 -: DATA_WIDTH];

                $display("[%0t] MASTER[%0d] READ beat[%0d]: data=0x%08h, resp=%0d, last=%0d",
                        $time,
                        mst_id,
                        b,
                        M_AXI_RDATA[DATA_WIDTH*(mst_id+1)-1 -: DATA_WIDTH],
                        M_AXI_RRESP[RESP_W*(mst_id+1)-1 -: RESP_W],
                        M_AXI_RLAST[mst_id]);
            end

            M_AXI_RREADY[mst_id] <= 1'b0;
        end
    endtask

    //=========================================================================
    // Scoreboard / Reference Model
    //=========================================================================
    // Map: {slv_id, word_idx[9:0]} -> expected 32-bit data
    reg [DATA_WIDTH-1:0] ref_mem [0:SLV_AMT-1][0:SLV_MEM_DEPTH-1];

    // Copy initial state
    initial begin
        for (init_s = 0; init_s < SLV_AMT; init_s = init_s + 1) begin
            for (init_i = 0; init_i < SLV_MEM_DEPTH; init_i = init_i + 1) begin
                ref_mem[init_s][init_i] = slv_mem[init_s][init_i];
            end
        end
    end

    //=========================================================================
    // Slave Address Mapping
    // Each slave has 8KB address space (SLV_ADDR_LEN = 13)
    // Slave 0: 0x00000000, Slave 1: 0x00002000, Slave 2: 0x00004000, ...
    //=========================================================================
    function automatic [1:0] get_slv_id;
        input [ADDR_WIDTH-1:0] addr;
        begin
            get_slv_id = addr[14:13];
        end
    endfunction

    function automatic [ADDR_WIDTH-1:0] slv_base_addr;
        input [1:0] slv_id;
        begin
            slv_base_addr = slv_id * 32'h2000;
        end
    endfunction

    //=========================================================================
    // Test Data Generation
    //=========================================================================
    function automatic [DATA_WIDTH-1:0] gen_test_data;
        input [ADDR_WIDTH-1:0] addr;
        input [7:0] beat;
        begin
            gen_test_data = {addr[15:0], beat[7:0], addr[7:0]};
        end
    endfunction

    //=========================================================================
    // Default Tie-offs for Unused Master Ports
    //=========================================================================
    initial begin
        M_AXI_AWVALID = {MST_AMT{1'b0}};
        M_AXI_WVALID  = {MST_AMT{1'b0}};
        M_AXI_WLAST   = {MST_AMT{1'b0}};
        M_AXI_BREADY  = {MST_AMT{1'b0}};
        M_AXI_ARVALID = {MST_AMT{1'b0}};
        M_AXI_RREADY  = {MST_AMT{1'b0}};
    end

    // Default slave ready = always ready (set in initial block per slave)
    initial begin
        S_AXI_AWREADY = {SLV_AMT{1'b1}};
        S_AXI_WREADY  = {SLV_AMT{1'b1}};
        S_AXI_ARREADY = {SLV_AMT{1'b1}};
        S_AXI_BID     = {W_SID*SLV_AMT{1'b0}};
        S_AXI_BRESP   = {RESP_W*SLV_AMT{1'b0}};
        S_AXI_BVALID  = {SLV_AMT{1'b0}};
        S_AXI_RID     = {W_SID*SLV_AMT{1'b0}};
        S_AXI_RDATA   = {DATA_WIDTH*SLV_AMT{1'b0}};
        S_AXI_RRESP   = {RESP_W*SLV_AMT{1'b0}};
        S_AXI_RLAST   = {SLV_AMT{1'b0}};
        S_AXI_RVALID  = {SLV_AMT{1'b0}};
    end

    // Default control signals
    initial begin
        arbiter_type  = 1'b0;     // Round-Robin
    end

    //=========================================================================
    // Fork slave handlers
    //=========================================================================
    integer slv_idx;
    initial begin
        for (slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin
            automatic int sid = slv_idx;
            fork
                slv_write_handler(sid);
                slv_read_handler(sid);
            join_none
        end
    end

    //=========================================================================
    // Main Test Sequence
    //=========================================================================
    axi_data_buf_t wdata_arr;
    axi_data_buf_t rdata_arr;
    reg [DATA_WIDTH-1:0] expected;
    integer              i, b;
    integer              err_cnt;
    reg [ADDR_WIDTH-1:0] test_addr;
    reg [ADDR_WIDTH-1:0] test_slv_base;

    // Per-thread variables for concurrent fork tests
    reg [ADDR_WIDTH-1:0] t2_addr0, t2_addr1, t2_addr2, t2_addr3;
    axi_data_buf_t t2_d0;
    axi_data_buf_t t2_d1;
    axi_data_buf_t t2_d2;
    axi_data_buf_t t2_d3;
    reg [ADDR_WIDTH-1:0] t6_wr_addr, t6_rd_addr;
    axi_data_buf_t t6_wdata;
    axi_data_buf_t t6_rdata;

    initial begin
        err_cnt = 0;

        // Wait for reset
        @(posedge rst_n);
        repeat (5) @(posedge clk);

        $display("============================================================");
        $display("[%0t] TESTBENCH STARTED", $time);
        $display("============================================================");

        //=================================================================
        // TEST 1: Single-beat write + read to each slave from master 0
        //=================================================================
        $display("\n--- TEST 1: Single-beat WR/RD to each slave ---");
        for (i = 0; i < SLV_AMT; i = i + 1) begin
            test_slv_base = slv_base_addr(i[1:0]);
            test_addr = test_slv_base + 32'h40;  // offset 0x40 within slave

            // Prepare write data
            for (b = 0; b < 4; b = b + 1)
                wdata_arr[b] = gen_test_data(test_addr, b[7:0]);

            // Write 4 beats
            axi_write(0, test_addr, 8'd3, 3'd2, 2'd1, 4'hA, wdata_arr);

            // Read back 4 beats
            axi_read(0, test_addr, 8'd3, 3'd2, 2'd1, 4'hA, rdata_arr);

            // Check
            for (b = 0; b < 4; b = b + 1) begin
                expected = gen_test_data(test_addr, b[7:0]);
                if (rdata_arr[b] !== expected) begin
                    $display("[%0t] ERROR: Slave%0d read mismatch! addr=0x%08h beat=%0d got=0x%08h exp=0x%08h",
                             $time, i, test_addr + (b*4), b, rdata_arr[b], expected);
                    err_cnt = err_cnt + 1;
                end
            end
        end
        $display("[%0t] TEST 1 COMPLETE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 2: Different masters write to different slaves
        //=================================================================
        $display("\n--- TEST 2: Multi-master concurrent writes ---");
        t2_addr0 = slv_base_addr(2'd0) + 32'h100;
        t2_addr1 = slv_base_addr(2'd1) + 32'h200;
        t2_addr2 = slv_base_addr(2'd2) + 32'h300;
        t2_addr3 = slv_base_addr(2'd3) + 32'h400;
        t2_d0[0] = 32'hAAAA_0000; t2_d0[1] = 32'hAAAA_0001;
        t2_d1[0] = 32'hBBBB_0000; t2_d1[1] = 32'hBBBB_0001;
        t2_d2[0] = 32'hCCCC_0000; t2_d2[1] = 32'hCCCC_0001;
        t2_d3[0] = 32'hDDDD_0000; t2_d3[1] = 32'hDDDD_0001;
        fork
            axi_write(0, t2_addr0, 8'd1, 3'd2, 2'd1, 4'h0, t2_d0);
            axi_write(1, t2_addr1, 8'd1, 3'd2, 2'd1, 4'h1, t2_d1);
            axi_write(2, t2_addr2, 8'd1, 3'd2, 2'd1, 4'h2, t2_d2);
            axi_write(3, t2_addr3, 8'd1, 3'd2, 2'd1, 4'h3, t2_d3);
        join

        // Read back from each slave using each master
        for (i = 0; i < 4; i = i + 1) begin
            test_addr = slv_base_addr(i[1:0]) + 32'h100 + (i * 32'h100);
            axi_read(i, test_addr, 8'd1, 3'd2, 2'd1, i[3:0], rdata_arr);
            // Simple check: data should be non-X
            for (b = 0; b < 2; b = b + 1) begin
                if (rdata_arr[b] === 32'hxxxx_xxxx) begin
                    $display("[%0t] ERROR: Master%0d read X data!", $time, i);
                    err_cnt = err_cnt + 1;
                end
            end
        end
        $display("[%0t] TEST 2 COMPLETE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 3: 4KB boundary crossing transaction
        //=================================================================
        $display("\n--- TEST 3: 4KB boundary crossing ---");
        begin
            // Address near end of 4KB page: 0x00000FF0
            // 4 beats x 4 bytes = 16 bytes → crosses to 0x00001000
            test_addr = 32'h0000_0FF0;
            for (b = 0; b < 8; b = b + 1)
                wdata_arr[b] = 32'hF00D_0000 + b;

            axi_write(0, test_addr, 8'd7, 3'd2, 2'd1, 4'h5, wdata_arr);

            // Read back
            axi_read(0, test_addr, 8'd7, 3'd2, 2'd1, 4'h5, rdata_arr);

            for (b = 0; b < 8; b = b + 1) begin
                expected = 32'hF00D_0000 + b;
                if (rdata_arr[b] !== expected) begin
                    $display("[%0t] ERROR: 4KB cross read mismatch! beat=%0d got=0x%08h exp=0x%08h",
                             $time, b, rdata_arr[b], expected);
                    err_cnt = err_cnt + 1;
                end
            end
        end
        $display("[%0t] TEST 3 COMPLETE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 4: Back-to-back transactions from same master
        //=================================================================
        $display("\n--- TEST 4: Back-to-back transactions ---");
        begin
            // Write 1
            test_addr = slv_base_addr(2'd0) + 32'h500;
            for (b = 0; b < 1; b = b + 1)
                wdata_arr[b] = 32'hB2B0_0001;
            axi_write(0, test_addr, 8'd0, 3'd2, 2'd1, 4'h0, wdata_arr);

            // Write 2 (back-to-back)
            test_addr = slv_base_addr(2'd0) + 32'h504;
            for (b = 0; b < 1; b = b + 1)
                wdata_arr[b] = 32'hB2B0_0002;
            axi_write(0, test_addr, 8'd0, 3'd2, 2'd1, 4'h0, wdata_arr);

            // Read 1
            test_addr = slv_base_addr(2'd0) + 32'h500;
            axi_read(0, test_addr, 8'd0, 3'd2, 2'd1, 4'h0, rdata_arr);
            if (rdata_arr[0] !== 32'hB2B0_0001) begin
                $display("[%0t] ERROR: B2B test - first write data mismatch! got=0x%08h",
                         $time, rdata_arr[0]);
                err_cnt = err_cnt + 1;
            end

            // Read 2
            test_addr = slv_base_addr(2'd0) + 32'h504;
            axi_read(0, test_addr, 8'd0, 3'd2, 2'd1, 4'h0, rdata_arr);
            if (rdata_arr[0] !== 32'hB2B0_0002) begin
                $display("[%0t] ERROR: B2B test - second write data mismatch! got=0x%08h",
                         $time, rdata_arr[0]);
                err_cnt = err_cnt + 1;
            end
        end
        $display("[%0t] TEST 4 COMPLETE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 5: Long burst (16 beats)
        //=================================================================
        $display("\n--- TEST 5: Long burst (16 beats) ---");
        begin
            test_addr = slv_base_addr(2'd1) + 32'h800;
            for (b = 0; b < 16; b = b + 1)
                wdata_arr[b] = 32'h1000_0000 + b;
            axi_write(1, test_addr, 8'd15, 3'd2, 2'd1, 4'h7, wdata_arr);
            axi_read(1, test_addr, 8'd15, 3'd2, 2'd1, 4'h7, rdata_arr);
            for (b = 0; b < 16; b = b + 1) begin
                expected = 32'h1000_0000 + b;
                if (rdata_arr[b] !== expected) begin
                    $display("[%0t] ERROR: Long burst mismatch! beat=%0d got=0x%08h exp=0x%08h",
                             $time, b, rdata_arr[b], expected);
                    err_cnt = err_cnt + 1;
                end
            end
        end
        $display("[%0t] TEST 5 COMPLETE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 6: Mixed read/write concurrent from different masters
        //=================================================================
        $display("\n--- TEST 6: Mixed concurrent read/write ---");
        t6_wr_addr = slv_base_addr(2'd0) + 32'h600;
        t6_rd_addr = slv_base_addr(2'd2) + 32'h700;
        t6_wdata[0] = 32'hCAFE_0000; t6_wdata[1] = 32'hCAFE_0001;
        t6_wdata[2] = 32'hCAFE_0002; t6_wdata[3] = 32'hCAFE_0003;
        fork
            axi_write(0, t6_wr_addr, 8'd3, 3'd2, 2'd1, 4'hA, t6_wdata);
            axi_read(2, t6_rd_addr, 8'd3, 3'd2, 2'd1, 4'hB, t6_rdata);
        join
        $display("[%0t] TEST 6 COMPLETE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 7: Different burst sizes (8-bit, 16-bit, 32-bit)
        //=================================================================
        $display("\n--- TEST 7: Different transfer sizes ---");
        begin
            // 8-bit writes (SIZE=0)
            test_addr = slv_base_addr(2'd0) + 32'hA00;
            for (b = 0; b < 4; b = b + 1)
                wdata_arr[b] = {4{8'hA0 + b[7:0]}};
            axi_write(0, test_addr, 8'd3, 3'd0, 2'd1, 4'hC, wdata_arr);
            axi_read(0, test_addr, 8'd3, 3'd0, 2'd1, 4'hC, rdata_arr);

            // 16-bit writes (SIZE=1)
            test_addr = slv_base_addr(2'd0) + 32'hB00;
            for (b = 0; b < 4; b = b + 1)
                wdata_arr[b] = {2{16'hB0B0}};
            axi_write(0, test_addr, 8'd3, 3'd1, 2'd1, 4'hD, wdata_arr);
            axi_read(0, test_addr, 8'd3, 3'd1, 2'd1, 4'hD, rdata_arr);
        end
        $display("[%0t] TEST 7 COMPLETE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // FINAL REPORT
        //=================================================================
        $display("\n============================================================");
        if (err_cnt == 0) begin
            $display("[%0t] ALL TESTS PASSED!", $time);
        end else begin
            $display("[%0t] TESTS FAILED with %0d errors!", $time, err_cnt);
        end
        $display("============================================================\n");

        #500;
        $finish;
    end

    //=========================================================================
    // Timeout watchdog
    //=========================================================================
    initial begin
        #5000000;  // 5ms timeout
        $display("[%0t] ERROR: Simulation timeout!", $time);
        $finish;
    end

    //=========================================================================
    // Waveform dump (FSDB for Verdi)
    //=========================================================================
    initial begin
        $fsdbDumpfile("axi_interconnect_tb.fsdb");
        $fsdbDumpvars(0, axi_interconnect_tb);
    end

endmodule
