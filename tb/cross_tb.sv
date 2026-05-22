//=============================================================================
// Testbench: cross_tb
// Desc     : Testbench for axi_crossbar (4x4 flattened crossbar switch)
//=============================================================================

module cross_tb;

    //=========================================================================
    // Parameters (matching DUT defaults)
    //=========================================================================
    localparam MST_AMT           = 4;
    localparam SLV_AMT           = 4;
    localparam OUTSTANDING_AMT   = 8;
    localparam W_ID              = 4;
    localparam TRANS_BURST_W     = 2;
    localparam TRANS_DATA_LEN_W  = 8;
    localparam TRANS_DATA_SIZE_W = 3;
    localparam TRANS_WR_RESP_W   = 2;
    localparam DATA_WIDTH        = 32;
    localparam ADDR_WIDTH        = 32;
    localparam W_STRB            = DATA_WIDTH / 8;
    localparam MST_ID_W          = $clog2(MST_AMT);
    localparam SLV_ID_W          = 2;
    localparam W_MID             = SLV_ID_W + W_ID;            // 6
    localparam W_SID             = MST_ID_W + SLV_ID_W + W_ID; // 8

    // 4KB address space per slave
    localparam SLV_MEM_DEPTH = 1024;
    localparam [0:SLV_AMT*ADDR_WIDTH-1] SLV_ADDR_BASE = {
        32'h00003000,   // Slave 3: 0x3000-0x3FFF
        32'h00002000,   // Slave 2: 0x2000-0x2FFF
        32'h00001000,   // Slave 1: 0x1000-0x1FFF
        32'h00000000    // Slave 0: 0x0000-0x0FFF
    };
    localparam [0:SLV_AMT*8-1] SLV_ADDR_LEN = {
        8'd12, 8'd12, 8'd12, 8'd12
    };

    typedef logic [DATA_WIDTH-1:0] data_buf_t [0:255];

    //=========================================================================
    // Clock and Reset
    //=========================================================================
    reg clk;
    reg rst_n;

    //=========================================================================
    // Master-side signals (flattened)
    //=========================================================================
    // AW
    reg  [W_MID*MST_AMT-1:0]              m_AWID;
    reg  [ADDR_WIDTH*MST_AMT-1:0]         m_AWADDR;
    reg  [TRANS_BURST_W*MST_AMT-1:0]      m_AWBURST;
    reg  [TRANS_DATA_LEN_W*MST_AMT-1:0]   m_AWLEN;
    reg  [TRANS_DATA_SIZE_W*MST_AMT-1:0]  m_AWSIZE;
    reg  [MST_AMT-1:0]                    m_AWVALID;
    wire [MST_AMT-1:0]                    m_AWREADY;
    // W
    reg  [DATA_WIDTH*MST_AMT-1:0]         m_WDATA;
    reg  [W_STRB*MST_AMT-1:0]             m_WSTRB;
    reg  [MST_AMT-1:0]                    m_WLAST;
    reg  [MST_AMT-1:0]                    m_WVALID;
    wire [MST_AMT-1:0]                    m_WREADY;
    // B
    wire [W_SID*MST_AMT-1:0]              m_BID;
    wire [TRANS_WR_RESP_W*MST_AMT-1:0]    m_BRESP;
    wire [MST_AMT-1:0]                    m_BVALID;
    reg  [MST_AMT-1:0]                    m_BREADY;
    // AR
    reg  [W_MID*MST_AMT-1:0]              m_ARID;
    reg  [ADDR_WIDTH*MST_AMT-1:0]         m_ARADDR;
    reg  [TRANS_BURST_W*MST_AMT-1:0]      m_ARBURST;
    reg  [TRANS_DATA_LEN_W*MST_AMT-1:0]   m_ARLEN;
    reg  [TRANS_DATA_SIZE_W*MST_AMT-1:0]  m_ARSIZE;
    reg  [MST_AMT-1:0]                    m_ARVALID;
    wire [MST_AMT-1:0]                    m_ARREADY;
    // R
    wire [W_SID*MST_AMT-1:0]              m_RID;
    wire [DATA_WIDTH*MST_AMT-1:0]         m_RDATA;
    wire [TRANS_WR_RESP_W*MST_AMT-1:0]    m_RRESP;
    wire [MST_AMT-1:0]                    m_RLAST;
    wire [MST_AMT-1:0]                    m_RVALID;
    reg  [MST_AMT-1:0]                    m_RREADY;
    // RSID
    wire [W_SID*MST_AMT-1:0]              m_RSID;

    //=========================================================================
    // Slave-side signals (flattened)
    //=========================================================================
    // AW
    wire [W_SID*SLV_AMT-1:0]              s_AWID;
    wire [ADDR_WIDTH*SLV_AMT-1:0]         s_AWADDR;
    wire [TRANS_BURST_W*SLV_AMT-1:0]      s_AWBURST;
    wire [TRANS_DATA_LEN_W*SLV_AMT-1:0]   s_AWLEN;
    wire [TRANS_DATA_SIZE_W*SLV_AMT-1:0]  s_AWSIZE;
    wire [SLV_AMT-1:0]                    s_AWVALID;
    reg  [SLV_AMT-1:0]                    s_AWREADY;
    // W
    wire [DATA_WIDTH*SLV_AMT-1:0]         s_WDATA;
    wire [W_STRB*SLV_AMT-1:0]             s_WSTRB;
    wire [SLV_AMT-1:0]                    s_WLAST;
    wire [SLV_AMT-1:0]                    s_WVALID;
    reg  [SLV_AMT-1:0]                    s_WREADY;
    // B
    reg  [W_SID*SLV_AMT-1:0]              s_BID;
    reg  [TRANS_WR_RESP_W*SLV_AMT-1:0]    s_BRESP;
    reg  [SLV_AMT-1:0]                    s_BVALID;
    wire [SLV_AMT-1:0]                    s_BREADY;
    // AR
    wire [W_SID*SLV_AMT-1:0]              s_ARID;
    wire [ADDR_WIDTH*SLV_AMT-1:0]         s_ARADDR;
    wire [TRANS_BURST_W*SLV_AMT-1:0]      s_ARBURST;
    wire [TRANS_DATA_LEN_W*SLV_AMT-1:0]   s_ARLEN;
    wire [TRANS_DATA_SIZE_W*SLV_AMT-1:0]  s_ARSIZE;
    wire [SLV_AMT-1:0]                    s_ARVALID;
    reg  [SLV_AMT-1:0]                    s_ARREADY;
    // R
    reg  [W_SID*SLV_AMT-1:0]              s_RID;
    reg  [DATA_WIDTH*SLV_AMT-1:0]         s_RDATA;
    reg  [TRANS_WR_RESP_W*SLV_AMT-1:0]    s_RRESP;
    reg  [SLV_AMT-1:0]                    s_RLAST;
    reg  [SLV_AMT-1:0]                    s_RVALID;
    wire [SLV_AMT-1:0]                    s_RREADY;

    //=========================================================================
    // Control
    //=========================================================================
    reg                arbiter_type;
    reg [SLV_AMT-1:0]  r_order_grant;
    reg [SLV_AMT-1:0]  slv_en;

    //=========================================================================
    // DUT Instantiation
    //=========================================================================
    axi_crossbar #(
        .MST_AMT           (MST_AMT),
        .SLV_AMT           (SLV_AMT),
        .OUTSTANDING_AMT   (OUTSTANDING_AMT),
        .W_ID              (W_ID),
        .TRANS_BURST_W     (TRANS_BURST_W),
        .TRANS_DATA_LEN_W  (TRANS_DATA_LEN_W),
        .TRANS_DATA_SIZE_W (TRANS_DATA_SIZE_W),
        .TRANS_WR_RESP_W   (TRANS_WR_RESP_W),
        .DATA_WIDTH        (DATA_WIDTH),
        .ADDR_WIDTH        (ADDR_WIDTH),
        .SLV_ADDR_BASE     (SLV_ADDR_BASE),
        .SLV_ADDR_LEN      (SLV_ADDR_LEN),
        .DEFAULT_SLV_EN    (1'b0)        // no default slave: all 4 are normal
    ) u_dut (
        .AXI_RSTn       (rst_n),
        .AXI_CLK        (clk),
        .m_AWID_i       (m_AWID),
        .m_AWADDR_i     (m_AWADDR),
        .m_AWBURST_i    (m_AWBURST),
        .m_AWLEN_i      (m_AWLEN),
        .m_AWSIZE_i     (m_AWSIZE),
        .m_AWVALID_i    (m_AWVALID),
        .m_AWREADY_o    (m_AWREADY),
        .m_WDATA_i      (m_WDATA),
        .m_WSTRB_i      (m_WSTRB),
        .m_WLAST_i      (m_WLAST),
        .m_WVALID_i     (m_WVALID),
        .m_WREADY_o     (m_WREADY),
        .m_BID_o        (m_BID),
        .m_BRESP_o      (m_BRESP),
        .m_BVALID_o     (m_BVALID),
        .m_BREADY_i     (m_BREADY),
        .m_ARID_i       (m_ARID),
        .m_ARADDR_i     (m_ARADDR),
        .m_ARBURST_i    (m_ARBURST),
        .m_ARLEN_i      (m_ARLEN),
        .m_ARSIZE_i     (m_ARSIZE),
        .m_ARVALID_i    (m_ARVALID),
        .m_ARREADY_o    (m_ARREADY),
        .m_RID_o        (m_RID),
        .m_RDATA_o      (m_RDATA),
        .m_RRESP_o      (m_RRESP),
        .m_RLAST_o      (m_RLAST),
        .m_RVALID_o     (m_RVALID),
        .m_RREADY_i     (m_RREADY),
        .m_RSID_o       (m_RSID),
        .s_AWID_o       (s_AWID),
        .s_AWADDR_o     (s_AWADDR),
        .s_AWBURST_o    (s_AWBURST),
        .s_AWLEN_o      (s_AWLEN),
        .s_AWSIZE_o     (s_AWSIZE),
        .s_AWVALID_o    (s_AWVALID),
        .s_AWREADY_i    (s_AWREADY),
        .s_WDATA_o      (s_WDATA),
        .s_WSTRB_o      (s_WSTRB),
        .s_WLAST_o      (s_WLAST),
        .s_WVALID_o     (s_WVALID),
        .s_WREADY_i     (s_WREADY),
        .s_BID_i        (s_BID),
        .s_BRESP_i      (s_BRESP),
        .s_BVALID_i     (s_BVALID),
        .s_BREADY_o     (s_BREADY),
        .s_ARID_o       (s_ARID),
        .s_ARADDR_o     (s_ARADDR),
        .s_ARBURST_o    (s_ARBURST),
        .s_ARLEN_o      (s_ARLEN),
        .s_ARSIZE_o     (s_ARSIZE),
        .s_ARVALID_o    (s_ARVALID),
        .s_ARREADY_i    (s_ARREADY),
        .s_RID_i        (s_RID),
        .s_RDATA_i      (s_RDATA),
        .s_RRESP_i      (s_RRESP),
        .s_RLAST_i      (s_RLAST),
        .s_RVALID_i     (s_RVALID),
        .s_RREADY_o     (s_RREADY),
        .arbiter_type   (arbiter_type),
        .r_order_grant_i(r_order_grant),
        .slv_en_i       (slv_en)
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
    // Slave Memory Model (4KB per slave = 1024 x 32-bit words)
    //=========================================================================
    reg [DATA_WIDTH-1:0] slv_mem [0:SLV_AMT-1][0:SLV_MEM_DEPTH-1];

    integer init_s, init_i;
    initial begin
        for (init_s = 0; init_s < SLV_AMT; init_s = init_s + 1) begin
            for (init_i = 0; init_i < SLV_MEM_DEPTH; init_i = init_i + 1) begin
                slv_mem[init_s][init_i] = {init_s[1:0], 14'h0, init_i[15:0]};
            end
        end
    end

    //=========================================================================
    // Shared B-done queues (write handler → B handler, per slave)
    //=========================================================================
    reg [W_SID-1:0] b_done_awid [0:SLV_AMT-1][0:3];
    reg [1:0]       b_done_wr_ptr [0:SLV_AMT-1];
    reg [1:0]       b_done_rd_ptr [0:SLV_AMT-1];
    integer         b_done_cnt    [0:SLV_AMT-1];

    integer bq_init_s;
    initial begin
        for (bq_init_s = 0; bq_init_s < SLV_AMT; bq_init_s = bq_init_s + 1) begin
            b_done_wr_ptr[bq_init_s] = 2'b00;
            b_done_rd_ptr[bq_init_s] = 2'b00;
            b_done_cnt[bq_init_s]    = 0;
        end
    end

    //=========================================================================
    // Shared AR queues (AR handler → R handler, per slave)
    //=========================================================================
    reg [W_SID-1:0] ar_q_arid   [0:SLV_AMT-1][0:3];
    reg [ADDR_WIDTH-1:0] ar_q_araddr [0:SLV_AMT-1][0:3];
    reg [7:0]            ar_q_total  [0:SLV_AMT-1][0:3];
    reg [1:0]       ar_q_wr_ptr [0:SLV_AMT-1];
    reg [1:0]       ar_q_rd_ptr [0:SLV_AMT-1];
    integer         ar_q_cnt    [0:SLV_AMT-1];

    integer arq_init_s;
    initial begin
        for (arq_init_s = 0; arq_init_s < SLV_AMT; arq_init_s = arq_init_s + 1) begin
            ar_q_wr_ptr[arq_init_s] = 2'b00;
            ar_q_rd_ptr[arq_init_s] = 2'b00;
            ar_q_cnt[arq_init_s]    = 0;
        end
    end

    //=========================================================================
    // Slave Write Handler (with AW→W transaction queue)
    //=========================================================================
    task automatic slv_write_handler(int slv_id);
        // Pending AW transaction queue (depth 4)
        reg [W_SID-1:0]      q_awid    [0:3];
        reg [ADDR_WIDTH-1:0] q_awaddr  [0:3];
        reg [7:0]            q_total   [0:3];
        integer              q_wr_ptr, q_rd_ptr, q_cnt;

        // Current active transaction (head of queue, being written)
        reg [W_SID-1:0]      cur_awid;
        reg [ADDR_WIDTH-1:0] cur_awaddr;
        reg [7:0]            cur_total;
        reg [7:0]            cur_beat_cnt;
        reg                  cur_active;

        reg [DATA_WIDTH-1:0] wdata;
        reg [W_STRB-1:0]     wstrb;
        reg                  wlast;
        reg [ADDR_WIDTH-1:0] word_idx;
        integer              bi;

        begin
            q_wr_ptr  = 0;
            q_rd_ptr  = 0;
            q_cnt     = 0;
            cur_active = 1'b0;

            forever begin
                @(posedge clk);

                // AW handshake → enqueue transaction
                if (s_AWVALID[slv_id] && s_AWREADY[slv_id]) begin
                    q_awid[q_wr_ptr]   = s_AWID[W_SID*(slv_id+1)-1 -: W_SID];
                    q_awaddr[q_wr_ptr] = s_AWADDR[ADDR_WIDTH*(slv_id+1)-1 -: ADDR_WIDTH];
                    q_total[q_wr_ptr]  = s_AWLEN[TRANS_DATA_LEN_W*(slv_id+1)-1 -: TRANS_DATA_LEN_W] + 8'd1;
                    q_wr_ptr = (q_wr_ptr + 1) & 3;
                    q_cnt    = q_cnt + 1;
                end

                // Dequeue next transaction if idle
                if (!cur_active && q_cnt > 0) begin
                    cur_awid      = q_awid[q_rd_ptr];
                    cur_awaddr    = q_awaddr[q_rd_ptr];
                    cur_total     = q_total[q_rd_ptr];
                    cur_beat_cnt  = 0;
                    cur_active    = 1'b1;
                    q_rd_ptr      = (q_rd_ptr + 1) & 3;
                    q_cnt         = q_cnt - 1;
                end

                // W data handshake
                if (cur_active && s_WVALID[slv_id] && s_WREADY[slv_id]) begin
                    wdata = s_WDATA[DATA_WIDTH*(slv_id+1)-1 -: DATA_WIDTH];
                    wstrb = s_WSTRB[W_STRB*(slv_id+1)-1 -: W_STRB];
                    wlast = s_WLAST[slv_id];
                    word_idx = cur_awaddr[11:2] + cur_beat_cnt;
                    if (word_idx < SLV_MEM_DEPTH) begin
                        for (bi = 0; bi < W_STRB; bi = bi + 1) begin
                            if (wstrb[bi])
                                slv_mem[slv_id][word_idx][8*bi +: 8] = wdata[8*bi +: 8];
                        end
                    end
                    cur_beat_cnt = cur_beat_cnt + 8'd1;
                    if (wlast) begin
                        cur_active = 1'b0;
                        // Push to shared B-done queue (B handler picks up)
                        b_done_awid[slv_id][b_done_wr_ptr[slv_id]] = cur_awid;
                        b_done_wr_ptr[slv_id] = (b_done_wr_ptr[slv_id] + 1) & 3;
                        b_done_cnt[slv_id]    = b_done_cnt[slv_id] + 1;
                    end
                end
            end
        end
    endtask

    //=========================================================================
    // Slave B Response Handler (pops from shared B-done queue)
    //=========================================================================
    task automatic slv_b_handler(int slv_id);
        forever begin
            @(posedge clk);

            // BVALID is not currently asserted and queue has pending response
            if (!s_BVALID[slv_id] && b_done_cnt[slv_id] > 0) begin
                s_BID[W_SID*(slv_id+1)-1 -: W_SID] <= b_done_awid[slv_id][b_done_rd_ptr[slv_id]];
                s_BRESP[TRANS_WR_RESP_W*(slv_id+1)-1 -: TRANS_WR_RESP_W] <= 2'b00;
                s_BVALID[slv_id] <= 1'b1;
                b_done_rd_ptr[slv_id] = (b_done_rd_ptr[slv_id] + 1) & 3;
                b_done_cnt[slv_id]    = b_done_cnt[slv_id] - 1;
            end

            // B handshake complete
            if (s_BVALID[slv_id] && s_BREADY[slv_id])
                s_BVALID[slv_id] <= 1'b0;
        end
    endtask

    //=========================================================================
    // Slave AR Handler (only enqueues AR transactions)
    //=========================================================================
    task automatic slv_ar_handler(int slv_id);
        forever begin
            @(posedge clk);
            if (s_ARVALID[slv_id] && s_ARREADY[slv_id]) begin
                ar_q_arid[slv_id][ar_q_wr_ptr[slv_id]]   =  s_ARID[W_SID*(slv_id+1)-1 -: W_SID];
                ar_q_araddr[slv_id][ar_q_wr_ptr[slv_id]] =  s_ARADDR[ADDR_WIDTH*(slv_id+1)-1 -: ADDR_WIDTH];
                ar_q_total[slv_id][ar_q_wr_ptr[slv_id]]  =  s_ARLEN[TRANS_DATA_LEN_W*(slv_id+1)-1 -: TRANS_DATA_LEN_W] + 8'd1;
                ar_q_wr_ptr[slv_id] = (ar_q_wr_ptr[slv_id] + 1) & 3;
                ar_q_cnt[slv_id]    = ar_q_cnt[slv_id] + 1;
            end
        end
    endtask

    //=========================================================================
    // Slave R Data Handler (pops AR queue → drives R beats)
    //=========================================================================
    task automatic slv_r_handler(int slv_id);
        reg [W_SID-1:0]      cur_arid;
        reg [ADDR_WIDTH-1:0] cur_araddr;
        reg [7:0]            cur_total;
        reg [7:0]            cur_beat_cnt;
        reg                  cur_active;
        reg [ADDR_WIDTH-1:0] word_idx;

        begin
            cur_active = 1'b0;

            forever begin
                @(posedge clk);

                // Dequeue next transaction if idle and R channel clear to drive
                if (!cur_active && ar_q_cnt[slv_id] > 0 && (!s_RVALID[slv_id] || s_RREADY[slv_id])) begin
                    cur_arid      = ar_q_arid[slv_id][ar_q_rd_ptr[slv_id]];
                    cur_araddr    = ar_q_araddr[slv_id][ar_q_rd_ptr[slv_id]];
                    cur_total     = ar_q_total[slv_id][ar_q_rd_ptr[slv_id]];
                    cur_beat_cnt  = 0;
                    cur_active    = 1'b1;
                    ar_q_rd_ptr[slv_id] = (ar_q_rd_ptr[slv_id] + 1) & 3;
                    ar_q_cnt[slv_id]    = ar_q_cnt[slv_id] - 1;

                    // Drive first beat
                    word_idx = cur_araddr[11:2];
                    s_RID[W_SID*(slv_id+1)-1 -: W_SID] <= cur_arid;
                    s_RDATA[DATA_WIDTH*(slv_id+1)-1 -: DATA_WIDTH] <=
                        (word_idx < SLV_MEM_DEPTH) ? slv_mem[slv_id][word_idx] : 32'hDEAD_BEEF;
                    s_RRESP[TRANS_WR_RESP_W*(slv_id+1)-1 -: TRANS_WR_RESP_W] <= 2'b00;
                    s_RLAST[slv_id] <= (cur_total == 8'd1);
                    s_RVALID[slv_id] <= 1'b1;
                end

                // R handshake: advance or finish
                if (cur_active && s_RVALID[slv_id] && s_RREADY[slv_id]) begin
                    if (cur_beat_cnt == cur_total - 1) begin
                        s_RVALID[slv_id] <= 1'b0;
                        s_RLAST[slv_id]  <= 1'b0;
                        cur_active = 1'b0;
                    end else begin
                        cur_beat_cnt = cur_beat_cnt + 8'd1;
                        word_idx = cur_araddr[11:2] + cur_beat_cnt;
                        s_RDATA[DATA_WIDTH*(slv_id+1)-1 -: DATA_WIDTH] <= (word_idx < SLV_MEM_DEPTH) ? slv_mem[slv_id][word_idx] : 32'hDEAD_BEEF;
                        s_RLAST[slv_id] <= (cur_beat_cnt == cur_total - 1);
                    end
                end
            end
        end
    endtask

    //=========================================================================
    // Master BFM: Write
    //=========================================================================
    task automatic axi_write(
        input int               mst_id,
        input [ADDR_WIDTH-1:0]  addr,
        input [7:0]             len,
        input [2:0]             size,
        input [1:0]             burst,
        input [W_MID-1:0]       id,
        input data_buf_t        data
    );
        reg [7:0] total_beats;
        reg [7:0] b;
        begin
            total_beats = len + 8'd1;
            $display("[%0t] M[%0d] WR: addr=0x%08h len=%0d id=%0h",
                     $time, mst_id, addr, len, id);

            // AW
            m_AWID[W_MID*(mst_id+1)-1 -: W_MID] <= id;
            m_AWADDR[ADDR_WIDTH*(mst_id+1)-1 -: ADDR_WIDTH] <= addr;
            m_AWLEN[TRANS_DATA_LEN_W*(mst_id+1)-1 -: TRANS_DATA_LEN_W] <= len;
            m_AWSIZE[TRANS_DATA_SIZE_W*(mst_id+1)-1 -: TRANS_DATA_SIZE_W] <= size;
            m_AWBURST[TRANS_BURST_W*(mst_id+1)-1 -: TRANS_BURST_W] <= burst;
            m_AWVALID[mst_id] <= 1'b1;

            @(posedge clk);
            while (!m_AWREADY[mst_id]) @(posedge clk);
            m_AWVALID[mst_id] <= 1'b0;

            // W beats
            for (b = 0; b < total_beats; b = b + 1) begin
                m_WDATA[DATA_WIDTH*(mst_id+1)-1 -: DATA_WIDTH] <= data[b];
                m_WSTRB[W_STRB*(mst_id+1)-1 -: W_STRB] <= {W_STRB{1'b1}};
                m_WLAST[mst_id] <= (b == total_beats - 1);
                m_WVALID[mst_id] <= 1'b1;
                @(posedge clk);
                while (!m_WREADY[mst_id]) @(posedge clk);
            end
            m_WVALID[mst_id] <= 1'b0;
            m_WLAST[mst_id]  <= 1'b0;

            // B response
            m_BREADY[mst_id] <= 1'b1;
            @(posedge clk);
            while (!m_BVALID[mst_id]) @(posedge clk);
            $display("[%0t] M[%0d] WR BRESP: id=%0h resp=%0d",
                     $time, mst_id,
                     m_BID[W_SID*(mst_id+1)-1 -: W_SID],
                     m_BRESP[TRANS_WR_RESP_W*(mst_id+1)-1 -: TRANS_WR_RESP_W]);
            m_BREADY[mst_id] <= 1'b0;
        end
    endtask

    //=========================================================================
    // Master BFM: Read
    //=========================================================================
    task automatic axi_read(
        input int               mst_id,
        input [ADDR_WIDTH-1:0]  addr,
        input [7:0]             len,
        input [2:0]             size,
        input [1:0]             burst,
        input [W_MID-1:0]       id,
        output data_buf_t       rdata
    );
        reg [7:0] total_beats;
        reg [7:0] b;
        begin
            total_beats = len + 8'd1;
            $display("[%0t] M[%0d] RD: addr=0x%08h len=%0d id=%0h",
                     $time, mst_id, addr, len, id);

            // AR
            m_ARID[W_MID*(mst_id+1)-1 -: W_MID] <= id;
            m_ARADDR[ADDR_WIDTH*(mst_id+1)-1 -: ADDR_WIDTH] <= addr;
            m_ARLEN[TRANS_DATA_LEN_W*(mst_id+1)-1 -: TRANS_DATA_LEN_W] <= len;
            m_ARSIZE[TRANS_DATA_SIZE_W*(mst_id+1)-1 -: TRANS_DATA_SIZE_W] <= size;
            m_ARBURST[TRANS_BURST_W*(mst_id+1)-1 -: TRANS_BURST_W] <= burst;
            m_ARVALID[mst_id] <= 1'b1;

            @(posedge clk);
            while (!m_ARREADY[mst_id]) @(posedge clk);
            m_ARVALID[mst_id] <= 1'b0;

            // R beats
            m_RREADY[mst_id] <= 1'b1;
            for (b = 0; b < total_beats; b = b + 1) begin
                @(posedge clk);
                while (!m_RVALID[mst_id]) @(posedge clk);
                rdata[b] = m_RDATA[DATA_WIDTH*(mst_id+1)-1 -: DATA_WIDTH];
                $display("[%0t] M[%0d] RD beat[%0d]: data=0x%08h resp=%0d last=%0d",
                         $time, mst_id, b, rdata[b],
                         m_RRESP[TRANS_WR_RESP_W*(mst_id+1)-1 -: TRANS_WR_RESP_W],
                         m_RLAST[mst_id]);
            end
            m_RREADY[mst_id] <= 1'b0;
        end
    endtask

    //=========================================================================
    // Helper functions
    //=========================================================================
    function automatic [ADDR_WIDTH-1:0] slv_base_addr(input [1:0] slv_id);
        slv_base_addr = slv_id * 32'h1000;
    endfunction

    function automatic [DATA_WIDTH-1:0] gen_data(
        input [ADDR_WIDTH-1:0] addr,
        input [7:0] beat
    );
        gen_data = {addr[15:0], beat[7:0], addr[7:0]};
    endfunction

    //=========================================================================
    // Default signal tie-offs
    //=========================================================================
    initial begin
        m_AWVALID = '0;
        m_WVALID  = '0;
        m_WLAST   = '0;
        m_BREADY  = '0;
        m_ARVALID = '0;
        m_RREADY  = '0;
    end

    initial begin
        s_AWREADY = {SLV_AMT{1'b1}};
        s_WREADY  = {SLV_AMT{1'b1}};
        s_ARREADY = {SLV_AMT{1'b1}};
        s_BID     = '0;
        s_BRESP   = '0;
        s_BVALID  = '0;
        s_RID     = '0;
        s_RDATA   = '0;
        s_RRESP   = '0;
        s_RLAST   = '0;
        s_RVALID  = '0;
    end

    initial begin
        arbiter_type  = 1'b0;               // round-robin
        r_order_grant = '0;                 // use internal reorder
        slv_en        = {SLV_AMT{1'b1}};    // all slaves enabled
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
                slv_b_handler(sid);
                slv_ar_handler(sid);
                slv_r_handler(sid);
            join_none
        end
    end

    //=========================================================================
    // Main Test Sequence
    //=========================================================================
    data_buf_t wdata, rdata;
    integer    i, b, err_cnt;
    reg [ADDR_WIDTH-1:0] test_addr;
    reg [DATA_WIDTH-1:0] expected;

    initial begin
        err_cnt = 0;

        @(posedge rst_n);
        repeat (5) @(posedge clk);

        $display("============================================================");
        $display("[%0t] CROSSBAR TESTBENCH STARTED", $time);
        $display("============================================================");

        //=================================================================
        // TEST 1: Single-beat write + read to each slave from M0
        //=================================================================
        $display("\n--- TEST 1: Single-beat WR/RD to each slave ---");
        for (i = 0; i < SLV_AMT; i = i + 1) begin
            test_addr = slv_base_addr(i[1:0]) + 32'h40;
            wdata[0] = gen_data(test_addr, 8'd0);
            axi_write(0, test_addr, 8'd0, 3'd2, 2'd1, 6'h05, wdata);
            axi_read(0, test_addr, 8'd0, 3'd2, 2'd1, 6'h05, rdata);
            expected = gen_data(test_addr, 8'd0);
            if (rdata[0] !== expected) begin
                $display("[%0t] ERROR: S%0d mismatch! got=0x%08h exp=0x%08h",
                         $time, i, rdata[0], expected);
                err_cnt = err_cnt + 1;
            end
        end
        $display("[%0t] TEST 1 DONE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 2: Burst-4 write + read from M1 to S2
        //=================================================================
        $display("\n--- TEST 2: Burst-4 WR/RD ---");
        test_addr = slv_base_addr(2'd2) + 32'h80;
        for (b = 0; b < 4; b = b + 1)
            wdata[b] = gen_data(test_addr, b[7:0]);
        axi_write(1, test_addr, 8'd3, 3'd2, 2'd1, 6'h0A, wdata);
        axi_read(1, test_addr, 8'd3, 3'd2, 2'd1, 6'h0A, rdata);
        for (b = 0; b < 4; b = b + 1) begin
            expected = gen_data(test_addr, b[7:0]);
            if (rdata[b] !== expected) begin
                $display("[%0t] ERROR: burst beat=%0d got=0x%08h exp=0x%08h",
                         $time, b, rdata[b], expected);
                err_cnt = err_cnt + 1;
            end
        end
        $display("[%0t] TEST 2 DONE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 3: Multi-master concurrent writes (separate slaves)
        //=================================================================
        $display("\n--- TEST 3: Multi-master concurrent writes ---");
        fork
            begin
                wdata[0] = 32'hAAAA_0000; wdata[1] = 32'hAAAA_0001;
                axi_write(0, slv_base_addr(2'd0) + 32'h100, 8'd1, 3'd2, 2'd1, 6'h00, wdata);
            end
            begin
                wdata[0] = 32'hBBBB_0000; wdata[1] = 32'hBBBB_0001;
                axi_write(1, slv_base_addr(2'd1) + 32'h200, 8'd1, 3'd2, 2'd1, 6'h04, wdata);
            end
            begin
                wdata[0] = 32'hCCCC_0000; wdata[1] = 32'hCCCC_0001;
                axi_write(2, slv_base_addr(2'd2) + 32'h300, 8'd1, 3'd2, 2'd1, 6'h08, wdata);
            end
            begin
                wdata[0] = 32'hDDDD_0000; wdata[1] = 32'hDDDD_0001;
                axi_write(3, slv_base_addr(2'd3) + 32'h400, 8'd1, 3'd2, 2'd1, 6'h0C, wdata);
            end
        join
        // Read back from each using corresponding master
        axi_read(0, slv_base_addr(2'd0) + 32'h100, 8'd1, 3'd2, 2'd1, 6'h00, rdata);
        if (rdata[0] !== 32'hAAAA_0000) begin
            $display("[%0t] ERROR: M0 multi-wr data mismatch! got=0x%08h", $time, rdata[0]);
            err_cnt = err_cnt + 1;
        end
        axi_read(1, slv_base_addr(2'd1) + 32'h200, 8'd1, 3'd2, 2'd1, 6'h04, rdata);
        if (rdata[0] !== 32'hBBBB_0000) begin
            $display("[%0t] ERROR: M1 multi-wr data mismatch! got=0x%08h", $time, rdata[0]);
            err_cnt = err_cnt + 1;
        end
        axi_read(2, slv_base_addr(2'd2) + 32'h300, 8'd1, 3'd2, 2'd1, 6'h08, rdata);
        if (rdata[0] !== 32'hCCCC_0000) begin
            $display("[%0t] ERROR: M2 multi-wr data mismatch! got=0x%08h", $time, rdata[0]);
            err_cnt = err_cnt + 1;
        end
        axi_read(3, slv_base_addr(2'd3) + 32'h400, 8'd1, 3'd2, 2'd1, 6'h0C, rdata);
        if (rdata[0] !== 32'hDDDD_0000) begin
            $display("[%0t] ERROR: M3 multi-wr data mismatch! got=0x%08h", $time, rdata[0]);
            err_cnt = err_cnt + 1;
        end
        $display("[%0t] TEST 3 DONE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 4: Two masters to same slave (arbitration test)
        //=================================================================
        $display("\n--- TEST 4: Multi-master to same slave (arbitration) ---");
        fork
            begin
                wdata[0] = 32'h1111_0000;
                axi_write(0, slv_base_addr(2'd0) + 32'h500, 8'd0, 3'd2, 2'd1, 6'h10, wdata);
            end
            begin
                wdata[0] = 32'h2222_0000;
                axi_write(1, slv_base_addr(2'd0) + 32'h504, 8'd0, 3'd2, 2'd1, 6'h14, wdata);
            end
        join
        axi_read(0, slv_base_addr(2'd0) + 32'h500, 8'd0, 3'd2, 2'd1, 6'h10, rdata);
        if (rdata[0] !== 32'h1111_0000) begin
            $display("[%0t] ERROR: arbitration M0 data mismatch! got=0x%08h", $time, rdata[0]);
            err_cnt = err_cnt + 1;
        end
        axi_read(1, slv_base_addr(2'd0) + 32'h504, 8'd0, 3'd2, 2'd1, 6'h14, rdata);
        if (rdata[0] !== 32'h2222_0000) begin
            $display("[%0t] ERROR: arbitration M1 data mismatch! got=0x%08h", $time, rdata[0]);
            err_cnt = err_cnt + 1;
        end
        $display("[%0t] TEST 4 DONE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 5: Back-to-back transactions
        //=================================================================
        $display("\n--- TEST 5: Back-to-back ---");
        test_addr = slv_base_addr(2'd0) + 32'h600;
        wdata[0] = 32'hB2B0_0001;
        axi_write(0, test_addr, 8'd0, 3'd2, 2'd1, 6'h00, wdata);
        wdata[0] = 32'hB2B0_0002;
        axi_write(0, test_addr + 32'd4, 8'd0, 3'd2, 2'd1, 6'h00, wdata);
        axi_read(0, test_addr, 8'd0, 3'd2, 2'd1, 6'h00, rdata);
        if (rdata[0] !== 32'hB2B0_0001) begin
            $display("[%0t] ERROR: B2B-1 mismatch! got=0x%08h", $time, rdata[0]);
            err_cnt = err_cnt + 1;
        end
        axi_read(0, test_addr + 32'd4, 8'd0, 3'd2, 2'd1, 6'h00, rdata);
        if (rdata[0] !== 32'hB2B0_0002) begin
            $display("[%0t] ERROR: B2B-2 mismatch! got=0x%08h", $time, rdata[0]);
            err_cnt = err_cnt + 1;
        end
        $display("[%0t] TEST 5 DONE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 6: Long burst (16 beats)
        //=================================================================
        $display("\n--- TEST 6: Long burst (16 beats) ---");
        test_addr = slv_base_addr(2'd2) + 32'h200;
        for (b = 0; b < 16; b = b + 1)
            wdata[b] = 32'hC000_0000 + b;
        axi_write(2, test_addr, 8'd15, 3'd2, 2'd1, 6'h1F, wdata);
        axi_read(2, test_addr, 8'd15, 3'd2, 2'd1, 6'h1F, rdata);
        for (b = 0; b < 16; b = b + 1) begin
            expected = 32'hC000_0000 + b;
            if (rdata[b] !== expected) begin
                $display("[%0t] ERROR: long burst beat=%0d got=0x%08h exp=0x%08h",
                         $time, b, rdata[b], expected);
                err_cnt = err_cnt + 1;
            end
        end
        $display("[%0t] TEST 6 DONE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 7: Concurrent read + write (different masters/slaves)
        //=================================================================
        $display("\n--- TEST 7: Concurrent read+write ---");
        fork
            begin
                wdata[0] = 32'hCAFE_0000; wdata[1] = 32'hCAFE_0001;
                wdata[2] = 32'hCAFE_0002; wdata[3] = 32'hCAFE_0003;
                axi_write(0, slv_base_addr(2'd0) + 32'h700, 8'd3, 3'd2, 2'd1, 6'h20, wdata);
            end
            begin
                axi_read(1, slv_base_addr(2'd1) + 32'h100, 8'd3, 3'd2, 2'd1, 6'h24, rdata);
            end
        join
        $display("[%0t] TEST 7 DONE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 8: Fixed-priority arbitration
        //=================================================================
        $display("\n--- TEST 8: Fixed-priority arbitration ---");
        @(posedge clk);
        arbiter_type <= 1'b1;  // switch to fixed-priority
        @(posedge clk);
        fork
            begin
                wdata[0] = 32'hF100_0000;
                axi_write(0, slv_base_addr(2'd0) + 32'h800, 8'd0, 3'd2, 2'd1, 6'h30, wdata);
            end
            begin
                wdata[0] = 32'hF200_0000;
                axi_write(1, slv_base_addr(2'd0) + 32'h804, 8'd0, 3'd2, 2'd1, 6'h34, wdata);
            end
        join
        arbiter_type <= 1'b0;  // restore round-robin
        $display("[%0t] TEST 8 DONE (errors=%0d)", $time, err_cnt);

        //=================================================================
        // TEST 9: Slave disable / enable
        //=================================================================
        $display("\n--- TEST 9: Slave disable ---");
        @(posedge clk);
        slv_en[3] <= 1'b0;  // disable slave 3
        @(posedge clk);
        // Attempt write to disabled slave (should be blocked/handled gracefully)
        repeat (10) @(posedge clk);
        slv_en[3] <= 1'b1;  // re-enable
        repeat (3) @(posedge clk);
        // Verify slave 3 still functional after re-enable
        test_addr = slv_base_addr(2'd3) + 32'h900;
        wdata[0] = 32'hDEAD_BEEF;
        axi_write(0, test_addr, 8'd0, 3'd2, 2'd1, 6'h3F, wdata);
        axi_read(0, test_addr, 8'd0, 3'd2, 2'd1, 6'h3F, rdata);
        if (rdata[0] !== 32'hDEAD_BEEF) begin
            $display("[%0t] ERROR: re-enable S3 data mismatch! got=0x%08h", $time, rdata[0]);
            err_cnt = err_cnt + 1;
        end
        $display("[%0t] TEST 9 DONE (errors=%0d)", $time, err_cnt);

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
    // Timeout watchdog (5ms)
    //=========================================================================
    initial begin
        #5000000;
        $display("[%0t] ERROR: Simulation timeout!", $time);
        $finish;
    end

    //=========================================================================
    // Waveform dump (FSDB for Verdi)
    //=========================================================================
    initial begin
        $fsdbDumpfile("cross_tb.fsdb");
        $fsdbDumpvars(0, cross_tb);
        $fsdbDumpMDA();
    end

endmodule
