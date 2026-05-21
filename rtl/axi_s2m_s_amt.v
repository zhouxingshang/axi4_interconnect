module axi_s2m_s_amt
#(
    parameter MASTER_ID         = 0,
    parameter W_CID             = 4,
    parameter W_ID              = 6,
    parameter W_ADDR            = 32,
    parameter W_DATA            = 32,
    parameter W_STRB            = (W_DATA/8),
    parameter W_SID             = W_CID + W_ID,
    parameter MST_AMT           = 4,
    parameter SLV_AMT           = 4,
    parameter MST_ID_FIELD_MSB  = W_SID-1,
    parameter MST_ID_FIELD_LSB  = W_ID
)
(
    input  wire                       AXI_RSTn,
    input  wire                       AXI_CLK,

    //=========================================================
    // Master side ports
    //=========================================================
    output reg  [W_SID-1:0]          M_BID,
    output reg  [1:0]                M_BRESP,
    output reg                       M_BVALID,
    input  wire                      M_BREADY,

    output reg  [W_SID-1:0]          M_RSID,
    output reg  [W_DATA-1:0]         M_RDATA,
    output reg  [1:0]                M_RRESP,
    output reg                       M_RLAST,
    output reg                       M_RVALID,
    input  wire                      M_RREADY,

    //=========================================================
    // Slave side ports
    //=========================================================
    input  wire [W_SID*SLV_AMT-1:0]  S_BID,
    input  wire [2*SLV_AMT-1:0]      S_BRESP,
    input  wire [SLV_AMT-1:0]        S_BVALID,
    output wire [SLV_AMT-1:0]        S_BREADY,

    input  wire [W_SID*SLV_AMT-1:0]  S_RID,
    input  wire [W_DATA*SLV_AMT-1:0] S_RDATA,
    input  wire [2*SLV_AMT-1:0]      S_RRESP,
    input  wire [SLV_AMT-1:0]        S_RLAST,
    input  wire [SLV_AMT-1:0]        S_RVALID,
    output wire [SLV_AMT-1:0]        S_RREADY,

    //=========================================================
    // Control
    //=========================================================
    input  wire [SLV_AMT-1:0]        r_order_grant,
    input  wire                      arbiter_type
);

localparam MST_ID_W = $clog2(MST_AMT);

//=============================================================
// Unpack slave buses
//=============================================================
wire [W_SID-1:0]   s_bid      [0:SLV_AMT-1];
wire [1:0]         s_bresp    [0:SLV_AMT-1];
wire               s_bvalid   [0:SLV_AMT-1];
wire               s_bready   [0:SLV_AMT-1];

wire [W_SID-1:0]   s_rid      [0:SLV_AMT-1];
wire [W_DATA-1:0]  s_rdata    [0:SLV_AMT-1];
wire [1:0]         s_rresp    [0:SLV_AMT-1];
wire               s_rlast    [0:SLV_AMT-1];
wire               s_rvalid   [0:SLV_AMT-1];
wire               s_rready   [0:SLV_AMT-1];

genvar si;
generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : UNPACK

        assign s_bid[si]    = S_BID[W_SID*si +: W_SID];
        assign s_bresp[si]  = S_BRESP[2*si +: 2];
        assign s_bvalid[si] = S_BVALID[si];
        assign S_BREADY[si] = s_bready[si];

        assign s_rid[si]    = S_RID[W_SID*si +: W_SID];
        assign s_rdata[si]  = S_RDATA[W_DATA*si +: W_DATA];
        assign s_rresp[si]  = S_RRESP[2*si +: 2];
        assign s_rlast[si]  = S_RLAST[si];
        assign s_rvalid[si] = S_RVALID[si];
        assign S_RREADY[si] = s_rready[si];

    end
endgenerate

//=============================================================
// Decode destination master index
//=============================================================
wire [MST_ID_W-1:0] s_bid_mst_idx [0:SLV_AMT-1];
wire [MST_ID_W-1:0] s_rid_mst_idx [0:SLV_AMT-1];

generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : DECODE

        assign s_bid_mst_idx[si]
            = s_bid[si][MST_ID_FIELD_MSB:MST_ID_FIELD_LSB];

        assign s_rid_mst_idx[si]
            = s_rid[si][MST_ID_FIELD_MSB:MST_ID_FIELD_LSB];

    end
endgenerate

//=============================================================
// Request select
//=============================================================
wire [SLV_AMT-1:0] BSELECT;
wire [SLV_AMT-1:0] RSELECT;
wire [SLV_AMT-1:0] RSELECT_in;

generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : SELECT

        assign BSELECT[si]
            = (s_bid_mst_idx[si] == MASTER_ID);

        assign RSELECT[si]
            = (s_rid_mst_idx[si] == MASTER_ID);

        assign RSELECT_in[si]
            = RSELECT[si] & r_order_grant[si];

    end
endgenerate

//=============================================================
// Arbitration
//=============================================================
wire [SLV_AMT-1:0] BGRANT;
wire [SLV_AMT-1:0] RGRANT;

axi_arbiter_param_rr #(
    .NUM(SLV_AMT)
) u_arb_b (
    .clk           (AXI_CLK),
    .rst_n         (AXI_RSTn),
    .arbiter_type  (arbiter_type),
    .req           (BSELECT & S_BVALID),
    .grant         (BGRANT)
);

axi_arbiter_param_rr #(
    .NUM(SLV_AMT)
) u_arb_r (
    .clk           (AXI_CLK),
    .rst_n         (AXI_RSTn),
    .arbiter_type  (arbiter_type),
    .req           (RSELECT_in & S_RVALID),
    .grant         (RGRANT)
);

//=============================================================
// Held grants
// Hold grant until handshake completes
//=============================================================
reg [SLV_AMT-1:0] bgrant_d;
reg [SLV_AMT-1:0] rgrant_d;

always @(posedge AXI_CLK or negedge AXI_RSTn) begin
    if(!AXI_RSTn) begin
        bgrant_d <= '0;
        rgrant_d <= '0;
    end
    else begin

        //=====================================================
        // B channel
        //=====================================================
        if(|bgrant_d) begin

            // Hold until handshake completes
            if(M_BVALID && M_BREADY)
                bgrant_d <= BGRANT;

            else
                bgrant_d <= bgrant_d;

        end
        else begin
            bgrant_d <= BGRANT;
        end

        //=====================================================
        // R channel
        //=====================================================
        if(|rgrant_d) begin

            // Hold until handshake completes
            if(M_RVALID && M_RREADY)
                rgrant_d <= RGRANT;

            else
                rgrant_d <= rgrant_d;

        end
        else begin
            rgrant_d <= RGRANT;
        end

    end
end

//=============================================================
// M-side B mux
// IMPORTANT:
// Use HELD GRANT instead of combinational grant
//=============================================================
always @(*) begin

    M_BID    = '0;
    M_BRESP  = '0;
    M_BVALID = 1'b0;

    for(int i = 0; i < SLV_AMT; i = i + 1) begin
        if(bgrant_d[i]) begin

            M_BID    = s_bid[i];
            M_BRESP  = s_bresp[i];
            M_BVALID = s_bvalid[i];

        end
    end
end

//=============================================================
// M-side R mux
// IMPORTANT:
// Use HELD GRANT instead of combinational grant
//=============================================================
always @(*) begin

    M_RSID   = '0;
    M_RDATA  = '0;
    M_RRESP  = '0;
    M_RLAST  = '0;
    M_RVALID = 1'b0;

    for(int i = 0; i < SLV_AMT; i = i + 1) begin
        if(rgrant_d[i]) begin

            M_RSID   = s_rid[i];
            M_RDATA  = s_rdata[i];
            M_RRESP  = s_rresp[i];
            M_RLAST  = s_rlast[i];
            M_RVALID = s_rvalid[i];

        end
    end
end

//=============================================================
// Slave READY generation
// READY and VALID now use SAME grant domain
//=============================================================
generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : READY_GEN

        assign s_bready[si]
            = bgrant_d[si] & M_BREADY;

        assign s_rready[si]
            = rgrant_d[si] & M_RREADY;

    end
endgenerate

endmodule