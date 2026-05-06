module axi_s2m_s_amt
#(
    parameter MASTER_ID       = 0,
    parameter W_CID           = 4,
    parameter W_ID            = 4,
    parameter W_ADDR          = 32,
    parameter W_DATA          = 32,
    parameter W_STRB          = (W_DATA/8),
    parameter W_SID           = W_CID + W_ID,
    parameter SLV_AMT         = 4,
    parameter MST_ID_FIELD_MSB = W_ID+1,
    parameter MST_ID_FIELD_LSB = W_ID
)
(
    input  wire                      AXI_RSTn,
    input  wire                      AXI_CLK,
    
    // Master side ports (single master)
    output reg   [W_ID-1:0]          M_BID,
    output reg   [1:0]               M_BRESP,
    output reg                       M_BVALID,
    input  wire                      M_BREADY,
    
    output reg   [W_SID-1:0]         M_RSID,
    output reg   [W_DATA-1:0]        M_RDATA,
    output reg   [1:0]               M_RRESP,
    output reg                       M_RLAST,
    output reg                       M_RVALID,
    input  wire                      M_RREADY,
    
    // Slave side ports (packed)
    input  wire  [W_SID*SLV_AMT-1:0]  S_BID,
    input  wire  [2*SLV_AMT-1:0]      S_BRESP,
    input  wire  [SLV_AMT-1:0]        S_BVALID,
    output wire  [SLV_AMT-1:0]        S_BREADY,
    
    input  wire  [W_SID*SLV_AMT-1:0]  S_RID,
    input  wire  [W_DATA*SLV_AMT-1:0] S_RDATA,
    input  wire  [2*SLV_AMT-1:0]      S_RRESP,
    input  wire  [SLV_AMT-1:0]        S_RLAST,
    input  wire  [SLV_AMT-1:0]        S_RVALID,
    output wire  [SLV_AMT-1:0]        S_RREADY,
    
    input  wire  [SLV_AMT-1:0]        r_order_grant,
    input  wire                       arbiter_type
);

localparam MST_ID_W = $clog2(SLV_AMT);
localparam NUM = SLV_AMT;

// Unpack slave arrays
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

// Extract master index from ID
wire [MST_ID_W-1:0] s_bid_mst_idx [0:SLV_AMT-1];
wire [MST_ID_W-1:0] s_rid_mst_idx [0:SLV_AMT-1];
generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : DECODE
        assign s_bid_mst_idx[si] = s_bid[si][MST_ID_FIELD_MSB:MST_ID_FIELD_LSB];
        assign s_rid_mst_idx[si] = s_rid[si][MST_ID_FIELD_MSB:MST_ID_FIELD_LSB];
    end
endgenerate

// Select logic
wire [SLV_AMT-1:0] BSELECT, RSELECT, RSELECT_in;
generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : SELECT
        assign BSELECT[si] = (s_bid_mst_idx[si] == MASTER_ID[MST_ID_W-1:0]);
        assign RSELECT[si] = (s_rid_mst_idx[si] == MASTER_ID[MST_ID_W-1:0]);
        assign RSELECT_in[si] = RSELECT[si] & r_order_grant[si];
    end
endgenerate

// Arbitration
wire [SLV_AMT-1:0] BGRANT, RGRANT;
axi_arbiter_param_rr #(.NUM(SLV_AMT)) u_arb_b (
    .clk(AXI_CLK), .rst_n(AXI_RSTn), .arbiter_type(arbiter_type),
    .req(BSELECT & {S_BVALID[SLV_AMT-1:0]}), .grant(BGRANT)
);
axi_arbiter_param_rr #(.NUM(SLV_AMT)) u_arb_r (
    .clk(AXI_CLK), .rst_n(AXI_RSTn), .arbiter_type(arbiter_type),
    .req(RSELECT_in & {S_RVALID[SLV_AMT-1:0]}), .grant(RGRANT)
);

// Pack bus for muxing
localparam NUM_B_WIDTH = W_ID + 2 + 1;
localparam NUM_R_WIDTH = W_SID + W_DATA + 2 + 1 + 1;
wire [NUM_B_WIDTH-1:0] bus_b [0:SLV_AMT-1];
wire [NUM_R_WIDTH-1:0] bus_r [0:SLV_AMT-1];
generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : PACK
        assign bus_b[si] = {s_bid[si][W_ID-1:0], s_bresp[si], s_bvalid[si]};
        assign bus_r[si] = {s_rid[si], s_rdata[si], s_rresp[si], s_rlast[si], s_rvalid[si]};
    end
endgenerate

`define M_BBUS {M_BID[W_ID-1:0], M_BRESP, M_BVALID}
always @(*) begin
    `M_BBUS = 0;
    for(int i = 0; i < SLV_AMT; i++) if(BGRANT[i]) `M_BBUS = bus_b[i];
end

`define M_RBUS {M_RSID, M_RDATA, M_RRESP, M_RLAST, M_RVALID}
always @(*) begin
    `M_RBUS = 0;
    for(int i = 0; i < SLV_AMT; i++) if(RGRANT[i]) `M_RBUS = bus_r[i];
end

// Ready generation
generate
    for(si = 0; si < SLV_AMT; si = si + 1) begin : READY_GEN
        assign s_bready[si] = BGRANT[si] & M_BREADY;
        assign s_rready[si] = RGRANT[si] & M_RREADY;
    end
endgenerate

endmodule