//=============================================================================
// Module: axi_arbiter_s2m_s_amt
// Desc  : Parameterized arbiter for Slave-to-Master B and R channels
//         - Supports arbitrary SLV_AMT requesters (parameterized)
//         - Handshake hold logic ensures grant persists until handshake completes
//         - Uses axi_arbiter_param_rr for arbitration
//=============================================================================
module axi_arbiter_s2m_s_amt #(
    parameter SLV_AMT = 4,          // Number of slaves (requesters)
    parameter USE_RLAST = 1         // 1: wait for RLAST at end of read burst
)(
    input  wire                  AXI_RSTn,
    input  wire                  AXI_CLK,
    
    // ---------- B Channel (Write Response) ----------
    input  wire  [SLV_AMT-1:0]   BSELECT,   // Decoded slave select (per slave)
    input  wire  [SLV_AMT-1:0]   BVALID,    // B valid from each slave
    input  wire  [SLV_AMT-1:0]   BREADY,    // B ready from master (common)
    output wire  [SLV_AMT-1:0]   BGRANT,    // Grant to slaves (one-hot)
    
    // ---------- R Channel (Read Data) ----------
    input  wire  [SLV_AMT-1:0]   RSELECT,   // Decoded slave select (per slave)
    input  wire  [SLV_AMT-1:0]   RVALID,    // R valid from each slave
    input  wire  [SLV_AMT-1:0]   RREADY,    // R ready from master (common)
    input  wire  [SLV_AMT-1:0]   RLAST,     // RLAST for each slave (optional)
    output wire  [SLV_AMT-1:0]   RGRANT,    // Grant to slaves (one-hot)
    
    // ---------- Common Control ----------
    input  wire                  arbiter_type   // 0: RR, 1: Fixed-Priority
);

//=============================================================================
// B Channel Arbiter with Handshake Hold
//=============================================================================
localparam STB_RUN = 1'b0, STB_WAIT = 1'b1;
reg  [SLV_AMT-1:0] bgrant_reg;
reg                 stateB;
wire [SLV_AMT-1:0] b_req;
wire [SLV_AMT-1:0] b_sel;

assign b_req = BSELECT & BVALID;

// Instantiate parameterized arbiter for B
axi_arbiter_param_rr #(
    .NUM(SLV_AMT)
) u_arb_b (
    .clk          (AXI_CLK),
    .rst_n        (AXI_RSTn),
    .arbiter_type (arbiter_type),
    .req          (b_req),
    .grant        (b_sel)
);

// State machine for B channel
always @(posedge AXI_CLK) begin
    if (!AXI_RSTn) begin
        bgrant_reg <= {SLV_AMT{1'b0}};
        stateB     <= STB_RUN;
    end else begin
        case (stateB)
            STB_RUN: begin
                if (|BGRANT) begin
                    // Current grant exists; check if handshake not yet done
                    if (~|(BGRANT & BREADY)) begin
                        bgrant_reg <= BGRANT;
                        stateB     <= STB_WAIT;
                    end
                end
            end
            STB_WAIT: begin
                // Wait for handshake completion (BVALID & BREADY on granted slave)
                if (|(BGRANT & BVALID & BREADY)) begin
                    stateB <= STB_RUN;
                end
            end
        endcase
    end
end

assign BGRANT = (stateB == STB_RUN) ? b_sel : bgrant_reg;

//=============================================================================
// R Channel Arbiter with Handshake Hold (optionally wait for RLAST)
//=============================================================================
localparam STR_RUN = 1'b0, STR_WAIT = 1'b1;
reg  [SLV_AMT-1:0] rgrant_reg;
reg                 stateR;
wire [SLV_AMT-1:0] r_req;
wire [SLV_AMT-1:0] r_sel;

assign r_req = RSELECT & RVALID;

// Instantiate parameterized arbiter for R
axi_arbiter_param_rr #(
    .NUM(SLV_AMT)
) u_arb_r (
    .clk          (AXI_CLK),
    .rst_n        (AXI_RSTn),
    .arbiter_type (arbiter_type),
    .req          (r_req),
    .grant        (r_sel)
);

// Handshake completion condition for R:
//   - If USE_RLAST is 1, need RLAST=1 on the granted slave to consider transaction done.
//   - If USE_RLAST is 0, just RVALID & RREADY is enough.
wire r_handshake_done;
generate
    if (USE_RLAST) begin
        assign r_handshake_done = |(RGRANT & RVALID & RREADY & RLAST);
    end else begin
        assign r_handshake_done = |(RGRANT & RVALID & RREADY);
    end
endgenerate

always @(posedge AXI_CLK) begin
    if (!AXI_RSTn) begin
        rgrant_reg <= {SLV_AMT{1'b0}};
        stateR     <= STR_RUN;
    end else begin
        case (stateR)
            STR_RUN: begin
                if (|RGRANT) begin
                    // Hold grant if handshake not yet complete
                    if (~|(RGRANT & RREADY)) begin
                        rgrant_reg <= RGRANT;
                        stateR     <= STR_WAIT;
                    end
                end
            end
            STR_WAIT: begin
                if (r_handshake_done) begin
                    stateR <= STR_RUN;
                end
            end
        endcase
    end
end

assign RGRANT = (stateR == STR_RUN) ? r_sel : rgrant_reg;

endmodule