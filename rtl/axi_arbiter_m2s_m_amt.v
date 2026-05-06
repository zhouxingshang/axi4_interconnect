module axi_arbiter_m2s_m_amt #(
    parameter W_CID = 4,
    parameter W_ID  = 4,
    parameter NUM   = 3          // Number of requesters (parameterized)
)(
    input  wire                  AXI_RSTn,
    input  wire                  AXI_CLK,
    
    // AW channel arbitration
    input  wire  [NUM-1:0]       AWSELECT, AWVALID, AWREADY,
    output wire  [NUM-1:0]       AWGRANT,
    
    // AR channel arbitration
    input  wire  [NUM-1:0]       ARSELECT, ARVALID, ARREADY,
    output wire  [NUM-1:0]       ARGRANT,
    
    input  wire                  arbiter_type    // 0: RR, 1: Fixed
);
    //=========================================================================
    // AW Arbitration Logic
    //=========================================================================
    localparam STAW_RUN = 1'b0, STAW_WAIT = 1'b1;
    reg [NUM-1:0] awgrant_reg;
    reg         stateAW = STAW_RUN;
    wire [NUM-1:0] aw_sel;

    // Parameterized RR/Fixed Arbiter Instance
    axi_arbiter_param_rr #(
        .NUM(NUM)
    ) u_arbiter_aw (
        .clk          (AXI_CLK),
        .rst_n        (AXI_RSTn),
        .arbiter_type (arbiter_type),
        .req          (AWSELECT & AWVALID),
        .grant        (aw_sel)
    );

    // Handshake Hold State Machine
    always @(posedge AXI_CLK) begin
        if (!AXI_RSTn) begin
            awgrant_reg <= '0;
            stateAW     <= STAW_RUN;
        end else begin
            case (stateAW)
                STAW_RUN: begin
                    if (|AWGRANT) begin
                        // Hold grant until handshake completes (AWREADY goes high)
                        if (~|(AWGRANT & AWREADY)) begin
                            awgrant_reg <= AWGRANT;
                            stateAW     <= STAW_WAIT;
                        end
                    end
                end
                STAW_WAIT: begin
                    if (|(AWGRANT & AWVALID & AWREADY)) begin
                        stateAW <= STAW_RUN;
                    end
                end
            endcase
        end
    end
    assign AWGRANT = (stateAW == STAW_RUN) ? aw_sel : awgrant_reg;

    //=========================================================================
    // AR Arbitration Logic (Mirrors AW)
    //=========================================================================
    localparam STAR_RUN = 1'b0, STAR_WAIT = 1'b1;
    reg [NUM-1:0] argrant_reg;
    reg         stateAR = STAR_RUN;
    wire [NUM-1:0] ar_sel;

    axi_arbiter_param_rr #(
        .NUM(NUM)
    ) u_arbiter_ar (
        .clk          (AXI_CLK),
        .rst_n        (AXI_RSTn),
        .arbiter_type (arbiter_type),
        .req          (ARSELECT & ARVALID),
        .grant        (ar_sel)
    );

    always @(posedge AXI_CLK) begin
        if (!AXI_RSTn) begin
            argrant_reg <= '0;
            stateAR     <= STAR_RUN;
        end else begin
            case (stateAR)
                STAR_RUN: begin
                    if (|ARGRANT) begin
                        if (~|(ARGRANT & ARREADY)) begin
                            argrant_reg <= ARGRANT;
                            stateAR     <= STAR_WAIT;
                        end
                    end
                end
                STAR_WAIT: begin
                    if (|(ARGRANT & ARVALID & ARREADY)) begin
                        stateAR <= STAR_RUN;
                    end
                end
            endcase
        end
    end
    assign ARGRANT = (stateAR == STAR_RUN) ? ar_sel : argrant_reg;

endmodule