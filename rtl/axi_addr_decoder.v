module axi_addr_decoder #(
    parameter SLV_AMT      = 4,
    parameter ADDR_WIDTH   = 32,
    parameter [0:SLV_AMT*ADDR_WIDTH-1] SLV_ADDR_BASE = {SLV_AMT{ADDR_WIDTH{1'b0}}},
    parameter [0:SLV_AMT*8-1]          SLV_ADDR_LEN  = {SLV_AMT{8'd12}}
)(
    input  wire [ADDR_WIDTH-1:0] addr,
    output wire [SLV_AMT-1:0]    sel
);
    genvar i;
    generate
        for(i=0; i<SLV_AMT; i=i+1) begin
            wire [ADDR_WIDTH-1:0] base = SLV_ADDR_BASE[ADDR_WIDTH*i +: ADDR_WIDTH];
            wire [7:0] len = SLV_ADDR_LEN[8*i +: 8];
            assign sel[i] = (addr[ADDR_WIDTH-1:len] == base[ADDR_WIDTH-1:len]);
        end
    endgenerate
endmodule