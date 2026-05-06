module axi_arbiter_param_rr #(
    parameter NUM = 4
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              arbiter_type, // 0: RR, 1: Fixed
    input  wire [NUM-1:0]    req,
    output wire [NUM-1:0]    grant
);
    localparam PTR_W = (NUM > 1) ? $clog2(NUM) : 1;

    reg  [PTR_W-1:0] rr_ptr;      // Points to next priority start index
    wire [NUM-1:0]   grant_nxt;
    wire [NUM-1:0]   fixed_grant;

    // ---------------------------------------------------------
    // Priority Encoder: Returns one-hot of the lowest index '1'
    // ---------------------------------------------------------
    function automatic [NUM-1:0] pri_enc;
        input [NUM-1:0] mask;
        integer i;
        begin
            pri_enc = {NUM{1'b0}};
            for(i = 0; i < NUM; i = i + 1) begin
                if(mask[i]) begin
                    pri_enc = 1'b1 << i;
                    break;
                end
            end
        end
    endfunction

    // ---------------------------------------------------------
    // Fixed Priority
    // ---------------------------------------------------------
    assign fixed_grant = pri_enc(req);

    // ---------------------------------------------------------
    // Round Robin Logic (if NUM > 1)
    // ---------------------------------------------------------
    generate
        if (NUM > 1) begin : rr_gen
            // Rotate request vector so that rr_ptr becomes logical index 0
            wire [2*NUM-1:0] req_rot = {req, req} >> rr_ptr;
            wire [NUM-1:0]   pri_rot = pri_enc(req_rot[NUM-1:0]);

            // Convert rotated one-hot back to absolute winner index
            reg [PTR_W-1:0] winner_idx;
            always @(*) begin
                winner_idx = 0;
                for(int i = 0; i < NUM; i = i + 1) begin
                    if(pri_rot[i]) begin
                        winner_idx = i[PTR_W-1:0];
                        break;
                    end
                end
            end

            // Map back to absolute one-hot grant
            assign grant_nxt = 1'b1 << ((winner_idx + rr_ptr) % NUM);

            // RR Pointer Update
            always @(posedge clk) begin
                if(!rst_n) begin
                    rr_ptr <= {PTR_W{1'b0}};
                end else if(grant != 0) begin
                    rr_ptr <= (winner_idx + 1'b1) % NUM;
                end
            end
        end else begin : single_requester
            // When only one requester, grant is simply the request
            assign grant_nxt = req;
            // rr_ptr unused, tie to 0
            always @(posedge clk) begin
                if(!rst_n) rr_ptr <= 0;
                else rr_ptr <= 0;
            end
        end
    endgenerate

    // Output MUX
    assign grant = (arbiter_type) ? fixed_grant : grant_nxt;

endmodule