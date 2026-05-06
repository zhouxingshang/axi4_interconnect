module aw_order_fifo
#(
    parameter DATA_WIDTH  = 11,   // {MST_ID_W, ALEN_W} width
    parameter FIFO_DEPTH  = 8,    // Number of entries (power of 2 recommended)
    parameter PTR_W       = $clog2(FIFO_DEPTH)
)
(
    input  wire                   clk,
    input  wire                   rst_n,
    
    // Write port
    input  wire                   wr_en,
    input  wire [DATA_WIDTH-1:0]  wr_data,
    output wire                   wr_full,
    
    // Read port
    input  wire                   rd_en,
    output wire [DATA_WIDTH-1:0]  rd_data,
    output wire                   rd_empty,
    
    // Status
    output wire [PTR_W:0]         item_cnt
);

// Internal registers
reg  [PTR_W-1:0]  wr_ptr;
reg  [PTR_W-1:0]  rd_ptr;
reg  [PTR_W:0]    fill_cnt;

// Memory array
reg [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];

// Read data output (combinational)
assign rd_data = mem[rd_ptr];

// Full/Empty flags
assign wr_full  = (fill_cnt == FIFO_DEPTH);
assign rd_empty = (fill_cnt == 0);
assign item_cnt = fill_cnt;

// Write pointer & data storage
always @(posedge clk) begin
    if (!rst_n) begin
        wr_ptr   <= '0;
        fill_cnt <= '0;
    end else begin
        if (wr_en && !wr_full) begin
            mem[wr_ptr] <= wr_data;
            wr_ptr      <= wr_ptr + 1'b1;
        end
        // Update fill_cnt for write
        if (wr_en && !wr_full) begin
            fill_cnt <= fill_cnt + 1'b1;
        end
        // Update fill_cnt for read
        if (rd_en && !rd_empty) begin
            fill_cnt <= fill_cnt - 1'b1;
        end
    end
end

// Read pointer
always @(posedge clk) begin
    if (!rst_n) begin
        rd_ptr   <= '0;
    end else if (rd_en && !rd_empty) begin
        rd_ptr   <= rd_ptr + 1'b1;
    end
end

endmodule