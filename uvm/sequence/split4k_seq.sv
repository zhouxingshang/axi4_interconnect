// 4KB Split Sequence: randomly picks a 4K boundary, crosses it once
class split4k_seq extends uvm_sequence #(axi_transaction);
    `uvm_object_utils(split4k_seq)
    int  mst_id   = 0;
    bit  is_write = 1;       // 1=write, 0=read

    rand bit [31:0] addr;
    rand bit [7:0]  len;
    rand bit [2:0]  size;
    rand bit [1:0]  burst;

    rand bit [31:0] boundary;     // 4K-aligned boundary to cross
    rand bit [11:0] pre_offset;   // bytes before the boundary
    rand bit [11:0] post_bytes;   // bytes after the boundary

    constraint c_cross_4k {
        // ---- constraints matching axi_transaction ----
        size  == 2;                               // 4 bytes/beat
        len   inside {[0:15]};                    // 1~16 beats
        burst == 2'b01;                           // INCR
        // ---- pick a random 4K boundary ----
        boundary[11:0] == 12'h000;
        boundary inside {[32'h1000:32'h7000]};
        // ---- cross with alignment ----
        // total_bytes = (len+1)*4, must be 4-byte aligned and ≤ 64
        pre_offset inside {4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60};
        post_bytes inside {4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60};
        pre_offset + post_bytes == (len + 1) * 4;
        // derive addr (word-aligned, since boundary and pre_offset are aligned)
        addr == boundary - pre_offset;
        // TEMP: force start address to 0x1ffc for 1-beat sub-transaction 1 test
        addr == 32'h1ffc;
    }

    function new(string n="split4k_seq");
        super.new(n);
    endfunction

    task body();
        axi_transaction t = axi_transaction::type_id::create("t");
        start_item(t);
        void'(t.randomize() with {
            mst_id   == local::mst_id;
            len      == local::len;
            burst    == local::burst;
            size     == local::size;
            is_write == local::is_write;
        });
        t.addr = addr;          // assign after randomization to bypass addr_range_c
        finish_item(t);
    endtask
endclass
