// Base Sequence: single random write or read
class axi_base_seq extends uvm_sequence #(axi_transaction);
    `uvm_object_utils(axi_base_seq)
    int          mst_id   = 0;
    bit          is_write = 1;      // 1=write, 0=read, set by test
    bit [31:0]   addr;              // set by test to share between WR/RD
    bit [3:0]    seq_id   = 0;      // unique per-sequence ID, set by test
    bit [7:0]    burst_len = 8'hFF; // -1 = randomize freely; set to share WR/RD len
    bit [7:0]    actual_len;         // captured from randomized t.len

    function new(string n="axi_base_seq");
        super.new(n);
    endfunction

    task body();
        axi_transaction t = axi_transaction::type_id::create("t");
        start_item(t);

        if(!t.randomize() with { mst_id == local::mst_id;
                                  is_write == local::is_write;
                                  addr == local::addr;
                                  id == local::seq_id;
                                  if (local::burst_len != 8'hFF)
                                      len == local::burst_len;
                                  // don't cross 4KB boundary
                                  (addr[11:0] + ((len + 1) << size)) <= 13'h1000; })
            `uvm_fatal("SEQ","randomize failed")

        actual_len = t.len;
        finish_item(t);
    endtask
endclass
