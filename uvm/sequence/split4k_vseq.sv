//=============================================================================
// Split4K Virtual Sequence: write-then-read across a 4KB boundary
//=============================================================================
`ifndef SPLIT4K_VSEQ_SV
`define SPLIT4K_VSEQ_SV

class split4k_vseq extends axi_virtual_sequence;

    `uvm_object_utils(split4k_vseq)

    function new(string name = "split4k_vseq");
        super.new(name);
    endfunction

    virtual task body();
        split4k_seq seq_wr, seq_rd;

        // Randomize write params
        seq_wr = split4k_seq::type_id::create("seq_wr");
        seq_wr.is_write = 1;
        seq_wr.mst_id   = $urandom_range(0, 3);
        if (!seq_wr.randomize())
            `uvm_fatal("SEQ", "split4k_seq write randomize failed")
        `uvm_info("VSEQ", $sformatf("split4k WR m=%0d: addr=0x%08h len=%0d size=%0d burst=%0d",
                 seq_wr.mst_id, seq_wr.addr, seq_wr.len, seq_wr.size, seq_wr.burst), UVM_NONE)
        start_on(seq_wr.mst_id, seq_wr);

        // Read back with same addr/len/size/burst, independent master
        seq_rd = split4k_seq::type_id::create("seq_rd");
        seq_rd.is_write = 0;
        seq_rd.mst_id   = $urandom_range(0, 3);
        seq_rd.addr     = seq_wr.addr;
        seq_rd.len      = seq_wr.len;
        seq_rd.size     = seq_wr.size;
        seq_rd.burst    = seq_wr.burst;
        `uvm_info("VSEQ", $sformatf("split4k RD m=%0d: addr=0x%08h len=%0d size=%0d burst=%0d",
                 seq_rd.mst_id, seq_rd.addr, seq_rd.len, seq_rd.size, seq_rd.burst), UVM_NONE)
        start_on(seq_rd.mst_id, seq_rd);
    endtask

endclass

`endif
