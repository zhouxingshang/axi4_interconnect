//=============================================================================
// Arbitration Virtual Sequence: 4 masters -> same slave concurrently, two phases
// Phase 1: all 4 masters write to slave 0
// Phase 2: all 4 masters read from slave 0
//=============================================================================
`ifndef ARB_VSEQ_SV
`define ARB_VSEQ_SV

class arb_vseq extends axi_virtual_sequence;

    `uvm_object_utils(arb_vseq)

    function new(string name = "arb_vseq");
        super.new(name);
    endfunction

    virtual task body();
        // Phase 1: all 4 masters write to same slave concurrently
        fork
            for (int m = 0; m < 4; m++) begin
                automatic int mid = m;
                axi_base_seq seq_wr = axi_base_seq::type_id::create($sformatf("seq_wr_m%0d", mid));
                seq_wr.mst_id   = mid;
                seq_wr.is_write = 1;
                seq_wr.addr     = 32'h100;
                start_on(mid, seq_wr);
            end
        join

        // Phase 2: all 4 masters read back from same address
        fork
            for (int m = 0; m < 4; m++) begin
                automatic int mid = m;
                axi_base_seq seq_rd = axi_base_seq::type_id::create($sformatf("seq_rd_m%0d", mid));
                seq_rd.mst_id   = mid;
                seq_rd.is_write = 0;
                seq_rd.addr     = 32'h100;
                start_on(mid, seq_rd);
            end
        join
    endtask

endclass

`endif
