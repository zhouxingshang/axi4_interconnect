//=============================================================================
// Burst Virtual Sequence: one multi-beat INCR burst per master, sequential
//=============================================================================
`ifndef BURST_VSEQ_SV
`define BURST_VSEQ_SV

class burst_vseq extends axi_virtual_sequence;

    `uvm_object_utils(burst_vseq)

    function new(string name = "burst_vseq");
        super.new(name);
    endfunction

    virtual task body();
        for (int m = 0; m < 4; m++) begin
            burst_seq seq = burst_seq::type_id::create($sformatf("bseq_m%0d", m));
            seq.mst_id    = m;
            seq.base_addr = m * 32'h2000;
            seq.burst_len = (m + 1) * 2 - 1;
            start_on(m, seq);
        end
    endtask

endclass

`endif
