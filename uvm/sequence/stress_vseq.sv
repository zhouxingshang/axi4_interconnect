//=============================================================================
// Stress Virtual Sequence: all 4 masters send randomized transactions concurrently
//=============================================================================
`ifndef STRESS_VSEQ_SV
`define STRESS_VSEQ_SV

class stress_vseq extends axi_virtual_sequence;

    `uvm_object_utils(stress_vseq)

    int tx_count = 32;   // transactions per master

    function new(string name = "stress_vseq");
        super.new(name);
    endfunction

    virtual task body();
        fork
            for (int m = 0; m < 4; m++) begin
                automatic int mid = m;
                stress_seq seq = stress_seq::type_id::create($sformatf("sseq_m%0d", mid));
                seq.mst_id = mid;
                seq.tx_count = tx_count;
                start_on(mid, seq);
            end
        join
    endtask

endclass

`endif
