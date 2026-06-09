//=============================================================================
// Outstanding Virtual Sequence: all 4 masters send outstanding transactions concurrently
// Each master fires `count` transactions back-to-back without waiting for responses
//=============================================================================
`ifndef OUTSTANDING_VSEQ_SV
`define OUTSTANDING_VSEQ_SV

class outstanding_vseq extends axi_virtual_sequence;

    `uvm_object_utils(outstanding_vseq)

    int count = 4;      // transactions per master

    function new(string name = "outstanding_vseq");
        super.new(name);
    endfunction

    virtual task body();
        fork
            for (int m = 0; m < 4; m++) begin
                automatic int mid = m;
                outstanding_seq seq = outstanding_seq::type_id::create($sformatf("oseq_m%0d", mid));
                seq.mst_id = mid;
                seq.count  = count;
                start_on(mid, seq);
            end
        join
    endtask

endclass

`endif
