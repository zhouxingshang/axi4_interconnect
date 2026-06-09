//=============================================================================
// AXI Virtual Sequence: coordinates sequences across multiple sequencers
//=============================================================================
`ifndef AXI_VIRTUAL_SEQUENCE_SV
`define AXI_VIRTUAL_SEQUENCE_SV

class axi_virtual_sequence extends uvm_sequence;

    axi_virtual_sequencer vsqr;

    `uvm_object_utils(axi_virtual_sequence)

    function new(string name = "axi_virtual_sequence");
        super.new(name);
    endfunction

    // Grab the virtual sequencer handle from the sequencer this sequence runs on
    virtual task pre_body();
        if (!$cast(vsqr, get_sequencer()))
            `uvm_fatal("VSQR", "Virtual sequence must run on axi_virtual_sequencer")
    endtask

    // Convenience: launch a child sequence on a specific master sequencer
    virtual task start_on(int mst_id, uvm_sequence #(axi_transaction) seq);
        if (mst_id < 0 || mst_id > 3)
            `uvm_fatal("VSQR", $sformatf("Invalid mst_id %0d in start_on", mst_id))
        seq.start(vsqr.sqr[mst_id]);
    endtask

endclass

`endif
