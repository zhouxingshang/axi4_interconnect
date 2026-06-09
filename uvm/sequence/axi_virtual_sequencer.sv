//=============================================================================
// AXI Virtual Sequencer: holds handles to all 4 master sequencers
//=============================================================================
`ifndef AXI_VIRTUAL_SEQUENCER_SV
`define AXI_VIRTUAL_SEQUENCER_SV

class axi_virtual_sequencer extends uvm_sequencer;

    axi_sequencer sqr[4];

    `uvm_component_utils(axi_virtual_sequencer)

    function new(string name = "axi_virtual_sequencer", uvm_component parent);
        super.new(name, parent);
    endfunction

endclass

`endif
