//=============================================================================
// AXI Master Agent: contains sequencer + driver + monitor per master
//=============================================================================
`ifndef AXI_MASTER_AGENT_SV
`define AXI_MASTER_AGENT_SV

class axi_master_agent extends uvm_agent;

    axi_sequencer sqr;
    axi_driver    drv;
    axi_monitor   mon;

    int mst_id;

    `uvm_component_utils(axi_master_agent)

    function new(string name = "axi_master_agent", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        sqr = axi_sequencer::type_id::create($sformatf("sqr_m%0d", mst_id), this);
        drv = axi_driver::type_id::create($sformatf("drv_m%0d", mst_id), this);
        mon = axi_monitor::type_id::create($sformatf("mon_m%0d", mst_id), this);
        sqr.mst_id = mst_id;
        drv.mst_id = mst_id;
        mon.mst_id = mst_id;
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        drv.seq_item_port.connect(sqr.seq_item_export);
    endfunction

endclass

`endif
