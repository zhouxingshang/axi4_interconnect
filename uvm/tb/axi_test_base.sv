// Base Test: sets up env, provides virtual sequence infrastructure
class axi_test_base extends uvm_test;
    `uvm_component_utils(axi_test_base)
    axi_env env;

    function new(string n="axi_test_base", uvm_component p);
        super.new(n,p);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        env = axi_env::type_id::create("env", this);
    endfunction

    function void end_of_elaboration_phase(uvm_phase phase);
        uvm_top.print_topology();
    endfunction

    // Convenience: run a virtual sequence on the virtual sequencer
    virtual task run_vseq(axi_virtual_sequence vseq);
        vseq.start(env.vsqr);
    endtask
endclass
