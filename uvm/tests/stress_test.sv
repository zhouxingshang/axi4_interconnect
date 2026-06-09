// Stress Test: 50 random transactions per master, concurrent
class stress_test extends axi_test_base;
    `uvm_component_utils(stress_test)
    function new(string n="stress_test", uvm_component p); super.new(n,p); endfunction
    task run_phase(uvm_phase phase);
        stress_vseq vseq;
        phase.raise_objection(this);
        phase.phase_done.set_drain_time(this, 0);
        fork
            begin
                #100000;
                `uvm_warning("TIMEOUT", "Simulation timeout at 6000000ns, stopping")
            end
            begin
                repeat(20) @(posedge env.vif.ACLK);  // wait for reset
                vseq = stress_vseq::type_id::create("vseq");
                run_vseq(vseq);
                #1000;
            end
        join_any
        disable fork;
        phase.drop_objection(this);
    endtask
endclass
