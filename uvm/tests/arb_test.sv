// Arbitration Test: 4 masters -> same slave concurrently, verify RR arbitration
class arb_test extends axi_test_base;
    `uvm_component_utils(arb_test)
    function new(string n="arb_test", uvm_component p); super.new(n,p); endfunction
    task run_phase(uvm_phase phase);
        arb_vseq vseq;
        phase.raise_objection(this);
        phase.phase_done.set_drain_time(this, 0);
        fork
            begin
                #6000;
                `uvm_warning("TIMEOUT", "Simulation timeout at 6000000ns, stopping")
            end
            begin
                repeat(20) @(posedge env.vif.ACLK);  // wait for reset
                vseq = arb_vseq::type_id::create("vseq");
                run_vseq(vseq);
                #1000;
            end
        join_any
        disable fork;
        phase.drop_objection(this);
    endtask
endclass
