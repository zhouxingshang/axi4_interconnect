// 4KB Split Test: cross-4K boundary transactions
class split4k_test extends axi_test_base;
    `uvm_component_utils(split4k_test)
    function new(string n="split4k_test", uvm_component p); super.new(n,p); endfunction
    task run_phase(uvm_phase phase);
        split4k_vseq vseq;
        phase.raise_objection(this);
        phase.phase_done.set_drain_time(this, 0);
        fork
            begin
                #6000;
                `uvm_warning("TIMEOUT", "Simulation timeout at 6000000ns, stopping")
            end
            begin
                repeat(20) @(posedge env.vif.ACLK);  // wait for reset
                vseq = split4k_vseq::type_id::create("vseq");
                run_vseq(vseq);
                #1000;
            end
        join_any
        disable fork;
        phase.drop_objection(this);
    endtask
endclass
