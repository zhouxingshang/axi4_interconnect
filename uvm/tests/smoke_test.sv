// Smoke Test: single write+read per master, basic connectivity
class smoke_test extends axi_test_base;
    `uvm_component_utils(smoke_test)

    function new(string n="smoke_test", uvm_component p);
        super.new(n,p);
    endfunction

    task run_phase(uvm_phase phase);
        smoke_vseq vseq;
        phase.raise_objection(this);
        fork
            begin
                #3000;
                `uvm_warning("TIMEOUT", "Simulation timeout at 2000000ns, stopping")
            end
            begin
                repeat(20) @(posedge env.vif.ACLK);  // wait for reset
                vseq = smoke_vseq::type_id::create("vseq");
                run_vseq(vseq);
                #1000;
            end
        join_any
        disable fork;
        phase.drop_objection(this);
    endtask
endclass
