// Outstanding Test: drives all 4 masters with outstanding transactions
class outstanding_test extends axi_test_base;

    `uvm_component_utils(outstanding_test)

    function new(string n = "outstanding_test", uvm_component p);
        super.new(n, p);
    endfunction

    task run_phase(uvm_phase phase);
        outstanding_vseq vseq;

        phase.raise_objection(this);
        phase.phase_done.set_drain_time(this, 0);

        fork
            begin
                #6000;
                `uvm_warning("TIMEOUT", "Simulation timeout, stopping")
            end
            begin
                repeat(20) @(posedge env.vif.ACLK);  // wait for reset

                vseq = outstanding_vseq::type_id::create("vseq");
                vseq.count = 8;                        // 8 transactions per master
                run_vseq(vseq);

                #1000;
            end
        join_any
        disable fork;

        phase.drop_objection(this);
    endtask

endclass
