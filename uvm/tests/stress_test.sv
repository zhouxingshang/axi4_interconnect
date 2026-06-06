// Stress Test: 50 random transactions per master, concurrent
class stress_test extends axi_test_base;
    `uvm_component_utils(stress_test)
    function new(string n="stress_test", uvm_component p); super.new(n,p); endfunction
    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        phase.phase_done.set_drain_time(this, 0);
        fork
            begin
                #100000;
                `uvm_warning("TIMEOUT", "Simulation timeout at 6000000ns, stopping")
            end
            begin
                repeat(20) @(posedge env.vif.ACLK);  // wait for reset
                fork
                    for(int m=0; m<4; m++) begin
                        automatic int mid = m;
                        stress_seq seq = stress_seq::type_id::create($sformatf("seq_m%0d", mid));
                        seq.mst_id = mid;
                        seq.start(env.master_agents[mid].sqr);
                    end
                join
                #1000;
            end
        join_any
        disable fork;
        phase.drop_objection(this);
    endtask
endclass
