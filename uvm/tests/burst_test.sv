// Burst Test: multi-beat INCR burst read/write
class burst_test extends axi_test_base;
    `uvm_component_utils(burst_test)
    function new(string n="burst_test", uvm_component p); super.new(n,p); endfunction
    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        phase.phase_done.set_drain_time(this, 0);
        fork
            begin
                #500000;
                `uvm_warning("TIMEOUT", "Simulation timeout at 6000000ns, stopping")
            end
            begin
                burst_seq seq;
                repeat(20) @(posedge env.vif.ACLK);  // wait for reset
                for(int m=0; m<4; m++) begin
                    seq = burst_seq::type_id::create("seq");
                    seq.mst_id=m; seq.base_addr=m*32'h2000; seq.burst_len=(m+1)*2-1;
                    seq.start(env.master_agents[m].sqr);
                end
                #1000;
            end
        join_any
        disable fork;
        phase.drop_objection(this);
    endtask
endclass
