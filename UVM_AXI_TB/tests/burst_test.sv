// Burst Test: multi-beat INCR burst read/write
class burst_test extends axi_test_base;
    `uvm_component_utils(burst_test)
    function new(string n="burst_test", uvm_component p); super.new(n,p); endfunction
    task run_phase(uvm_phase phase);
        burst_seq seq; phase.raise_objection(this);
        for(int m=0; m<4; m++) begin
            seq = burst_seq::type_id::create("seq");
            seq.mst_id=m; seq.base_addr=m*32'h2000; seq.burst_len=(m+1)*2-1;
            seq.start(env.master_agents[m].sqr);
        end
        #1000; phase.drop_objection(this);
    endtask
endclass
