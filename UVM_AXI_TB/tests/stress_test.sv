// Stress Test: 50 random transactions per master, concurrent
class stress_test extends axi_test_base;
    `uvm_component_utils(stress_test)
    function new(string n="stress_test", uvm_component p); super.new(n,p); endfunction
    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        fork
            for(int m=0; m<4; m++) begin
                automatic int mid=m;
                stress_seq seq = stress_seq::type_id::create("seq");
                seq.mst_id=mid;
                seq.start(env.master_agents[mid].sqr);
            end
        join
        #1000; phase.drop_objection(this);
    endtask
endclass
