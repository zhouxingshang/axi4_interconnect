// Smoke Test: single write+read per master, basic connectivity
class smoke_test extends axi_test_base;
    `uvm_component_utils(smoke_test)
    
    function new(string n="smoke_test", uvm_component p); 
        super.new(n,p); 
    endfunction

    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        for(int m=0; m<4; m++) begin
            axi_base_seq seq = axi_base_seq::type_id::create("seq");
            seq.mst_id = m;
            seq.start(env.master_agents[m].sqr);
        end
        #1000;
        phase.drop_objection(this);
    endtask
endclass
