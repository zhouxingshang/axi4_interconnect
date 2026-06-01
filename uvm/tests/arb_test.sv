// Arbitration Test: 4 masters -> same slave, verify RR arbitration
class arb_test extends axi_test_base;
    `uvm_component_utils(arb_test)
    function new(string n="arb_test", uvm_component p); super.new(n,p); endfunction
    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        fork
            for(int m=0; m<4; m++) begin
                automatic int mid=m;
                axi_base_seq seq = axi_base_seq::type_id::create("seq");
                seq.mst_id=mid;
                seq.start(env.master_agents[mid].sqr);
            end
        join
        #1000; phase.drop_objection(this);
    endtask
endclass
