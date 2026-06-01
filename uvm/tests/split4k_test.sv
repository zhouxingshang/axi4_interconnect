// 4KB Split Test: cross-4K boundary transactions
class split4k_test extends axi_test_base;
    `uvm_component_utils(split4k_test)
    function new(string n="split4k_test", uvm_component p); super.new(n,p); endfunction
    task run_phase(uvm_phase phase);
        split4k_seq seq; phase.raise_objection(this);
        for(int len_idx=0; len_idx<3; len_idx++) begin
            seq = split4k_seq::type_id::create("seq");
            seq.mst_id=0;
            seq.start(env.master_agents[0].sqr);
        end
        #1000; phase.drop_objection(this);
    endtask
endclass
