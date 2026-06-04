// Smoke Test: single write+read per master, basic connectivity
class smoke_test extends axi_test_base;
    `uvm_component_utils(smoke_test)
    
    function new(string n="smoke_test", uvm_component p); 
        super.new(n,p); 
    endfunction

    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        fork
            begin
                #1000;
                `uvm_fatal("TIMEOUT", "Simulation timeout at 1000000ns")
            end
            begin
                // wait for reset to complete
                repeat(20) @(posedge env.vif.ACLK);
                for(int m=0; m<4; m++) begin
                    axi_base_seq seq_wr, seq_rd;
                    seq_wr = axi_base_seq::type_id::create($sformatf("seq_wr_%0d", m));
                    seq_wr.mst_id   = m;
                    seq_wr.is_write = 1;
                    seq_wr.start(env.master_agents[m].sqr);

                    seq_rd = axi_base_seq::type_id::create($sformatf("seq_rd_%0d", m));
                    seq_rd.mst_id   = m;
                    seq_rd.is_write = 0;
                    seq_rd.start(env.master_agents[m].sqr);
                end
                #1000;
            end
        join_any
        phase.drop_objection(this);
    endtask
endclass
