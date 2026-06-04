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
                #3000;
                `uvm_warning("TIMEOUT", "Simulation timeout at 2000000ns, stopping")
            end
            begin
                bit [31:0] shared_addr;
                bit [3:0]  id_cnt;
                bit [31:0] slv_base[4] = '{32'h0000, 32'h2000, 32'h4000, 32'h6000};
                // wait for reset to complete
                repeat(20) @(posedge env.vif.ACLK);
                for(int m=0; m<4; m++) begin
                    for(int s=0; s<4; s++) begin
                        axi_base_seq seq_wr, seq_rd;
                        shared_addr = (slv_base[s] | ($urandom & 32'h1FFF)) & ~32'h3;

                        seq_wr = axi_base_seq::type_id::create($sformatf("seq_wr_m%0d_s%0d", m, s));
                        seq_wr.mst_id   = m;
                        seq_wr.is_write = 1;
                        seq_wr.addr     = shared_addr;
                        seq_wr.seq_id   = id_cnt++;
                        seq_wr.start(env.master_agents[m].sqr);

                        seq_rd = axi_base_seq::type_id::create($sformatf("seq_rd_m%0d_s%0d", m, s));
                        seq_rd.mst_id   = m;
                        seq_rd.is_write = 0;
                        seq_rd.addr     = shared_addr;
                        seq_rd.seq_id   = id_cnt++;
                        seq_rd.start(env.master_agents[m].sqr);
                    end
                end
                #1000;
            end
        join_any
        phase.drop_objection(this);
    endtask
endclass
