// Arbitration Test: 4 masters -> same slave concurrently, verify RR arbitration
class arb_test extends axi_test_base;
    `uvm_component_utils(arb_test)
    function new(string n="arb_test", uvm_component p); super.new(n,p); endfunction
    task run_phase(uvm_phase phase);
        axi_base_seq seq_wr, seq_rd;
        phase.raise_objection(this);
        phase.phase_done.set_drain_time(this, 0);
        fork
            begin
                #6000;
                `uvm_warning("TIMEOUT", "Simulation timeout at 6000000ns, stopping")
            end
            begin
                repeat(20) @(posedge env.vif.ACLK);  // wait for reset
                // Phase 1: all 4 masters write to same slave concurrently
                fork
                    for(int m=0; m<4; m++) begin
                        automatic int mid = m;
                        seq_wr = axi_base_seq::type_id::create($sformatf("seq_wr_m%0d", mid));
                        seq_wr.mst_id   = mid;
                        seq_wr.is_write = 1;
                        seq_wr.addr     = 32'h100;      // all target slave 0
                        seq_wr.start(env.master_agents[mid].sqr);
                    end
                join
                // Phase 2: all 4 masters read back from same address
                fork
                    for(int m=0; m<4; m++) begin
                        automatic int mid = m;
                        seq_rd = axi_base_seq::type_id::create($sformatf("seq_rd_m%0d", mid));
                        seq_rd.mst_id   = mid;
                        seq_rd.is_write = 0;
                        seq_rd.addr     = 32'h100;      // same address, slave 0
                        seq_rd.start(env.master_agents[mid].sqr);
                    end
                join
                #1000;
            end
        join_any
        disable fork;
        phase.drop_objection(this);
    endtask
endclass
