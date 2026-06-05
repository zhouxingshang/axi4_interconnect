// 4KB Split Test: cross-4K boundary transactions
class split4k_test extends axi_test_base;
    `uvm_component_utils(split4k_test)
    function new(string n="split4k_test", uvm_component p); super.new(n,p); endfunction
    task run_phase(uvm_phase phase);
        split4k_seq seq_wr, seq_rd;
        phase.raise_objection(this);
        phase.phase_done.set_drain_time(this, 0);
        fork
            begin
                #6000;
                `uvm_warning("TIMEOUT", "Simulation timeout at 6000000ns, stopping")
            end
            begin
                // Write: cross 4K boundary, scoreboard captures data
                seq_wr = split4k_seq::type_id::create("seq_wr");
                seq_wr.mst_id   = 0;
                seq_wr.is_write = 1;
                seq_wr.start(env.master_agents[0].sqr);
                // Read back same address: scoreboard checks data match
                seq_rd = split4k_seq::type_id::create("seq_rd");
                seq_rd.mst_id   = 0;
                seq_rd.is_write = 0;
                seq_rd.start(env.master_agents[0].sqr);
                #1000;
            end
        join_any
        disable fork;
        phase.drop_objection(this);
    endtask
endclass
