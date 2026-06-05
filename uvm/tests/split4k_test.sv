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
                // Randomize write params, copy addr/len/size/burst to read for data match
                seq_wr = split4k_seq::type_id::create("seq_wr");
                seq_wr.is_write = 1;
                seq_wr.mst_id   = $urandom_range(0, 3);
                if (!seq_wr.randomize())
                    `uvm_fatal("SEQ", "split4k_seq write randomize failed")
                `uvm_info("TEST", $sformatf("split4k WR m=%0d: addr=0x%08h len=%0d size=%0d burst=%0d",
                         seq_wr.mst_id, seq_wr.addr, seq_wr.len, seq_wr.size, seq_wr.burst), UVM_NONE)
                seq_wr.start(env.master_agents[seq_wr.mst_id].sqr);
                // Read back from independent master, scoreboard checks cross-master data match
                seq_rd = split4k_seq::type_id::create("seq_rd");
                seq_rd.is_write = 0;
                seq_rd.mst_id   = $urandom_range(0, 3);
                seq_rd.addr     = seq_wr.addr;
                seq_rd.len      = seq_wr.len;
                seq_rd.size     = seq_wr.size;
                seq_rd.burst    = seq_wr.burst;
                `uvm_info("TEST", $sformatf("split4k RD m=%0d: addr=0x%08h len=%0d size=%0d burst=%0d",
                         seq_rd.mst_id, seq_rd.addr, seq_rd.len, seq_rd.size, seq_rd.burst), UVM_NONE)
                seq_rd.start(env.master_agents[seq_rd.mst_id].sqr);
                #1000;
            end
        join_any
        disable fork;
        phase.drop_objection(this);
    endtask
endclass
