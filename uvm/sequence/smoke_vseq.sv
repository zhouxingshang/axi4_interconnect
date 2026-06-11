//=============================================================================
// Smoke Virtual Sequence: 4x4 crossbar write+read per master/slave pair
//=============================================================================
`ifndef SMOKE_VSEQ_SV
`define SMOKE_VSEQ_SV

class smoke_vseq extends axi_virtual_sequence;

    `uvm_object_utils(smoke_vseq)

    function new(string name = "smoke_vseq");
        super.new(name);
    endfunction

    virtual task body();
        bit [31:0] shared_addr;
        bit [3:0]  id_cnt;
        bit [31:0] slv_base[4] = '{32'h0000, 32'h2000, 32'h4000, 32'h6000};

        for (int m = 0; m < 4; m++) begin
            for (int s = 0; s < 4; s++) begin
                axi_base_seq seq_wr, seq_rd;
                shared_addr = (slv_base[s] | ($urandom & 32'h1FFF)) & ~32'h3;
                `uvm_info("VSEQ", $sformatf("m=%0d s=%0d shared_addr=0x%08h", m, s, shared_addr), UVM_NONE)

                seq_wr = axi_base_seq::type_id::create($sformatf("seq_wr_m%0d_s%0d", m, s));
                seq_wr.mst_id   = m;
                seq_wr.is_write = 1;
                seq_wr.addr     = shared_addr;
                seq_wr.seq_id   = id_cnt++;
                `uvm_info("VSEQ", $sformatf("m=%0d s=%0d WR start id=%0d", m, s, seq_wr.seq_id), UVM_NONE)
                start_on(m, seq_wr);
                `uvm_info("VSEQ", $sformatf("m=%0d s=%0d WR done  id=%0d", m, s, seq_wr.seq_id), UVM_NONE)

                seq_rd = axi_base_seq::type_id::create($sformatf("seq_rd_m%0d_s%0d", m, s));
                seq_rd.mst_id    = m;
                seq_rd.is_write  = 0;
                seq_rd.addr      = shared_addr;
                seq_rd.burst_len = seq_wr.actual_len;   // same len as write
                seq_rd.seq_id    = id_cnt++;
                `uvm_info("VSEQ", $sformatf("m=%0d s=%0d RD start id=%0d", m, s, seq_rd.seq_id), UVM_NONE)
                start_on(m, seq_rd);
                `uvm_info("VSEQ", $sformatf("m=%0d s=%0d RD done  id=%0d", m, s, seq_rd.seq_id), UVM_NONE)
            end
        end
    endtask

endclass

`endif
