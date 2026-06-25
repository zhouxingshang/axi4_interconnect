    //=======================================================================
    // 1) Route comparison: refm prediction vs slave-side actual
    //=======================================================================

    task collect_master_aw();
        axi_aw_item t; int key;
        forever begin
            aw_r_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            if(aw_s_pending.exists(key) && aw_s_pending[key].size() > 0) begin
                do_aw_route_check(t, aw_s_pending[key][0]);
                void'(aw_s_pending[key].pop_front());
                if (aw_s_pending[key].size() == 0)
                    aw_s_pending.delete(key);
            end else begin
                aw_m_pending[key] = t;
            end
        end
    endtask

    task collect_slave_aw();
        axi_aw_item t; int key;
        forever begin
            slv_aw_fifo.get(t);
            if(t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            if(aw_m_pending.exists(key)) begin
                do_aw_route_check(aw_m_pending[key], t);
                aw_m_pending.delete(key);
            end else begin
                if (!aw_s_pending.exists(key))
                    aw_s_pending[key] = {};
                aw_s_pending[key].push_back(t);
            end
        end
    endtask

    function void do_aw_route_check(axi_aw_item m_t, axi_aw_item s_t);
        int key = {m_t.mst_id[1:0], m_t.id[3:0]};
        int exp;
        bit is_split;
        bit[31:0] sub2_addr;
        int sub2_slv;
        exp = refm.predict_slv_id(m_t.addr); route_cnt++;
        // Route check: for split writes, sub2 may route to different slave
        is_split = addr_decoder::crosses_4k(m_t.addr, m_t.len, m_t.size);
        if(exp != s_t.slv_id) begin
            if (is_split) begin
                sub2_addr = {m_t.addr[31:12] + 1'b1, 12'h0};
                sub2_slv  = refm.predict_slv_id(sub2_addr);
                if (sub2_slv != s_t.slv_id) begin route_err++;
                    `uvm_error("ROUTE",$sformatf("WR M[%0d] addr=0x%08h exp S%0d got S%0d",
                              m_t.mst_id, m_t.addr, exp, s_t.slv_id))
                end
            end else begin route_err++;
                `uvm_error("ROUTE",$sformatf("WR M[%0d] addr=0x%08h exp S%0d got S%0d",
                          m_t.mst_id, m_t.addr, exp, s_t.slv_id))
            end
        end
        // W cross-check: record routing for S-side W beat ordering
        // Push key to the current S-side AW's slave
        slv_aw_order[s_t.slv_id].push_back(key);
        // If split: also push key to sub2's slave (not covered by the paired S-side AW)
        if (is_split) begin
            sub2_addr = {m_t.addr[31:12] + 1'b1, 12'h0};
            sub2_slv  = refm.predict_slv_id(sub2_addr);
            if (sub2_slv != s_t.slv_id) begin
                slv_aw_order[sub2_slv].push_back(key);
            end
        end
        // B cross-check: track split for B response merge (1 or 2 S-side Bs)
        if (!b_s_exp.exists(key)) begin
            b_s_exp[key] = is_split ? 2 : 1;
        end
    endfunction

    task collect_master_ar();
        axi_ar_item t; int key;
        forever begin
            ar_r_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            if(ar_s_pending.exists(key) && ar_s_pending[key].size() > 0) begin
                // Pair with all queued S-side ARs (supports split reads)
                while (ar_s_pending[key].size() > 0) begin
                    do_ar_route_check(t, ar_s_pending[key][0]);
                    void'(ar_s_pending[key].pop_front());
                end
                if (ar_pair_got[key] == ar_pair_exp[key]) begin
                    ar_s_pending.delete(key);
                    ar_pair_exp.delete(key);
                    ar_pair_got.delete(key);
                end
            end else begin
                ar_m_pending[key] = t;
            end
        end
    endtask

    task collect_slave_ar();
        axi_ar_item t; int key;
        forever begin
            slv_ar_fifo.get(t);
            if(t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            if(ar_m_pending.exists(key)) begin
                do_ar_route_check(ar_m_pending[key], t);
                // Only delete M-side pending when all expected pairs completed
                if (ar_pair_got[key] == ar_pair_exp[key]) begin
                    ar_m_pending.delete(key);
                    ar_pair_exp.delete(key);
                    ar_pair_got.delete(key);
                end
            end else begin
                if (!ar_s_pending.exists(key))
                    ar_s_pending[key] = {};
                ar_s_pending[key].push_back(t);
            end
        end
    endtask

    function void do_ar_route_check(axi_ar_item m_t, axi_ar_item s_t);
        int key = {m_t.mst_id[1:0], m_t.id[3:0]};
        int exp = refm.predict_slv_id(m_t.addr); route_cnt++;
        // Route check: for split reads, sub2 may route to different slave
        if(exp != s_t.slv_id) begin
            bit is_split;
            bit[31:0] sub2_addr;
            int sub2_slv;
            is_split = addr_decoder::crosses_4k(m_t.addr, m_t.len, m_t.size);
            if (is_split) begin
                sub2_addr = {m_t.addr[31:12] + 1'b1, 12'h0};
                sub2_slv  = refm.predict_slv_id(sub2_addr);
                if (sub2_slv != s_t.slv_id) begin route_err++;
                    `uvm_error("ROUTE",$sformatf("RD M[%0d] addr=0x%08h exp S%0d got S%0d",
                              m_t.mst_id, m_t.addr, exp, s_t.slv_id))
                end
            end else begin route_err++;
                `uvm_error("ROUTE",$sformatf("RD M[%0d] addr=0x%08h exp S%0d got S%0d",
                          m_t.mst_id, m_t.addr, exp, s_t.slv_id))
            end
        end
        // AR 1:2 pairing: track expected pairs for split reads
        if (!ar_pair_exp.exists(key)) begin
            ar_pair_exp[key] = addr_decoder::crosses_4k(m_t.addr, m_t.len, m_t.size) ? 2 : 1;
        end
        if (!ar_pair_got.exists(key))
            ar_pair_got[key] = 1;
        else
            ar_pair_got[key]++;
        // R cross-check: track split for RLAST/RRESP merge (1 or 2 S-side RLASTs)
        if (!r_s_exp.exists(key)) begin
            r_s_exp[key] = ar_pair_exp[key];
        end
    endfunction
