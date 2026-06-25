    //=======================================================================
    // 2) Data comparison: AR→R address tracking + refm.shadow vs R actual
    //=======================================================================

    task process_ar_data();
        axi_ar_item t; int key; ar_info_t info;
        forever begin
            ar_d_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            info.addr=t.addr; info.len=t.len; info.size=t.size; info.burst=t.burst;
            ar_info_pool[key].push_back(info);
            if(!r_beat_cnt.exists(key)) r_beat_cnt[key]=0;
        end
    endtask

    task process_r_data();
        axi_r_item t; int key; ar_info_t info; bit[31:0] addr, exp_val; int bpb;
        forever begin
            r_d_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            if(ar_info_pool[key].size() == 0) begin
                data_err++;
                `uvm_error("DATA","R beat with no matching AR"); continue;
            end
            info = ar_info_pool[key][0];   // oldest AR with this ID
            bpb = 1 << info.size;
            addr = (info.burst == 2'b00) ? info.addr : (info.addr + (r_beat_cnt[key] * bpb));
            data_cnt++;
            exp_val = refm.read_shadow(addr);
            if(t.data !== exp_val) begin data_err++;
                `uvm_error("DATA",$sformatf("M[%0d] addr=0x%08h exp=0x%08h act=0x%08h",
                          t.mst_id, addr, exp_val, t.data))
            end
            // R cross-check: store M-side R data in rd_out
            if(rd_out[key].size() > 0) begin
                rd_out[key][0].m_r_data.push_back(t.data);
                rd_out[key][0].m_r_resp.push_back(t.resp);
                if (!m_r_id.exists(key))
                    m_r_id[key] = t.id;
                if (m_r_id[key] !== t.id) begin r_xchk_err++;
                    `uvm_error("R_XCHK", $sformatf("M-side RID inconsistent key=%0d exp=%0d got=%0d",
                              key, m_r_id[key], t.id))
                end
                if(t.last) begin
                    r_m_done[key] = 1;
                    r_try_xchk(key);
                end
            end
            r_beat_cnt[key]++;
            if(t.last) begin
                if(r_beat_cnt[key] != info.len+1) begin
                    data_err++;
                    `uvm_error("DATA",$sformatf("RD beat cnt M[%0d] id=%0d exp=%0d got=%0d",
                              t.mst_id, t.id, info.len+1, r_beat_cnt[key]))
                end
                void'(ar_info_pool[key].pop_front());
                r_beat_cnt.delete(key);
            end
        end
    endtask

    //=======================================================================
    // 3) Slave-side monitoring + cross-check against M-side
    //=======================================================================

    task process_slv_w();
        axi_w_item t;
        int key;
        bit found;
        forever begin
            slv_w_fifo.get(t);
            if (slv_aw_order[t.slv_id].size() == 0) begin
                `uvm_error("SLV_W", $sformatf("S[%0d] W beat with no pending AW in slv_aw_order", t.slv_id))
                continue;
            end
            key = slv_aw_order[t.slv_id][0];
            found = 0;
            if (wr_out.exists(key)) begin
                foreach (wr_out[key][j]) begin
                    if (wr_out[key][j].s_w_data.size() < wr_out[key][j].len + 1) begin
                        wr_out[key][j].s_w_data.push_back(t.data);
                        `uvm_info("SLV_W", $sformatf("S[%0d] W key=%0d beat=%0d data=0x%08h last=%b",
                                   t.slv_id, key, wr_out[key][j].s_w_data.size()-1, t.data, t.last), UVM_HIGH)
                        if (t.last) begin
                            void'(slv_aw_order[t.slv_id].pop_front());
                            if (wr_out[key][j].m_w_data.size() == wr_out[key][j].len + 1)
                                w_do_xchk(key, j);
                        end
                        found = 1;
                        break;
                    end
                end
            end
            if (!found) begin
                slv_w_buf[key].push_back(t);
                `uvm_info("SLV_W", $sformatf("S[%0d] W buffered key=%0d (AW not ready)", t.slv_id, key), UVM_HIGH)
            end
        end
    endtask

    task process_slv_b();
        axi_b_item t; int key;
        forever begin
            slv_b_fifo.get(t);
            key = {t.mst_id[1:0], t.id[3:0]};
            if (t.resp != 2'b00) begin
                `uvm_error("SLV_RESP", $sformatf("S[%0d] BRESP error mst=%0d resp=%b",
                          t.slv_id, t.mst_id, t.resp))
            end
            if (!b_s_exp.exists(key))
                b_s_exp[key] = 1;
            if (!b_s_got.exists(key)) begin
                b_s_got[key]   = 1;
                b_s_mresp[key] = t.resp;
                b_s_id[key]    = t.id;
            end else begin
                b_s_got[key]++;
                b_s_mresp[key] = b_s_mresp[key] | t.resp;
                if (b_s_id[key] !== t.id) begin
                    b_xchk_err++;
                    `uvm_error("B_XCHK", $sformatf("S-side BID inconsistent key=%0d exp=%0d got=%0d",
                              key, b_s_id[key], t.id))
                end
            end
            `uvm_info("SLV_B", $sformatf("S[%0d] B mst=%0d id=%0d resp=%0d got=%0d/exp=%0d",
                       t.slv_id, t.mst_id, t.id, t.resp, b_s_got[key], b_s_exp[key]), UVM_HIGH)
            if (b_s_got[key] == b_s_exp[key] && b_m_pending.exists(key)) begin
                do_b_xchk(b_m_pending[key], b_s_mresp[key], b_s_id[key], key);
                b_m_pending.delete(key);
            end
        end
    endtask

    task process_slv_r();
        axi_r_item t; int key;
        forever begin
            slv_r_fifo.get(t);
            key = {t.mst_id[1:0], t.id[3:0]};
            if (t.resp != 2'b00) begin
                `uvm_error("SLV_RESP", $sformatf("S[%0d] RRESP error mst=%0d resp=%b",
                          t.slv_id, t.mst_id, t.resp))
            end
            if (rd_out[key].size() > 0) begin
                rd_out[key][0].s_r_data.push_back(t.data);
                rd_out[key][0].s_r_resp.push_back(t.resp);
                if (!s_r_id.exists(key))
                    s_r_id[key] = t.id;
                if (s_r_id[key] !== t.id) begin
                    r_xchk_err++;
                    `uvm_error("R_XCHK", $sformatf("S-side RID inconsistent key=%0d exp=%0d got=%0d",
                              key, s_r_id[key], t.id))
                end
                if (!r_s_exp.exists(key)) r_s_exp[key] = 1;
                if (!r_s_got.exists(key)) begin
                    r_s_got[key]        = 0;
                    r_s_merge_resp[key] = 2'b00;
                end
                r_s_merge_resp[key] = r_s_merge_resp[key] | t.resp;
                if (t.last) begin
                    r_s_got[key]++;
                    r_try_xchk(key);
                end
            end
            `uvm_info("SLV_R", $sformatf("S[%0d] R mst=%0d id=%0d beat=%0d data=0x%08h last=%b",
                       t.slv_id, t.mst_id, t.id, rd_out.exists(key) ? rd_out[key][0].s_r_data.size() : 0, t.data, t.last), UVM_HIGH)
        end
    endtask

    //---- Cross-check helper functions ----

    function void w_do_xchk(int key, int j);
        automatic int len = wr_out[key][j].len + 1;
        w_xchk_cnt++;
        for (int i = 0; i < len; i++) begin
            if (i < wr_out[key][j].m_w_data.size() && i < wr_out[key][j].s_w_data.size()) begin
                if (wr_out[key][j].m_w_data[i] !== wr_out[key][j].s_w_data[i]) begin
                    w_xchk_err++;
                    `uvm_error("W_XCHK", $sformatf("W data mismatch key=%0d beat=%0d M=0x%08h S=0x%08h",
                        key, i, wr_out[key][j].m_w_data[i], wr_out[key][j].s_w_data[i]))
                end
            end
        end
        if (wr_out[key][j].b_received) begin
            void'(wr_out[key].pop_front());
            if (wr_out[key].size() == 0) begin
                for (int i = 0; i < wr_order.size(); i++)
                    if (wr_order[i] == key) begin wr_order.delete(i); break; end
            end
        end
    endfunction

    function void do_b_xchk(axi_b_item m_t, bit[1:0] s_merged_resp, bit[3:0] s_id, int key);
        b_xchk_cnt++;
        if (m_t.resp !== s_merged_resp) begin b_xchk_err++;
            `uvm_error("B_XCHK", $sformatf("BRESP mismatch key=%0d M=%0d S_merged=%0d",
                      key, m_t.resp, s_merged_resp))
        end
        if (m_t.id !== s_id) begin b_xchk_err++;
            `uvm_error("B_XCHK", $sformatf("BID mismatch key=%0d M=%0d S=%0d",
                      key, m_t.id, s_id))
        end
    endfunction

    function void r_do_xchk(int key, bit[1:0] merged_resp);
        automatic int len = rd_out[key][0].len + 1;
        bit[1:0] m_final_resp;
        r_xchk_cnt++;
        if (m_r_id.exists(key) && s_r_id.exists(key)) begin
            if (m_r_id[key] !== s_r_id[key]) begin
                r_xchk_err++;
                `uvm_error("R_XCHK", $sformatf("RID mismatch key=%0d M=%0d S=%0d",
                          key, m_r_id[key], s_r_id[key]))
            end
        end
        if (rd_out[key][0].m_r_resp.size() > 0) begin
            m_final_resp = rd_out[key][0].m_r_resp[rd_out[key][0].m_r_resp.size()-1];
            if (m_final_resp !== merged_resp) begin
                r_xchk_err++;
                `uvm_error("R_XCHK", $sformatf("Merged RRESP mismatch key=%0d M=%0d S=%0d",
                          key, m_final_resp, merged_resp))
            end
        end
        for (int i = 0; i < len; i++) begin
            if (i < rd_out[key][0].m_r_data.size() && i < rd_out[key][0].s_r_data.size()) begin
                if (rd_out[key][0].m_r_data[i] !== rd_out[key][0].s_r_data[i]) begin
                    r_xchk_err++;
                    `uvm_error("R_XCHK", $sformatf("R data mismatch key=%0d beat=%0d M=0x%08h S=0x%08h",
                        key, i, rd_out[key][0].m_r_data[i], rd_out[key][0].s_r_data[i]))
                end
            end
        end
    endfunction

    function void r_try_xchk(int key);
        if (!r_m_done.exists(key) || !r_m_done[key]) return;
        if (!r_s_got.exists(key) || r_s_got[key] != r_s_exp[key]) return;
        if (rd_out[key].size() == 0) return;
        if (rd_out[key][0].m_r_data.size() != rd_out[key][0].len + 1) return;
        if (rd_out[key][0].s_r_data.size() != rd_out[key][0].len + 1) return;
        r_do_xchk(key, r_s_merge_resp[key]);
        void'(rd_out[key].pop_front());
        if (rd_out[key].size() == 0) begin
            for (int i = 0; i < rd_order.size(); i++)
                if (rd_order[i] == key) begin rd_order.delete(i); break; end
        end
        r_s_exp.delete(key);
        r_s_got.delete(key);
        r_s_merge_resp.delete(key);
        r_m_done.delete(key);
        s_r_id.delete(key);
        m_r_id.delete(key);
    endfunction

    //---- Replay function for buffered S-side W beats ----

    function void replay_slv_w_beat(axi_w_item t);
        int key; bit found;
        if (slv_aw_order[t.slv_id].size() == 0) return;
        key = slv_aw_order[t.slv_id][0];
        found = 0;
        if (wr_out.exists(key)) begin
            foreach (wr_out[key][j]) begin
                if (wr_out[key][j].s_w_data.size() < wr_out[key][j].len + 1) begin
                    wr_out[key][j].s_w_data.push_back(t.data);
                    if (t.last) begin
                        void'(slv_aw_order[t.slv_id].pop_front());
                        if (wr_out[key][j].m_w_data.size() == wr_out[key][j].len + 1)
                            w_do_xchk(key, j);
                    end
                    found = 1;
                    break;
                end
            end
        end
        if (!found) begin
            slv_w_buf[key].push_back(t);
        end
    endfunction
