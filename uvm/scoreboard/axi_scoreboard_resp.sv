    //=======================================================================
    // 4) Response checking: BRESP/RRESP validation, beat count tracking
    //=======================================================================

    task process_aw_resp();
        axi_aw_item t; int key; wr_out_t entry;
        forever begin
            aw_p_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            entry.mst_id=t.mst_id;
            entry.id=t.id;
            entry.len=t.len;
            entry.w_beat_cnt=0;
            entry.b_received=0;
            wr_out[key].push_back(entry);
            wr_order.push_back(key);
            wr_cnt++;
            if (slv_w_buf.exists(key)) begin
                foreach (slv_w_buf[key][i])
                    replay_slv_w_beat(slv_w_buf[key][i]);
                slv_w_buf[key].delete();
            end
            if (b_buf.exists(key)) begin
                foreach (b_buf[key][i])
                    replay_b_item(b_buf[key][i]);
                b_buf[key].delete();
            end
        end
    endtask

    task process_w_resp();
        axi_w_item t; int key; bit found;
        forever begin
            w_p_fifo.get(t);
            if(wr_order.size() == 0) continue;
            found=0;
            foreach(wr_order[i]) begin
                key=wr_order[i];
                foreach(wr_out[key][j]) begin
                    if(wr_out[key][j].mst_id == t.mst_id && wr_out[key][j].w_beat_cnt <= wr_out[key][j].len) begin
                        wr_out[key][j].w_beat_cnt++;
                        wr_out[key][j].m_w_data.push_back(t.data);
                        if(t.last && wr_out[key][j].w_beat_cnt != wr_out[key][j].len+1) begin
                            if(wr_out[key][j].w_beat_cnt < wr_out[key][j].len+1) begin
                                wr_out[key][j].len = wr_out[key][j].len - wr_out[key][j].w_beat_cnt;
                                wr_out[key][j].w_beat_cnt = 0;
                            end else begin
                                resp_err++;
                                `uvm_error("RESP",$sformatf("WLAST mismatch M[%0d] id=%0d",
                                          wr_out[key][j].mst_id,wr_out[key][j].id))
                            end
                        end
                        if(t.last && wr_out[key][j].s_w_data.size() == wr_out[key][j].m_w_data.size())
                            w_do_xchk(key, j);
                        if (t.last && b_buf.exists(key)) begin
                            foreach (b_buf[key][k])
                                replay_b_item(b_buf[key][k]);
                            b_buf[key].delete();
                        end
                        found=1; break;
                    end
                end
                if(found) break;
            end
            if(!found) begin resp_err++;
                `uvm_error("RESP","W beat with no matching outstanding AW")
            end
        end
    endtask

    task process_b_resp();
        axi_b_item t; int key;
        forever begin
            b_p_fifo.get(t);
            key = {t.mst_id[1:0], t.id[3:0]};
            if(wr_out[key].size() == 0) begin
                b_buf[key].push_back(t);
                `uvm_info("RESP", $sformatf("B buffered M[%0d] id=%0d (AW not ready)", t.mst_id, t.id), UVM_HIGH)
                continue;
            end
            wr_out[key][0].b_received=1;
            if(t.resp!=2'b00) begin
                resp_err++;
                `uvm_error("RESP",$sformatf("BRESP error M[%0d] id=%0d resp=%b",t.mst_id,t.id,t.resp))
            end
            if(wr_out[key][0].w_beat_cnt < wr_out[key][0].len+1) begin
                b_buf[key].push_back(t);
                `uvm_info("RESP", $sformatf("B buffered M[%0d] id=%0d (W not done, w_beat=%0d)", t.mst_id, t.id, wr_out[key][0].w_beat_cnt), UVM_HIGH)
                continue;
            end
            if (b_s_got.exists(key) && b_s_got[key] == b_s_exp[key]) begin
                do_b_xchk(t, b_s_mresp[key], b_s_id[key], key);
            end else begin
                b_m_pending[key] = t;
            end
            b_order.push_back(key);
            if (wr_out[key][0].s_w_data.size() == wr_out[key][0].len + 1) begin
                void'(wr_out[key].pop_front());
                if(wr_out[key].size() == 0) begin
                    for(int i=0; i<wr_order.size(); i++)
                        if(wr_order[i]==key) begin wr_order.delete(i); break; end
                end
            end
        end
    endtask

    task process_ar_resp();
        axi_ar_item t; int key; rd_out_t entry;
        forever begin
            ar_p_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {t.mst_id[1:0], t.id[3:0]};
            entry.mst_id=t.mst_id;
            entry.id=t.id;
            entry.len=t.len;
            entry.r_beat_cnt=0;
            rd_out[key].push_back(entry);
            rd_order.push_back(key); r_seen[key] = 1; rd_cnt++;
        end
    endtask

    task process_r_resp();
        axi_r_item t; int key;
        forever begin
            r_p_fifo.get(t);
            key = {t.mst_id[1:0], t.id[3:0]};
            if(rd_out[key].size() == 0) begin
                if (!r_seen.exists(key)) begin
                    resp_err++;
                    `uvm_error("RESP",$sformatf("RID mismatch M[%0d] id=%0d",t.mst_id,t.id))
                end
                continue;
            end
            rd_out[key][0].r_beat_cnt++;
            if(t.resp!=2'b00) begin
                resp_err++;
                `uvm_error("RESP",$sformatf("RRESP error M[%0d] id=%0d resp=%b",t.mst_id,t.id,t.resp))
            end
            if(t.last && rd_out[key][0].r_beat_cnt != rd_out[key][0].len+1) begin
                resp_err++;
                `uvm_error("RESP",$sformatf("RLAST mismatch M[%0d] id=%0d",t.mst_id,t.id))
            end
            if(rd_order.size()>0 && rd_order[0]!=key) reorder_cnt++;
            if(t.last) begin
            end
        end
    endtask

    //---- Replay function for buffered B items ----

    function void replay_b_item(axi_b_item t);
        int key; key = {t.mst_id[1:0], t.id[3:0]};
        if (wr_out[key].size() == 0) begin
            b_buf[key].push_back(t);
            return;
        end
        if (wr_out[key][0].w_beat_cnt < wr_out[key][0].len + 1) begin
            b_buf[key].push_back(t);
            return;
        end
        wr_out[key][0].b_received = 1;
        if (t.resp != 2'b00) begin resp_err++;
            `uvm_error("RESP", $sformatf("BRESP error M[%0d] id=%0d resp=%b", t.mst_id, t.id, t.resp))
        end
        if (b_s_got.exists(key) && b_s_got[key] == b_s_exp[key]) begin
            do_b_xchk(t, b_s_mresp[key], b_s_id[key], key);
        end else begin
            b_m_pending[key] = t;
        end
        b_order.push_back(key);
        if (wr_out[key][0].s_w_data.size() == wr_out[key][0].len + 1) begin
            void'(wr_out[key].pop_front());
            if (wr_out[key].size() == 0) begin
                for (int i = 0; i < wr_order.size(); i++)
                    if (wr_order[i] == key) begin wr_order.delete(i); break; end
            end
        end
    endfunction
