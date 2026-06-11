// =============================================================================
// FILE: uvm/env/axi_protocol_checker.sv (重构优化版)
// =============================================================================

// 声明多通道独立的 Analysis Imp 后缀
`uvm_analysis_imp_decl(_aw)
`uvm_analysis_imp_decl(_w)
`uvm_analysis_imp_decl(_b)
`uvm_analysis_imp_decl(_ar)
`uvm_analysis_imp_decl(_r)

class axi_protocol_checker extends uvm_component;
    `uvm_component_utils(axi_protocol_checker)

    // 通道传输订阅端口
    uvm_analysis_imp_aw#(axi_aw_item, axi_protocol_checker) aw_imp;
    uvm_analysis_imp_w#(axi_w_item,   axi_protocol_checker) w_imp;
    uvm_analysis_imp_b#(axi_b_item,   axi_protocol_checker) b_imp;
    uvm_analysis_imp_ar#(axi_ar_item, axi_protocol_checker) ar_imp;
    uvm_analysis_imp_r#(axi_r_item,   axi_protocol_checker) r_imp;

    // -------------------------------------------------------------------------
    // 核心重构数据结构：Master 域隔离表
    // Key (int): mst_id -> 每个 Master 拥有完全独立的专属 FIFO 追踪队列
    // -------------------------------------------------------------------------
    axi_transaction wr_tbl[int][$]; // 写通道挂起事务表
    axi_transaction rd_tbl[int][$]; // 读通道挂起事务表

    // W-before-AW tolerance: buffer W beats that arrive before their AW
    axi_w_item w_pending[int][$];    // key=mst_id, queue of orphan W beats

    // 4KB split tracking: per-master flag for W beats between split sub-transactions
    bit split_pending[int];

    // 性能与错误统计计数器
    int total_aw_count = 0;
    int total_w_count  = 0;
    int total_b_count  = 0;
    int total_ar_count = 0;
    int total_r_count  = 0;
    int error_count    = 0;

    function new(string name="axi_protocol_checker", uvm_component parent=null);
        super.new(name, parent);
        aw_imp = new("aw_imp", this);
        w_imp  = new("w_imp",  this);
        b_imp  = new("b_imp",  this);
        ar_imp = new("ar_imp", this);
        r_imp  = new("r_imp",  this);
    endfunction

    // =========================================================================
    // 1. WRITE ADDRESS CHANNEL (AW)
    // =========================================================================
    virtual function void write_aw(axi_aw_item item);
        axi_transaction tx;
        if (!item.is_master_side) return;   // only track master-side
        total_aw_count++;
        // Clear 4KB split pending when new AW arrives (second sub-transaction)
        split_pending[item.mst_id] = 1'b0;
        
        tx = axi_transaction::type_id::create("tx");
        tx.mst_id        = item.mst_id;
        tx.id            = item.id;
        tx.addr          = item.addr;
        tx.len           = item.len;
        tx.size          = item.size;
        tx.burst         = item.burst;
        tx.is_write      = 1;
        tx.actual_wbeats = 0;

        // 【关键修复】：依据当前事务的 mst_id，精准推入该 Master 专属队列的末尾
        wr_tbl[item.mst_id].push_back(tx);

        // Replay any W beats that arrived before this AW (W-before-AW tolerance)
        if (w_pending.exists(item.mst_id)) begin
            foreach (w_pending[item.mst_id][i])
                write_w(w_pending[item.mst_id][i]);
            w_pending[item.mst_id].delete();
        end

        `uvm_info("PROT_AW", $sformatf("AW Recorded: M[%0d] ID=0x%0x, ADDR=0x%0x, LEN=%0d. Current Master Pending Depth=%0d",
                  item.mst_id, item.id, item.addr, item.len, wr_tbl[item.mst_id].size()), UVM_HIGH)
    endfunction

    // =========================================================================
    // 2. WRITE DATA CHANNEL (W)
    // =========================================================================
    virtual function void write_w(axi_w_item item);
        int m_id; axi_transaction tx;
        if (!item.is_master_side) return;   // only track master-side
        m_id = item.mst_id;
        total_w_count++;

        // 核心校验：检查当前发数据的 Master 旗下是否登记过对应 AW 请求
        if (!wr_tbl.exists(m_id) || wr_tbl[m_id].size() == 0) begin
            // 4KB split: W beats may arrive before second AW
            if (split_pending.exists(m_id) && split_pending[m_id]) begin
                total_w_count--;
                return;
            end
            // W-before-AW: buffer the W beat, process when AW arrives
            w_pending[m_id].push_back(item);
            total_w_count--;
            return;
        end

        // 【关键修复】：根据 AXI4 规范，单个 Master 内部的 W Burst 顺序必须与 AW 严格一致。
        // 直接锁定该 Master 队列的首元素(最老的 AW)，杜绝跨 Master 串包混淆。
        tx = wr_tbl[m_id][0];
        tx.actual_wbeats++;

        // 边界与 WLAST 强一致性校验 (4KB split aware)
        if (item.last) begin
            if (tx.actual_wbeats != (tx.len + 1)) begin
                // Check for 4KB split: sent bytes + remaining would cross 4KB boundary
                int bpb = 1 << tx.size;
                int bytes_sent = tx.actual_wbeats * bpb;
                bit crosses_4k = ((tx.addr[11:0] + bytes_sent) >= 13'h1000);
                if (!crosses_4k) begin
                    `uvm_error("PROT_ERR_WLEN", $sformatf("Protocol Error: M[%0d] WLAST Mismatch! AWLEN expects %0d beats, but got WLAST at beat %0d. (ADDR=0x%0x)",
                               m_id, tx.len + 1, tx.actual_wbeats, tx.addr))
                    error_count++;
                end else begin
                    // Valid 4KB split: mark pending for orphan W beat tolerance
                    split_pending[m_id] = 1'b1;
                end
            end

            // 该笔写事务数据传输彻底结束，安全弹出
            void'(wr_tbl[m_id].pop_front());
            `uvm_info("PROT_W_DONE", $sformatf("W Burst completed successfully for M[%0d]. Remaining pending AW=%0d", m_id, wr_tbl[m_id].size()), UVM_HIGH)
        end
        else begin
            // 异常校验：未拉高 WLAST 但计数已超额 (4KB split aware)
            if (tx.actual_wbeats >= (tx.len + 1)) begin
                int bpb = 1 << tx.size;
                int bytes_sent = tx.actual_wbeats * bpb;
                bit crosses_4k = ((tx.addr[11:0] + bytes_sent) >= 13'h1000);
                if (!crosses_4k) begin
                    `uvm_error("PROT_ERR_WLEN_OVER", $sformatf("Protocol Error: M[%0d] Missing WLAST! Received %0d beats, which already reaches/exceeds AWLEN=%0d. (ADDR=0x%0x)",
                               m_id, tx.actual_wbeats, tx.len, tx.addr))
                    error_count++;
                end
            end
        end
    endfunction

    // =========================================================================
    // 3. WRITE RESPONSE CHANNEL (B)
    // =========================================================================
    virtual function void write_b(axi_b_item item);
        total_b_count++;
        // 可根据实际需要添加 BRESP 逻辑校验
    endfunction

    // =========================================================================
    // 4. READ ADDRESS CHANNEL (AR)
    // =========================================================================
    virtual function void write_ar(axi_ar_item item);
        axi_transaction tx;
        if (!item.is_master_side) return;   // only track master-side
        total_ar_count++;
        
        tx = axi_transaction::type_id::create("tx");
        tx.mst_id        = item.mst_id;
        tx.id            = item.id;
        tx.addr          = item.addr;
        tx.len           = item.len;
        tx.size          = item.size;
        tx.burst         = item.burst;
        tx.is_write      = 0;
        tx.actual_rbeats = 0;

        // 同步实施读通道的 Master 域隔离
        rd_tbl[item.mst_id].push_back(tx);
    endfunction

    // =========================================================================
    // 5. READ DATA CHANNEL (R)
    // =========================================================================
    virtual function void write_r(axi_r_item item);
        int m_id; bit id_found;
        if (!item.is_master_side) return;   // only track master-side
        m_id = item.mst_id;
        id_found = 0;
        total_r_count++;

        if (!rd_tbl.exists(m_id) || rd_tbl[m_id].size() == 0) begin
            `uvm_error("PROT_ERR_ORPHAN_R", $sformatf("Protocol Violation: R Beat detected on M[%0d] but no matching AR request exists!", m_id))
            error_count++;
            return;
        end

        // 【拓展修复】：AXI 标准允许单个 Master 内部不同 ARID 的读数据发生交叉交织（Interleaving）。
        // 因此，在当前 Master 的局部专属队列中检索匹配 ID 的事务。
        foreach (rd_tbl[m_id][i]) begin
            if (rd_tbl[m_id][i].id == item.id) begin
                axi_transaction tx = rd_tbl[m_id][i];
                tx.actual_rbeats++;
                id_found = 1;

                if (item.last) begin
                    if (tx.actual_rbeats != (tx.len + 1)) begin
                        `uvm_error("PROT_ERR_RLEN", $sformatf("Protocol Error: M[%0d] RLAST Mismatch for ID=0x%0x! ARLEN expects %0d beats, but got RLAST at beat %0d.", 
                                   m_id, tx.id, tx.len + 1, tx.actual_rbeats))
                        error_count++;
                    end
                    // 读事务全包收齐，将该事务从局部队列中注销
                    rd_tbl[m_id].delete(i);
                end 
                else begin
                    if (tx.actual_rbeats >= (tx.len + 1)) begin
                        `uvm_error("PROT_ERR_RLEN_OVER", $sformatf("Protocol Error: M[%0d] Missing RLAST for ID=0x%0x! Received %0d beats, which already reaches/exceeds ARLEN=%0d.", 
                                   m_id, tx.id, tx.actual_rbeats, tx.len))
                        error_count++;
                    end
                end
                break; // 找到目标，跳出局部循环
            end
        end

        if (!id_found) begin
            `uvm_error("PROT_ERR_RID_MISMATCH", $sformatf("Protocol Error: M[%0d] Received R Beat with ID=0x%0x, but no matching pending AR ID found in this Master's tracker!", m_id, item.id))
            error_count++;
        end
    endfunction

    // =========================================================================
    // 报告阶段：打印仿真最终的协议检测统计
    // =========================================================================
    virtual function void report_phase(uvm_phase phase);
        `uvm_info("PROT_CHKER_SUMMARY", $sformatf("\n==================================================\n  AXI PROTOCOL CHECKER仿真报告:\n  AW次数: %0d | W次数: %0d | B次数: %0d\n  AR次数: %0d | R次数: %0d\n  总计发现协议违例错误(Error): %0d\n==================================================", 
                  total_aw_count, total_w_count, total_b_count, total_ar_count, total_r_count, error_count), UVM_LOW)
    endfunction

endclass