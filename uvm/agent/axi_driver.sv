//=============================================================================
// AXI Master Driver: AW/W decoupled, B/R blocking for sequence synchronization
//=============================================================================
`ifndef AXI_DRIVER_SV
`define AXI_DRIVER_SV

class axi_driver extends uvm_driver #(axi_transaction);

    virtual axi_if vif;
    int mst_id;

    // Channel FIFOs for decoupling
    uvm_tlm_fifo #(axi_transaction) aw_fifo;
    uvm_tlm_fifo #(axi_transaction) w_fifo;
    uvm_tlm_fifo #(axi_transaction) ar_fifo;

    // Pending response lists for ID matching
    axi_transaction b_pending[$];
    axi_transaction r_pending[$];

    // Synchronization: main thread waits for B/R completion before next item
    int b_done_cnt = 0;  // incremented when B received, decremented when waited
    int r_done_cnt = 0;

    `uvm_component_utils(axi_driver)

    function new(string name = "axi_driver", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        aw_fifo = new("aw_fifo", this);
        w_fifo  = new("w_fifo", this);
        ar_fifo = new("ar_fifo", this);
    endfunction

    task run_phase(uvm_phase phase);
        reset_signals();
        fork
            get_and_dispatch();    // fetch items, push to FIFOs, wait for response, item_done
            drive_aw_channel();    // AW independently
            drive_w_channel();     // W  independently
            receive_b_channel();   // B  collector
            drive_ar_channel();    // AR independently
            receive_r_channel();   // R  collector
        join
    endtask

    task reset_signals();
        @(negedge vif.ACLK);
        // AW channel
        vif.M_AWVALID[mst_id]        = 1'b0;
        vif.M_AWID[mst_id*4+:4]      = 4'b0;
        vif.M_AWADDR[mst_id*32+:32]  = 32'b0;
        vif.M_AWLEN[mst_id*8+:8]     = 8'b0;
        vif.M_AWSIZE[mst_id*3+:3]    = 3'b0;
        vif.M_AWBURST[mst_id*2+:2]   = 2'b0;
        // W channel
        vif.M_WVALID[mst_id]         = 1'b0;
        vif.M_WDATA[mst_id*32+:32]   = 32'b0;
        vif.M_WSTRB[mst_id*4+:4]     = 4'b0;
        vif.M_WLAST[mst_id]          = 1'b0;
        // B channel
        vif.M_BREADY[mst_id]         = 1'b0;
        // AR channel
        vif.M_ARVALID[mst_id]        = 1'b0;
        vif.M_ARID[mst_id*4+:4]      = 4'b0;
        vif.M_ARADDR[mst_id*32+:32]  = 32'b0;
        vif.M_ARLEN[mst_id*8+:8]     = 8'b0;
        vif.M_ARSIZE[mst_id*3+:3]    = 3'b0;
        vif.M_ARBURST[mst_id*2+:2]   = 2'b0;
        // R channel
        vif.M_RREADY[mst_id]         = 1'b0;
    endtask

    //===================================================================
    // Main fetch-dispatch loop: one item at a time, blocked until response
    //===================================================================
    task get_and_dispatch();
        forever begin
            seq_item_port.get_next_item(req);
            `uvm_info("DRIVER", $sformatf("M[%0d] dispatching: %s", mst_id, req.convert2string()), UVM_MEDIUM)

            if (req.is_write) begin
                aw_fifo.put(req);          // hand off to AW thread
                w_fifo.put(req);           // hand off to W thread
                b_done_cnt++;              // expect one B response
                while (b_done_cnt > 0) @(posedge vif.ACLK);  // wait until B received
            end else begin
                ar_fifo.put(req);          // hand off to AR thread
                r_done_cnt++;              // expect one RLAST
                while (r_done_cnt > 0) @(posedge vif.ACLK);  // wait until RLAST received
            end

            seq_item_port.item_done();
        end
    endtask

    //===================================================================
    // AW Channel: decoupled, can send AW ahead of W
    //===================================================================
    task drive_aw_channel();
        axi_transaction t;
        forever begin
            aw_fifo.get(t);

            vif.M_AWID[mst_id*4+:4]     <= t.id;
            vif.M_AWADDR[mst_id*32+:32] <= t.addr;
            vif.M_AWLEN[mst_id*8+:8]    <= t.len;
            vif.M_AWSIZE[mst_id*3+:3]   <= t.size;
            vif.M_AWBURST[mst_id*2+:2]  <= t.burst;
            vif.M_AWVALID[mst_id]       <= 1'b1;

            `uvm_info("TRACE", $sformatf("M[%0d] DRV AW start addr=0x%08h id=%0d", mst_id, t.addr, t.id), UVM_MEDIUM)

            do begin
                @(posedge vif.ACLK);
            end while (!vif.M_AWREADY[mst_id]);

            vif.M_AWVALID[mst_id] <= 1'b0;
            b_pending.push_back(t);
        end
    endtask

    //===================================================================
    // W Channel: decoupled, W beats driven independently from AW
    //===================================================================
    task drive_w_channel();
        axi_transaction t;
        forever begin
            w_fifo.get(t);
            for (int b = 0; b <= t.len; b++) begin
                // 1. 无缝更新当前拍的数据总线
                vif.M_WVALID[mst_id]       <= 1'b1;
                vif.M_WDATA[mst_id*32+:32] <= t.data[b];
                vif.M_WSTRB[mst_id*4+:4]   <= t.strb[b];
                vif.M_WLAST[mst_id]        <= (b == t.len) ? 1'b1 : 1'b0;
                
                `uvm_info("TRACE", $sformatf("M[%0d] DRV W beat %0d/%0d last=%b", mst_id, b, t.len, (b == t.len)), UVM_MEDIUM)
                
                // 2. 仅在这一拍数据的生命周期内，消耗时钟沿等待硬件 READY
                do begin
                    @(posedge vif.ACLK);
                end while (!vif.M_WREADY[mst_id]);
                
                // 3. 运行到这里说明当前拍握手成功。
                // 如果是最后一拍，拉低 VALID/LAST；如果不是，下一轮 loop 会直接把下拍数据覆盖上来
                if (b == t.len) begin
                    vif.M_WVALID[mst_id] <= 1'b0;
                    vif.M_WLAST[mst_id]  <= 1'b0;
                    //@(posedge vif.ACLK);  // ensure deassertion takes effect before next transaction
                end
            end
        end
    endtask

    //===================================================================
    // B Channel: collect response, match by ID, signal done
    //===================================================================
    task receive_b_channel();
        bit [3:0] bid;
        int idx;

        vif.M_BREADY[mst_id] <= 1'b0;

        forever begin
            @(posedge vif.ACLK);
            vif.M_BREADY[mst_id] <= 1'b1;

            if (vif.M_BVALID[mst_id] && vif.M_BREADY[mst_id]) begin
                bid = vif.M_BID[mst_id*4+:4];

                $display("[%0t] M[%0d] DRV BDEBUG: Handshake Succeeded! BID=%0d b_pending_size=%0d",
                         $time, mst_id, bid, b_pending.size());

                idx = -1;
                foreach (b_pending[i]) begin
                    if (b_pending[i].id == bid) begin idx = i; break; end
                end

                if (idx >= 0) begin
                    b_pending[idx].resp = vif.M_BRESP[mst_id*2+:2];
                    `uvm_info("TRACE", $sformatf("M[%0d] DRV B resp=%0d id=%0d", mst_id, b_pending[idx].resp, bid), UVM_MEDIUM)
                    b_pending.delete(idx);
                    b_done_cnt--;
                end else begin
                    `uvm_error("DRV", $sformatf("M[%0d] unexpected B id=%0d", mst_id, bid))
                end
            end
        end
    endtask

    //===================================================================
    // AR Channel: decoupled
    //===================================================================
    task drive_ar_channel();
        axi_transaction t;
        forever begin
            ar_fifo.get(t);
            vif.M_ARID[mst_id*4+:4]     <= t.id;
            vif.M_ARADDR[mst_id*32+:32] <= t.addr;
            vif.M_ARLEN[mst_id*8+:8]    <= t.len;
            vif.M_ARSIZE[mst_id*3+:3]   <= t.size;
            vif.M_ARBURST[mst_id*2+:2]  <= t.burst;
            vif.M_ARVALID[mst_id]       <= 1'b1;
            
            `uvm_info("TRACE", $sformatf("M[%0d] DRV AR start addr=0x%08h len=%0d", mst_id, t.addr, t.len), UVM_MEDIUM)
            
            do begin
                @(posedge vif.ACLK);
            end while (!vif.M_ARREADY[mst_id]);
            
            vif.M_ARVALID[mst_id] <= 1'b0;
            r_pending.push_back(t);
        end
    endtask

    //===================================================================
    // R Channel: collect beats, match by RID, signal done on RLAST
    //===================================================================
    task receive_r_channel();
        int beat_cnt[int];
        bit [3:0] rid;
        int idx;

        vif.M_RREADY[mst_id] <= 1'b0;

        forever begin
            @(posedge vif.ACLK);
            vif.M_RREADY[mst_id] <= 1'b1;

            if (vif.M_RVALID[mst_id] && vif.M_RREADY[mst_id]) begin
                rid = vif.M_RID[mst_id*4+:4];
                idx = -1;

                foreach (r_pending[i]) begin
                    if (r_pending[i].id == rid) begin 
                        idx = i; break; 
                    end
                end

                if (idx >= 0) begin
                    if (!beat_cnt.exists(rid)) 
                        beat_cnt[rid] = 0;

                    r_pending[idx].data[beat_cnt[rid]] = vif.M_RDATA[mst_id*32+:32];
                    r_pending[idx].resp                = vif.M_RRESP[mst_id*2+:2];
                    r_pending[idx].rlast               = vif.M_RLAST[mst_id];

                    if (vif.M_RLAST[mst_id] !== (beat_cnt[rid] == r_pending[idx].len)) begin
                        `uvm_error("PROTOCOL_VIOLATION",
                            $sformatf("M[%0d] RLAST error! beat_cnt=%0d, expected len=%0d, RLAST=%1b",
                            mst_id, beat_cnt[rid], r_pending[idx].len, vif.M_RLAST[mst_id]))
                    end

                    if (vif.M_RLAST[mst_id]) begin
                        beat_cnt.delete(rid);
                        r_pending.delete(idx);
                        r_done_cnt--;
                    end else begin
                        beat_cnt[rid] = beat_cnt[rid] + 1;
                    end

                end else begin
                    `uvm_error("DRV_ERR", $sformatf("M[%0d] unexpected R id=%0d received", mst_id, rid))
                end
            end
        end
    endtask

endclass

`endif
