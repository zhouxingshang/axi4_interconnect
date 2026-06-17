//=============================================================================
// AXI Master Driver: AW/W decoupled, Outstanding Supported
//=============================================================================
`ifndef AXI_DRIVER_SV
`define AXI_DRIVER_SV

class axi_driver extends uvm_driver #(axi_transaction);

    virtual axi_if vif;
    int mst_id;

    // Channel FIFOs for decoupling (Size set to 0 for unbounded in build_phase)
    uvm_tlm_fifo #(axi_transaction) aw_fifo;
    uvm_tlm_fifo #(axi_transaction) w_fifo;
    uvm_tlm_fifo #(axi_transaction) ar_fifo;

    // Pending response lists for ID matching
    axi_transaction b_pending[$];
    axi_transaction r_pending[$];

    //===================================================================
    // Outstanding Control Properties
    //===================================================================
    int max_write_outstanding = 4;
    int max_read_outstanding  = 4;

    int write_in_flight = 0;       // current inflight write count
    int read_in_flight  = 0;       // current inflight read count

    `uvm_component_utils(axi_driver)

    function new(string name = "axi_driver", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        // Unbounded depth to prevent FIFO from blocking outstanding
        aw_fifo = new("aw_fifo", this, 0);
        w_fifo  = new("w_fifo", this, 0);
        ar_fifo = new("ar_fifo", this, 0);
    endfunction

    task run_phase(uvm_phase phase);
        reset_signals();
        fork
            get_and_dispatch();    // non-blocking dispatch, throttled by outstanding limit
            drive_aw_channel();    // AW independently
            drive_w_channel();     // W  independently
            receive_b_channel();   // B  async collector
            drive_ar_channel();    // AR independently
            receive_r_channel();   // R  async collector
        join
    endtask

    task reset_signals();
        @(posedge vif.ACLK);
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
    // Main fetch-dispatch loop: non-blocking, throttled by outstanding limit
    //===================================================================
    task get_and_dispatch();
        forever begin
            seq_item_port.get_next_item(req);
            `uvm_info("DRIVER", $sformatf("M[%0d] fetched item: %s", mst_id, req.convert2string()), UVM_HIGH)

            if (req.is_write) begin
                while (write_in_flight >= max_write_outstanding) begin
                    @(posedge vif.ACLK);
                end
                write_in_flight++;
                aw_fifo.put(req);          // hand off to AW thread
                w_fifo.put(req);           // hand off to W thread
            end else begin
                while (read_in_flight >= max_read_outstanding) begin
                    @(posedge vif.ACLK);
                end
                read_in_flight++;
                ar_fifo.put(req);          // hand off to AR thread
            end

            // Release sequencer immediately for next item
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
            b_pending.push_back(t); // hand off to B channel for ID matching
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
                vif.M_WVALID[mst_id]       <= 1'b1;
                vif.M_WDATA[mst_id*32+:32] <= t.data[b];
                vif.M_WSTRB[mst_id*4+:4]   <= t.strb[b];
                vif.M_WLAST[mst_id]        <= (b == t.len) ? 1'b1 : 1'b0;

                `uvm_info("TRACE", $sformatf("M[%0d] DRV W beat %0d/%0d last=%b", mst_id, b, t.len, (b == t.len)), UVM_HIGH)

                do begin
                    @(posedge vif.ACLK);
                end while (!vif.M_WREADY[mst_id]);

                if (b == t.len) begin
                    vif.M_WVALID[mst_id] <= 1'b0;
                    vif.M_WLAST[mst_id]  <= 1'b0;
                end
            end
        end
    endtask

    //===================================================================
    // B Channel: collect response, match by ID, async response
    //===================================================================
    task receive_b_channel();
        axi_transaction rsp;
        bit [3:0] bid;
        int idx;

        vif.M_BREADY[mst_id] <= 1'b1; // keep ready high for best outstanding perf

        forever begin
            @(posedge vif.ACLK);

            if (vif.M_BVALID[mst_id] && vif.M_BREADY[mst_id]) begin
                bid = vif.M_BID[mst_id*4+:4];

                $display("[%0t] M[%0d] DRV B Handshake! BID=%0d b_pending_size=%0d",
                         $time, mst_id, bid, b_pending.size());

                idx = -1;
                foreach (b_pending[i]) begin
                    if (b_pending[i].id == bid) begin idx = i; break; end
                end

                if (idx >= 0) begin
                    b_pending[idx].resp = vif.M_BRESP[mst_id*2+:2];
                    `uvm_info("TRACE", $sformatf("M[%0d] DRV B resp=%0d id=%0d", mst_id, b_pending[idx].resp, bid), UVM_MEDIUM)

                    // Clone and send response back to sequence asynchronously
                    $cast(rsp, b_pending[idx].clone());
                    rsp.set_id_info(b_pending[idx]);
                    seq_item_port.put_response(rsp);

                    b_pending.delete(idx);
                    write_in_flight--; // release one inflight write slot
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

            `uvm_info("TRACE", $sformatf("M[%0d] DRV AR start addr=0x%08h len=%0d id=%0d", mst_id, t.addr, t.len, t.id), UVM_MEDIUM)

            do begin
                @(posedge vif.ACLK);
            end while (!vif.M_ARREADY[mst_id]);

            vif.M_ARVALID[mst_id] <= 1'b0;
            r_pending.push_back(t); // hand off to R channel for data collection
        end
    endtask

    //===================================================================
    // R Channel: collect beats, match by RID, signal done on RLAST
    //===================================================================
    task receive_r_channel();
        axi_transaction rsp;
        int beat_cnt[int];
        bit [3:0] rid;
        int idx;

        vif.M_RREADY[mst_id] <= 1'b1; // keep ready high

        forever begin
            @(posedge vif.ACLK);

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
                        // Full read burst done, clone and send back to sequence
                        $cast(rsp, r_pending[idx].clone());
                        rsp.set_id_info(r_pending[idx]);
                        seq_item_port.put_response(rsp);

                        beat_cnt.delete(rid);
                        r_pending.delete(idx);
                        read_in_flight--; // release one inflight read slot
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
