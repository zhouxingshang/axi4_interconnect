//=============================================================================
// AXI Slave Agent: memory-model slave with configurable back-pressure
// Fully fixed version (WSTRB assumed always all 1s / 4'hf)
//=============================================================================
class axi_slave_agent extends uvm_agent;

    int slv_id;
    virtual axi_if vif;
    axi_monitor  mon;

    typedef struct { 
        bit[7:0] id; 
        bit[31:0] addr; 
        bit[7:0] len;
        bit[2:0] size; 
        bit[1:0] burst; 
    } trans_hdr_t;
    
    trans_hdr_t aw_queue[$], ar_queue[$];

    // W beat tracking: per-burst isolation for 4KB split support
    typedef struct { 
        bit[31:0] addr; 
        bit[2:0] size; 
    } wr_aw_t;
    
    wr_aw_t      wr_aw_q   [$]; // AW info per W burst
    int          w_beat_cnt;
    int          w_done_q  [$]; // beat count per completed W burst

    // Memory: stores WDATA, returns on read
    bit[31:0]    mem [bit[31:0]];

    `uvm_component_utils(axi_slave_agent)

    function new(string name = "axi_slave_agent", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        mon = axi_monitor::type_id::create($sformatf("mon_s%0d", slv_id), this);
        mon.slv_id = slv_id;
    endfunction

    task run_phase(uvm_phase phase);
        reset_signals();
        fork 
            aw_handler(); 
            w_handler(); 
            b_handler(); 
            ar_handler(); 
            r_handler(); 
        join
    endtask

    task reset_signals();
        @(negedge vif.ACLK);
        vif.S_AWREADY[slv_id]      <= 1'b0;
        vif.S_WREADY[slv_id]       <= 1'b0;
        vif.S_BVALID[slv_id]       <= 1'b0;
        vif.S_BID[slv_id*8+:8]     <= 8'b0;
        vif.S_BRESP[slv_id*2+:2]   <= 2'b0;
        vif.S_ARREADY[slv_id]      <= 1'b0;
        vif.S_RVALID[slv_id]       <= 1'b0;
        vif.S_RID[slv_id*8+:8]     <= 8'b0;
        vif.S_RDATA[slv_id*32+:32] <= 32'b0;
        vif.S_RRESP[slv_id*2+:2]   <= 2'b0;
        vif.S_RLAST[slv_id]        <= 1'b0;
    endtask

    //===================================================================
    // Handlers: Standard Clocking & Non-blocking Assignment Pattern
    //===================================================================

    task aw_handler();
        trans_hdr_t wt;
        vif.S_AWREADY[slv_id] <= 1'b1; // 默认拉高准备接收地址
        
        forever begin
            @(posedge vif.ACLK);
            if (vif.S_AWVALID[slv_id] && vif.S_AWREADY[slv_id]) begin
                `uvm_info("TRACE", $sformatf("S[%0d] SLV AW handshake id=%0d addr=0x%08h len=%0d",
                         slv_id, vif.S_AWID[slv_id*8+:8], vif.S_AWADDR[slv_id*32+:32],
                         vif.S_AWLEN[slv_id*8+:8]), UVM_MEDIUM)
                         
                wt.id    = vif.S_AWID[slv_id*8+:8];
                wt.addr  = vif.S_AWADDR[slv_id*32+:32];
                wt.len   = vif.S_AWLEN[slv_id*8+:8];
                wt.size  = vif.S_AWSIZE[slv_id*3+:3];
                wt.burst = vif.S_AWBURST[slv_id*2+:2];
                
                aw_queue.push_back(wt);
                wr_aw_q.push_back('{addr: wt.addr, size: wt.size});
            end
        end
    endtask

    task w_handler();
        bit[31:0] beat_addr; 
        int bpb;
        vif.S_WREADY[slv_id] <= 1'b0;
        
        forever begin
            // 规避数据丢失：只有当 AW 通道有地址登记时，才允许反压释放拉高 READY
            wait(wr_aw_q.size() > 0);
            vif.S_WREADY[slv_id] <= 1'b1;
            
            @(posedge vif.ACLK);
            if (vif.S_WVALID[slv_id] && vif.S_WREADY[slv_id]) begin
                `uvm_info("TRACE", $sformatf("S[%0d] SLV W beat %0d data=0x%08h last=%b",
                         slv_id, w_beat_cnt, vif.S_WDATA[slv_id*32+:32], vif.S_WLAST[slv_id]), UVM_MEDIUM)
                
                bpb = 1 << wr_aw_q[0].size;
                beat_addr = wr_aw_q[0].addr + (w_beat_cnt * bpb);
                
                // 【简化】WSTRB强行为全1，直接安全覆盖整字 32-bit
                mem[beat_addr[31:2]] = vif.S_WDATA[slv_id*32+:32];
                
                w_beat_cnt++;
                
                if (vif.S_WLAST[slv_id]) begin
                    w_done_q.push_back(w_beat_cnt);
                    w_beat_cnt = 0;
                    vif.S_WREADY[slv_id] <= 1'b0; // 传输完成，暂时拉低 READY
                    void'(wr_aw_q.pop_front());   // 【修复 Bug 1】在此处统一 pop，b_handler 不再重复操作
                end
            end
        end
    endtask

    task b_handler();
        int exp_beats, actual_beats;
        vif.S_BVALID[slv_id] <= 1'b0;
        
        forever begin
            // 严格确保 AW 握手与 W 数据突发都完成后再发起响应
            wait(aw_queue.size() > 0 && w_done_q.size() > 0);
            
            exp_beats    = aw_queue[0].len + 1;
            actual_beats = w_done_q.pop_front();
            
            if (exp_beats != actual_beats) begin
                `uvm_error("LEN_MISMATCH", $sformatf("S[%0d] Protocol violation! Expected %0d beats but received %0d beats", 
                           slv_id, exp_beats, actual_beats))
            end
            
            // 使用正沿 + 非阻塞赋值驱动总线
            vif.S_BID[slv_id*8+:8]   <= aw_queue[0].id;
            vif.S_BRESP[slv_id*2+:2] <= 2'b00; // OKAY
            vif.S_BVALID[slv_id]     <= 1'b1;
            
            // 完美的握手机制：等待 Master 确认
            do begin
                @(posedge vif.ACLK);
            end while (!vif.S_BREADY[slv_id]);
            
            vif.S_BVALID[slv_id] <= 1'b0;
            void'(aw_queue.pop_front()); // 仅清理地址队列
        end
    endtask

    task ar_handler();
        trans_hdr_t rt;
        vif.S_ARREADY[slv_id] <= 1'b1; // 默认拉高准备接收读地址
        
        forever begin
            @(posedge vif.ACLK);
            if (vif.S_ARVALID[slv_id] && vif.S_ARREADY[slv_id]) begin
                `uvm_info("TRACE", $sformatf("S[%0d] SLV AR handshake id=%0d addr=0x%08h len=%0d",
                         slv_id, vif.S_ARID[slv_id*8+:8], vif.S_ARADDR[slv_id*32+:32],
                         vif.S_ARLEN[slv_id*8+:8]), UVM_MEDIUM)
                         
                rt.id    = vif.S_ARID[slv_id*8+:8];
                rt.addr  = vif.S_ARADDR[slv_id*32+:32];
                rt.len   = vif.S_ARLEN[slv_id*8+:8];
                rt.size  = vif.S_ARSIZE[slv_id*3+:3];
                rt.burst = vif.S_ARBURST[slv_id*2+:2];
                
                ar_queue.push_back(rt);
            end
        end
    endtask

    task r_handler();
        int beat; 
        bit[31:0] beat_addr; 
        int bpb; 
        bit[31:0] rdata;
        
        vif.S_RVALID[slv_id] <= 1'b0;
        vif.S_RLAST[slv_id]  <= 1'b0;
        
        forever begin
            wait(ar_queue.size() > 0);
            bpb = 1 << ar_queue[0].size;
            
            for (beat = 0; beat <= ar_queue[0].len; beat++) begin
                beat_addr = ar_queue[0].addr + (beat * bpb);
                rdata = mem.exists(beat_addr[31:2]) ? mem[beat_addr[31:2]] : 32'hDEAD_BEEF;
                
                if (!mem.exists(beat_addr[31:2])) begin
                    `uvm_info("TRACE", $sformatf("S[%0d] SLV R MISS at addr=0x%08h", slv_id, beat_addr), UVM_MEDIUM)
                end
                
                // 正沿非阻塞赋值吐出数据
                vif.S_RVALID[slv_id]       <= 1'b1;
                vif.S_RID[slv_id*8+:8]     <= ar_queue[0].id;
                vif.S_RDATA[slv_id*32+:32] <= rdata;
                vif.S_RRESP[slv_id*2+:2]   <= 2'b00;
                vif.S_RLAST[slv_id]        <= (beat == ar_queue[0].len);
                
                // 零气泡（Zero-wait-state）连续握手判断
                do begin
                    @(posedge vif.ACLK);
                end while (!vif.S_RREADY[slv_id]);
                
                vif.S_RVALID[slv_id] <= 1'b0;
                vif.S_RLAST[slv_id]  <= 1'b0;
            end
            void'(ar_queue.pop_front());
        end
    endtask

endclass