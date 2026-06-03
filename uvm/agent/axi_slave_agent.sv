//=============================================================================
// AXI Slave Agent: memory-model slave with configurable back-pressure
//=============================================================================
class axi_slave_agent extends uvm_agent;

    int slv_id;
    virtual axi_if vif;
    axi_monitor   mon;

    int aw_ready_delay_min = 0, aw_ready_delay_max = 3;
    int  w_ready_delay_min = 0,  w_ready_delay_max = 3;
    int ar_ready_delay_min = 0, ar_ready_delay_max = 3;
    int  b_valid_delay_min = 0,  b_valid_delay_max = 3;
    int  r_valid_delay_min = 0,  r_valid_delay_max = 3;

    typedef struct { bit[7:0] id; bit[31:0] addr; bit[7:0] len;
                     bit[2:0] size; bit[1:0] burst; } trans_hdr_t;
    trans_hdr_t aw_queue[$], ar_queue[$];

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
        fork aw_handler(); w_handler(); b_handler(); ar_handler(); r_handler(); join
    endtask

    task aw_handler();
        trans_hdr_t wt;
        forever begin
            delay(aw_ready_delay_min, aw_ready_delay_max); 
            vif.S_AWREADY[slv_id] =1; 
            @(posedge vif.ACLK);
            if(vif.S_AWVALID[slv_id] && vif.S_AWREADY[slv_id]) begin
                wt.id=vif.S_AWID[slv_id*8+:8]; 
                wt.addr=vif.S_AWADDR[slv_id*32+:32];
                wt.len=vif.S_AWLEN[slv_id*8+:8]; 
                wt.size=vif.S_AWSIZE[slv_id*3+:3];
                wt.burst=vif.S_AWBURST[slv_id*2+:2]; 
                aw_queue.push_back(wt); 
                vif.S_AWREADY[slv_id] =0;
            end
        end
    endtask

    task w_handler();
        forever begin
            delay(w_ready_delay_min, w_ready_delay_max); 
            vif.S_WREADY[slv_id] =1; 
            @(posedge vif.ACLK);
            if(vif.S_WVALID[slv_id] && vif.S_WREADY[slv_id]) begin
                vif.S_WREADY[slv_id] =0;
            end
        end
    endtask

    task b_handler();
        forever begin
            wait(aw_queue.size()>0);
            delay(b_valid_delay_min, b_valid_delay_max);
            vif.S_BVALID[slv_id] =1; 
            vif.S_BID[slv_id*8+:8] =aw_queue[0].id;
            vif.S_BRESP[slv_id*2+:2] =2'b00; 
            @(posedge vif.ACLK);
            while(!vif.S_BREADY[slv_id]) 
            @(posedge vif.ACLK);
            vif.S_BVALID[slv_id] =0; 
            void'(aw_queue.pop_front());
        end
    endtask

    task ar_handler();
        trans_hdr_t rt;
        forever begin
            delay(ar_ready_delay_min, ar_ready_delay_max); 
            vif.S_ARREADY[slv_id] =1; 
            @(posedge vif.ACLK);
            if(vif.S_ARVALID[slv_id] && vif.S_ARREADY[slv_id]) begin
                rt.id=vif.S_ARID[slv_id*8+:8]; 
                rt.addr=vif.S_ARADDR[slv_id*32+:32];
                rt.len=vif.S_ARLEN[slv_id*8+:8]; 
                rt.size=vif.S_ARSIZE[slv_id*3+:3];
                rt.burst=vif.S_ARBURST[slv_id*2+:2]; 
                ar_queue.push_back(rt); 
                vif.S_ARREADY[slv_id] =0;
            end
        end
    endtask

    task r_handler();
        int beat;
        forever begin
            wait(ar_queue.size()>0);
            for(beat=0; beat<=ar_queue[0].len; beat++) begin
                delay(r_valid_delay_min, r_valid_delay_max);
                vif.S_RVALID[slv_id] =1; 
                vif.S_RID[slv_id*8+:8] =ar_queue[0].id;
                vif.S_RDATA[slv_id*32+:32] =$urandom; 
                vif.S_RRESP[slv_id*2+:2] =2'b00;
                vif.S_RLAST[slv_id] =(beat==ar_queue[0].len); 
                @(posedge vif.ACLK);
                while(!vif.S_RREADY[slv_id]) 
                @(posedge vif.ACLK);
            end
            vif.S_RVALID[slv_id] =0; void'(ar_queue.pop_front());
        end
    endtask

    task delay(int min_d, int max_d);
        int d;
        if(max_d>0) begin 
            d=min_d+($urandom%(max_d-min_d+1)); 
            repeat(d) @(posedge vif.ACLK); 
        end
    endtask

endclass
