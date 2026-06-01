//=============================================================================
// AW Channel Monitor: master-side + slave-side dual capture for routing check
//=============================================================================
class axi_aw_monitor extends uvm_monitor;
    `uvm_component_utils(axi_aw_monitor)

    virtual axi_if vif;
    int side;  // 0..3 = master-side, -1 = slave-side (all 4 slaves)
    uvm_analysis_port #(axi_aw_item) ap;

    function new(string name="axi_aw_monitor", uvm_component parent);
        super.new(name,parent); ap=new("ap",this);
    endfunction

    task run_phase(uvm_phase phase);
        if(side>=0) monitor_master_aw(); else monitor_slave_aw();
    endtask

    task monitor_master_aw();
        axi_aw_item t;
        forever begin
            @(posedge vif.ACLK);
            if(vif.M_AWVALID[side] && vif.M_AWREADY[side]) begin
                t = axi_aw_item::type_id::create("t");
                t.is_master_side=1; t.mst_id=side; t.slv_id=-1;
                t.id=vif.M_AWID[side*4+:4]; t.addr=vif.M_AWADDR[side*32+:32];
                t.len=vif.M_AWLEN[side*8+:8]; t.size=vif.M_AWSIZE[side*3+:3];
                t.burst=vif.M_AWBURST[side*2+:2]; t.timestamp=$time;
                ap.write(t);
            end
        end
    endtask

    task monitor_slave_aw();
        axi_aw_item t; int s;
        forever begin
            @(posedge vif.ACLK);
            for(s=0; s<4; s++) begin
                if(vif.S_AWVALID[s] && vif.S_AWREADY[s]) begin
                    t = axi_aw_item::type_id::create("t");
                    t.is_master_side=0; t.slv_id=s;
                    t.mst_id=vif.S_AWID[s*8+6+:2];  // upper 2 bits = mst index
                    t.id=vif.S_AWID[s*8+:4];        // lower 4 bits = original ID
                    t.addr=vif.S_AWADDR[s*32+:32]; t.len=vif.S_AWLEN[s*8+:8];
                    t.size=vif.S_AWSIZE[s*3+:3]; t.burst=vif.S_AWBURST[s*2+:2];
                    t.timestamp=$time; ap.write(t);
                end
            end
        end
    endtask
endclass
