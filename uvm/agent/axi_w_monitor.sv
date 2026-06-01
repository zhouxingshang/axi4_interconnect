//=============================================================================
// W Channel Monitor: master-side + slave-side, per-beat data capture
//=============================================================================
class axi_w_monitor extends uvm_monitor;
    `uvm_component_utils(axi_w_monitor)
    virtual axi_if vif;
    int side;
    uvm_analysis_port #(axi_w_item) ap;
    function new(string name="axi_w_monitor", uvm_component parent);
        super.new(name,parent); ap=new("ap",this);
    endfunction
    task run_phase(uvm_phase phase);
        if(side>=0) monitor_master_w(); else monitor_slave_w();
    endtask
    task monitor_master_w();
        axi_w_item t;
        forever begin
            @(posedge vif.ACLK);
            if(vif.M_WVALID[side] && vif.M_WREADY[side]) begin
                t=axi_w_item::type_id::create("t");
                t.is_master_side=1; t.mst_id=side; t.slv_id=-1;
                t.data=vif.M_WDATA[side*32+:32]; t.strb=vif.M_WSTRB[side*4+:4];
                t.last=vif.M_WLAST[side]; t.timestamp=$time; ap.write(t);
            end
        end
    endtask
    task monitor_slave_w();
        axi_w_item t; int s;
        forever begin
            @(posedge vif.ACLK);
            for(s=0; s<4; s++) begin
                if(vif.S_WVALID[s] && vif.S_WREADY[s]) begin
                    t=axi_w_item::type_id::create("t");
                    t.is_master_side=0; t.slv_id=s; t.mst_id=-1;
                    t.data=vif.S_WDATA[s*32+:32]; t.strb=vif.S_WSTRB[s*4+:4];
                    t.last=vif.S_WLAST[s]; t.timestamp=$time; ap.write(t);
                end
            end
        end
    endtask
endclass
