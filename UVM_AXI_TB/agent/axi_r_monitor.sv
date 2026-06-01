//=============================================================================
// R Channel Monitor: master-side + slave-side, per-beat R data capture
// Used for reorder detection and data integrity checking
//=============================================================================
class axi_r_monitor extends uvm_monitor;
    `uvm_component_utils(axi_r_monitor)
    virtual axi_if vif; int side;
    uvm_analysis_port #(axi_r_item) ap;
    function new(string name="axi_r_monitor", uvm_component parent);
        super.new(name,parent); ap=new("ap",this);
    endfunction
    task run_phase(uvm_phase phase);
        if(side>=0) monitor_master_r(); else monitor_slave_r();
    endtask
    task monitor_master_r();
        axi_r_item t;
        forever begin @(posedge vif.ACLK);
            if(vif.M_RVALID[side] && vif.M_RREADY[side]) begin
                t=axi_r_item::type_id::create("t"); t.is_master_side=1; t.mst_id=side; t.slv_id=-1;
                t.id=vif.M_RID[side*4+:4]; t.data=vif.M_RDATA[side*32+:32];
                t.resp=vif.M_RRESP[side*2+:2]; t.last=vif.M_RLAST[side]; t.timestamp=$time;
                ap.write(t);
            end
        end
    endtask
    task monitor_slave_r();
        axi_r_item t; int s;
        forever begin @(posedge vif.ACLK);
            for(s=0; s<4; s++) begin
                if(vif.S_RVALID[s] && vif.S_RREADY[s]) begin
                    t=axi_r_item::type_id::create("t"); t.is_master_side=0; t.slv_id=s;
                    t.mst_id=vif.S_RID[s*8+6+:2]; t.id=vif.S_RID[s*8+:4];
                    t.data=vif.S_RDATA[s*32+:32]; t.resp=vif.S_RRESP[s*2+:2];
                    t.last=vif.S_RLAST[s]; t.timestamp=$time; ap.write(t);
                end
            end
        end
    endtask
endclass
