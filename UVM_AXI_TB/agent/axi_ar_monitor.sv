//=============================================================================
// AR Channel Monitor: master-side + slave-side dual capture
//=============================================================================
class axi_ar_monitor extends uvm_monitor;
    `uvm_component_utils(axi_ar_monitor)
    virtual axi_if vif; int side;
    uvm_analysis_port #(axi_ar_item) ap;
    function new(string name="axi_ar_monitor", uvm_component parent);
        super.new(name,parent); ap=new("ap",this);
    endfunction
    task run_phase(uvm_phase phase);
        if(side>=0) monitor_master_ar(); else monitor_slave_ar();
    endtask
    task monitor_master_ar();
        axi_ar_item t;
        forever begin @(posedge vif.ACLK);
            if(vif.M_ARVALID[side] && vif.M_ARREADY[side]) begin
                t=axi_ar_item::type_id::create("t"); t.is_master_side=1; t.mst_id=side; t.slv_id=-1;
                t.id=vif.M_ARID[side*4+:4]; t.addr=vif.M_ARADDR[side*32+:32];
                t.len=vif.M_ARLEN[side*8+:8]; t.size=vif.M_ARSIZE[side*3+:3];
                t.burst=vif.M_ARBURST[side*2+:2]; t.timestamp=$time; ap.write(t);
                `uvm_info("AR_MON",$sformatf("M[%0d] AR id=%0d addr=0x%08h len=%0d",side,t.id,t.addr,t.len),UVM_HIGH)
            end
        end
    endtask
    task monitor_slave_ar();
        axi_ar_item t; int s;
        forever begin @(posedge vif.ACLK);
            for(s=0; s<4; s++) begin
                if(vif.S_ARVALID[s] && vif.S_ARREADY[s]) begin
                    t=axi_ar_item::type_id::create("t"); t.is_master_side=0; t.slv_id=s;
                    t.mst_id=vif.S_ARID[s*8+6+:2]; t.id=vif.S_ARID[s*8+:4];
                    t.addr=vif.S_ARADDR[s*32+:32]; t.len=vif.S_ARLEN[s*8+:8];
                    t.size=vif.S_ARSIZE[s*3+:3]; t.burst=vif.S_ARBURST[s*2+:2];
                    t.timestamp=$time; ap.write(t);
                    `uvm_info("AR_MON",$sformatf("S[%0d] AR id=%0d mst=%0d addr=0x%08h",s,t.id,t.mst_id,t.addr),UVM_HIGH)
                end
            end
        end
    endtask
endclass
