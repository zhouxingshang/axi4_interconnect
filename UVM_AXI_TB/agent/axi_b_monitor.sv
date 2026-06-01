//=============================================================================
// B Channel Monitor: captures B from master-side AND slave-side
// Master-side: BID (stripped), BRESP; Slave-side: BID (with mst prefix), BRESP
//=============================================================================
class axi_b_monitor extends uvm_monitor;
    `uvm_component_utils(axi_b_monitor)
    virtual axi_if vif; int side;
    uvm_analysis_port #(axi_b_item) ap;
    function new(string name="axi_b_monitor", uvm_component parent);
        super.new(name,parent); ap=new("ap",this);
    endfunction
    task run_phase(uvm_phase phase);
        if(side>=0) monitor_master_b(); else monitor_slave_b();
    endtask
    task monitor_master_b();
        axi_b_item t;
        forever begin @(posedge vif.ACLK);
            if(vif.M_BVALID[side] && vif.M_BREADY[side]) begin
                t=axi_b_item::type_id::create("t"); t.is_master_side=1;
                t.mst_id=side; t.slv_id=-1;
                t.id=vif.M_BID[side*4+:4]; t.resp=vif.M_BRESP[side*2+:2];
                t.timestamp=$time; ap.write(t);
                `uvm_info("B_MON",$sformatf("M[%0d] B id=%0d resp=%0d",side,t.id,t.resp),UVM_HIGH)
            end
        end
    endtask
    task monitor_slave_b();
        axi_b_item t; int s;
        forever begin @(posedge vif.ACLK);
            for(s=0; s<4; s++) begin
                if(vif.S_BVALID[s] && vif.S_BREADY[s]) begin
                    t=axi_b_item::type_id::create("t"); t.is_master_side=0;
                    t.slv_id=s; t.mst_id=vif.S_BID[s*8+6+:2];
                    t.id=vif.S_BID[s*8+:4]; t.resp=vif.S_BRESP[s*2+:2];
                    t.timestamp=$time; ap.write(t);
                    `uvm_info("B_MON",$sformatf("S[%0d] B id=%0d mst=%0d",s,t.id,t.mst_id),UVM_HIGH)
                end
            end
        end
    endtask
endclass
