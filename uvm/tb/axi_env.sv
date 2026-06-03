//=============================================================================
// AXI Environment: 4 master agents + 4 slave agents
// Each agent contains a unified 5-channel monitor (AW/W/B/AR/R)
// 3 scoreboards (routing/data/response) + reference model + coverage
//=============================================================================
class axi_env extends uvm_env;
    `uvm_component_utils(axi_env)

    virtual axi_if vif;

    // Agents
    axi_master_agent master_agents[4];
    axi_slave_agent  slave_agents[4];

    // Scoreboards + Reference Model
    routing_sb   r_sb;
    data_sb      d_sb;
    response_sb  resp_sb;
    reference_model     refm;
    axi_cov             cov;
    axi_protocol_checker proto_chk;

    function new(string n="axi_env", uvm_component p); super.new(n,p); endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if(!uvm_config_db #(virtual axi_if)::get(this,"","vif",vif))
            `uvm_fatal("NOVIF","vif not found")

        // Agents (each contains driver + sequencer + unified monitor)
        for(int i=0; i<4; i++) begin
            master_agents[i]=axi_master_agent::type_id::create($sformatf("mst_ag%0d",i),this);
            master_agents[i].mst_id=i;
            slave_agents[i] =axi_slave_agent::type_id::create($sformatf("slv_ag%0d",i),this);
            slave_agents[i].slv_id=i;
        end

        // Scoreboards + Reference Model
        r_sb  =routing_sb::type_id::create("r_sb",this);
        d_sb  =data_sb::type_id::create("d_sb",this);
        resp_sb=response_sb::type_id::create("resp_sb",this);
        refm  =reference_model::type_id::create("refm",this);
        cov      =axi_cov::type_id::create("cov",this);
        proto_chk=axi_protocol_checker::type_id::create("proto_chk",this);
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);

        // Distribute vif to all agents and their monitors
        for(int i=0; i<4; i++) begin
            master_agents[i].drv.vif = vif;
            master_agents[i].mon.vif = vif;
            slave_agents[i].vif = vif;
            slave_agents[i].mon.vif = vif;
        end

        //==== Master-side connections (per agent) ====
        for(int i=0; i<4; i++) begin
            // AW -> routing (mst) + data + response + refm + cov + proto_chk
            master_agents[i].mon.ap_aw.connect(r_sb.mst_aw_ap);
            master_agents[i].mon.ap_aw.connect(d_sb.aw_ap);
            master_agents[i].mon.ap_aw.connect(resp_sb.aw_ap);
            master_agents[i].mon.ap_aw.connect(refm.aw_ap);
            master_agents[i].mon.ap_aw.connect(cov.aw_ap);
            master_agents[i].mon.ap_aw.connect(proto_chk.aw_ap);

            // W -> data + response + proto_chk
            master_agents[i].mon.ap_w.connect(d_sb.w_ap);
            master_agents[i].mon.ap_w.connect(resp_sb.w_ap);
            master_agents[i].mon.ap_w.connect(proto_chk.w_ap);

            // B -> response + cov + proto_chk
            master_agents[i].mon.ap_b.connect(resp_sb.b_ap);
            master_agents[i].mon.ap_b.connect(cov.b_ap);
            master_agents[i].mon.ap_b.connect(proto_chk.b_ap);

            // AR -> routing (mst) + data + response + refm + cov + proto_chk
            master_agents[i].mon.ap_ar.connect(r_sb.mst_ar_ap);
            master_agents[i].mon.ap_ar.connect(d_sb.ar_ap);
            master_agents[i].mon.ap_ar.connect(resp_sb.ar_ap);
            master_agents[i].mon.ap_ar.connect(refm.ar_ap);
            master_agents[i].mon.ap_ar.connect(cov.ar_ap);
            master_agents[i].mon.ap_ar.connect(proto_chk.ar_ap);

            // R -> data + response + cov + proto_chk
            master_agents[i].mon.ap_r.connect(d_sb.r_ap);
            master_agents[i].mon.ap_r.connect(resp_sb.r_ap);
            master_agents[i].mon.ap_r.connect(cov.r_ap);
            master_agents[i].mon.ap_r.connect(proto_chk.r_ap);
        end

        //==== Slave-side connections (per agent) ====
        for(int i=0; i<4; i++) begin
            // AW/AR -> routing (slv port)
            slave_agents[i].mon.ap_aw.connect(r_sb.slv_aw_ap);
            slave_agents[i].mon.ap_ar.connect(r_sb.slv_ar_ap);
            // W -> data sb
            slave_agents[i].mon.ap_w.connect(d_sb.w_ap);
            // B/R -> response sb
            slave_agents[i].mon.ap_b.connect(resp_sb.b_ap);
            slave_agents[i].mon.ap_r.connect(resp_sb.r_ap);
        end
    endfunction

endclass
