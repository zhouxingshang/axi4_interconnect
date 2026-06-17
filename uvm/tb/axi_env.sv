//=============================================================================
// AXI Environment: 4 master agents + 4 slave agents
// Each agent contains a unified 5-channel monitor (AW/W/B/AR/R)
// Unified scoreboard + reference model + coverage
//=============================================================================
class axi_env extends uvm_env;
    `uvm_component_utils(axi_env)

    virtual axi_if vif;

    // Agents
    axi_master_agent master_agents[4];
    axi_slave_agent  slave_agents[4];

    // Virtual Sequencer
    axi_virtual_sequencer vsqr;

    // Scoreboard + Reference Model
    axi_scoreboard      sb;
    reference_model     refm;
    axi_cov             cov;
    axi_protocol_checker proto_chk;

    function new(string n="axi_env", uvm_component p); 
        super.new(n,p); 
    endfunction

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

        // Virtual Sequencer
        vsqr = axi_virtual_sequencer::type_id::create("vsqr", this);

        // Scoreboard + Reference Model
        sb        = axi_scoreboard::type_id::create("sb",this);
        refm      = reference_model::type_id::create("refm",this);
        cov       = axi_cov::type_id::create("cov",this);
        proto_chk = axi_protocol_checker::type_id::create("proto_chk",this);
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);

        // Connect scoreboard to reference model
        sb.refm = refm;

        // Connect master sequencers to virtual sequencer
        for(int i=0; i<4; i++) begin
            vsqr.sqr[i] = master_agents[i].sqr;
        end

        // Distribute vif to all agents and their monitors
        for(int i=0; i<4; i++) begin
            master_agents[i].drv.vif = vif;
            master_agents[i].mon.vif = vif;
            slave_agents[i].vif = vif;
            slave_agents[i].mon.vif = vif;
        end

        //==== Master-side connections (per agent) ====
        for(int i=0; i<4; i++) begin
            // AW -> scoreboard + refm + cov + proto_chk
            master_agents[i].mon.ap_aw.connect(sb.aw_imp);
            master_agents[i].mon.ap_aw.connect(refm.aw_ap);
            master_agents[i].mon.ap_aw.connect(cov.aw_ap);
            master_agents[i].mon.ap_aw.connect(proto_chk.aw_imp);

            // W -> scoreboard + refm + proto_chk
            master_agents[i].mon.ap_w.connect(sb.w_imp);
            master_agents[i].mon.ap_w.connect(refm.w_ap);
            master_agents[i].mon.ap_w.connect(proto_chk.w_imp);

            // B -> scoreboard + cov + proto_chk
            master_agents[i].mon.ap_b.connect(sb.b_imp);
            master_agents[i].mon.ap_b.connect(cov.b_ap);
            master_agents[i].mon.ap_b.connect(proto_chk.b_imp);

            // AR -> scoreboard + refm + cov + proto_chk
            master_agents[i].mon.ap_ar.connect(sb.ar_imp);
            master_agents[i].mon.ap_ar.connect(refm.ar_ap);
            master_agents[i].mon.ap_ar.connect(cov.ar_ap);
            master_agents[i].mon.ap_ar.connect(proto_chk.ar_imp);

            // R -> scoreboard + cov + proto_chk
            master_agents[i].mon.ap_r.connect(sb.r_imp);
            master_agents[i].mon.ap_r.connect(cov.r_ap);
            master_agents[i].mon.ap_r.connect(proto_chk.r_imp);
        end

        //==== Slave-side connections (per agent) ====
        for(int i=0; i<4; i++) begin
            slave_agents[i].mon.ap_aw.connect(sb.slv_aw_imp);
            slave_agents[i].mon.ap_ar.connect(sb.slv_ar_imp);
            slave_agents[i].mon.ap_w.connect(sb.slv_w_imp);
            slave_agents[i].mon.ap_b.connect(sb.slv_b_imp);
            slave_agents[i].mon.ap_r.connect(sb.slv_r_imp);
        end
    endfunction

endclass
