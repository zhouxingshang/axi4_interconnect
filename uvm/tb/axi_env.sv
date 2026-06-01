//=============================================================================
// AXI Environment: 4 master agents + 4 slave agents
// 5 channel monitors (AW/W/B/AR/R) x 2 sides (master/slave)
// 3 scoreboards (routing/data/response) + reference model + coverage
//=============================================================================
class axi_env extends uvm_env;
    `uvm_component_utils(axi_env)

    // Agents
    axi_master_agent master_agents[4];
    axi_slave_agent  slave_agents[4];

    // Master-side channel monitors (per master: aw/w/b/ar/r)
    axi_aw_monitor aw_m_mon[4]; axi_w_monitor w_m_mon[4]; axi_b_monitor b_m_mon[4];
    axi_ar_monitor ar_m_mon[4]; axi_r_monitor r_m_mon[4];

    // Slave-side channel monitors (one per channel, covers all 4 slaves)
    axi_aw_monitor aw_s_mon; axi_w_monitor w_s_mon; axi_b_monitor b_s_mon;
    axi_ar_monitor ar_s_mon; axi_r_monitor r_s_mon;

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
        // Master agents (each has driver + sequencer, monitor optional)
        for(int i=0; i<4; i++) begin
            master_agents[i]=axi_master_agent::type_id::create($sformatf("mst_ag%0d",i),this);
            master_agents[i].mst_id=i;
            slave_agents[i] =axi_slave_agent::type_id::create($sformatf("slv_ag%0d",i),this);
            slave_agents[i].slv_id=i;
        end

        // Master-side channel monitors (one per master)
        for(int i=0; i<4; i++) begin
            aw_m_mon[i]=axi_aw_monitor::type_id::create($sformatf("aw_m%0d",i),this); aw_m_mon[i].side=i;
            w_m_mon[i] =axi_w_monitor::type_id::create($sformatf("w_m%0d",i),this);  w_m_mon[i].side=i;
            b_m_mon[i] =axi_b_monitor::type_id::create($sformatf("b_m%0d",i),this);  b_m_mon[i].side=i;
            ar_m_mon[i]=axi_ar_monitor::type_id::create($sformatf("ar_m%0d",i),this); ar_m_mon[i].side=i;
            r_m_mon[i] =axi_r_monitor::type_id::create($sformatf("r_m%0d",i),this);  r_m_mon[i].side=i;
        end

        // Slave-side channel monitors (one each, covers all 4 slaves)
        aw_s_mon=axi_aw_monitor::type_id::create("aw_s",this); aw_s_mon.side=-1;
        w_s_mon =axi_w_monitor::type_id::create("w_s",this);  w_s_mon.side=-1;
        b_s_mon =axi_b_monitor::type_id::create("b_s",this);  b_s_mon.side=-1;
        ar_s_mon=axi_ar_monitor::type_id::create("ar_s",this); ar_s_mon.side=-1;
        r_s_mon =axi_r_monitor::type_id::create("r_s",this);  r_s_mon.side=-1;

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

        // Master-side AW -> routing (mst port) + data (addr context) + response (outstanding)
        for(int i=0; i<4; i++) begin
            aw_m_mon[i].ap.connect(r_sb.mst_aw_ap);
            aw_m_mon[i].ap.connect(d_sb.aw_ap);
            aw_m_mon[i].ap.connect(resp_sb.aw_ap);

            w_m_mon[i].ap.connect(d_sb.w_ap);
            w_m_mon[i].ap.connect(resp_sb.w_ap);

            b_m_mon[i].ap.connect(resp_sb.b_ap);

            ar_m_mon[i].ap.connect(r_sb.mst_ar_ap);
            ar_m_mon[i].ap.connect(d_sb.ar_ap);
            ar_m_mon[i].ap.connect(resp_sb.ar_ap);

            r_m_mon[i].ap.connect(d_sb.r_ap);
            r_m_mon[i].ap.connect(resp_sb.r_ap);

            // Connect to reference model
            aw_m_mon[i].ap.connect(refm.aw_ap);
            ar_m_mon[i].ap.connect(refm.ar_ap);
        end

        // Slave-side AW/AR -> routing (slv port)
        aw_s_mon.ap.connect(r_sb.slv_aw_ap);
        ar_s_mon.ap.connect(r_sb.slv_ar_ap);

        // Connect channel monitors to coverage + protocol checker
        for(int i=0; i<4; i++) begin
            aw_m_mon[i].ap.connect(cov.aw_ap);  aw_m_mon[i].ap.connect(proto_chk.aw_ap);
            ar_m_mon[i].ap.connect(cov.ar_ap);  ar_m_mon[i].ap.connect(proto_chk.ar_ap);
            b_m_mon[i].ap.connect(cov.b_ap);    b_m_mon[i].ap.connect(proto_chk.b_ap);
            r_m_mon[i].ap.connect(cov.r_ap);    r_m_mon[i].ap.connect(proto_chk.r_ap);
            w_m_mon[i].ap.connect(proto_chk.w_ap);
        end

        // Slave-side W -> data (verify data arrives at correct slave)
        w_s_mon.ap.connect(d_sb.w_ap);
        // Slave-side B/R -> response (verify slave-side BID/RID)
        b_s_mon.ap.connect(resp_sb.b_ap);
        r_s_mon.ap.connect(resp_sb.r_ap);
    endfunction

endclass
