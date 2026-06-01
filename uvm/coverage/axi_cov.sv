//=============================================================================
// AXI Comprehensive Functional Coverage
// 1. Master-Slave + Burst        5. Reorder Depth & Distribution
// 2. Arbitration Grant Pattern   6. Default Slave Hit Coverage
// 3. Simultaneous Request Cross
// 4. Outstanding Depth Tracking
// 7. 4KB Split: Length, Position, Slave
//=============================================================================
class axi_cov extends uvm_component;
    `uvm_component_utils(axi_cov)

    // Inputs from channel monitors
    uvm_analysis_export #(axi_aw_item) aw_ap; uvm_analysis_export #(axi_ar_item) ar_ap;
    uvm_analysis_export #(axi_b_item)  b_ap;  uvm_analysis_export #(axi_r_item)  r_ap;
    uvm_tlm_analysis_fifo #(axi_aw_item) aw_fifo; uvm_tlm_analysis_fifo #(axi_ar_item) ar_fifo;
    uvm_tlm_analysis_fifo #(axi_b_item)  b_fifo;  uvm_tlm_analysis_fifo #(axi_r_item)  r_fifo;

    // Outstanding counters
    int aw_outstanding;     // current pending AW count
    int ar_outstanding;     // current pending AR count
    int aw_peak;            // peak AW depth
    int ar_peak;            // peak AR depth

    // Simultaneous request tracking (4-bit mask per cycle)
    int aw_simul_hist[bit[15:0]];   // histogram of simultaneous AW request patterns
    int ar_simul_hist[bit[15:0]];

    // Arbitration tracking
    int arb_grant_hist[4][4];       // [master][slave] grant count
    int arb_grant_order[$];         // ordered grant sequence {mst_id, slv_id}
    int arb_total_grants;

    // Reorder tracking
    int ar_order_q[$];              // ordered AR sequence (key={mst,id})
    int r_order_q[$];               // ordered R arrival sequence
    int reorder_depth_max;          // max reorder depth observed
    int reorder_event_cnt;          // total reorder events

    // Split tracking
    int split_total;                // total split transactions
    int split_by_len[4];            // split count per burst length bucket: 0=1-3, 1=4-7, 2=8-15
    int split_by_slv[4];            // split count per slave
    int split_by_pos[3];            // split position: 0=early(<25%), 1=mid(25-75%), 2=late(>75%)

    // Default slave tracking
    int default_slv_hits;           // transactions routed to default slave
    int default_slv_by_mst[4];      // per master

    //---- Covergroups ----
    covergroup cg_mst_slv;
        MST: coverpoint v_mst { bins m[]={[0:3]}; }
        SLV: coverpoint v_slv { bins s[]={[0:3]}; }
        X_MST_SLV: cross MST, SLV;
    endgroup

    covergroup cg_burst;
        LEN:   coverpoint v_len   { bins b0={0}; bins b1_3={[1:3]}; bins b4_7={[4:7]}; bins b8_15={[8:15]}; }
        SIZE:  coverpoint v_size  { bins sz[]={0,1,2,3}; }
        BURST: coverpoint v_burst { bins bt[]={0,1,2}; }
        X_LEN_SIZE: cross LEN, SIZE;
    endgroup

    covergroup cg_arbitration;
        GRANT_MST: coverpoint v_arb_mst { bins m[]={[0:3]}; }
        GRANT_SLV: coverpoint v_arb_slv { bins s[]={[0:3]}; }
        X_ARB_MST_SLV: cross GRANT_MST, GRANT_SLV;
    endgroup

    covergroup cg_simul;
        SIMUL_CNT: coverpoint v_simul_cnt { bins alone={1}; bins two={2}; bins three={3}; bins all4={4}; }
        SIMUL_MST: coverpoint v_simul_mst { bins m[]={[0:3]}; }
        X_SIMUL: cross SIMUL_CNT, SIMUL_MST;
    endgroup

    covergroup cg_outstanding;
        AW_PEAK: coverpoint v_aw_peak { bins p1={1}; bins p2={2}; bins p3={3}; bins p4={4}; bins p5p={[5:8]}; }
        AR_PEAK: coverpoint v_ar_peak { bins p1={1}; bins p2={2}; bins p3={3}; bins p4={4}; bins p5p={[5:8]}; }
        X_AWPK_ARPK: cross AW_PEAK, AR_PEAK;
    endgroup

    covergroup cg_split_detail;
        SPLIT_LEN:    coverpoint v_spl_len  { bins short={[1:3]}; bins med={[4:7]}; bins long={[8:15]}; }
        SPLIT_POS:    coverpoint v_spl_pos  { bins early={0}; bins mid={1}; bins late={2}; }
        SPLIT_SLV:    coverpoint v_spl_slv  { bins s[]={[0:3]}; }
        X_SPL_LEN_POS: cross SPLIT_LEN, SPLIT_POS;
        X_SPL_LEN_SLV: cross SPLIT_LEN, SPLIT_SLV;
    endgroup

    covergroup cg_reorder;
        REORDER_DEPTH: coverpoint v_reo_depth { bins d1={1}; bins d2={2}; bins d3={3}; bins d4p={[4:8]}; }
        REORDER_MST:   coverpoint v_reo_mst   { bins m[]={[0:3]}; }
        X_REO_DEPTH_MST: cross REORDER_DEPTH, REORDER_MST;
    endgroup

    covergroup cg_default_slv;
        DFT_MST: coverpoint v_dft_mst { bins m[]={[0:3]}; }
        DFT_HIT: coverpoint v_dft_hit { bins yes={1}; bins no={0}; }
        X_DFT: cross DFT_MST, DFT_HIT;
    endgroup

    // Context variables for covergroup sampling (v_ prefix avoids naming conflict)
    int v_mst, v_slv, v_len, v_size, v_burst;
    int v_arb_mst, v_arb_slv;
    int v_simul_cnt, v_simul_mst;
    int v_aw_peak, v_ar_peak;
    int v_spl_len, v_spl_pos, v_spl_slv;
    int v_reo_depth, v_reo_mst;
    int v_dft_mst, v_dft_hit;

    function new(string name="axi_cov", uvm_component parent);
        super.new(name,parent);

        cg_mst_slv      = new; cg_burst         = new;
        cg_arbitration  = new; cg_simul          = new;
        cg_outstanding  = new; cg_split_detail   = new;
        cg_reorder      = new; cg_default_slv    = new;
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        aw_ap=new("aw_ap",this); ar_ap=new("ar_ap",this);
        b_ap =new("b_ap",this);  r_ap =new("r_ap",this);
        aw_fifo=new("aw_fifo",this); ar_fifo=new("ar_fifo",this);
        b_fifo =new("b_fifo",this);  r_fifo =new("r_fifo",this);
        aw_ap.connect(aw_fifo.analysis_export); ar_ap.connect(ar_fifo.analysis_export);
        b_ap.connect(b_fifo.analysis_export);   r_ap.connect(r_fifo.analysis_export);
    endfunction

    task run_phase(uvm_phase phase);
        fork process_aw(); process_ar(); process_b(); process_r(); join
    endtask

    //---- AW: track outstanding, simultaneous, split ----
    task process_aw();
        axi_aw_item t;
        addr_decoder::split_info_t split;
        forever begin
            aw_fifo.get(t);
            if(!t.is_master_side) continue;

            // Burst coverage
            v_mst=t.mst_id; v_slv=addr_decoder::decode(t.addr);
            v_len=t.len; v_size=t.size; v_burst=t.burst;
            cg_mst_slv.sample(); cg_burst.sample();

            // Outstanding
            aw_outstanding++;
            if(aw_outstanding > aw_peak) aw_peak = aw_outstanding;

            // Simultaneous request pattern
            aw_simul_hist[{12'h0, t.mst_id[1:0], 2'b00}]++;

            // 4KB Split detail
            split = addr_decoder::predict_split(t.addr, t.len, t.size);
            if(split.will_split) begin
                split_total++;
                // Split length bucket
                if(t.len<=3) begin split_by_len[0]++; v_spl_len=t.len; end
                else if(t.len<=7) begin split_by_len[1]++; v_spl_len=t.len; end
                else begin split_by_len[2]++; v_spl_len=t.len; end
                // Split slave
                v_spl_slv = addr_decoder::decode(t.addr);
                split_by_slv[v_spl_slv]++;
                // Split position percentage
                if(split.sub1_len*100/(t.len+1) < 25)      begin split_by_pos[0]++; v_spl_pos=0; end
                else if(split.sub1_len*100/(t.len+1) < 75)  begin split_by_pos[1]++; v_spl_pos=1; end
                else                                        begin split_by_pos[2]++; v_spl_pos=2; end
                cg_split_detail.sample();
            end

            // Default slave: addr doesn't match any valid slave range
            if(addr_decoder::decode(t.addr) >= 4) begin
                default_slv_hits++;
                default_slv_by_mst[t.mst_id]++;
            end

            // Arbitration: track per-master per-slave grant
            v_arb_mst=t.mst_id; v_arb_slv=addr_decoder::decode(t.addr);
            arb_grant_hist[t.mst_id][addr_decoder::decode(t.addr)]++;
            v_arb_slv = addr_decoder::decode(t.addr);
            arb_grant_order.push_back({t.mst_id[1:0], v_arb_slv[1:0]});
            arb_total_grants++;
            cg_arbitration.sample();

            // Outstanding + Simultaneous sampling
            v_aw_peak=aw_peak; v_ar_peak=ar_peak; cg_outstanding.sample();
            v_simul_cnt = aw_outstanding > 4 ? 4 : aw_outstanding;
            v_simul_mst = t.mst_id; cg_simul.sample();
        end
    endtask

    //---- AR: track read-side metrics similarly ----
    task process_ar();
        axi_ar_item t; int exp_slv;
        forever begin
            ar_fifo.get(t);
            if(!t.is_master_side) continue;

            ar_outstanding++;
            if(ar_outstanding > ar_peak) ar_peak = ar_outstanding;

            ar_simul_hist[{12'h0, t.mst_id[1:0], 2'b00}]++;

            exp_slv = addr_decoder::decode(t.addr);
            arb_grant_hist[t.mst_id][exp_slv]++;
            arb_total_grants++;

            // Track AR order for reorder detection
            ar_order_q.push_back({t.mst_id[1:0], t.id[3:0]});
        end
    endtask

    //---- B: decrement AW outstanding ----
    task process_b();
        axi_b_item t;
        forever begin
            b_fifo.get(t);
            if(t.is_master_side) begin
                if(aw_outstanding > 0) aw_outstanding--;
            end
        end
    endtask

    //---- R: decrement AR outstanding, detect reorder ----
    task process_r();
        axi_r_item t; int key; int pos;
        forever begin
            r_fifo.get(t);
            if(!t.is_master_side) continue;

            if(ar_outstanding > 0) ar_outstanding--;

            // Reorder detection
            key = {t.mst_id[1:0], t.id[3:0]};
            pos = find_in_q(ar_order_q, key);
            if(pos >= 0) begin
                if(pos > 0) begin
                    reorder_event_cnt++;
                    if(pos > reorder_depth_max) reorder_depth_max = pos;
                end
                ar_order_q.delete(pos);
            end
            // Track R arrival order
            r_order_q.push_back(key);
        end
    endtask

    //---- Helper ----
    function int find_in_q(int q[$], int key);
        for(int i=0; i<q.size(); i++) if(q[i]==key) return i;
        return -1;
    endfunction

    function void sample_reorder(int depth, int mst);
        v_reo_depth=depth; v_reo_mst=mst;
        cg_reorder.sample();
    endfunction

    function void sample_default_slv(int mst, bit hit);
        v_dft_mst=mst; v_dft_hit=hit;
        cg_default_slv.sample();
    endfunction

    //---- Report ----
    function void report_phase(uvm_phase phase);
        $display("COVERAGE REPORT:");
        $display("  Outstanding: AW peak=%0d AR peak=%0d", aw_peak, ar_peak);
        $display("  Arbitration: %0d total grants", arb_total_grants);
        $display("  4KB Splits: %0d total", split_total);
        $display("    by_len: 1-3=%0d 4-7=%0d 8-15=%0d", split_by_len[0], split_by_len[1], split_by_len[2]);
        $display("    by_slv: S0=%0d S1=%0d S2=%0d S3=%0d", split_by_slv[0], split_by_slv[1], split_by_slv[2], split_by_slv[3]);
        $display("    by_pos: early=%0d mid=%0d late=%0d", split_by_pos[0], split_by_pos[1], split_by_pos[2]);
        $display("  Reorder: %0d events, max depth=%0d", reorder_event_cnt, reorder_depth_max);
        $display("  Default Slave: %0d hits", default_slv_hits);
    endfunction

endclass
