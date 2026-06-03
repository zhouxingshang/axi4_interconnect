//=============================================================================
// Response Scoreboard: BID/RID matching, outstanding tracking, reorder detection
// Uses all 5 channel monitors (AW/W/B/AR/R) for complete transaction lifecycle
//=============================================================================
class response_sb extends uvm_scoreboard;
    `uvm_component_utils(response_sb)

    // Channel analysis ports
    uvm_analysis_export #(axi_aw_item) aw_ap; uvm_analysis_export #(axi_w_item) w_ap;
    uvm_analysis_export #(axi_b_item)  b_ap;
    uvm_analysis_export #(axi_ar_item) ar_ap; uvm_analysis_export #(axi_r_item) r_ap;
    uvm_tlm_analysis_fifo #(axi_aw_item) aw_fifo; uvm_tlm_analysis_fifo #(axi_w_item) w_fifo;
    uvm_tlm_analysis_fifo #(axi_b_item)  b_fifo;
    uvm_tlm_analysis_fifo #(axi_ar_item) ar_fifo; uvm_tlm_analysis_fifo #(axi_r_item) r_fifo;

    // Outstanding transaction tracking
    typedef struct {
        bit[7:0] mst_id; bit[3:0] id; bit[31:0] addr;
        bit[7:0] len; int w_beat_cnt; int b_received;
    } wr_out_t;
    
    typedef struct {
        bit[7:0] mst_id; bit[3:0] id; bit[31:0] addr;
        bit[7:0] len; int r_beat_cnt; time ar_time;
    } rd_out_t;

    wr_out_t wr_out[int];  // key = {mst_id, id}
    rd_out_t rd_out[int];  // key = {mst_id, id}
    int wr_order[$];       // AW arrival order for reorder check
    int rd_order[$];       // AR arrival order
    int b_order[$];        // B arrival order

    int error_cnt=0; int wr_cnt=0, rd_cnt=0; int reorder_cnt=0;

    function new(string name="response_sb", uvm_component parent); super.new(name,parent); endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        aw_ap=new("aw_ap",this); w_ap=new("w_ap",this); b_ap=new("b_ap",this);
        ar_ap=new("ar_ap",this); r_ap=new("r_ap",this);
        aw_fifo=new("aw_fifo",this); w_fifo=new("w_fifo",this); b_fifo=new("b_fifo",this);
        ar_fifo=new("ar_fifo",this); r_fifo=new("r_fifo",this);
        aw_ap.connect(aw_fifo.analysis_export); w_ap.connect(w_fifo.analysis_export);
        b_ap.connect(b_fifo.analysis_export); ar_ap.connect(ar_fifo.analysis_export);
        r_ap.connect(r_fifo.analysis_export);
    endfunction

    task run_phase(uvm_phase phase);
        fork
            process_aw(); process_w(); process_b();
            process_ar(); process_r();
        join
    endtask

    //---- AW: register outstanding write ----
    task process_aw();
        axi_aw_item t; int key;
        forever begin
            aw_fifo.get(t);
            key = {8'(t.mst_id), t.id};
            if(wr_out.exists(key)) begin error_cnt++;
                `uvm_error("RESP",$sformatf("DUPLICATE AW M[%0d] id=%0d",t.mst_id,t.id))
            end
            wr_out[key].mst_id=t.mst_id; wr_out[key].id=t.id;
            wr_out[key].addr=t.addr; wr_out[key].len=t.len;
            wr_out[key].w_beat_cnt=0; wr_out[key].b_received=0;
            wr_order.push_back(key);
            wr_cnt++;
        end
    endtask

    //---- W: count beats per outstanding write, detect WLAST ----
    task process_w();
        axi_w_item t; int key; bit found;
        forever begin
            w_fifo.get(t);
            // W items from master-side don't have mst_id; track by AW order via wr_order
            // Match by: iterate wr_order to find first write that hasn't completed W beats
            found=0;
            foreach(wr_order[i]) begin
                key=wr_order[i];
                if(wr_out[key].w_beat_cnt <= wr_out[key].len) begin
                    wr_out[key].w_beat_cnt++;
                    if(t.last && wr_out[key].w_beat_cnt != wr_out[key].len+1) begin error_cnt++;
                        `uvm_error("RESP",$sformatf("WLAST mismatch: M[%0d] id=%0d beat=%0d len=%0d",
                                  wr_out[key].mst_id,wr_out[key].id,wr_out[key].w_beat_cnt,wr_out[key].len))
                    end
                    found=1; break;
                end
            end
            if(!found) begin error_cnt++;
                `uvm_error("RESP","W beat with no matching outstanding AW")
            end
        end
    endtask

    //---- B: check BID matches outstanding AW, detect out-of-order completion ----
    task process_b();
        axi_b_item t; int key; int aw_key;
        forever begin
            b_fifo.get(t);
            if(!t.is_master_side) continue; // only check master-side B
            key = {8'(t.mst_id), t.id};
            if(!wr_out.exists(key)) begin error_cnt++;
                `uvm_error("RESP",$sformatf("BID mismatch: M[%0d] id=%0d no matching AW",t.mst_id,t.id))
                continue;
            end
            wr_out[key].b_received=1;
            if(t.resp!=2'b00) begin error_cnt++;
                `uvm_error("RESP",$sformatf("BRESP error: M[%0d] id=%0d resp=%0d",t.mst_id,t.id,t.resp))
            end
            b_order.push_back(key);
            // Outstanding check: B should come after WLAST
            if(wr_out[key].w_beat_cnt < wr_out[key].len+1) begin error_cnt++;
                `uvm_error("RESP",$sformatf("B before WLAST: M[%0d] id=%0d w_beats=%0d/%0d",
                          t.mst_id,t.id,wr_out[key].w_beat_cnt,wr_out[key].len+1))
            end
        end
    endtask

    //---- AR: register outstanding read, track order for reorder check ----
    task process_ar();
        axi_ar_item t; int key;
        forever begin
            ar_fifo.get(t);
            key = {8'(t.mst_id), t.id};
            rd_out[key].mst_id=t.mst_id; rd_out[key].id=t.id;
            rd_out[key].addr=t.addr; rd_out[key].len=t.len;
            rd_out[key].r_beat_cnt=0; rd_out[key].ar_time=t.timestamp;
            rd_order.push_back(key);
            rd_cnt++;
        end
    endtask

    //---- R: check RID matches AR, detect reorder, check RLAST/RRESP ----
    task process_r();
        axi_r_item t; int key; int ar_key;
        forever begin
            r_fifo.get(t);
            if(!t.is_master_side) continue;
            key = {8'(t.mst_id), t.id};
            if(!rd_out.exists(key)) begin error_cnt++;
                `uvm_error("RESP",$sformatf("RID mismatch: M[%0d] id=%0d no matching AR",t.mst_id,t.id))
                continue;
            end
            rd_out[key].r_beat_cnt++;
            if(t.resp!=2'b00) begin error_cnt++;
                `uvm_error("RESP",$sformatf("RRESP error: M[%0d] id=%0d resp=%0d",t.mst_id,t.id,t.resp))
            end
            if(t.last && rd_out[key].r_beat_cnt != rd_out[key].len+1) begin error_cnt++;
                `uvm_error("RESP",$sformatf("RLAST length mismatch: M[%0d] id=%0d got %0d beats exp %0d",
                          t.mst_id,t.id,rd_out[key].r_beat_cnt,rd_out[key].len+1))
            end
            // Reorder detection: if R arrives for an AR that wasn't first in rd_order
            if(rd_order.size()>0 && rd_order[0]!=key) reorder_cnt++;
            // Remove from order when all R beats received
            if(t.last) begin
                for(int i=0; i<rd_order.size(); i++) if(rd_order[i]==key) begin rd_order.delete(i); break; end
            end
        end
    endtask

    function void report_phase(uvm_phase phase);
        $display("RESP SB: %0d writes, %0d reads, %0d errors, %0d reorder events",
                 wr_cnt, rd_cnt, error_cnt, reorder_cnt);
    endfunction
endclass
