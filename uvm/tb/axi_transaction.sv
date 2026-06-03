//=============================================================================
// Unified AXI Transaction Item
//=============================================================================
`ifndef AXI_TRANSACTION_SV
`define AXI_TRANSACTION_SV

class axi_transaction extends uvm_sequence_item;

    //---- Control ----
    rand bit                is_write;       // 1=write, 0=read

    //---- Address phase ----
    rand bit [3:0]          id;             // transaction ID
    rand bit [31:0]         addr;           // start address
    rand bit [7:0]          len;            // burst length (beats = len + 1)
    rand bit [2:0]          size;           // bytes per beat = 2^size
    rand bit [1:0]          burst;          // burst type: FIXED=00, INCR=01, WRAP=10
    rand bit [3:0]          strb [];        // W strobes per beat (writes only)

    //---- Data (write data or expected read data) ----
    rand bit [31:0]         data [];        // data payload per beat

    //---- Response (filled by monitor) ----
    bit [1:0]               resp;           // BRESP or RRESP
    bit                     rlast;          // RLAST (reads only)

    //---- Meta (filled by monitor / reference model) ----
    int                     mst_id;         // which master sent this
    int                     slv_id;         // which slave it was routed to
    bit                     is_split;       // was this split across 4KB?

    //---- Constraints ----
    constraint addr_range_c {
        addr inside {[0:32'h7FFF]};              // within slave address map
    }

    constraint addr_align_c {
        (size == 0) -> (addr[1:0] == 2'b00);   // byte access: no alignment needed
        (size == 1) -> (addr[0]   == 1'b0);    // halfword: even address
        (size >= 2) -> (addr[1:0] == 2'b00);   // word+: word aligned
    }

    constraint burst_len_c {
        len inside {[0:15]};                    // up to 16 beats
    }

    constraint burst_type_c {
        burst inside {0, 1};                // FIXED, INCR, WRAP
    }

    constraint id_c {
        id inside {[0:15]};                     // 4-bit ID range
    }

    constraint data_size_c {
        data.size() < 128;
        strb.size() < 128;
        if (is_write) {
            data.size() == len + 1;
            strb.size() == len + 1;
        }
    }

    `uvm_object_utils_begin(axi_transaction)
        `uvm_field_int (is_write, UVM_DEFAULT)
        `uvm_field_int (id,       UVM_DEFAULT)
        `uvm_field_int (addr,     UVM_DEFAULT)
        `uvm_field_int (len,      UVM_DEFAULT)
        `uvm_field_int (size,     UVM_DEFAULT)
        `uvm_field_int (burst,    UVM_DEFAULT)
        `uvm_field_int (resp,     UVM_DEFAULT)
        `uvm_field_int (mst_id,   UVM_DEFAULT)
        `uvm_field_int (slv_id,   UVM_DEFAULT)
    `uvm_object_utils_end

    function new(string name = "axi_transaction");
        super.new(name);
        data = new[16];
        strb = new[16];
    endfunction

    function void post_randomize();
        if (is_write) begin
            foreach (strb[i]) strb[i] = 4'hF;
        end
    endfunction

    function string convert2string();
        return $sformatf("%s id=%0d addr=0x%08h len=%0d size=%0d burst=%0d mst=%0d slv=%0d",
                         is_write ? "WR" : "RD", id, addr, len, size, burst, mst_id, slv_id);
    endfunction

endclass

`endif
