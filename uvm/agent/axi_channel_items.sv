//=============================================================================
// AXI Channel Transaction Items: per-channel monitor capture types
//=============================================================================

//---- AW Channel Item ----
class axi_aw_item extends uvm_sequence_item;
    bit is_master_side;          // 1 = M-side, 0 = S-side
    int  mst_id;                 // master index
    int  slv_id;                 // slave index (-1 if master-side)
    bit [3:0]  id;              // original transaction ID
    bit [31:0] addr;
    bit [7:0]  len;
    bit [2:0]  size;
    bit [1:0]  burst;
    time       timestamp;
    `uvm_object_utils_begin(axi_aw_item)
        `uvm_field_int(is_master_side,UVM_DEFAULT) `uvm_field_int(mst_id,UVM_DEFAULT)
        `uvm_field_int(slv_id,UVM_DEFAULT) `uvm_field_int(id,UVM_DEFAULT)
        `uvm_field_int(addr,UVM_DEFAULT) `uvm_field_int(len,UVM_DEFAULT)
    `uvm_object_utils_end
    function new(string name="axi_aw_item"); super.new(name); endfunction
endclass

//---- W Channel Item (single beat) ----
class axi_w_item extends uvm_sequence_item;
    bit is_master_side;
    int  mst_id, slv_id;
    bit [31:0] data;
    bit [3:0]  strb;
    bit        last;
    time       timestamp;
    `uvm_object_utils_begin(axi_w_item)
        `uvm_field_int(is_master_side,UVM_DEFAULT) `uvm_field_int(mst_id,UVM_DEFAULT)
        `uvm_field_int(slv_id,UVM_DEFAULT) `uvm_field_int(data,UVM_DEFAULT)
    `uvm_object_utils_end
    function new(string name="axi_w_item"); super.new(name); endfunction
endclass

//---- B Channel Item ----
class axi_b_item extends uvm_sequence_item;
    bit is_master_side;
    int  mst_id, slv_id;
    bit [3:0] id;
    bit [1:0] resp;
    time      timestamp;
    `uvm_object_utils_begin(axi_b_item)
        `uvm_field_int(is_master_side,UVM_DEFAULT) `uvm_field_int(mst_id,UVM_DEFAULT)
        `uvm_field_int(slv_id,UVM_DEFAULT) `uvm_field_int(id,UVM_DEFAULT)
        `uvm_field_int(resp,UVM_DEFAULT)
    `uvm_object_utils_end
    function new(string name="axi_b_item"); super.new(name); endfunction
endclass

//---- AR Channel Item ----
class axi_ar_item extends uvm_sequence_item;
    bit is_master_side;
    int  mst_id, slv_id;
    bit [3:0]  id;
    bit [31:0] addr;
    bit [7:0]  len;
    bit [2:0]  size;
    bit [1:0]  burst;
    time       timestamp;
    `uvm_object_utils_begin(axi_ar_item)
        `uvm_field_int(is_master_side,UVM_DEFAULT) `uvm_field_int(mst_id,UVM_DEFAULT)
        `uvm_field_int(slv_id,UVM_DEFAULT) `uvm_field_int(id,UVM_DEFAULT)
        `uvm_field_int(addr,UVM_DEFAULT) `uvm_field_int(len,UVM_DEFAULT)
    `uvm_object_utils_end
    function new(string name="axi_ar_item"); super.new(name); endfunction
endclass

//---- R Channel Item (single beat) ----
class axi_r_item extends uvm_sequence_item;
    bit is_master_side;
    int  mst_id, slv_id;
    bit [3:0]  id;
    bit [31:0] data;
    bit [1:0]  resp;
    bit        last;
    time       timestamp;
    `uvm_object_utils_begin(axi_r_item)
        `uvm_field_int(is_master_side,UVM_DEFAULT) `uvm_field_int(mst_id,UVM_DEFAULT)
        `uvm_field_int(slv_id,UVM_DEFAULT) `uvm_field_int(id,UVM_DEFAULT)
        `uvm_field_int(data,UVM_DEFAULT) `uvm_field_int(resp,UVM_DEFAULT)
    `uvm_object_utils_end
    function new(string name="axi_r_item"); super.new(name); endfunction
endclass
