// Burst Sequence: multi-beat INCR burst
class burst_seq extends uvm_sequence #(axi_transaction);
    `uvm_object_utils(burst_seq)
    int mst_id=0; 
    bit[31:0] base_addr=0; 
    bit[7:0] burst_len=3;

    function new(string n="burst_seq"); 
        super.new(n); 
    endfunction

    task body();
        axi_transaction t = axi_transaction::type_id::create("t");
        start_item(t);
        if(!t.randomize() with { mst_id==local::mst_id; addr==local::base_addr;
            len==local::burst_len; burst==2'b01; size==2;
            is_write dist {1:=1, 0:=1}; }) `uvm_fatal("SEQ","randomize failed")
        finish_item(t);
        get_response(t);
    endtask
endclass
