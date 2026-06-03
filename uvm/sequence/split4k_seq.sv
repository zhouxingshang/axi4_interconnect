// 4KB Split Sequence: crosses 4K boundary
class split4k_seq extends uvm_sequence #(axi_transaction);
    `uvm_object_utils(split4k_seq)
    int mst_id=0;

    function new(string n="split4k_seq"); 
        super.new(n); 
    endfunction
    
    task body();
        axi_transaction t = axi_transaction::type_id::create("t");
        start_item(t);
        if(!t.randomize() with { mst_id==local::mst_id; addr==32'h0FF0;
            len==7; burst==2'b01; size==2; is_write dist {1:=1, 0:=1};
        }) `uvm_fatal("SEQ","randomize failed")
        finish_item(t);
    endtask
endclass
