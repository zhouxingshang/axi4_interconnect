// Outstanding Sequence: sends multiple AW/AR before waiting for responses
class outstanding_seq extends uvm_sequence #(axi_transaction);
    `uvm_object_utils(outstanding_seq)
    int mst_id=0; int count=4;
    function new(string n="outstanding_seq"); super.new(n); endfunction
    task body();
        axi_transaction t; int i;
        for(i=0; i<count; i++) begin
            t = axi_transaction::type_id::create("t");
            start_item(t);
            if(!t.randomize() with { mst_id==local::mst_id; addr inside {[0:16'h1FFF]};
                len==0; is_write dist {1:=1, 0:=1}; }) `uvm_fatal("SEQ","randomize failed")
            finish_item(t);
        end
    endtask
endclass
