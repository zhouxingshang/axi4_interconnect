// Stress Sequence: randomized multi-master concurrent transactions
class stress_seq extends uvm_sequence #(axi_transaction);
    `uvm_object_utils(stress_seq)
    int mst_id=0; 
    int tx_count=50;

    function new(string n="stress_seq"); 
        super.new(n); 
    endfunction

    task body();
        axi_transaction t; int i;
        for(i=0; i<tx_count; i++) begin
            t = axi_transaction::type_id::create("t");
            start_item(t);
            if(!t.randomize() with { mst_id==local::mst_id; len inside {[0:7]};
                burst inside {0,1,2}; size inside {0,1,2,3}; }) `uvm_fatal("SEQ","randomize failed")
            finish_item(t);
        end
    endtask
endclass
