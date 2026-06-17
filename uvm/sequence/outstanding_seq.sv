// Outstanding Sequence: sends multiple AW/AR before waiting for responses
class outstanding_seq extends uvm_sequence #(axi_transaction);
    `uvm_object_utils(outstanding_seq)
    int mst_id=0; 
    int count=4;

    function new(string n="outstanding_seq"); 
        super.new(n); 
    endfunction

    task body();
        fork
            // Thread 1: fire all transactions, non-blocking
            begin
                for (int i = 0; i < count; i++) begin
                    axi_transaction t = axi_transaction::type_id::create("t");
                    start_item(t);
                    if (!t.randomize() with {
                        mst_id == local::mst_id;
                        addr inside {[0:16'h1FFF]};
                        len == 0;
                        is_write dist {1:=1, 0:=1};
                    }) `uvm_fatal("SEQ", "randomize failed")
                    finish_item(t);
                    `uvm_info("OUTSTANDING_SEQ", $sformatf("M[%0d] fired #%0d: %s", mst_id, i, t.convert2string()), UVM_MEDIUM)
                end
            end

            // Thread 2: collect all responses asynchronously
            begin
                axi_transaction rsp;
                for (int i = 0; i < count; i++) begin
                    get_response(rsp);
                    `uvm_info("OUTSTANDING_SEQ", $sformatf("M[%0d] got response #%0d: resp=%0d id=%0d",
                                                           mst_id, i, rsp.resp, rsp.id), UVM_MEDIUM)
                end
            end
        join
    endtask
endclass
