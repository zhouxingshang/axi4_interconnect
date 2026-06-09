//=============================================================================
// AXI UVM Package: includes all UVM components
//=============================================================================
`ifndef AXI_PKG_SV
`define AXI_PKG_SV

package axi_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"

    //---- Parameters ----
    localparam MST_AMT    = 4;
    localparam SLV_AMT    = 4;
    localparam W_ID       = 4;
    localparam W_CID      = 2;
    localparam W_SID      = $clog2(MST_AMT) + W_CID + W_ID;
    localparam ADDR_WIDTH = 32;
    localparam DATA_WIDTH = 32;
    localparam LEN_W      = 8;
    localparam SIZE_W     = 3;
    localparam BURST_W    = 2;
    localparam RESP_W     = 2;
    localparam W_STRB     = DATA_WIDTH / 8;

    // Address map: bits [31:13] select slave, 8KB per slave
    localparam bit [ADDR_WIDTH-1:0] SLV_ADDR_BASE [0:SLV_AMT-1] = '{
        32'h0000_0000,   // slave 0
        32'h0000_2000,   // slave 1
        32'h0000_4000,   // slave 2
        32'h0000_6000    // slave 3
    };
    localparam bit [7:0] SLV_ADDR_LEN = 8'd13;

    //---- Transaction ----
    `include "axi_transaction.sv"

    //---- Channel transaction items ----
    `include "axi_channel_items.sv"

    //---- Interface ----

    //---- Agent components ----
    `include "axi_sequencer.sv"
    `include "axi_driver.sv"
    `include "axi_monitor.sv"
    `include "axi_master_agent.sv"
    `include "axi_slave_agent.sv"

    //---- Sequences ----
    `include "axi_virtual_sequencer.sv"
    `include "axi_virtual_sequence.sv"
    `include "axi_base_seq.sv"
    `include "burst_seq.sv"
    `include "outstanding_seq.sv"
    `include "outstanding_vseq.sv"
    `include "split4k_seq.sv"
    `include "stress_seq.sv"
    `include "smoke_vseq.sv"
    `include "burst_vseq.sv"
    `include "split4k_vseq.sv"
    `include "stress_vseq.sv"
    `include "arb_vseq.sv"

    //---- Model ----
    `include "addr_decoder.sv"
    `include "memory_model.sv"
    `include "reference_model.sv"

    //---- Scoreboard ----
    `include "axi_scoreboard.sv"

    //---- Coverage ----
    `include "axi_cov.sv"
    `include "axi_protocol_checker.sv"

    //---- Environment ----
    `include "axi_env.sv"

    //---- Tests ----
    `include "axi_test_base.sv"
    `include "smoke_test.sv"
    `include "burst_test.sv"
    `include "arb_test.sv"
    `include "split4k_test.sv"
    `include "stress_test.sv"
    `include "outstanding_test.sv"

endpackage

`endif
