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
    `include "../agent/axi_channel_items.sv"

    //---- Interface ----

    //---- Agent components ----
    `include "../agent/axi_sequencer.sv"
    `include "../agent/axi_driver.sv"
    `include "../agent/axi_monitor.sv"
    `include "../agent/axi_master_agent.sv"
    `include "../agent/axi_slave_agent.sv"

    //---- Sequences ----
    `include "../sequence/axi_base_seq.sv"
    `include "../sequence/burst_seq.sv"
    `include "../sequence/outstanding_seq.sv"
    `include "../sequence/split4k_seq.sv"
    `include "../sequence/stress_seq.sv"

    //---- Model ----
    `include "../model/addr_decoder.sv"
    `include "../model/memory_model.sv"
    `include "../model/reference_model.sv"

    //---- Scoreboard ----
    `include "../scoreboard/routing_sb.sv"
    `include "../scoreboard/data_sb.sv"
    `include "../scoreboard/response_sb.sv"

    //---- Coverage ----
    `include "../coverage/axi_cov.sv"
    `include "../coverage/axi_protocol_checker.sv"

    //---- Environment ----
    `include "axi_env.sv"

    //---- Tests ----
    `include "axi_test_base.sv"
    `include "../tests/smoke_test.sv"
    `include "../tests/burst_test.sv"
    `include "../tests/arb_test.sv"
    `include "../tests/split4k_test.sv"
    `include "../tests/stress_test.sv"

endpackage

`endif
