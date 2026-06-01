//=============================================================================
// UVM AXI Interconnect Verification Filelist
// Usage: vcs -f flist.f -full64 -sverilog ...
//=============================================================================

//---- Global options ----
+define+UVM_NO_DPI
-timescale=1ns/1ps

//---- Include directories ----
+incdir+./UVM_AXI_TB/tb
+incdir+./UVM_AXI_TB/agent
+incdir+./UVM_AXI_TB/sequence
+incdir+./UVM_AXI_TB/model
+incdir+./UVM_AXI_TB/scoreboard
+incdir+./UVM_AXI_TB/coverage
+incdir+./UVM_AXI_TB/tests

//---- UVM library (adjust path to your installation) ----
// $UVM_HOME/src/uvm_pkg.sv

//---- RTL: AXI Interconnect DUT ----
./rtl/axi_fifo_sync.v
./rtl/aw_order_fifo.v
./rtl/axi_arbiter_param_rr.v
./rtl/axi_arbiter_m2s_m_amt.v
./rtl/axi_m2s_m_amt.v
./rtl/axi_s2m_s_amt.v
./rtl/axi_crossbar.v
./rtl/axi_default_slave.v
./rtl/cross_4k_if.v
./rtl/sid_buffer.v
./rtl/reorder.v
./rtl/axi_split_b_merge.v
./rtl/axi_split_r_merge.v
./rtl/axi_interconnect.v

//---- UVM TB: Interface (must come before package — agents reference axi_if) ----
./UVM_AXI_TB/tb/axi_if.sv

//---- UVM TB: Package (includes all other TB files via `include) ----
./UVM_AXI_TB/tb/axi_pkg.sv

//---- UVM TB: Top-level module ----
./UVM_AXI_TB/tb/tb_top.sv
