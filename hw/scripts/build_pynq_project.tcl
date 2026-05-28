# =============================================================================
# Project      : Adaptive QAM Modem
# File         : build_pynq_project.tcl
# Description  : Vivado TCL script to build the PYNQ-Z2 loopback project
# =============================================================================

set project_name "qam_modem"
set project_dir  "./hw/build/vivado_project"
set part         "xc7z020clg400-1"
set rtl_path     "./hw/hdl/src"

# Create project
# Create project
create_project -force $project_name $project_dir -part $part

set has_board [expr {[llength [get_board_parts -quiet tul.com.tw:pynq-z2:part0:1.0]] > 0}]
if {$has_board} {
    set_property board_part "tul.com.tw:pynq-z2:part0:1.0" [current_project]
} else {
    puts "WARNING: PYNQ-Z2 board files not installed. Falling back to default xc7z020 part."
}

# Add RTL sources
add_files [glob $rtl_path/*.v]

# Create Block Design
create_bd_design "system"

# Add Zynq Processing System
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7

# Apply board preset or manual config
if {$has_board} {
    apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {make_external "FIXED_IO, DDR" apply_board_preset "1"}  [get_bd_cells ps7]
} else {
    # Configure 100MHz clock and enable AXI GP0
    set_property -dict [list CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} CONFIG.PCW_USE_M_AXI_GP0 {1}] [get_bd_cells ps7]
    apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {make_external "FIXED_IO, DDR" Master "Disable" Slave "Disable" }  [get_bd_cells ps7]
}

# Add our custom RTL module as a block
create_bd_cell -type module -reference qam_loopback_axi qam_0

# Connect AXI-Lite using automation (this adds interconnect and resets automatically)
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/ps7/M_AXI_GP0} Slave {/qam_0/s_axi} ddr_cas_latency {8} intc_ip {New AXI Interconnect} master_apm {0}}  [get_bd_intf_pins qam_0/s_axi]

# Validate and Save
validate_bd_design
save_bd_design

# Generate target FIRST so the BD is fully resolved
generate_target all [get_files  $project_dir/$project_name.srcs/sources_1/bd/system/system.bd]

# Workaround for Vivado Ubuntu 22.04 bug where OS release variables leak into Verilog headers
exec bash -c "find $project_dir -name '*.v' -exec sed -i -e '/^VERSION_ID=/d' -e '/^VERSION_CODENAME=/d' {} +"

# Create HDL wrapper for the block design
set wrapper_path [make_wrapper -files [get_files -norecurse system.bd] -top]

# Run workaround again for the wrapper
exec bash -c "find $project_dir -name '*.v' -exec sed -i -e '/^VERSION_ID=/d' -e '/^VERSION_CODENAME=/d' {} +"

add_files -norecurse -force $wrapper_path
set_property top system_wrapper [current_fileset]
update_compile_order -fileset sources_1

# Force wait for hierarchy update
puts "Waiting for hierarchy update..."
update_compile_order -fileset sources_1
puts "=================================================================="
puts "CURRENT TOP MODULE (Fileset): [get_property top [current_fileset]]"
puts "=================================================================="

# Launch synthesis and implementation
reset_run synth_1
launch_runs synth_1 -jobs 2
wait_on_run synth_1

launch_runs impl_1 -to_step write_bitstream -jobs 2
wait_on_run impl_1

# Export results for PYNQ
file copy -force $project_dir/$project_name.runs/impl_1/system_wrapper.bit ./sw/pynq/qam_modem.bit
file copy -force $project_dir/$project_name.gen/sources_1/bd/system/hw_handoff/system.hwh ./sw/pynq/qam_modem.hwh

puts "=================================================================="
puts "Build Complete!"
puts "Bitstream and HWH files have been copied to the project root directory:"
puts " -> qam_modem.bit"
puts " -> qam_modem.hwh"
puts "=================================================================="
