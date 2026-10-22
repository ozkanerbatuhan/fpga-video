# Full hardware build: project, block design, synthesis, implementation,
# bitstream, reports and the .xsa for Vitis.
#
#   vivado -mode batch -source hw/build.tcl            (from the repo root)
#   vivado -mode batch -source hw/build.tcl -tclargs bd   (stop after the block design)
#
# Output: hw/build/  (project), hw/reports/ (committed), hw/build/video_top.xsa
set root  [file normalize [file join [file dirname [info script]] ..]]
set bdir  $root/hw/build
set rdir  $root/hw/reports
set stop  [expr {[llength $argv] > 0 ? [lindex $argv 0] : "all"}]
file mkdir $rdir

set_param board.repoPaths [list "$::env(APPDATA)/Xilinx/Vivado/2025.2/xhub/board_store/xilinx_board_store/XilinxBoardStore/Vivado/2025.2/boards"]
create_project video $bdir -part xc7z020clg484-1 -force
set_property board_part avnet.com:zedboard:part0:1.4 [current_project]
set_property target_language Verilog [current_project]

add_files [glob $root/rtl/*.sv] [glob $root/hw/hdl/*.v] $root/rtl/font8x8.mem
set_property file_type {Memory File} [get_files font8x8.mem]
add_files -fileset constrs_1 $root/hw/zedboard_video.xdc
update_compile_order -fileset sources_1

# ------------------------------------------------------------------ block design
create_bd_design system

set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7 ps7]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} $ps
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_USE_S_AXI_HP0 {1} \
    CONFIG.PCW_S_AXI_HP0_DATA_WIDTH {64} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
    CONFIG.PCW_USE_FABRIC_INTERRUPT {0}] $ps

# clocks: 148.5 MHz processing (1080p60 pixel rate), 74.25 MHz 720p60 pixel clock
set cw [create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz clk_wiz]
set_property -dict [list \
    CONFIG.PRIM_IN_FREQ {100} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {148.5} \
    CONFIG.CLKOUT2_USED {true} \
    CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {74.25} \
    CONFIG.RESET_TYPE {ACTIVE_LOW} \
    CONFIG.RESET_PORT {resetn}] $cw
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins clk_wiz/clk_in1]
connect_bd_net [get_bd_pins ps7/FCLK_RESET0_N] [get_bd_pins clk_wiz/resetn]

set clk  [get_bd_pins clk_wiz/clk_out1]
set pclk [get_bd_pins clk_wiz/clk_out2]

create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst_proc
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst_pix
foreach r {rst_proc rst_pix} {
    connect_bd_net [get_bd_pins ps7/FCLK_RESET0_N] [get_bd_pins $r/ext_reset_in]
    connect_bd_net [get_bd_pins clk_wiz/locked] [get_bd_pins $r/dcm_locked]
}
connect_bd_net $clk  [get_bd_pins rst_proc/slowest_sync_clk]
connect_bd_net $pclk [get_bd_pins rst_pix/slowest_sync_clk]
set aresetn [get_bd_pins rst_proc/peripheral_aresetn]

# processing: AXI DMA (simple mode) <-> pipeline
set dma [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma dma]
set_property -dict [list \
    CONFIG.c_include_sg {0} \
    CONFIG.c_sg_length_width {20} \
    CONFIG.c_m_axis_mm2s_tdata_width {16} \
    CONFIG.c_s_axis_s2mm_tdata_width {16} \
    CONFIG.c_m_axi_mm2s_data_width {64} \
    CONFIG.c_m_axi_s2mm_data_width {64} \
    CONFIG.c_mm2s_burst_size {64} \
    CONFIG.c_s2mm_burst_size {64}] $dma

create_bd_cell -type module -reference vproc_axi vproc
connect_bd_intf_net [get_bd_intf_pins dma/M_AXIS_MM2S] [get_bd_intf_pins vproc/s_axis]
connect_bd_intf_net [get_bd_intf_pins vproc/m_axis] [get_bd_intf_pins dma/S_AXIS_S2MM]

# display: VDMA (read only) -> 2x upscaler -> AXI4-Stream to video out -> ADV7511
set vdma [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_vdma vdma]
set_property -dict [list \
    CONFIG.c_include_s2mm {0} \
    CONFIG.c_include_sg {0} \
    CONFIG.c_m_axis_mm2s_tdata_width {16} \
    CONFIG.c_m_axi_mm2s_data_width {64} \
    CONFIG.c_mm2s_max_burst_length {32} \
    CONFIG.c_mm2s_linebuffer_depth {1024} \
    CONFIG.c_num_fstores {3} \
    CONFIG.c_use_mm2s_fsync {0}] $vdma

create_bd_cell -type module -reference upscale_axi upscale
connect_bd_intf_net [get_bd_intf_pins vdma/M_AXIS_MM2S] [get_bd_intf_pins upscale/s_axis]

set vtc [create_bd_cell -type ip -vlnv xilinx.com:ip:v_tc vtc]
set_property -dict [list \
    CONFIG.HAS_AXI4_LITE {false} \
    CONFIG.enable_detection {false} \
    CONFIG.VIDEO_MODE {720p} \
    CONFIG.VID_PPC {1} \
    CONFIG.GEN_VIDEO_FORMAT {YUV_422}] $vtc

set vo [create_bd_cell -type ip -vlnv xilinx.com:ip:v_axi4s_vid_out vid_out]
set_property -dict [list \
    CONFIG.C_HAS_ASYNC_CLK {1} \
    CONFIG.C_S_AXIS_VIDEO_FORMAT {0} \
    CONFIG.C_S_AXIS_VIDEO_DATA_WIDTH {8} \
    CONFIG.C_ADDR_WIDTH {12}] $vo
connect_bd_intf_net [get_bd_intf_pins upscale/m_axis] [get_bd_intf_pins vid_out/video_in]
connect_bd_intf_net [get_bd_intf_pins vtc/vtiming_out] [get_bd_intf_pins vid_out/vtiming_in]
connect_bd_net [get_bd_pins vid_out/vtg_ce] [get_bd_pins vtc/gen_clken]

create_bd_cell -type module -reference hdmi_out u_hdmi
connect_bd_net [get_bd_pins vid_out/vid_data]         [get_bd_pins u_hdmi/vid_data]
connect_bd_net [get_bd_pins vid_out/vid_hsync]        [get_bd_pins u_hdmi/vid_hsync]
connect_bd_net [get_bd_pins vid_out/vid_vsync]        [get_bd_pins u_hdmi/vid_vsync]
connect_bd_net [get_bd_pins vid_out/vid_active_video] [get_bd_pins u_hdmi/vid_active_video]
foreach p {hd_clk hd_hsync hd_vsync hd_de} { create_bd_port -dir O $p }
create_bd_port -dir O -from 15 -to 0 hd_d
create_bd_port -dir O -from 3 -to 0 led
foreach p {hd_clk hd_d hd_hsync hd_vsync hd_de led} {
    connect_bd_net [get_bd_pins u_hdmi/$p] [get_bd_ports $p]
}

# HDMI I2C and control GPIO
set iic [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_iic iic_hdmi]
set_property CONFIG.IIC_FREQ_KHZ {100} $iic
create_bd_intf_port -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 iic_hdmi
connect_bd_intf_net [get_bd_intf_pins iic_hdmi/IIC] [get_bd_intf_ports iic_hdmi]

set gpio [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio gpio]
set_property -dict [list CONFIG.C_GPIO_WIDTH {5} CONFIG.C_ALL_OUTPUTS {1}] $gpio
connect_bd_net [get_bd_pins gpio/gpio_io_o] [get_bd_pins u_hdmi/ctrl]

# clocks / resets
foreach p {vproc/aclk upscale/aclk dma/s_axi_lite_aclk dma/m_axi_mm2s_aclk dma/m_axi_s2mm_aclk
           vdma/s_axi_lite_aclk vdma/m_axi_mm2s_aclk vdma/m_axis_mm2s_aclk
           vid_out/aclk iic_hdmi/s_axi_aclk gpio/s_axi_aclk ps7/M_AXI_GP0_ACLK ps7/S_AXI_HP0_ACLK} {
    connect_bd_net $clk [get_bd_pins $p]
}
foreach p {vtc/clk vid_out/vid_io_out_clk u_hdmi/pix_clk} {
    connect_bd_net $pclk [get_bd_pins $p]
}
foreach p {vproc/aresetn upscale/aresetn dma/axi_resetn vdma/axi_resetn
           iic_hdmi/s_axi_aresetn gpio/s_axi_aresetn vid_out/aresetn} {
    connect_bd_net $aresetn [get_bd_pins $p]
}
connect_bd_net [get_bd_pins rst_pix/peripheral_aresetn] [get_bd_pins vtc/resetn]
connect_bd_net [get_bd_pins rst_pix/peripheral_reset]   [get_bd_pins vid_out/vid_io_out_reset]

# AXI-Lite control from GP0
set ic [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect ic_ctrl]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {5}] $ic
connect_bd_intf_net [get_bd_intf_pins ps7/M_AXI_GP0] [get_bd_intf_pins ic_ctrl/S00_AXI]
set i 0
foreach s {vproc/s_axi dma/S_AXI_LITE vdma/S_AXI_LITE iic_hdmi/S_AXI gpio/S_AXI} {
    connect_bd_intf_net [get_bd_intf_pins ic_ctrl/M0${i}_AXI] [get_bd_intf_pins $s]
    incr i
}
connect_bd_net $clk [get_bd_pins ic_ctrl/aclk]
connect_bd_net $aresetn [get_bd_pins ic_ctrl/aresetn]

# memory traffic to HP0
set im [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect ic_mem]
set_property -dict [list CONFIG.NUM_SI {3} CONFIG.NUM_MI {1}] $im
connect_bd_intf_net [get_bd_intf_pins dma/M_AXI_MM2S]  [get_bd_intf_pins ic_mem/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins dma/M_AXI_S2MM]  [get_bd_intf_pins ic_mem/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins vdma/M_AXI_MM2S] [get_bd_intf_pins ic_mem/S02_AXI]
connect_bd_intf_net [get_bd_intf_pins ic_mem/M00_AXI]  [get_bd_intf_pins ps7/S_AXI_HP0]
connect_bd_net $clk [get_bd_pins ic_mem/aclk]
connect_bd_net $aresetn [get_bd_pins ic_mem/aresetn]

assign_bd_address
set_property offset 0x43C00000 [get_bd_addr_segs {ps7/Data/SEG_vproc_reg0}]
set_property range 64K [get_bd_addr_segs {ps7/Data/SEG_vproc_reg0}]
validate_bd_design
save_bd_design
write_bd_tcl -force $rdir/system_bd.tcl

make_wrapper -files [get_files system.bd] -top
add_files -norecurse [file join $bdir video.gen sources_1 bd system hdl system_wrapper.v]
set_property top system_wrapper [current_fileset]
update_compile_order -fileset sources_1

if {$stop eq "bd"} { puts "BD_DONE"; exit }

# ------------------------------------------------------------------ implementation
set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
launch_runs synth_1 -jobs 8
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} { error "synthesis failed" }
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { error "implementation failed" }

open_run impl_1
report_timing_summary -max_paths 10 -file $rdir/timing_summary.txt
report_utilization -file $rdir/utilization.txt
report_utilization -hierarchical -hierarchical_depth 4 -file $rdir/utilization_hier.txt
report_power -file $rdir/power.txt
report_clock_interaction -file $rdir/clock_interaction.txt
report_cdc -file $rdir/cdc.txt

write_hw_platform -fixed -include_bit -force $bdir/video_top.xsa
file copy -force [glob $bdir/video.runs/impl_1/*.bit] $bdir/video_top.bit
puts "BUILD_DONE"
