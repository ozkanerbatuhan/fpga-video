# Dumps the CONFIG.* properties of the IPs used in the block design.
set_param board.repoPaths [list {C:/Users/ozkan/AppData/Roaming/Xilinx/Vivado/2025.2/xhub/board_store/xilinx_board_store/XilinxBoardStore/Vivado/2025.2/boards}]
create_project -in_memory -part xc7z020clg484-1
set_property board_part avnet.com:zedboard:part0:1.4 [current_project]
create_bd_design probe
foreach {name vlnv} {
    vid_out xilinx.com:ip:v_axi4s_vid_out
    vtc     xilinx.com:ip:v_tc
    dma     xilinx.com:ip:axi_dma
    vdma    xilinx.com:ip:axi_vdma
    iic     xilinx.com:ip:axi_iic
    gpio    xilinx.com:ip:axi_gpio
} {
    set c [create_bd_cell -type ip -vlnv $vlnv $name]
    puts "=== $name [get_property VLNV $c]"
    foreach p [lsort [list_property $c CONFIG.*]] {
        puts "$p = [get_property $p $c]"
    }
}
