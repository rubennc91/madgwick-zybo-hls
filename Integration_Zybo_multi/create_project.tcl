# Uso: vivado -mode batch -source create_project.tcl
#
# Integracion de VARIAS IMUs (hasta 4 Pmod NAV) con UN solo nucleo Madgwick en la Zybo Z7-10:
#
#   nav_spi_ctrl_0..3 (VHDL, uno por Pmod) --AXI-Stream--> r2f_mux (HLS: raw2float + reparto
#   round-robin, TID = indice de la IMU) --AXI-Stream--> mad_multi (HLS: opt2 con un estado
#   por IMU). El PS lo controla todo por AXI-Lite (sin DMA).
#
# Estructura de carpetas esperada (este script se ejecuta desde Integration_Zybo_multi):
#   PMOD_NAV_Ctrller/ip_repo/nav_spi_ctrl_v1.0/                    (package_nav_ip.tcl)
#   HLS_madgwick_multi/r2f_mux_prj/sol1/impl/ip/                   (run_hls_r2f_mux.tcl)
#   HLS_madgwick_multi/madgwick_multi_prj/sol1/impl/ip/            (run_hls_madgwick_multi.tcl)

set PART      "xc7z010clg400-1"
set PROJ_NAME "zybo_multi"
set NIMU      4

set NAV_IP_REPO   [file normalize "../PMOD_NAV_Ctrller/ip_repo"]
set MUX_IP_REPO   [file normalize "../HLS_madgwick_multi/r2f_mux_prj/sol1/impl/ip"]
set MULTI_IP_REPO [file normalize "../HLS_madgwick_multi/madgwick_multi_prj/sol1/impl/ip"]

foreach p [list $NAV_IP_REPO $MUX_IP_REPO $MULTI_IP_REPO] {
    if {![file isdirectory $p]} {
        puts "AVISO: no existe '$p'."
        puts "  -> Ejecuta antes package_nav_ip.tcl y los dos run_hls_*.tcl de HLS_madgwick_multi."
    }
}

create_project -force $PROJ_NAME ./$PROJ_NAME -part $PART

# Board files de Digilent para la Zybo Z7-10 (ver comentarios en Integration_Zybo_dual/create_project.tcl)
set zybo_boards [get_board_parts -quiet *zybo-z7-10*]
if {[llength $zybo_boards] == 0} {
    puts "ERROR: no encuentro los board files de la Zybo Z7-10 (get_board_parts *zybo*)."
    close_project
    return
}
set ZYBO_BOARD [lindex [lsort -dictionary $zybo_boards] end]
puts "Usando board_part: $ZYBO_BOARD"
set_property board_part $ZYBO_BOARD [current_project]

set_property ip_repo_paths [list $NAV_IP_REPO $MUX_IP_REPO $MULTI_IP_REPO] [current_project]
update_ip_catalog

create_bd_design "system"

# --- Zynq-7 Processing System ------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" \
             Master "Disable" Slave "Disable"} [get_bd_cells ps7]
set_property -dict [list CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} CONFIG.PCW_USE_M_AXI_GP0 {1}] [get_bd_cells ps7]

# --- IPs ---------------------------------------------------------------------
for {set i 0} {$i < $NIMU} {incr i} {
    create_bd_cell -type ip -vlnv user.org:user:nav_spi_ctrl:1.0 nav_spi_ctrl_$i
}
create_bd_cell -type ip -vlnv xilinx.com:hls:r2f_mux_top:1.0 r2f_mux
create_bd_cell -type ip -vlnv xilinx.com:hls:madgwick_multi_top:1.0 mad_multi

# --- Reloj y reset -----------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins proc_sys_reset_0/slowest_sync_clk]
connect_bd_net [get_bd_pins ps7/FCLK_RESET0_N] [get_bd_pins proc_sys_reset_0/ext_reset_in]

set clk_pins [list ps7/M_AXI_GP0_ACLK r2f_mux/ap_clk mad_multi/ap_clk]
set rst_pins [list r2f_mux/ap_rst_n mad_multi/ap_rst_n]
for {set i 0} {$i < $NIMU} {incr i} {
    lappend clk_pins nav_spi_ctrl_$i/clk
    lappend rst_pins nav_spi_ctrl_$i/rstn
}
foreach p $clk_pins { connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins $p] }
foreach p $rst_pins { connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] [get_bd_pins $p] }

# --- AXI-Lite: PS -> SmartConnect -> NIMU nav_spi_ctrl + r2f_mux + mad_multi -
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 axi_lite_sc
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI [expr {$NIMU + 2}]] [get_bd_cells axi_lite_sc]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins axi_lite_sc/aclk]
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] [get_bd_pins axi_lite_sc/aresetn]
connect_bd_intf_net [get_bd_intf_pins ps7/M_AXI_GP0] [get_bd_intf_pins axi_lite_sc/S00_AXI]
for {set i 0} {$i < $NIMU} {incr i} {
    connect_bd_intf_net [get_bd_intf_pins axi_lite_sc/M0${i}_AXI] [get_bd_intf_pins nav_spi_ctrl_$i/s_axi]
}
connect_bd_intf_net [get_bd_intf_pins axi_lite_sc/M0${NIMU}_AXI] [get_bd_intf_pins mad_multi/s_axi_ctrl]
connect_bd_intf_net [get_bd_intf_pins axi_lite_sc/M0[expr {$NIMU + 1}]_AXI] [get_bd_intf_pins r2f_mux/s_axi_ctrl]

# --- AXI4-Stream: nav_spi_ctrl_i -> r2f_mux/s_axisI ; r2f_mux -> mad_multi ----
for {set i 0} {$i < $NIMU} {incr i} {
    connect_bd_intf_net [get_bd_intf_pins nav_spi_ctrl_$i/m_axis] [get_bd_intf_pins r2f_mux/s_axis$i]
}
connect_bd_intf_net [get_bd_intf_pins r2f_mux/m_axis] [get_bd_intf_pins mad_multi/s_axis_imu]

# --- Pines SPI externos (nombres fijos: spi_*_0 .. spi_*_3, los usa el XDC) ----
for {set i 0} {$i < $NIMU} {incr i} {
    foreach s {sclk mosi miso cs_n} {
        make_bd_pins_external -name spi_${s}_$i [get_bd_pins nav_spi_ctrl_$i/spi_$s]
    }
}

# --- Direcciones fijas (las usa main.c) -----------------------------------------
#   nav_spi_ctrl_i 0x40000000 + 0x10000*i (4K) ; mad_multi 0x40040000 ; r2f_mux 0x40050000
if {[catch {
    for {set i 0} {$i < $NIMU} {incr i} {
        assign_bd_address -offset [expr {0x40000000 + 0x10000 * $i}] -range 0x1000 \
            -target_address_space [get_bd_addr_spaces ps7/Data] [get_bd_addr_segs nav_spi_ctrl_$i/s_axi/reg0]
    }
    assign_bd_address -offset 0x40040000 -range 0x10000 -target_address_space [get_bd_addr_spaces ps7/Data] [get_bd_addr_segs mad_multi/s_axi_ctrl/Reg]
    assign_bd_address -offset 0x40050000 -range 0x10000 -target_address_space [get_bd_addr_spaces ps7/Data] [get_bd_addr_segs r2f_mux/s_axi_ctrl/Reg]
} err]} {
    puts "AVISO: asignacion explicita de direcciones fallo ($err); se usa la automatica."
    puts "       Revisa las direcciones impresas por build_project.tcl y ajusta main.c."
    assign_bd_address
}

validate_bd_design
save_bd_design

make_wrapper -files [get_files ./${PROJ_NAME}/${PROJ_NAME}.srcs/sources_1/bd/system/system.bd] -top
add_files -norecurse ./${PROJ_NAME}/${PROJ_NAME}.gen/sources_1/bd/system/hdl/system_wrapper.v
set_property top system_wrapper [current_fileset]

add_files -fileset constrs_1 -norecurse [file normalize "./zybo_z7_10_pmod_nav_x4.xdc"]
update_compile_order -fileset sources_1

puts ""
puts "Proyecto creado: ./${PROJ_NAME}"
puts "Revisa zybo_z7_10_pmod_nav_x4.xdc (pines de los 4 Pmod NAV) antes de sintetizar."
puts "Para generar bitstream: source build_project.tcl"
