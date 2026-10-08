# Uso: vivado -mode batch -source create_project.tcl
#
# Crea el proyecto de integracion completo para la Zybo Z7-10:
#   nav_spi_ctrl (VHDL, empaquetado como IP) -> raw2float_top (HLS) ->
#   madgwick_stream_top (HLS), todo controlado por un Zynq-7 Processing
#   System via AXI-Lite, con el camino de datos en AXI4-Stream punto a punto
#   (sin DMA: solo 9 palabras por trama, el PS no necesita tocarlas).
#
# Estructura de carpetas esperada (ajusta set NAV_IP_REPO / HLS *_IP_REPO si
# la tuya es distinta):
#   Madgwick_Filter_HLS/
#     PMOD_NAV_Ctrller/ip_repo/nav_spi_ctrl_v1.0/        (tras package_nav_ip.tcl)
#     HLS_raw2float/raw2float_prj/sol1/impl/ip/           (tras export_design)
#     HLS_madgwick_stream/madgwick_stream_prj/sol1/impl/ip/
#     Integration_Zybo/   <- este script se ejecuta desde aqui

set PART      "xc7z010clg400-1"
set PROJ_NAME "zybo_opt2"

set NAV_IP_REPO        [file normalize "../PMOD_NAV_Ctrller/ip_repo"]
set RAW2FLOAT_IP_REPO  [file normalize "../HLS_raw2float/raw2float_prj/sol1/impl/ip"]
set MADGWICK_IP_REPO   [file normalize "../HLS_madgwick_stream_opt2/madgwick_stream_opt2_prj/sol1/impl/ip"]

foreach p [list $NAV_IP_REPO $RAW2FLOAT_IP_REPO $MADGWICK_IP_REPO] {
    if {![file isdirectory $p]} {
        puts "AVISO: no existe '$p'."
        puts "  -> Ejecuta antes package_nav_ip.tcl (PMOD_NAV_Ctrller) y los dos"
        puts "     run_hls_*.tcl (export_design) para generar estos repositorios de IP."
    }
}

create_project -force $PROJ_NAME ./$PROJ_NAME -part $PART

# Board files de Digilent para la Zybo Z7-10: OBLIGATORIOS para que el PS7
# quede bien configurado (DDR3L de la placa, UART1 en MIO48/49 para el
# USB-UART, SD, QSPI, Ethernet...). Sin ellos el PS7 queda con la
# configuracion generica por defecto: sin UART (no hay printf) y con una
# memoria DDR que no es la de la placa.
# Instalacion: copiar la carpeta new/board_files/zybo-z7-10 del repositorio
# https://github.com/Digilent/vivado-boards a
#   C:/Xilinx/Vivado/2023.2/data/boards/board_files/
# y reiniciar Vivado.
set zybo_boards [get_board_parts -quiet *zybo-z7-10*]
if {[llength $zybo_boards] == 0} {
    puts "ERROR: no encuentro los board files de la Zybo Z7-10."
    puts "  -> Instalalos como se explica en el comentario de arriba y reinicia Vivado."
    puts "  -> Comprueba con: get_board_parts *zybo*"
    close_project
    return
}
set ZYBO_BOARD [lindex [lsort -dictionary $zybo_boards] end]
puts "Usando board_part: $ZYBO_BOARD"
set_property board_part $ZYBO_BOARD [current_project]

set_property ip_repo_paths [list $NAV_IP_REPO $RAW2FLOAT_IP_REPO $MADGWICK_IP_REPO] [current_project]
update_ip_catalog

create_bd_design "system"

# ---------------------------------------------------------------------------
# Zynq-7 Processing System
# ---------------------------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" \
             Master "Disable" Slave "Disable"} [get_bd_cells ps7]

# FCLK_CLK0 a 100 MHz para todo el sistema (PL + logica AXI).
set_property -dict [list CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100}] [get_bd_cells ps7]
# Habilita el puerto maestro AXI-Lite general (el PS controla los
# perifericos de la PL) y la entrada de interrupciones PL->PS.
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_USE_FABRIC_INTERRUPT {1} \
    CONFIG.PCW_IRQ_F2P_INTR {1} \
] [get_bd_cells ps7]

# ---------------------------------------------------------------------------
# IPs del camino de datos
# ---------------------------------------------------------------------------
create_bd_cell -type ip -vlnv user.org:user:nav_spi_ctrl:1.0 nav_spi_ctrl_0
create_bd_cell -type ip -vlnv xilinx.com:hls:raw2float_top:1.0 raw2float_top_0
create_bd_cell -type ip -vlnv xilinx.com:hls:madgwick_stream_top:1.0 madgwick_stream_top_0

# ---------------------------------------------------------------------------
# Reset de la PL (derivado del reset del PS)
# ---------------------------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins proc_sys_reset_0/slowest_sync_clk]
connect_bd_net [get_bd_pins ps7/FCLK_RESET0_N] [get_bd_pins proc_sys_reset_0/ext_reset_in]

# nav_spi_ctrl.vhd declara sus puertos de reloj/reset como "clk"/"rstn"
# (genericos, no con el prefijo s_axi_*, segun confirma el log de
# package_nav_ip.tcl: "Inferred bus interface 'clk' ..." / "'rstn' ...").
# Los dos nucleos HLS si usan la convencion ap_clk/ap_rst_n.
foreach clk_pin {nav_spi_ctrl_0/clk raw2float_top_0/ap_clk madgwick_stream_top_0/ap_clk ps7/M_AXI_GP0_ACLK} {
    connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins $clk_pin]
}
foreach rst_pin {nav_spi_ctrl_0/rstn raw2float_top_0/ap_rst_n madgwick_stream_top_0/ap_rst_n} {
    connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] [get_bd_pins $rst_pin]
}

# ---------------------------------------------------------------------------
# AXI-Lite: PS (M_AXI_GP0) -> SmartConnect -> 3 esclavos de control
# ---------------------------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 axi_lite_sc
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {3}] [get_bd_cells axi_lite_sc]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins axi_lite_sc/aclk]
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] [get_bd_pins axi_lite_sc/aresetn]

connect_bd_intf_net [get_bd_intf_pins ps7/M_AXI_GP0] [get_bd_intf_pins axi_lite_sc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_lite_sc/M00_AXI] [get_bd_intf_pins nav_spi_ctrl_0/s_axi]
connect_bd_intf_net [get_bd_intf_pins axi_lite_sc/M01_AXI] [get_bd_intf_pins raw2float_top_0/s_axi_ctrl]
connect_bd_intf_net [get_bd_intf_pins axi_lite_sc/M02_AXI] [get_bd_intf_pins madgwick_stream_top_0/s_axi_ctrl]

# ---------------------------------------------------------------------------
# AXI4-Stream punto a punto: nav_spi_ctrl -> raw2float -> madgwick_stream
# (conexion directa, sin interconnect: un solo maestro por esclavo)
# ---------------------------------------------------------------------------
connect_bd_intf_net [get_bd_intf_pins nav_spi_ctrl_0/m_axis] [get_bd_intf_pins raw2float_top_0/s_axis]
connect_bd_intf_net [get_bd_intf_pins raw2float_top_0/m_axis] [get_bd_intf_pins madgwick_stream_top_0/s_axis_imu]

# ---------------------------------------------------------------------------
# Interrupcion de nav_spi_ctrl -> PS (IRQ_F2P)
# ---------------------------------------------------------------------------
connect_bd_net [get_bd_pins nav_spi_ctrl_0/irq] [get_bd_pins ps7/IRQ_F2P]

# ---------------------------------------------------------------------------
# Señales físicas SPI hacia el exterior (Pmod NAV) -- se amarran al XDC
# ---------------------------------------------------------------------------
make_bd_pins_external  [get_bd_pins nav_spi_ctrl_0/spi_sclk]
make_bd_pins_external  [get_bd_pins nav_spi_ctrl_0/spi_mosi]
make_bd_pins_external  [get_bd_pins nav_spi_ctrl_0/spi_miso]
make_bd_pins_external  [get_bd_pins nav_spi_ctrl_0/spi_cs_n]

# Asigna automaticamente en el mapa de memoria del PS (/ps7/Data) los 3
# segmentos AXI-Lite que quedaron sin asignar (nav_spi_ctrl, raw2float_top,
# madgwick_stream_top) -- si no se hace esto, el SmartConnect no sabe que
# direccion le corresponde a cada esclavo y la implementacion fallaria mas
# adelante con un "black box".
assign_bd_address

validate_bd_design
save_bd_design

make_wrapper -files [get_files ./${PROJ_NAME}/${PROJ_NAME}.srcs/sources_1/bd/system/system.bd] -top
add_files -norecurse ./${PROJ_NAME}/${PROJ_NAME}.gen/sources_1/bd/system/hdl/system_wrapper.v
set_property top system_wrapper [current_fileset]

add_files -fileset constrs_1 -norecurse [file normalize "./zybo_z7_10_pmod_nav.xdc"]

update_compile_order -fileset sources_1

puts ""
puts "Proyecto creado: ./${PROJ_NAME}"
puts "Revisa/edita zybo_z7_10_pmod_nav.xdc (pines del Pmod NAV) antes de sintetizar."
puts "Para generar bitstream: launch_runs impl_1 -to_step write_bitstream -jobs 4"
