# Uso: vivado -mode batch -source create_project.tcl
#
# LINEA BASE EN SOFTWARE DE EXTREMO A EXTREMO (v0.6):
# el ARM hace TODO. En la PL solo hay un AXI Quad SPI por Pmod NAV (modo estandar, 2 chip selects) y nada mas:
#   Pmod NAV i -- AXI Quad SPI i (SCK = 100 MHz / 32 = 3,125 MHz, modo SPI 3) -- AXI-Lite -- PS (ARM)
# No hay nav_spi_ctrl, ni arbitro, ni nucleo Madgwick: la adquisicion por SPI, la conversion y el filtro corren en el ARM.
# Es la forma habitual de hacerlo (un AXI Quad SPI por sensor; ~700 LUT por sensor, como en el sistema de Galindo).
#
# Pines: los mismos conectores que Integration_Zybo_multi (IMU0 = JE, IMU1 = JD, IMU2 = JC, IMU3 = JA).
# Cambia NIMU (1..4) segun las placas conectadas.

set PART      "xc7z010clg400-1"
set PROJ_NAME "zybo_swspi"
set NIMU      2

create_project -force $PROJ_NAME ./$PROJ_NAME -part $PART

set zybo_boards [get_board_parts -quiet *zybo-z7-10*]
if {[llength $zybo_boards] == 0} {
    puts "ERROR: no encuentro los board files de la Zybo Z7-10 (get_board_parts *zybo*)."
    close_project
    return
}
set ZYBO_BOARD [lindex [lsort -dictionary $zybo_boards] end]
puts "Usando board_part: $ZYBO_BOARD"
set_property board_part $ZYBO_BOARD [current_project]

create_bd_design "system"

create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} [get_bd_cells ps7]
set_property -dict [list CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} CONFIG.PCW_USE_M_AXI_GP0 {1}] [get_bd_cells ps7]

create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins proc_sys_reset_0/slowest_sync_clk]
connect_bd_net [get_bd_pins ps7/FCLK_RESET0_N] [get_bd_pins proc_sys_reset_0/ext_reset_in]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins ps7/M_AXI_GP0_ACLK]

create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 axi_lite_sc
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI $NIMU] [get_bd_cells axi_lite_sc]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins axi_lite_sc/aclk]
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] [get_bd_pins axi_lite_sc/aresetn]
connect_bd_intf_net [get_bd_intf_pins ps7/M_AXI_GP0] [get_bd_intf_pins axi_lite_sc/S00_AXI]

for {set i 0} {$i < $NIMU} {incr i} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:axi_quad_spi:3.2 qspi_$i
    # Modo estandar (registros SPICR/SPISR/SPIDTR/SPIDRR), FIFO de 16, 2 chip selects (CS_AG y CS_M), SCK = ext_spi_clk / 32.
    # Cada propiedad por separado: si una falla, las demas se aplican igualmente (antes un solo error anulaba todo el set_property).
    foreach {p v} {CONFIG.C_SPI_MODE 0  CONFIG.C_NUM_SS_BITS 2  CONFIG.C_NUM_TRANSFER_BITS 8  CONFIG.C_FIFO_DEPTH 16 \
                   CONFIG.C_TYPE_OF_AXI4_INTERFACE 0  CONFIG.C_USE_STARTUP 0  CONFIG.C_SCK_RATIO 32} {
        if {[catch {set_property $p $v [get_bd_cells qspi_$i]} err]} {
            puts "AVISO: qspi_$i: no pude fijar $p=$v ($err)"
        }
    }
    connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins qspi_$i/s_axi_aclk]
    connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins qspi_$i/ext_spi_clk]
    connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] [get_bd_pins qspi_$i/s_axi_aresetn]
    connect_bd_intf_net [get_bd_intf_pins axi_lite_sc/M0${i}_AXI] [get_bd_intf_pins qspi_$i/AXI_LITE]
    # Puerto externo del interfaz SPI: Vivado genera en el wrapper spi_i_io0_io (MOSI), spi_i_io1_io (MISO),
    # spi_i_sck_io y spi_i_ss_io[1:0] (con los IOBUF incluidos). Si los nombres cambian, ajusta el XDC.
    make_bd_intf_pins_external -name spi_$i [get_bd_intf_pins qspi_$i/SPI_0]
}

# Direcciones fijas (las usa main.c): qspi_i en 0x40000000 + 0x10000 * i
if {[catch {
    for {set i 0} {$i < $NIMU} {incr i} {
        assign_bd_address -offset [expr {0x40000000 + 0x10000 * $i}] -range 0x10000 \
            -target_address_space [get_bd_addr_spaces ps7/Data] [get_bd_addr_segs qspi_$i/AXI_LITE/Reg]
    }
} err]} {
    puts "AVISO: la asignacion explicita de direcciones fallo ($err); se usa la automatica."
    puts "       Revisa las direcciones impresas por build_project.tcl y ajusta QSPI_BASE en main.c."
    assign_bd_address
}


# Comprobacion: lo que el firmware supone (2 bits de SS, SCK = 100/32 MHz, sin STARTUP). Los bits de SS son criticos: sin ellos el magnetometro no recibe CS.
set bad 0
for {set i 0} {$i < $NIMU} {incr i} {
    set c [get_bd_cells qspi_$i]
    set ss [get_property CONFIG.C_NUM_SS_BITS $c]; set ra [get_property CONFIG.C_SCK_RATIO $c]; set st [get_property CONFIG.C_USE_STARTUP $c]
    puts "qspi_$i: C_NUM_SS_BITS=$ss C_SCK_RATIO=$ra C_USE_STARTUP=$st"
    if {$ss != 2} { puts "ERROR: qspi_$i tiene $ss bits de SS (se necesitan 2)"; incr bad }
    if {$ra != 32} { puts "AVISO: qspi_$i tiene C_SCK_RATIO=$ra (se espera 32 = 3,125 MHz; en la GUI: 16 x 2). Si es otro, ajusta el texto de main.c o el IP." }
    if {$st != 0} { puts "AVISO: qspi_$i tiene STARTUP activado (debe estar desmarcado)" }
}
if {$bad > 0} { error "Parametros del AXI Quad SPI incorrectos: corrigelos en el block design (qspi_0, qspi_1) antes de sintetizar." }

validate_bd_design
save_bd_design

make_wrapper -files [get_files ./${PROJ_NAME}/${PROJ_NAME}.srcs/sources_1/bd/system/system.bd] -top
add_files -norecurse ./${PROJ_NAME}/${PROJ_NAME}.gen/sources_1/bd/system/hdl/system_wrapper.v
set_property top system_wrapper [current_fileset]

add_files -fileset constrs_1 -norecurse [file normalize "./zybo_z7_10_swspi.xdc"]
update_compile_order -fileset sources_1

puts ""
puts "Proyecto creado: ./${PROJ_NAME}"
puts "Antes de sintetizar: abre ./${PROJ_NAME}/${PROJ_NAME}.gen/sources_1/bd/system/hdl/system_wrapper.v y comprueba que los puertos"
puts "se llaman spi_<i>_io0_io, spi_<i>_io1_io, spi_<i>_sck_io y spi_<i>_ss_io (ver zybo_z7_10_swspi.xdc)."
puts "Para generar bitstream y .xsa: source build_project.tcl"