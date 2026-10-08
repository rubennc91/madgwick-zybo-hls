# Uso: vivado -mode batch -source package_nav_ip.tcl
#
# Empaqueta nav_pkg.vhd + spi_engine.vhd + nav_spi_ctrl.vhd (ya simulados y
# verificados en Vivado xsim) como un IP reutilizable de Vivado, listo para
# instanciar en el diseno de bloques (block design) del proyecto de
# integracion. Usa ipx::package_project para que todo quede scripted, sin
# pasar por el asistente grafico "Create and Package New IP".
#
# IMPORTANTE: ejecutar este script estando en la carpeta PMOD_NAV_Ctrller
# (donde estan nav_pkg.vhd, spi_engine.vhd, nav_spi_ctrl.vhd), o ajustar las
# rutas de add_files mas abajo.

set ip_name    "nav_spi_ctrl"
set ip_version "1.0"
set ip_dir     [file normalize "./ip_repo/${ip_name}_v${ip_version}"]
set part       "xc7z010clg400-1"

file delete -force $ip_dir
file mkdir $ip_dir

create_project -force ${ip_name}_pkg_prj ./${ip_name}_pkg_prj -part $part

add_files -norecurse {nav_pkg.vhd spi_engine.vhd nav_spi_ctrl.vhd}
set_property file_type {VHDL 2008} [get_files {nav_pkg.vhd spi_engine.vhd nav_spi_ctrl.vhd}]
update_compile_order -fileset sources_1

ipx::package_project -root_dir $ip_dir -vendor user.org -library user -taxonomy /UserIP \
    -import_files -set_current true

set core [ipx::current_core]
set_property name          $ip_name                                   $core
set_property display_name  "NAV SPI Controller (LSM9DS1 / Pmod NAV)"  $core
set_property description   "FSM de adquisicion SPI del LSM9DS1 (Pmod NAV): configuracion automatica, lectura en rafaga por canal AXI4-Stream, registros de control/estado por AXI4-Lite." $core
set_property vendor_display_name "Proyecto Madgwick FPGA" $core
set_property version       $ip_version                                $core
# (No existe la propiedad 'top_level_hdl_file' en un componente IP-XACT;
#  el top ya queda fijado por ipx::package_project a partir del top del
#  proyecto -- aqui era nav_spi_ctrl.vhd porque es el unico que no es
#  instanciado por ningun otro de los tres ficheros, asi que no hace falta
#  fijarlo a mano.)

# Las interfaces AXI4-Lite (s_axi_*) y AXI4-Stream (m_axis_*) se detectan
# automaticamente por el nombre de los puertos; si el asistente no las
# infiere bien, se pueden forzar manualmente aqui, por ejemplo:
#   ipx::infer_bus_interface {s_axi_awaddr s_axi_awvalid ...} \
#       xilinx.com:interface:aximm_rtl:1.0 $core
# En la practica, con el prefijo estandar s_axi_/m_axis_ y las senales
# tpico (awvalid, awready, wdata, ... / tdata, tvalid, tready, tlast),
# Vivado las reconoce sin intervencion.

ipx::create_xgui_files        $core
ipx::update_checksums         $core
ipx::save_core                $core

close_project

puts "IP '$ip_name' v$ip_version empaquetado en: $ip_dir"
puts "Anade este repositorio de IP al proyecto de integracion con:"
puts "  set_property ip_repo_paths {$ip_dir/..} \[current_project\]"
puts "  update_ip_catalog"