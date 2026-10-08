# Uso: vivado -mode batch -source run_sim_nav_spi_ctrl.tcl
# (o abrelo en el Tcl Console de Vivado con "source run_sim_nav_spi_ctrl.tcl")
#
# Crea un proyecto de simulacion para la FSM completa (nav_spi_ctrl) con el
# modelo de esclavo LSM9DS1 del testbench, y lanza xsim.

create_project -force nav_spi_ctrl_sim ./nav_spi_ctrl_sim -part xc7z010clg400-1

add_files -norecurse {nav_pkg.vhd spi_engine.vhd nav_spi_ctrl.vhd}
set_property file_type {VHDL 2008} [get_files {nav_pkg.vhd spi_engine.vhd nav_spi_ctrl.vhd}]

add_files -fileset sim_1 -norecurse {tb_nav_spi_ctrl.vhd}
set_property file_type {VHDL 2008} [get_files tb_nav_spi_ctrl.vhd]
set_property top tb_nav_spi_ctrl [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

launch_simulation
run 300 us

# Para ver las formas de onda a mano en vez de solo el log:
#   En la consola Tcl de Vivado, tras "launch_simulation":
#     add_wave -recursive /tb_nav_spi_ctrl/*
#     run 300 us
#
# Comprobar en el log de la consola (o con "get_msg_config") los REPORT y
# ASSERT: "TEST FALLIDO"/"FALLO ..." en severity error indican un problema;
# si solo aparecen los "report" informativos sin ningun ERROR, la prueba
# basica (config + primera rafaga de muestras + SAMPLE_COUNT) ha pasado.
