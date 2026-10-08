# Uso: vivado -mode batch -source run_sim_spi_engine.tcl
# (o abrelo en el Tcl Console de Vivado con "source run_sim_spi_engine.tcl")
#
# Crea un proyecto de simulacion para probar spi_engine de forma aislada
# y lanza xsim.

create_project -force spi_engine_sim ./spi_engine_sim -part xc7z010clg400-1

add_files -norecurse {spi_engine.vhd}
set_property file_type {VHDL 2008} [get_files spi_engine.vhd]

add_files -fileset sim_1 -norecurse {tb_spi_engine.vhd}
set_property file_type {VHDL 2008} [get_files tb_spi_engine.vhd]
set_property top tb_spi_engine [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

launch_simulation
run 5 us

# Para ver las formas de onda a mano en vez de solo el log:
#   En la consola Tcl de Vivado, tras "launch_simulation":
#     add_wave -recursive /tb_spi_engine/*
#     run 5 us
