# Uso: vitis_hls -f run_hls_raw2float.tcl
open_project -reset raw2float_prj
set_top raw2float_top
add_files raw2float.cpp
add_files -tb tb_raw2float.cpp

open_solution -reset sol1 -flow_target vivado
set_part {xc7z010clg400-1}            ;# Zybo (Zynq-7010)
create_clock -period 10 -name default ;# 100 MHz

csim_design
csynth_design
export_design -format ip_catalog
exit
