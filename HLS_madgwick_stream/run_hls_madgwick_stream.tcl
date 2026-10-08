# Uso: vitis_hls -f run_hls_madgwick_stream.tcl
# Copia aqui madgwick_f2.c y madgwick_f2.h (los originales de tu proyecto)
# antes de ejecutar, igual que hiciste para run_hls_1imu.tcl.
open_project -reset madgwick_stream_prj
set_top madgwick_stream_top
add_files madgwick_stream.cpp
add_files -tb tb_madgwick_stream.cpp
add_files -tb madgwick_f2.c
add_files -tb madgwick_f2.h

open_solution -reset sol1 -flow_target vivado
set_part {xc7z010clg400-1}             ;# Zybo (Zynq-7010)
create_clock -period 10 -name default  ;# 100 MHz

csim_design
csynth_design
export_design -format ip_catalog
exit
