# Uso: vitis_hls -f run_hls_madgwick_multi.tcl
# Nucleo Madgwick opt2 con UN estado por IMU (hasta 4), identificadas por TID.
# csim: 4 IMUs sinteticas -> r2f_mux_top -> madgwick_multi_top, comparado bit a bit con 4 nucleos de una IMU.
open_project -reset madgwick_multi_prj
set_top madgwick_multi_top
add_files madgwick_multi.cpp
add_files madgwick_multi.h
add_files madgwick_prog.h
add_files -tb tb_multi.cpp
add_files -tb madgwick_single.cpp
add_files -tb r2f_mux.cpp
add_files -tb r2f_mux.h

open_solution -reset sol1 -flow_target vivado
set_part {xc7z010clg400-1}
create_clock -period 10 -name default

csim_design
csynth_design
export_design -format ip_catalog
exit
