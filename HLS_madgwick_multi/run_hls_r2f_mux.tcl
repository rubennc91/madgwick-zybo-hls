# Uso: vitis_hls -f run_hls_r2f_mux.tcl
# raw2float + multiplexor de 4 IMUs (round-robin, TID = indice de la IMU).
# No ejecuta csim aqui: el banco de pruebas conjunto esta en run_hls_madgwick_multi.tcl.
open_project -reset r2f_mux_prj
set_top r2f_mux_top
add_files r2f_mux.cpp
add_files r2f_mux.h

open_solution -reset sol1 -flow_target vivado
set_part {xc7z010clg400-1}
create_clock -period 10 -name default

csynth_design
export_design -format ip_catalog
exit
