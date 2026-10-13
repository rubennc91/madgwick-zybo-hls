# Uso: vitis_hls -f run_hls_r2f_tid.tcl
# raw2float con TID de paso (un solo puerto de entrada). El reparto de las 4 IMUs lo hace
# axis_rr_mux4.vhd (en Integration_Zybo_multi). El banco de pruebas conjunto esta en
# run_hls_madgwick_multi.tcl.
open_project -reset r2f_tid_prj
set_top r2f_tid_top
add_files r2f_tid.cpp
add_files r2f_tid.h

open_solution -reset sol1 -flow_target vivado
set_part {xc7z010clg400-1}
create_clock -period 10 -name default

csynth_design
export_design -format ip_catalog
exit
