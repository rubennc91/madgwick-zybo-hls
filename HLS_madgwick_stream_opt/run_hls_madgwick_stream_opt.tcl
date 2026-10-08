# Uso: vitis_hls -f run_hls_madgwick_stream_opt.tcl
# Igual que run_hls_madgwick_stream.tcl: copia aqui madgwick_f2.c y madgwick_f2.h
# (los originales de tu proyecto) para el testbench.
open_project -reset madgwick_stream_opt_prj
set_top madgwick_stream_top
add_files madgwick_stream.cpp
add_files -tb tb_madgwick_stream.cpp
add_files -tb madgwick_f2.c
add_files -tb madgwick_f2.h

open_solution -reset sol1 -flow_target vivado
set_part {xc7z010clg400-1}
create_clock -period 10 -name default

csim_design
csynth_design
# Resumen util: latencia y recursos (compara con el de la version original)
puts "=== Informe: madgwick_stream_opt_prj/sol1/syn/report/madgwick_stream_top_csynth.rpt ==="
export_design -format ip_catalog
exit
