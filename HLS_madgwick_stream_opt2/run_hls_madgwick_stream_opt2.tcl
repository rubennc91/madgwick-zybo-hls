# Uso: vitis_hls -f run_hls_madgwick_stream_opt2.tcl
# Testbench y referencia madgwick_f2.c/.h se reutilizan de ../HLS_madgwick_stream
# (la interfaz de madgwick_stream.h es identica).
# madgwick_prog.h lo genera gen_prog.py (python3 gen_prog.py > madgwick_prog.h).
open_project -reset madgwick_stream_opt2_prj
set_top madgwick_stream_top
add_files madgwick_stream.cpp
add_files madgwick_stream.h
add_files madgwick_prog.h
add_files -tb ../HLS_madgwick_stream/tb_madgwick_stream.cpp
add_files -tb ../HLS_madgwick_stream/madgwick_f2.c
add_files -tb ../HLS_madgwick_stream/madgwick_f2.h

open_solution -reset sol1 -flow_target vivado
set_part {xc7z010clg400-1}
create_clock -period 10 -name default

csim_design
csynth_design
puts "=== Informe: madgwick_stream_opt2_prj/sol1/syn/report/madgwick_stream_top_csynth.rpt ==="
export_design -format ip_catalog
exit
