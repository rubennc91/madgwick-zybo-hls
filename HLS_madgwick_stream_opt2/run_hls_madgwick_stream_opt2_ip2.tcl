# Uso: vitis_hls -f run_hls_madgwick_stream_opt2_ip2.tcl
#
# Genera el MISMO nucleo opt2, pero empaquetado como IP "mad_opt2" (en lugar de
# "madgwick_stream_top"). Sirve para instanciar a la vez el nucleo original y el
# opt2 en el mismo diseno de bloques (Integration_Zybo_dual): dos IP con el mismo
# nombre (VLNV) no pueden convivir en un catalogo de Vivado.
# El codigo fuente no cambia: -Dmadgwick_stream_top=mad_opt2 renombra solo la
# funcion principal al compilar. No ejecuta csim (ya validado con el nombre
# original en run_hls_madgwick_stream_opt2.tcl).
open_project -reset mad_opt2_prj
set_top mad_opt2
add_files madgwick_stream.cpp -cflags "-Dmadgwick_stream_top=mad_opt2"
add_files madgwick_stream.h
add_files madgwick_prog.h

open_solution -reset sol1 -flow_target vivado
set_part {xc7z010clg400-1}
create_clock -period 10 -name default

csynth_design
export_design -format ip_catalog
exit
