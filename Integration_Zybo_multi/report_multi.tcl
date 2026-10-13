# Uso (desde Integration_Zybo_multi, con la implementacion ya hecha):
#   vivado -mode batch -source report_multi.tcl
# Escribe en ./reports/ el timing, la utilizacion (total y por jerarquia) y la potencia post-implementacion.
set PROJ_NAME "zybo_multi"
open_project ./${PROJ_NAME}/${PROJ_NAME}.xpr
open_run impl_1
file mkdir reports
report_timing_summary -delay_type min_max -max_paths 10 -file reports/timing_multi.rpt
report_utilization                                        -file reports/util_multi.rpt
report_utilization -hierarchical -hierarchical_depth 3    -file reports/util_hier_multi.rpt
report_power                                              -file reports/power_multi.rpt
puts "WNS = [get_property SLACK [get_timing_paths -max_paths 1 -setup]]  WHS = [get_property SLACK [get_timing_paths -max_paths 1 -hold]]"
puts "Informes escritos en ./reports/"
