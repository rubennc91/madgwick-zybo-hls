# Uso: source build_project.tcl
# (ejecutar desde la carpeta Integration_Zybo, con Vivado ya abierto en modo
# interactivo -- vivado -mode tcl -- y el proyecto YA CREADO por
# create_project.tcl en una ejecucion anterior)
#
# Abre el proyecto existente (no lo vuelve a crear) y encadena: comprobacion
# de direcciones AXI-Lite, sintesis, implementacion + bitstream, reportes de
# utilizacion/timing, y exportacion del .xsa para Vitis.
#
# Si una ejecucion anterior de synth_1/impl_1 se quedo "colgada" o a medias,
# este script la resetea primero con reset_run para evitar arrancar sobre un
# estado inconsistente.

set PROJ_NAME "madgwick_zybo_z710"
set XPR_PATH  "./${PROJ_NAME}/${PROJ_NAME}.xpr"

if {[catch {current_project}]} {
    if {![file exists $XPR_PATH]} {
        puts "ERROR: no encuentro '$XPR_PATH'."
        puts "  -> Ejecuta primero 'source create_project.tcl' desde esta carpeta,"
        puts "     o ajusta PROJ_NAME/XPR_PATH si tu proyecto se llama distinto."
        return
    }
    open_project $XPR_PATH
    puts "Proyecto abierto: $XPR_PATH"
} else {
    puts "Ya habia un proyecto abierto: [current_project]"
}

# ---------------------------------------------------------------------------
# 1) Direcciones AXI-Lite
# ---------------------------------------------------------------------------
# get_bd_addr_segs necesita el diseno de bloques ABIERTO en memoria, no solo
# el proyecto -- open_project no lo abre automaticamente.
set bd_file "./${PROJ_NAME}/${PROJ_NAME}.srcs/sources_1/bd/system/system.bd"
if {[get_bd_designs -quiet system] eq ""} {
    open_bd_design $bd_file
}

puts "\n--- Direcciones AXI-Lite asignadas ---"
foreach seg [get_bd_addr_segs] {
    puts "  [get_property NAME $seg] -> offset [get_property OFFSET $seg]  range [get_property RANGE $seg]"
}

# ---------------------------------------------------------------------------
# 2) Sintesis
# ---------------------------------------------------------------------------
reset_run synth_1
puts "\n--- Lanzando synth_1 ---"
launch_runs synth_1 -jobs 4
wait_on_run synth_1

set synth_status [get_property STATUS [get_runs synth_1]]
set synth_progress [get_property PROGRESS [get_runs synth_1]]
puts "synth_1: $synth_progress / $synth_status"
if {[string match "*ERROR*" $synth_status] || $synth_progress != "100%"} {
    puts "ERROR: la sintesis no termino bien. Revisa el log en:"
    puts "  ./${PROJ_NAME}/${PROJ_NAME}.runs/synth_1/runme.log"
    return
}

# ---------------------------------------------------------------------------
# 3) Implementacion + bitstream
# ---------------------------------------------------------------------------
reset_run impl_1
puts "\n--- Lanzando impl_1 (hasta write_bitstream) ---"
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

set impl_status [get_property STATUS [get_runs impl_1]]
set impl_progress [get_property PROGRESS [get_runs impl_1]]
puts "impl_1: $impl_progress / $impl_status"
if {[string match "*ERROR*" $impl_status] || $impl_progress != "100%"} {
    puts "ERROR: la implementacion no termino bien. Revisa el log en:"
    puts "  ./${PROJ_NAME}/${PROJ_NAME}.runs/impl_1/runme.log"
    return
}

# ---------------------------------------------------------------------------
# 4) Utilizacion y timing
# ---------------------------------------------------------------------------
open_run impl_1
puts "\n--- Utilizacion de recursos ---"
report_utilization

puts "\n--- Resumen de timing (busca el WNS, Worst Negative Slack) ---"
report_timing_summary -delay_type min_max

# ---------------------------------------------------------------------------
# 5) Exportar .xsa para Vitis
# ---------------------------------------------------------------------------
write_hw_platform -fixed -include_bit -force ./${PROJ_NAME}.xsa
puts "\nHardware exportado: ./${PROJ_NAME}.xsa"
puts "Listo para importar en Vitis."