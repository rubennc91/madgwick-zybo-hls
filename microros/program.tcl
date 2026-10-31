# Programa la Zybo con el bitstream de Integration_Zybo_multi y un .elf de micro-ROS, sin abrir Vitis.
# Uso (en una terminal con el entorno de Vitis, p. ej. "Vitis 2023.2 Command Prompt"):
#   xsct program.tcl system_multi.bit ps7_init.tcl microros_udp_119_100ms.elf
# Sin probar en la placa en el momento de escribirlo; sigue la secuencia habitual de xsct para el Zynq-7000.
if {$argc < 3} { puts "uso: xsct program.tcl <bitstream.bit> <ps7_init.tcl> <programa.elf>"; exit 1 }
set bit  [lindex $argv 0]
set init [lindex $argv 1]
set elf  [lindex $argv 2]

connect
targets -set -nocase -filter {name =~ "APU*"}
rst -system
after 1000
fpga -file $bit
targets -set -nocase -filter {name =~ "APU*"}
source $init
ps7_init
ps7_post_config
targets -set -nocase -filter {name =~ "ARM*#0"}
dow $elf
con
puts "Programa en ejecucion. Deja las placas quietas ~10 s mientras se mide el sesgo del giroscopio."
disconnect
