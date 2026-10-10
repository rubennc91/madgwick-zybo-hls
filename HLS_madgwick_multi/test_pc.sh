#!/bin/sh
# Prueba en el PC (sin Vitis HLS): 4 IMUs sinteticas -> r2f_mux -> madgwick_multi, comparado bit a bit con
# el mismo nucleo de una sola IMU. Usa cabeceras minimas de sustitucion (tb_stub/), no las de Xilinx.
set -e
cd "$(dirname "$0")"
F="-O2 -std=c++14 -Itb_stub -I."
g++ $F -c madgwick_multi.cpp -o /tmp/mm_multi.o
g++ $F -c madgwick_single.cpp -o /tmp/mm_single.o
g++ $F -c r2f_mux.cpp -o /tmp/mm_mux.o
g++ $F tb_multi.cpp /tmp/mm_multi.o /tmp/mm_single.o /tmp/mm_mux.o -o /tmp/mm_tb
/tmp/mm_tb
