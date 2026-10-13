#!/bin/sh
# Prueba en el PC (sin Vitis HLS): 4 IMUs sinteticas con cadencias distintas -> r2f_tid -> madgwick_multi
# (dt distinto por IMU), comparado bit a bit con el mismo nucleo de una sola IMU.
# Usa cabeceras minimas de sustitucion (tb_stub/), no las de Xilinx.
set -e
cd "$(dirname "$0")"
T="${TMPDIR:-/tmp}"
F="-O2 -std=c++14 -Itb_stub -I."
g++ $F -c madgwick_multi.cpp -o $T/mm_multi.o
g++ $F -c madgwick_single.cpp -o $T/mm_single.o
g++ $F -c r2f_tid.cpp -o $T/mm_r2f.o
g++ $F tb_multi.cpp $T/mm_multi.o $T/mm_single.o $T/mm_r2f.o -o $T/mm_tb
$T/mm_tb
