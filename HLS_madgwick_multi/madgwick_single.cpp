// Solo para el banco de pruebas: el mismo nucleo compilado para UNA IMU (NIMU=1) con otro nombre,
// que sirve de referencia bit a bit de madgwick_multi_top.
#define NIMU 1
#define madgwick_multi_top madgwick_single
#include "madgwick_multi.cpp"
