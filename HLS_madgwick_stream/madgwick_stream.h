#ifndef MADGWICK_STREAM_H
#define MADGWICK_STREAM_H

#include "hls_stream.h"
#include "ap_axi_sdata.h"

// Mismo tipo de palabra AXI4-Stream que produce raw2float_top (float + TLAST).
typedef hls::axis<float, 0, 0, 0> ap_axis_float_t;

// -----------------------------------------------------------------------------
// madgwick_stream_top
//
// Variante de streaming de madgwick_hls_1imu: en vez de recibir imu_in[9] por
// AXI-Lite (que exigiria que el PS copiara cada muestra), lee UNA trama de 9
// floats por AXI4-Stream (mismo orden que raw2float_top: gx,gy,gz,ax,ay,az,
// mx,my,mz) y hace EXACTAMENTE una actualizacion del filtro por invocacion.
//
// beta/dt/reset/q_out se mantienen por AXI-Lite, igual que en la version
// original: son parametros de control que el PS solo necesita tocar de vez
// en cuando (beta/dt casi nunca, q_out para leer la orientacion resultante
// cuando haga falta), no en cada muestra.
//
// Protocolo de bloque: ap_ctrl_hs (igual que raw2float_top). En el diseno de
// bloques, ap_start se amarra a '1' para que el nucleo encadene
// automaticamente una actualizacion tras otra en cuanto raw2float_top le
// entrega la siguiente trama.
//
// El algoritmo interno (madgwick_imu/madgwick_ahrs) es IDENTICO, caracter por
// caracter, al de madgwick_hls_1imu.cpp -- no se ha tocado para no introducir
// nuevas divergencias numericas respecto a lo ya validado con csim/hardware.
// -----------------------------------------------------------------------------
void madgwick_stream_top(hls::stream<ap_axis_float_t>& s_axis_imu,
                          float q_out[4],
                          float beta,
                          float dt,
                          int reset);

#endif
