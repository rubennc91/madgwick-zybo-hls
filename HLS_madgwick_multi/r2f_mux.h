#ifndef R2F_MUX_H
#define R2F_MUX_H

#include "hls_stream.h"
#include "ap_int.h"
#include "ap_axi_sdata.h"

#define NIMU_IDW 2

typedef hls::axis<ap_int<32>, 0, 0, 0>         ap_axis_raw_t;   // entrada: entero crudo (nav_spi_ctrl)
typedef hls::axis<float,      0, NIMU_IDW, 0>  ap_axis_fid_t;   // salida : float escalado + TID

// -----------------------------------------------------------------------------
// r2f_mux_top : raw2float + multiplexor de 4 IMUs
//
// Recibe tramas de 9 enteros crudos por 4 puertos AXI4-Stream (uno por
// nav_spi_ctrl), las convierte a float con el mismo escalado que raw2float_top
// y las saca por UN puerto AXI4-Stream con TID = indice de la IMU (0..3).
// Reparto por turnos (round-robin): en cada pasada mira los 4 puertos empezando
// por el siguiente al ultimo atendido, de modo que una IMU rapida no deja sin
// servicio a las demas. Una trama completa (9 palabras) se atiende de una vez.
//
// Protocolo de bloque ap_ctrl_hs con auto-restart (lo activa el PS, igual que
// con raw2float_top): cada invocacion atiende como mucho UNA trama.
// fs_g_reg (AXI-Lite): rango del giroscopio, igual que en raw2float_top.
// -----------------------------------------------------------------------------
void r2f_mux_top(hls::stream<ap_axis_raw_t>& s_axis0,
                 hls::stream<ap_axis_raw_t>& s_axis1,
                 hls::stream<ap_axis_raw_t>& s_axis2,
                 hls::stream<ap_axis_raw_t>& s_axis3,
                 hls::stream<ap_axis_fid_t>& m_axis,
                 ap_uint<2> fs_g_reg);

#endif
