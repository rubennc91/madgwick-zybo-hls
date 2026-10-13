#ifndef R2F_TID_H
#define R2F_TID_H

#include "hls_stream.h"
#include "ap_int.h"
#include "ap_axi_sdata.h"

#ifndef NIMU_IDW
#define NIMU_IDW 2
#endif

typedef hls::axis<ap_int<32>, 0, NIMU_IDW, 0> ap_axis_rawid_t;   // entrada: entero crudo + TID (del arbitro)
typedef hls::axis<float,      0, NIMU_IDW, 0> ap_axis_fid_t;     // salida : float escalado + TID

// -----------------------------------------------------------------------------
// r2f_tid_top : raw2float con TID de paso. UN solo puerto de entrada y UNO de salida.
//
// Sustituye a r2f_mux_top. El reparto entre las 4 IMUs lo hace ahora un arbitro
// AXI-Stream en VHDL (axis_rr_mux4.vhd: round-robin, trama completa, TID = IMU),
// asi que aqui solo se convierte entero -> float con el mismo escalado que
// raw2float_top y se copia el TID de cada palabra.
//
// Protocolo de bloque ap_ctrl_hs con auto-restart (lo activa el PS): cada
// invocacion convierte UNA trama de 9 palabras.
// fs_g_reg (AXI-Lite, offset 0x10): rango del giroscopio, igual que antes.
// -----------------------------------------------------------------------------
void r2f_tid_top(hls::stream<ap_axis_rawid_t>& s_axis,
                 hls::stream<ap_axis_fid_t>& m_axis,
                 ap_uint<2> fs_g_reg);

#endif
