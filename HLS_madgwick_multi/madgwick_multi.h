#ifndef MADGWICK_MULTI_H
#define MADGWICK_MULTI_H

#include "hls_stream.h"
#include "ap_int.h"
#include "ap_axi_sdata.h"

// Numero de IMUs que atiende UN solo nucleo. El identificador viaja en TID.
#ifndef NIMU
#define NIMU 4
#endif
#define NIMU_IDW 2                       // ancho de TID (2 bits -> hasta 4 IMUs)

typedef hls::axis<float, 0, NIMU_IDW, 0> ap_axis_id_t;   // float + TLAST + TID

// -----------------------------------------------------------------------------
// madgwick_multi_top
//
// Es el nucleo opt2 (interprete de programa en ROM; mismo algoritmo, mismo
// programa) con UN ESTADO q_state[NIMU][4] POR IMU. Cada invocacion lee una
// trama de 9 floats (gx..mz) cuyo TID (en la primera palabra) dice a que IMU
// pertenece, actualiza el estado de ESA IMU y deja su cuaternion en q_out.
//
//  q_out[4*i .. 4*i+3]  cuaternion de la IMU i (AXI-Lite)
//  frame_cnt[i]         numero de tramas procesadas de la IMU i (AXI-Lite);
//                       permite leer q_out de forma coherente (contador antes
//                       y despues de leer, como se hace con SAMPLE_COUNT)
//  reset                MASCARA de bits: el bit i a 1 reinicia la IMU i en su
//                       siguiente trama (el valor se captura al arrancar cada
//                       invocacion, igual que en el nucleo de una IMU)
//  beta, dt             comunes a todas las IMUs
//
// Una trama con TID >= NIMU se consume y se descarta.
// -----------------------------------------------------------------------------
void madgwick_multi_top(hls::stream<ap_axis_id_t>& s_axis_imu,
                        float q_out[4 * NIMU],
                        unsigned int frame_cnt[NIMU],
                        float beta,
                        float dt,
                        int reset);

#endif
