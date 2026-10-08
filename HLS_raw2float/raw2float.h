#ifndef RAW2FLOAT_H
#define RAW2FLOAT_H

#include "hls_stream.h"
#include "ap_int.h"
#include "ap_axi_sdata.h"

// Tipos AXI4-Stream con canal TLAST (sin TID/TDEST/TUSER -> anchos a 0).
// hls::axis<T,WUser,WId,WDest> anade automaticamente .last/.keep/.strb
// y Vitis HLS los mapea a las senales TLAST/TKEEP/TSTRB del puerto axis.
typedef hls::axis<ap_int<32>, 0, 0, 0> ap_axis_in_t;   // entrada: entero crudo (32b, con signo)
typedef hls::axis<float,      0, 0, 0> ap_axis_out_t;  // salida : float ya escalado

// -----------------------------------------------------------------------------
// raw2float_top
//
// Convierte, en hardware, UNA trama de 9 muestras crudas (enteras, ya
// extendidas a 32 bits con signo) que saca nav_spi_ctrl por AXI4-Stream, en 9
// floats listos para entrar al nucleo Madgwick (madgwick_stream_top).
//
// Protocolo de bloque: ap_ctrl_hs (el que Vitis HLS anade por defecto junto
// con el bundle AXI-Lite "ctrl"), es decir, procesa EXACTAMENTE una trama por
// invocacion (ap_start -> ... -> ap_done) y luego se queda ap_idle. Para que
// funcione en modo "siempre corriendo" sin que el PS tenga que relanzarlo,
// en el diseno de bloques (block design) se amarra la senal ap_start a un
// IP de constante a '1': con ap_ctrl_hs, mantener ap_start en alto hace que
// el nucleo se reinicie solo cada vez que termina (ap_done), que es
// exactamente el comportamiento "auto-restart" documentado por Xilinx para
// este protocolo. Ver integration_bd.tcl.
//
// Orden de la trama de entrada y salida (igual que nav_spi_ctrl.vhd):
//   0: gx  1: gy  2: gz   (giroscopio, cuentas LSB)
//   3: ax  4: ay  5: az   (acelerometro, cuentas LSB)
//   6: mx  7: my  8: mz   (magnetometro, cuentas LSB)
//
// Escalado:
//  - Giroscopio: hace falta el factor LSB->rad/s correcto, porque Madgwick
//    INTEGRA la velocidad angular (qDot = 0.5*q*omega*dt); un factor erroneo
//    se traduce directamente en deriva o en una rotacion a velocidad
//    incorrecta. Sensibilidad LSM9DS1 (datasheet, "G_So"):
//      FS_G=00 (245 dps)  -> 8.75  mdps/LSB
//      FS_G=01 (500 dps)  -> 17.50 mdps/LSB
//      FS_G=10/11 (2000dps) -> 70.0 mdps/LSB
//    mdps/LSB * (pi/180)/1000 = rad/s por LSB.
//  - Acelerometro y magnetometro: Madgwick normaliza internamente estos dos
//    vectores (recipNorm = rsqrt(ax^2+ay^2+az^2), etc.), asi que su escala
//    FISICA absoluta no afecta al resultado -- solo tiene que ser una escala
//    FIJA y razonable para que los productos intermedios (ax*ax, etc.) no
//    desborden el rango de un float ni se queden en denormales. Por eso aqui
//    se usa un divisor fijo (no dependiente de FS_XL) para accel/mag.
//
// fs_g_reg: debe contener el MISMO valor que el campo FS_G del registro
//           RANGE_CFG de nav_spi_ctrl (ver nav_pkg.vhd, REGOFF_RANGE_CFG).
//           El software debe escribir el mismo valor aqui y alli para que la
//           escala del giroscopio sea coherente con la configuracion real
//           del sensor.
// -----------------------------------------------------------------------------
void raw2float_top(hls::stream<ap_axis_in_t>&  s_axis,
                    hls::stream<ap_axis_out_t>& m_axis,
                    ap_uint<2> fs_g_reg);

#endif
