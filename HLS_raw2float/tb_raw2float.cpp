#include <cstdio>
#include <cmath>
#include "raw2float.h"

// Testbench minimo: inyecta una trama de 9 muestras crudas con valores
// conocidos y comprueba que:
//  1) El orden/TLAST de la trama de salida es correcto (TLAST solo en i=8).
//  2) El giroscopio se escala con el factor correspondiente a FS_G.
//  3) Accel/mag usan la escala fija (solo importa que sea > 0 y estable,
//     ya que Madgwick los normaliza despues).
int main() {
    hls::stream<ap_axis_in_t>  s_axis("s_axis");
    hls::stream<ap_axis_out_t> m_axis("m_axis");

    // Trama de prueba: gx,gy,gz,ax,ay,az,mx,my,mz (cuentas crudas LSM9DS1)
    int32_t raw_frame[9] = { 1000, -2000, 500, 4000, -4000, 16000, 100, 200, -300 };

    for (int i = 0; i < 9; i++) {
        ap_axis_in_t w;
        w.data = raw_frame[i];
        w.last = (i == 8) ? 1 : 0;
        w.keep = -1;
        s_axis.write(w);
    }

    ap_uint<2> fs_g = 1; // 500 dps -> 17.50 mdps/LSB
    float expected_gyro_scale = 17.50e-3f * 0.017453293f;

    raw2float_top(s_axis, m_axis, fs_g); // una invocacion = una trama (ap_ctrl_hs)

    int errors = 0;
    for (int i = 0; i < 9; i++) {
        if (m_axis.empty()) {
            printf("ERROR: faltan palabras de salida en i=%d\n", i);
            errors++;
            break;
        }
        ap_axis_out_t w = m_axis.read();
        float got = w.data;
        float expected = (i < 3)
                            ? (float)raw_frame[i] * expected_gyro_scale
                            : (float)raw_frame[i] * (1.0f / 16384.0f);
        int expected_last = (i == 8) ? 1 : 0;

        if (fabsf(got - expected) > 1e-6f) {
            printf("ERROR muestra %d: got=%f expected=%f\n", i, got, expected);
            errors++;
        }
        if ((int)w.last != expected_last) {
            printf("ERROR TLAST muestra %d: got=%d expected=%d\n", i, (int)w.last, expected_last);
            errors++;
        }
    }

    if (errors == 0) {
        printf("TEST OK: raw2float_top escala y entrama correctamente.\n");
        return 0;
    } else {
        printf("TEST FALLIDO: %d error(es).\n", errors);
        return 1;
    }
}
