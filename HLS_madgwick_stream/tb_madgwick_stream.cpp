// Testbench: igual que tb_madgwick_1imu.cpp (misma referencia madgwick_f2.c,
// mismas entradas sinteticas y misma tolerancia -- ver esa cabecera para la
// justificacion de por que no se exige coincidencia bit-a-bit), pero
// empujando cada trama por el AXI4-Stream de entrada en vez de escribir
// directamente en un array imu_in[9].
#include <cstdio>
#include <cstdlib>
#include <cmath>

extern "C" {
#include "madgwick_f2.h"
extern volatile float beta;
}
#include "madgwick_stream.h"

static float rnd(float lo, float hi) {
    return lo + (hi - lo) * ((float)rand() / (float)RAND_MAX);
}

static void push_frame(hls::stream<ap_axis_float_t>& s, const float v[9]) {
    for (int k = 0; k < 9; k++) {
        ap_axis_float_t w;
        w.data = v[k];
        w.last = (k == 8) ? 1 : 0;
        w.keep = -1;
        s.write(w);
    }
}

int main() {
    const float BETA = 0.5f;
    const float DT = 0.1f;
    const int N = 500;
    const float TOL = 0.08f;

    beta = BETA;

    MadgwickFilt ref;
    ref.q0 = -1.0f; ref.q1 = 0.0f; ref.q2 = 0.0f; ref.q3 = 0.2f;

    hls::stream<ap_axis_float_t> s_axis_imu("s_axis_imu");
    float q[4];
    float zero[9] = {0,0,0,0,0,0,0,0,0};

    push_frame(s_axis_imu, zero);
    madgwick_stream_top(s_axis_imu, q, BETA, DT, 1);   // reset (igual consume la trama)

    float maxErr = 0.0f;
    float sumErr = 0.0f;
    int   nErr   = 0;
    srand(1234);

    for (int n = 0; n < N; n++) {
        float gx = rnd(-0.05f, 0.05f), gy = rnd(-0.05f, 0.05f), gz = rnd(-0.05f, 0.05f);
        float ax = rnd(-0.03f, 0.03f), ay = rnd(-0.03f, 0.03f), az = -1.0f + rnd(-0.01f, 0.01f);
        float mx = 0.20f + rnd(-0.005f, 0.005f);
        float my = 0.05f + rnd(-0.005f, 0.005f);
        float mz = -0.40f + rnd(-0.005f, 0.005f);
        if (n % 100 == 99) { mx = 0.0f; my = 0.0f; mz = 0.0f; }

        float v[9] = {gx, gy, gz, ax, ay, az, mx, my, mz};

        MadgwickAHRSupdate(&ref, gx, gy, gz, ax, ay, az, mx, my, mz);

        push_frame(s_axis_imu, v);
        madgwick_stream_top(s_axis_imu, q, BETA, DT, 0);

        float r[4] = {ref.q0, ref.q1, ref.q2, ref.q3};
        for (int j = 0; j < 4; j++) {
            float e = fabsf(q[j] - r[j]);
            if (e > maxErr) maxErr = e;
            sumErr += e;
            nErr++;
        }
    }

    printf("Error maximo  |q_hls - q_ref| = %g (tolerancia %g)\n", maxErr, TOL);
    printf("Error promedio|q_hls - q_ref| = %g\n", sumErr / nErr);
    if (maxErr < TOL) {
        printf("TEST OK\n");
        return 0;
    }
    printf("TEST FALLIDO\n");
    return 1;
}
