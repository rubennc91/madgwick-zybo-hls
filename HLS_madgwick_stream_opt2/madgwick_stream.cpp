#include "madgwick_stream.h"
#include "madgwick_prog.h"

// ---------------------------------------------------------------------------
// Madgwick MARG como INTERPRETE de un programa en ROM.
//
//  * Un solo sumador/restador, un solo multiplicador y un solo comparador en
//    coma flotante (5 DSP en total). No hay nucleos frsqrt/fsqrt: la inversa de
//    la raiz se calcula con Newton-Raphson sobre el mismo multiplicador.
//  * El control es un contador de programa + una ROM de ~550 palabras en lugar
//    de una maquina de estados de cientos de estados con multiplexores enormes.
//  * El algoritmo es el de HLS_madgwick_stream_opt (mismo orden de operaciones;
//    el programa lo genera gen_prog.py). La unica diferencia numerica es que
//    rsqrt/sqrt usan Newton-Raphson (error relativo ~1e-7, como float).
//
// A 119 Hz hay ~840.000 ciclos por muestra y el programa necesita unos 4.000.
// ---------------------------------------------------------------------------

typedef union { float f; unsigned int u; } fu_t;
static inline unsigned int f2u(float x) { fu_t t; t.f = x; return t.u; }
static inline float u2f(unsigned int x) { fu_t t; t.u = x; return t.f; }
static inline bool is_zero(float x) { return (f2u(x) & 0x7FFFFFFFu) == 0u; }

void madgwick_stream_top(hls::stream<ap_axis_float_t>& s_axis_imu,
                          float q_out[4],
                          float beta,
                          float dt,
                          int reset) {
#pragma HLS INTERFACE axis     port=s_axis_imu
#pragma HLS INTERFACE s_axilite port=q_out  bundle=ctrl
#pragma HLS INTERFACE s_axilite port=beta   bundle=ctrl
#pragma HLS INTERFACE s_axilite port=dt     bundle=ctrl
#pragma HLS INTERFACE s_axilite port=reset  bundle=ctrl
#pragma HLS INTERFACE s_axilite port=return bundle=ctrl
#pragma HLS ALLOCATION operation instances=fmul limit=1
#pragma HLS ALLOCATION operation instances=fadd limit=1
#pragma HLS ALLOCATION operation instances=fsub limit=1
#pragma HLS ALLOCATION operation instances=fcmp limit=1

    static float q_state[4];

    float R[MP_NREG];                       // banco de registros

    // Lee siempre la trama completa (9 floats) aunque vayamos a resetear: no
    // dejar palabras sin consumir en el stream (TLAST desincronizado).
    for (int k = 0; k < 9; k++) {
#pragma HLS PIPELINE II=1
        ap_axis_float_t w = s_axis_imu.read();
        R[k] = w.data;
    }

    if (reset != 0) {
        q_state[0] = -1.0f;
        q_state[1] = 0.0f;
        q_state[2] = 0.0f;
        q_state[3] = 0.2f;
    } else {
        for (int k = 0; k < 4; k++) R[9 + k] = q_state[k];
        R[13] = beta;
        R[14] = dt;
        for (int k = 0; k < MP_NCONST; k++) R[MP_CBASE + k] = u2f(MP_CONST_BITS[k]);

        // Rama: sin acelerometro -> solo giroscopio; sin magnetometro -> IMU.
        const bool acc0 = is_zero(R[3]) && is_zero(R[4]) && is_zero(R[5]);
        const bool mag0 = is_zero(R[6]) && is_zero(R[7]) && is_zero(R[8]);
        const int prog = acc0 ? MP_GYRO : (mag0 ? MP_IMU : MP_AHRS);

    RUN:
        for (int pc = MP_START[prog]; pc < MP_END[prog]; pc++) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT min=57 max=320
            const unsigned int ins = MP_ROM[pc];
            const unsigned int op = ins & 0xFu;
            const unsigned int d  = (ins >> 4)  & 0x7Fu;
            const unsigned int a  = (ins >> 11) & 0x7Fu;
            const unsigned int b  = (ins >> 18) & 0x7Fu;

            const float x = R[a];
            const float y = R[b];

            // Los tres operadores se calculan siempre y se elige el resultado:
            // cada nucleo recibe x e y directamente, sin multiplexores delante.
            // La resta es una suma con el bit de signo de y invertido.
            const unsigned int smask = (op == OP_SUB) ? 0x80000000u : 0u;
            const float sum  = x + u2f(f2u(y) ^ smask);
            const float prod = x * y;
            const float gt   = (x > y) ? 1.0f : 0.0f;
            const float rsq0 = u2f(0x5f3759dfu - (f2u(x) >> 1));

            float r;
            if (op == OP_MUL)          r = prod;
            else if (op == OP_GT)      r = gt;
            else if (op == OP_RSQINIT) r = rsq0;
#ifdef MP_HOST_EXACT
            else if (op == OP_RSQRT)   r = 1.0f / __builtin_sqrtf(x);
            else if (op == OP_SQRT)    r = __builtin_sqrtf(x);
#endif
            else                       r = sum;     // ADD / SUB
            R[d] = r;
        }

        for (int k = 0; k < 4; k++) q_state[k] = R[MP_OUT[prog][k]];
    }

    for (int j = 0; j < 4; j++) {
        q_out[j] = q_state[j];
    }
}
