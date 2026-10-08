#include "madgwick_stream.h"
#include "hls_math.h"

// ---------------------------------------------------------------------------
// madgwick_imu / madgwick_ahrs: IDENTICAS a madgwick_hls_1imu.cpp (no se han
// modificado), para no introducir ninguna divergencia numerica nueva
// respecto a lo ya validado en csim/hardware con esa version.
// ---------------------------------------------------------------------------
static void madgwick_imu(float q[4],
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float beta, float dt) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float recipNorm;
    float s0, s1, s2, s3;

    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = hls::rsqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        float sNorm2 = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (sNorm2 > 1e-20f) {
            recipNorm = hls::rsqrt(sNorm2);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;
        }

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = hls::rsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q[0] = q0 * recipNorm;
    q[1] = q1 * recipNorm;
    q[2] = q2 * recipNorm;
    q[3] = q3 * recipNorm;
}

static void madgwick_ahrs(float q[4],
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float beta, float dt) {
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        madgwick_imu(q, gx, gy, gz, ax, ay, az, beta, dt);
        return;
    }

    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float recipNorm;
    float s0, s1, s2, s3;

    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = hls::rsqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        recipNorm = hls::rsqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        float _2q0mx = 2.0f * q0 * mx;
        float _2q0my = 2.0f * q0 * my;
        float _2q0mz = 2.0f * q0 * mz;
        float _2q1mx = 2.0f * q1 * mx;
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _2q0q2 = 2.0f * q0 * q2;
        float _2q2q3 = 2.0f * q2 * q3;
        float q0q0 = q0 * q0;
        float q0q1 = q0 * q1;
        float q0q2 = q0 * q2;
        float q0q3 = q0 * q3;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q3q3 = q3 * q3;

        float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        float _2bx = hls::sqrt(hx * hx + hy * hy);
        float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        float _4bx = 2.0f * _2bx;
        float _4bz = 2.0f * _2bz;

        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        float sNorm2 = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (sNorm2 > 1e-20f) {
            recipNorm = hls::rsqrt(sNorm2);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;
        }

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = hls::rsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q[0] = q0 * recipNorm;
    q[1] = q1 * recipNorm;
    q[2] = q2 * recipNorm;
    q[3] = q3 * recipNorm;
}

// ---------------------------------------------------------------------------
// Top: 1 IMU, estado interno, entrada por AXI4-Stream, control por AXI-Lite.
// ---------------------------------------------------------------------------
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
#pragma HLS ALLOCATION operation instances=fmul limit=4
#pragma HLS ALLOCATION operation instances=fadd limit=4

    static float q_state[4];

    // Lee siempre la trama completa (9 floats), aunque vayamos a resetear,
    // para no dejar palabras a medio consumir en el stream de entrada: eso
    // desincronizaria el protocolo TLAST con raw2float_top en la siguiente
    // invocacion.
    float in[9];
    for (int k = 0; k < 9; k++) {
#pragma HLS PIPELINE II=1
        ap_axis_float_t w = s_axis_imu.read();
        in[k] = w.data;
    }

    if (reset != 0) {
        q_state[0] = -1.0f;
        q_state[1] = 0.0f;
        q_state[2] = 0.0f;
        q_state[3] = 0.2f;
    } else {
        madgwick_ahrs(q_state,
                      in[0], in[1], in[2],
                      in[3], in[4], in[5],
                      in[6], in[7], in[8],
                      beta, dt);
    }

    for (int j = 0; j < 4; j++) {
        q_out[j] = q_state[j];
    }
}
