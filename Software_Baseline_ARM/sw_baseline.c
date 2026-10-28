/* sw_baseline.c -- Madgwick MARG en el ARM Cortex-A9 con LA MISMA ENTRADA que el nucleo de la FPGA.
 *
 * Mide (1) tiempo de una actualizacion del filtro (min/media/p50/p99/max, con cache caliente y fria),
 *     (2) diferencia numerica del cuaternion frente al que dio el nucleo (SW_QCORE) con las mismas palabras,
 *     (3) carga de CPU si el filtro se ejecutase a la frecuencia de muestreo.
 * El algoritmo es el mismo que madgwick_hls.cpp / el modelo float32 de replay.py (mismo orden de operaciones),
 * salvo 1/sqrt, que aqui es 1.0f/sqrtf(x) en vez de la semilla magica + Newton-Raphson del nucleo.
 *
 * Vitis 2023.2 (flujo SDT, bare-metal, ps7_cortexa9_0): anade este fichero + sw_input.h al proyecto. No necesita xtime_l.h ni -lm
 *   (el BSP SDT no trae xtime_l.h y el enlazador no anade libm: el temporizador global se lee directamente y la raiz usa VSQRT).
 *   Compila dos veces (Debug -O0 y Release -O3) y apunta ambos resultados: el compilador cambia mucho el tiempo.
 * En PC (prueba de la logica, el tiempo no vale):  gcc -O2 -ffp-contract=off -DHOST sw_baseline.c -o sw -lm */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "sw_input.h"

#ifdef HOST
  #include <time.h>
  static uint64_t now_ticks(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (uint64_t)t.tv_sec * 1000000000ull + t.tv_nsec; }
  static double ticks_hz(void) { return 1.0e9; }
  static void cache_cold(void) {}
#else
  #include "xil_cache.h"
  #include "sleep.h"
  /* Temporizador global del Cortex-A9 leido directamente (como main.c): el BSP de Vitis 2023.2 (flujo SDT) no trae xtime_l.h */
  #define GT_BASE 0xF8F00200u
  static uint64_t now_ticks(void) {
      volatile uint32_t *g = (volatile uint32_t *)GT_BASE;
      uint32_t hi, lo, hi2;
      do { hi = g[1]; lo = g[0]; hi2 = g[1]; } while (hi != hi2);   /* lectura coherente de los 64 bits */
      return ((uint64_t)hi << 32) | lo;
  }
  /* Las macros COUNTS_PER_SECOND de algunos BSP no son fiables: se calibra contra usleep (0,5 s), como main.c */
  static double ticks_hz(void) {
      volatile uint32_t *ctl = (volatile uint32_t *)(GT_BASE + 8u); if (!(*ctl & 1u)) *ctl |= 1u;   /* arranca el contador si estaba parado */
      uint64_t a = now_ticks(); usleep(500000); uint64_t b = now_ticks(); return (double)(b - a) * 2.0; }
  static void cache_cold(void) { Xil_DCacheFlush(); }      /* L1+L2: el siguiente acceso va a DDR */
#endif

/* Sin libm: el enlazador de Vitis 2023.2 (flujo SDT) no anade -lm. En el ARM la raiz sale de la instruccion VSQRT (la FPU del A9). */
#ifdef HOST
  #define SQRTF(x) sqrtf(x)
  #define SQRTD(x) sqrt(x)
#else
  static inline float sqrtf_hw(float x) { float r; __asm__("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x)); return r; }
  static inline double sqrtd_hw(double x) { double r; __asm__("vsqrt.f64 %0, %1" : "=w"(r) : "w"(x)); return r; }
  #define SQRTF(x) sqrtf_hw(x)
  #define SQRTD(x) sqrtd_hw(x)
#endif
static inline float rs(float x) { return 1.0f / SQRTF(x); }

static void madgwick_imu(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float beta, float dt) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float qd1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qd2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qd3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qd4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        float r = rs(ax * ax + ay * ay + az * az); ax *= r; ay *= r; az *= r;
        float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2, _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        float n2 = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (n2 > 1e-20f) { r = rs(n2); s0 *= r; s1 *= r; s2 *= r; s3 *= r; }
        qd1 -= beta * s0; qd2 -= beta * s1; qd3 -= beta * s2; qd4 -= beta * s3;
    }
    q0 += qd1 * dt; q1 += qd2 * dt; q2 += qd3 * dt; q3 += qd4 * dt;
    float r = rs(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q[0] = q0 * r; q[1] = q1 * r; q[2] = q2 * r; q[3] = q3 * r;
}

static void madgwick_ahrs(float q[4], float gx, float gy, float gz, float ax, float ay, float az,
                          float mx, float my, float mz, float beta, float dt) {
    if (mx == 0.0f && my == 0.0f && mz == 0.0f) { madgwick_imu(q, gx, gy, gz, ax, ay, az, beta, dt); return; }
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float qd1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qd2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qd3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qd4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        float r = rs(ax * ax + ay * ay + az * az); ax *= r; ay *= r; az *= r;
        r = rs(mx * mx + my * my + mz * mz); mx *= r; my *= r; mz *= r;
        float _2q0mx = 2.0f * q0 * mx, _2q0my = 2.0f * q0 * my, _2q0mz = 2.0f * q0 * mz, _2q1mx = 2.0f * q1 * mx;
        float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3, _2q0q2 = 2.0f * q0 * q2, _2q2q3 = 2.0f * q2 * q3;
        float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3, q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
        float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;
        float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        float _2bx = SQRTF(hx * hx + hy * hy);
        float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        float _4bx = 2.0f * _2bx, _4bz = 2.0f * _2bz;
        float h = 0.5f;
        float e1 = 2.0f * q1q3 - _2q0q2 - ax, e2 = 2.0f * q0q1 + _2q2q3 - ay;
        float m1 = _2bx * (h - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx;
        float m2 = _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my;
        float m3 = _2bx * (q0q2 + q1q3) + _2bz * (h - q1q1 - q2q2) - mz;
        float s0 = -_2q2 * e1 + _2q1 * e2 - _2bz * q2 * m1 + (-_2bx * q3 + _2bz * q1) * m2 + _2bx * q2 * m3;
        float s1 = _2q3 * e1 + _2q0 * e2 - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * m1 + (_2bx * q2 + _2bz * q0) * m2 + (_2bx * q3 - _4bz * q1) * m3;
        float s2 = -_2q0 * e1 + _2q3 * e2 - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * m1 + (_2bx * q1 + _2bz * q3) * m2 + (_2bx * q0 - _4bz * q2) * m3;
        float s3 = _2q1 * e1 + _2q2 * e2 + (-_4bx * q3 + _2bz * q1) * m1 + (-_2bx * q0 + _2bz * q2) * m2 + _2bx * q1 * m3;
        float n2 = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (n2 > 1e-20f) { r = rs(n2); s0 *= r; s1 *= r; s2 *= r; s3 *= r; }
        qd1 -= beta * s0; qd2 -= beta * s1; qd3 -= beta * s2; qd4 -= beta * s3;
    }
    q0 += qd1 * dt; q1 += qd2 * dt; q2 += qd3 * dt; q3 += qd4 * dt;
    float r = rs(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q[0] = q0 * r; q[1] = q1 * r; q[2] = q2 * r; q[3] = q3 * r;
}

/* ---- utilidades de medida ---- */
static uint32_t T_WARM[SW_N], T_COLD[SW_N];
static float QOUT[SW_N][4];
static int cmp_u32(const void *a, const void *b) { uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b; return (x > y) - (x < y); }
static double ang_deg(const float *a, const float *b) {               /* angulo entre dos cuaterniones, en double */
    double a0 = a[0], a1 = -a[1], a2 = -a[2], a3 = -a[3], b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3];
    double w = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3, x = a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2, y = a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1, z = a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0;
    double sv = SQRTD(x * x + y * y + z * z), aw = w < 0 ? -w : w;
    double t = sv / aw;                                  /* tan(angulo/2): aqui los cuaterniones coinciden a ~1e-5, serie de arctan suficiente */
    double t2 = t * t, at = t * (1.0 - t2 / 3.0 + t2 * t2 / 5.0 - t2 * t2 * t2 / 7.0);
    return 2.0 * at * 57.29577951308232;
}
static void report(const char *name, uint32_t *t, int n, double hz, double ovh) {
    qsort(t, n, sizeof(uint32_t), cmp_u32);
    double sum = 0; for (int i = 0; i < n; i++) sum += t[i];
    double us = 1.0e6 / hz;
    printf("%-14s n=%d  min %.3f  media %.3f  p50 %.3f  p99 %.3f  p99.9 %.3f  max %.3f us  (resta %.3f us de timer)\r\n", name, n,
           t[0] * us - ovh, sum / n * us - ovh, t[n / 2] * us - ovh, t[(int)(n * 0.99)] * us - ovh, t[(int)(n * 0.999)] * us - ovh, t[n - 1] * us - ovh, ovh);
}

int main(void) {
    const double hz = ticks_hz();
    printf("\r\n=== Madgwick en ARM, misma entrada que el nucleo: %d tramas, dt %.3f us (%.2f Hz), reloj de medida %.3f MHz ===\r\n", SW_N, SW_DT * 1e6, 1.0 / SW_DT, hz / 1e6);
    /* sobrecoste del temporizador: dos lecturas seguidas */
    uint32_t ov = 0xFFFFFFFFu; for (int i = 0; i < 1000; i++) { uint64_t a = now_ticks(), b = now_ticks(); if ((uint32_t)(b - a) < ov) ov = (uint32_t)(b - a); }
    const double ovh = ov * 1.0e6 / hz;

    /* (1) cache caliente: se ejecuta toda la secuencia y se cronometra cada actualizacion */
    float q[4] = { SW_QCORE[0][0], SW_QCORE[0][1], SW_QCORE[0][2], SW_QCORE[0][3] };
    for (int c = 0; c < 4; c++) QOUT[0][c] = q[c];
    for (int k = 1; k < SW_N; k++) {
        const float *x = SW_IN[k]; const float beta = (k < SW_BETA_SWITCH_K) ? SW_BETA_START : SW_BETA_RUN;
        uint64_t a = now_ticks();
        madgwick_ahrs(q, x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], beta, SW_DT);
        uint64_t b = now_ticks();
        T_WARM[k - 1] = (uint32_t)(b - a); for (int c = 0; c < 4; c++) QOUT[k][c] = q[c];
    }
    /* carga de CPU: se calcula ANTES de ordenar T_WARM (report() lo ordena) */
    double sum = 0; uint32_t tmax = 0; for (int i = 0; i < SW_N - 1; i++) { sum += T_WARM[i]; if (T_WARM[i] > tmax) tmax = T_WARM[i]; }
    const double mean_us = sum / (SW_N - 1) * 1.0e6 / hz - ovh, max_us = tmax * 1.0e6 / hz - ovh;
    report("cache caliente", T_WARM, SW_N - 1, hz, ovh);

    /* (2) cache fria: se vacia L1+L2 antes de cada actualizacion (peor caso: otra tarea ha desalojado todo) */
    int nc = 0;
    {
        float qc[4] = { QOUT[SW_BETA_SWITCH_K][0], QOUT[SW_BETA_SWITCH_K][1], QOUT[SW_BETA_SWITCH_K][2], QOUT[SW_BETA_SWITCH_K][3] };
        for (int i = SW_BETA_SWITCH_K + 1; i < SW_N && nc < 2000; i++, nc++) {
            const float *x = SW_IN[i];
            cache_cold();
            uint64_t a = now_ticks();
            madgwick_ahrs(qc, x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], SW_BETA_RUN, SW_DT);
            uint64_t b = now_ticks();
            T_COLD[nc] = (uint32_t)(b - a);
        }
    }
    if (nc > 10) report("cache fria", T_COLD, nc, hz, ovh);

    /* (3) diferencia numerica frente al cuaternion que dio el nucleo (se omiten los primeros 2 s tras bajar beta) */
    double mx = 0, sm = 0; int n = 0; int k0 = SW_BETA_SWITCH_K + (int)(2.0 / SW_DT);
    for (int k = k0; k < SW_N; k++) { double e = ang_deg(QOUT[k], SW_QCORE[k]); if (e > mx) mx = e; sm += e; n++; }
    printf("ARM float vs nucleo FPGA (t > %.1f s): media %.3e  max %.3e grados\r\n", k0 * SW_DT, n ? sm / n : 0.0, mx);

    /* (4) carga de CPU a la frecuencia de muestreo, con el tiempo medio y el maximo con cache caliente */
    printf("Carga de CPU a %.1f Hz (1 IMU): media %.4f %%  peor caso %.4f %%  | 4 IMUs: media %.3f %%  peor caso %.3f %%\r\n",
           1.0 / SW_DT, mean_us * 1e-6 / SW_DT * 100.0, max_us * 1e-6 / SW_DT * 100.0, 4 * mean_us * 1e-6 / SW_DT * 100.0, 4 * max_us * 1e-6 / SW_DT * 100.0);
    printf("(el filtro NO incluye la adquisicion SPI ni la salida: en el sistema de software esa parte es la que consume el tiempo)\r\n");
#ifdef HOST
    FILE *f = fopen("sw_out.csv", "w"); for (int k = 0; k < SW_N; k++) fprintf(f, "%.9g,%.9g,%.9g,%.9g\n", QOUT[k][0], QOUT[k][1], QOUT[k][2], QOUT[k][3]); fclose(f);
#endif
    return 0;
}
