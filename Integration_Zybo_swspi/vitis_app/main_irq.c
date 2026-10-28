/* main_irq.c -- LINEA BASE EN SOFTWARE CON INTERRUPCIONES (diseno zybo_swspi, igual que main.c pero sin sondeo continuo)
 *
 * Misma adquisicion que main.c (AXI Quad SPI por registros, SCK 3,125 MHz, mismo orden de lectura, mismo filtro) pero el ARM
 * NO sondea todo el tiempo: un temporizador privado del Cortex-A9 genera una interrupcion cada (periodo nominal / POLL_DIV);
 * la rutina de interrupcion lee el estado de cada IMU y, si hay dato nuevo (GDA), lee, calibra y filtra. El resto del tiempo
 * el bucle principal ejecuta una tarea de fondo (el MISMO bucle que main_pl.c) y, segun la fase, imprime por la UART.
 *
 * Por que un temporizador y no la interrupcion de dato listo (INT del Pmod NAV): ese pin no esta en el diseno ni en el XDC
 * (hay que comprobar el puente/jumper de la placa) y exigiria cambiar el hardware. Con el temporizador la unica parte que no
 * se puede evitar es la que se quiere medir: el procesador sigue haciendo TODAS las transferencias SPI (por registros, esperando),
 * ahora dentro de la rutina de interrupcion, y el dato puede esperar hasta un periodo del temporizador antes de leerse.
 *
 * Mide (por IMU, con el temporizador global): lectura SPI, filtro, deteccion->cuaternion (sin contar la espera al temporizador),
 * intervalo entre tramas y tramas perdidas (entre interrupciones consecutivas con dato, > periodo + tick + 100 us); y (global): numero de interrupciones, duracion media y maxima
 * de la rutina, porcentaje de CPU en la rutina y CPU LIBRE = iteraciones del bucle de fondo / iteraciones de referencia sin interrupciones
 * (misma definicion que main_pl.c: 100 % = el procesador no pierde nada).
 * Fases: A sin salida, B una linea cada ~100 ms, C una linea por trama nueva de la IMU0 (la rutina sigue activa mientras se imprime).
 *
 * Uso: bitstream zybo_swspi, placas quietas, UART 115200 8N1. ODR_HZ = 119 o 952, N_IMU = 2 o 4, POLL_DIV = 2 por defecto. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#ifdef HOST
  #include <math.h>
  #include "host_sim.h"
#else
  #include "xil_io.h"
  #include "xil_exception.h"
  #include "sleep.h"
  #define GT_BASE 0xF8F00200u
  static uint64_t gt_now(void) {
      volatile uint32_t *g = (volatile uint32_t *)GT_BASE; uint32_t hi, lo, hi2;
      do { hi = g[1]; lo = g[0]; hi2 = g[1]; } while (hi != hi2);
      return ((uint64_t)hi << 32) | lo;
  }
#endif

#ifndef N_IMU
#define N_IMU          2            /* IMUs conectadas (1..4) = NIMU del create_project.tcl */
#endif
#define QSPI_BASE(i)   (0x40000000u + 0x10000u * (uint32_t)(i))
#ifndef POLL_DIV
#define POLL_DIV       2            /* interrupciones por periodo de muestreo nominal (2 = una cada medio periodo) */
#endif
#ifndef ODR_HZ
#define ODR_HZ         119
#endif
#if ODR_HZ == 952
  #define ODR_CODE 6u
  #define DT_DIV 8.0f
#elif ODR_HZ == 119
  #define ODR_CODE 3u
  #define DT_DIV 1.0f
#else
  #error "ODR_HZ debe ser 119 o 952"
#endif
#define PRINT_10HZ     ((ODR_HZ + 5) / 10)
#ifndef PHASE_SECONDS
#define PHASE_SECONDS  30
#endif
#ifndef PHASE_C_SECONDS
#define PHASE_C_SECONDS 5
#endif
#define BETA_START     1.0f
#define BETA_RUN       0.1f
#define BETA_START_MS  3000
#define FS_G_DPS_LSB   17.5e-3f     /* 500 dps */
#define GYRO_BIAS_FRAMES (2 * ODR_HZ)
static const float DT_IMU[4] = { 8.280e-3f / DT_DIV, 8.307e-3f / DT_DIV, 8.480e-3f / DT_DIV, 8.057e-3f / DT_DIV };
static const int32_t MAG_PRESET[4][3] = {{1537, 1372, -2025}, {-1952, 2124, -1233}, {-2188, 2152, -1273}, {1508, 1242, -1704}};

#ifdef HOST
  #define G_HZ host_gt_hz()
#else
  #define G_HZ 333.333333e6         /* temporizador global y privado = CPU/2; fijo (la calibracion con usleep varia ~1 % entre arranques) */
#endif
static double g_hz;
static double us(uint64_t ticks) { return (double)ticks * 1e6 / g_hz; }

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



/* ---- AXI Quad SPI, modo estandar ---- */
enum { SRR = 0x40, SPICR = 0x60, SPISR = 0x64, SPIDTR = 0x68, SPIDRR = 0x6C, SPISSR = 0x70, RXOCY = 0x78 };
#define CR_SPE (1u << 1)
#define CR_MASTER (1u << 2)
#define CR_CPOL (1u << 3)
#define CR_CPHA (1u << 4)
#define CR_TXRST (1u << 5)
#define CR_RXRST (1u << 6)
#define CR_MANSS (1u << 7)
#define CR_INHIBIT (1u << 8)
#define CR_RUN (CR_SPE | CR_MASTER | CR_CPOL | CR_CPHA | CR_MANSS)   /* modo SPI 3, chip select manual */
#define SR_RX_EMPTY (1u << 0)
#define SR_TX_EMPTY (1u << 2)

static void qspi_init(int i) {
    const uint32_t b = QSPI_BASE(i);
    Xil_Out32(b + SRR, 0x0000000Au);                                   /* reset del nucleo */
    Xil_Out32(b + SPICR, CR_RUN | CR_INHIBIT | CR_TXRST | CR_RXRST);
    Xil_Out32(b + SPISSR, 0x3u);                                       /* ningun chip select activo (activos a nivel bajo) */
}

/* Una transaccion: tx[0..n-1] -> rx[0..n-1] bajo un mismo CS (ss = 0: A/G, 1: mag). Bloqueante. Devuelve 0 si bien. */
static int qspi_xfer(int imu, int ss, const uint8_t *tx, uint8_t *rx, int n) {
    const uint32_t b = QSPI_BASE(imu);
    Xil_Out32(b + SPISSR, (~(1u << ss)) & 0x3u);
    for (int k = 0; k < n; k++) Xil_Out32(b + SPIDTR, tx[k]);
    Xil_Out32(b + SPICR, CR_RUN);                                      /* quita el inhibit: empieza a desplazar */
    uint32_t guard = 0;
    while (!(Xil_In32(b + SPISR) & SR_TX_EMPTY)) { if (++guard > 2000000u) return -1; }
    guard = 0;
    for (;;) {                                                         /* espera a tener los n bytes recibidos */
        if (!(Xil_In32(b + SPISR) & SR_RX_EMPTY) && (int)(Xil_In32(b + RXOCY) & 0xFFu) + 1 >= n) break;
        if (++guard > 2000000u) return -2;
    }
    for (int k = 0; k < n; k++) rx[k] = (uint8_t)Xil_In32(b + SPIDRR);
    Xil_Out32(b + SPICR, CR_RUN | CR_INHIBIT);
    Xil_Out32(b + SPISSR, 0x3u);
    return 0;
}

enum { CS_AG = 0, CS_MAG = 1 };
static int reg_write(int imu, int cs, uint8_t addr, uint8_t val) { uint8_t t[2] = { addr, val }, r[2]; return qspi_xfer(imu, cs, t, r, 2); }
static int reg_read1(int imu, int cs, uint8_t addr, uint8_t *val) { uint8_t t[2] = { (uint8_t)(0x80u | addr), 0 }, r[2]; int e = qspi_xfer(imu, cs, t, r, 2); *val = r[1]; return e; }
static int reg_burst6(int imu, int cs, uint8_t addr, int16_t v[3]) {   /* 6 bytes de datos; el mag necesita el bit MS (0x40) */
    uint8_t t[7] = { (uint8_t)(0x80u | (cs == CS_MAG ? 0x40u : 0u) | addr), 0, 0, 0, 0, 0, 0 }, r[7];
    int e = qspi_xfer(imu, cs, t, r, 7);
    for (int k = 0; k < 3; k++) v[k] = (int16_t)(((uint16_t)r[2 + 2 * k] << 8) | r[1 + 2 * k]);
    return e;
}

/* Configuracion identica a CFG_PROGRAM de nav_pkg (ODR g/xl segun ODR_HZ, 80 Hz mag, 500 dps, +-2 g, bajo consumo del mag) */
static int imu_config(int i) {
    uint8_t w;
    if (reg_read1(i, CS_AG, 0x0F, &w) || w != 0x68) { printf("IMU%d: WHO_AM_I A/G = 0x%02X (esperado 0x68)\r\n", i, w); return -1; }
    if (reg_read1(i, CS_MAG, 0x0F, &w) || w != 0x3D) { printf("IMU%d: WHO_AM_I mag = 0x%02X (esperado 0x3D)\r\n", i, w); return -1; }
    reg_write(i, CS_AG, 0x10, (ODR_CODE << 5) | (1u << 3));   /* CTRL_REG1_G */
    reg_write(i, CS_AG, 0x20, (ODR_CODE << 5));               /* CTRL_REG6_XL */
    reg_write(i, CS_AG, 0x22, 0x44);                          /* BDU, IF_ADD_INC */
    reg_write(i, CS_MAG, 0x20, (7u << 2));                    /* CTRL_REG1_M: ODR 80 Hz */
    reg_write(i, CS_MAG, 0x22, 0x00);                         /* modo continuo */
    reg_write(i, CS_MAG, 0x24, 0x40);                         /* BDU_M */
    reg_write(i, CS_MAG, 0x23, 0x00);                         /* CTRL_REG4_M: OMZ bajo consumo */
    return 0;
}

/* ---- estado y estadisticas por IMU ---- */
typedef struct { uint64_t n; double sum; double mx; double mn; } stat_t;
static void st_clear(stat_t *s) { s->n = 0; s->sum = 0; s->mx = 0; s->mn = 1e30; }
static void st_add(stat_t *s, double v) { s->n++; s->sum += v; if (v > s->mx) s->mx = v; if (v < s->mn) s->mn = v; }

typedef struct {
    float q[4]; volatile uint32_t seq;            /* seq impar = la rutina esta escribiendo q */
    int16_t m[3]; int16_t goff[3];
    uint64_t t_start, t_prev, tk_prev; int have_prev;
    stat_t spi, flt, lat, itv; uint64_t lost, frames, polls, sat;
    uint64_t busy_ticks;
} imu_t;
static imu_t G[4];
static uint64_t tick_period;           /* periodo del temporizador en ciclos */
static uint64_t tick_t0;               /* instante de entrada de la interrupcion en curso (criterio de tramas perdidas) */

/* Procesa una IMU: devuelve 1 si habia dato nuevo. Se llama desde la rutina de interrupcion. */
static int imu_step(int i) {
    imu_t *s = &G[i];
    uint8_t st; s->polls++;
    if (reg_read1(i, CS_AG, 0x17, &st)) { s->sat++; return 0; }
    if (!(st & 0x02u)) return 0;                                       /* GDA: no hay dato nuevo */
    const uint64_t t0 = gt_now();
    int16_t g[3], a[3], mg[3];
    reg_burst6(i, CS_AG, 0x18, g);
    reg_burst6(i, CS_AG, 0x28, a);
    uint8_t sm; reg_read1(i, CS_MAG, 0x27, &sm);
    if (sm & 0x08u) { reg_burst6(i, CS_MAG, 0x28, mg); for (int k = 0; k < 3; k++) s->m[k] = mg[k]; }
    const uint64_t t1 = gt_now();
    const float gx = -(float)(g[0] - s->goff[0]) * FS_G_DPS_LSB * 0.0174532925f;
    const float gy = -(float)(g[1] - s->goff[1]) * FS_G_DPS_LSB * 0.0174532925f;
    const float gz = -(float)(g[2] - s->goff[2]) * FS_G_DPS_LSB * 0.0174532925f;
    const float mx = -(float)(s->m[0] - MAG_PRESET[i][0]), my = (float)(s->m[1] - MAG_PRESET[i][1]), mz = (float)(s->m[2] - MAG_PRESET[i][2]);
    const float beta = (us(t0 - s->t_start) < BETA_START_MS * 1000.0) ? BETA_START : BETA_RUN;
    s->seq++;
    madgwick_ahrs(s->q, gx, gy, gz, (float)a[0], (float)a[1], (float)a[2], mx, my, mz, beta, DT_IMU[i]);
    s->seq++;
    const uint64_t t2 = gt_now();
    st_add(&s->spi, us(t1 - t0)); st_add(&s->flt, us(t2 - t1)); st_add(&s->lat, us(t2 - t0));
    s->busy_ticks += t2 - t0;
    if (s->have_prev && s->frames > 2) {
        const double itv = us(t0 - s->t_prev); st_add(&s->itv, itv);
        const double per = DT_IMU[i] * 1e6;
        /* con un temporizador el dato se lee entre 0 y un periodo del temporizador despues de estar listo: el intervalo normal es < periodo + tick;
         * se cuenta perdida una trama si el intervalo supera periodo + tick + 100 us (una perdida real da ~2 periodos +- tick) */
        /* el criterio usa el instante de ENTRADA de la interrupcion (multiplo exacto del tick), no el de lectura, para no contar el retardo
         * de la rutina hasta llegar a cada sensor (hasta ~350 us con 4 IMU): normal <= periodo + tick; una perdida real da >= 2 periodos - tick */
        const double itk = us(tick_t0 - s->tk_prev);
        if (itk > per + us(tick_period) + 100.0) s->lost += (uint64_t)(itk / per + 0.5) - 1;
    }
    s->t_prev = t0; s->tk_prev = tick_t0; s->have_prev = 1; s->frames++;
    return 1;
}

/* ---- temporizador privado + GIC (registros directos del Cortex-A9; el temporizador privado es la interrupcion 29) ---- */
#define PT_BASE   0xF8F00600u
#define PT_LOAD   0x00u
#define PT_CTRL   0x08u
#define PT_ISR    0x0Cu
#define GICC_BASE 0xF8F00100u
#define GICD_BASE 0xF8F01000u
#define PT_IRQ_ID 29u

static volatile uint64_t isr_ticks, isr_busy, isr_max, isr_over;
static volatile int irq_enabled;

static void tick_work(void) {
    const uint64_t a = gt_now();
    tick_t0 = a;
    for (int i = 0; i < N_IMU; i++) imu_step(i);
    const uint64_t d = gt_now() - a;
    isr_ticks++; isr_busy += d; if (d > isr_max) isr_max = d; if (d > tick_period) isr_over++;
}

#ifdef HOST
static uint64_t host_next_tick; static int host_in_isr;
static void host_irq_pump(void) {      /* en el PC no hay interrupciones: se emulan al entrar en el bucle de fondo */
    if (!irq_enabled || host_in_isr) return;
    while (gt_now() >= host_next_tick) { host_in_isr = 1; tick_work(); host_in_isr = 0; host_next_tick += tick_period; }
}
static void irq_global_off(void) { irq_enabled = 0; }
static void irq_global_on(void)  { irq_enabled = 1; host_next_tick = gt_now() + tick_period; }
static void irq_setup(void) {}
#else
static void host_irq_pump(void) {}
static void tick_isr(void *arg) {
    (void)arg;
    const uint32_t iar = Xil_In32(GICC_BASE + 0x0Cu);
    if ((iar & 0x3FFu) == PT_IRQ_ID) { Xil_Out32(PT_BASE + PT_ISR, 1u); tick_work(); }
    Xil_Out32(GICC_BASE + 0x10u, iar);                                 /* EOI */
}
static void irq_setup(void) {
    Xil_Out32(PT_BASE + PT_CTRL, 0u);                                  /* temporizador parado */
    Xil_Out32(PT_BASE + PT_LOAD, (uint32_t)tick_period - 1u);
    Xil_Out32(PT_BASE + PT_ISR, 1u);
    Xil_Out32(GICD_BASE + 0x000u, 0u);                                 /* distribuidor: configurar con el deshabilitado */
    { const uint32_t pa = GICD_BASE + 0x400u + (PT_IRQ_ID & ~3u), sh = (PT_IRQ_ID & 3u) * 8u;           /* prioridad: un byte de ICDIPR */
      Xil_Out32(pa, (Xil_In32(pa) & ~(0xFFu << sh)) | (0xA0u << sh)); }
    Xil_Out32(GICD_BASE + 0x100u, Xil_In32(GICD_BASE + 0x100u) | (1u << PT_IRQ_ID));   /* ICDISER0: habilita la 29 */
    Xil_Out32(GICD_BASE + 0x000u, 1u);
    Xil_Out32(GICC_BASE + 0x04u, 0xF0u);                               /* mascara de prioridad */
    Xil_Out32(GICC_BASE + 0x00u, 0x07u);                               /* interfaz de CPU habilitada */
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT, (Xil_ExceptionHandler)tick_isr, NULL);
}
static void irq_global_on(void)  { irq_enabled = 1; Xil_Out32(PT_BASE + PT_ISR, 1u); Xil_Out32(PT_BASE + PT_CTRL, (1u << 0) | (1u << 1) | (1u << 2)); Xil_ExceptionEnable(); }  /* enable | auto-reload | IRQ */
static void irq_global_off(void) { Xil_ExceptionDisable(); Xil_Out32(PT_BASE + PT_CTRL, 0u); irq_enabled = 0; }
#endif

/* ---- tarea de fondo: la misma que main_pl.c (cuenta iteraciones; representa "otro trabajo" del procesador) ---- */
static volatile uint32_t g_sink;
__attribute__((noinline)) static uint64_t bg_chunk(void) { uint32_t x = g_sink; for (int k = 0; k < 4096; k++) x = x * 1664525u + 1013904223u; g_sink = x; host_irq_pump(); return 4096u; }
__attribute__((noinline)) static uint64_t bg_run_until(uint64_t t_end) {
    uint64_t n = 0;
    while (gt_now() < t_end) n += bg_chunk();
    return n;
}
static uint64_t bg_run_seconds(double secs) { return bg_run_until(gt_now() + (uint64_t)(secs * g_hz)); }
static double g_bg_ref;

static void clear_stats(void) {
    for (int i = 0; i < N_IMU; i++) {
        st_clear(&G[i].spi); st_clear(&G[i].flt); st_clear(&G[i].lat); st_clear(&G[i].itv);
        G[i].lost = G[i].frames = G[i].polls = G[i].sat = 0; G[i].busy_ticks = 0; G[i].have_prev = 0;
    }
    isr_ticks = isr_busy = isr_max = isr_over = 0;
}

static void read_q0(float q[4], uint32_t *frames) {   /* instantanea coherente del cuaternion de la IMU0 */
    uint32_t a;
    do { a = G[0].seq; for (int k = 0; k < 4; k++) q[k] = G[0].q[k]; *frames = (uint32_t)G[0].frames; } while ((a & 1u) || a != G[0].seq);
}
static void print_line(void) {
    float q[4]; uint32_t f; read_q0(q, &f);
    printf("%lu,%d,%d,%d,%d\r\n", (unsigned long)f, (int)(q[0] * 10000.0f), (int)(q[1] * 10000.0f), (int)(q[2] * 10000.0f), (int)(q[3] * 10000.0f));
}

/* mode: 0 = A (nada), 1 = B (una linea cada ~100 ms), 2 = C (una linea por trama nueva de la IMU0) */
static void run_phase(const char *name, int mode, double secs) {
    irq_global_off(); clear_stats();
    uint32_t printed = 0; uint64_t bg_iters = 0, print_ticks = 0;
    const uint64_t t0 = gt_now(), t_end = t0 + (uint64_t)(secs * g_hz);
    uint64_t next_b = t0 + (uint64_t)(0.1 * g_hz); uint32_t last = 0;
    irq_global_on();
    while (gt_now() < t_end) {
        if (mode == 0) { bg_iters = bg_run_until(t_end); break; }
        if (mode == 1) {
            bg_iters += bg_run_until(next_b < t_end ? next_b : t_end);
            if (gt_now() >= t_end) break;
            next_b += (uint64_t)(0.1 * g_hz);
            const uint64_t h = gt_now(); print_line(); printed++; print_ticks += gt_now() - h;
            continue;
        }
        host_irq_pump();
        { float q[4]; uint32_t f; read_q0(q, &f); if (f == last) continue; last = f; }
        { const uint64_t h = gt_now(); print_line(); printed++; print_ticks += gt_now() - h; }
    }
    irq_global_off();
    const double s = (double)(gt_now() - t0) / g_hz;
    printf("\r\n--- %s: %.1f s, %lu lineas impresas ---\r\n", name, s, (unsigned long)printed);
    for (int i = 0; i < N_IMU; i++) {
        imu_t *g = &G[i];
        printf("IMU%d: %lu tramas (%.2f Hz), perdidas %lu, lecturas de estado %lu (errores SPI %lu)\r\n", i, (unsigned long)g->frames, g->frames / s, (unsigned long)g->lost, (unsigned long)g->polls, (unsigned long)g->sat);
        printf("   lectura SPI  media %.1f  max %.1f us | filtro  media %.2f  max %.2f us | deteccion->cuaternion  media %.1f  max %.1f us (sin la espera al temporizador)\r\n",
               g->spi.n ? g->spi.sum / g->spi.n : 0, g->spi.mx, g->flt.n ? g->flt.sum / g->flt.n : 0, g->flt.mx, g->lat.n ? g->lat.sum / g->lat.n : 0, g->lat.mx);
        printf("   intervalo entre tramas  media %.1f  min %.1f  max %.1f us (nominal %.1f)\r\n", g->itv.n ? g->itv.sum / g->itv.n : 0, g->itv.n ? g->itv.mn : 0, g->itv.mx, DT_IMU[i] * 1e6);
    }
    printf("Temporizador: una interrupcion cada %.1f us (%lu en la fase); rutina media %.1f us, max %.1f us, desbordes (rutina > periodo) %lu\r\n",
           us(tick_period), (unsigned long)isr_ticks, isr_ticks ? us(isr_busy) / (double)isr_ticks : 0.0, us(isr_max), (unsigned long)isr_over);
    if (isr_ticks == 0) printf("ERROR: no ha llegado ninguna interrupcion del temporizador (revisa GIC / controlador de excepciones); las cifras de esta fase no valen\r\n");
    printf("CPU en la rutina de interrupcion (estado + SPI + filtro): %.2f %%\r\n", us(isr_busy) / (s * 1e6) * 100.0);
    if (mode == 2) printf("CPU libre: sin bucle de fondo (la CPU imprime sin parar); tiempo imprimiendo %.1f %%\r\n", us(print_ticks) / (s * 1e6) * 100.0);
    else printf("CPU libre para otra tarea (bucle de fondo / referencia sin interrupciones): %.1f %%\r\n", (double)bg_iters / s / g_bg_ref * 100.0);
}

int main(void) {
    g_hz = G_HZ;
#ifndef HOST
    if (!(Xil_In32(GT_BASE + 8u) & 1u)) Xil_Out32(GT_BASE + 8u, Xil_In32(GT_BASE + 8u) | 1u);   /* arranca el temporizador global (antes lo hacia la calibracion) */
    { const uint64_t a = gt_now(); for (volatile int k = 0; k < 1000; k++) { } if (gt_now() == a) { printf("ERROR: el temporizador global no cuenta\r\n"); return -1; } }
#endif
    {   /* periodo del temporizador: el menor periodo nominal entre las IMU en uso, dividido por POLL_DIV */
        float pmin = DT_IMU[0]; for (int i = 1; i < N_IMU; i++) if (DT_IMU[i] < pmin) pmin = DT_IMU[i];
        tick_period = (uint64_t)((double)pmin / POLL_DIV * g_hz);
    }
    printf("\r\n=== Linea base en software con interrupciones: %d IMU, ODR %d Hz, SPI %.3f MHz, temporizador %.3f MHz, POLL_DIV %d ===\r\n", N_IMU, ODR_HZ, 100.0 / 32.0, g_hz / 1e6, POLL_DIV);
    for (int i = 0; i < N_IMU; i++) {
        qspi_init(i);
        if (imu_config(i)) { printf("fallo al configurar la IMU %d: se detiene\r\n", i); return -1; }
        G[i].q[0] = -1.0f; G[i].q[1] = 0; G[i].q[2] = 0; G[i].q[3] = 0.2f;
        G[i].m[0] = G[i].m[1] = G[i].m[2] = 0;
    }
    printf("Midiendo el sesgo del giroscopio (~2 s, placas quietas)...\r\n");
    for (int i = 0; i < N_IMU; i++) {
        int32_t acc[3] = {0, 0, 0}; int n = 0;
        while (n < GYRO_BIAS_FRAMES) {
            uint8_t st; int16_t g[3];
            if (reg_read1(i, CS_AG, 0x17, &st) || !(st & 0x02u)) continue;
            reg_burst6(i, CS_AG, 0x18, g); for (int k = 0; k < 3; k++) acc[k] += g[k]; n++;
            int16_t a[3]; reg_burst6(i, CS_AG, 0x28, a);
        }
        for (int k = 0; k < 3; k++) G[i].goff[k] = (int16_t)(acc[k] / n);
        printf("IMU%d: sesgo giro = (%d, %d, %d) cuentas\r\n", i, G[i].goff[0], G[i].goff[1], G[i].goff[2]);
    }
    irq_setup();
    g_bg_ref = (double)bg_run_seconds(2.0) / 2.0;       /* referencia sin interrupciones */
    printf("Referencia de CPU (sin interrupciones): %.3e iteraciones/s\r\n", g_bg_ref);
    const uint64_t t0 = gt_now(); for (int i = 0; i < N_IMU; i++) G[i].t_start = t0;
    run_phase("FASE A: sin salida por la UART (la CPU solo ejecuta el bucle de fondo)", 0, PHASE_SECONDS);
    run_phase("FASE B: una linea cada ~100 ms (10 Hz) por la UART", 1, PHASE_SECONDS);
    run_phase("FASE C: una linea por trama por la UART", 2, PHASE_C_SECONDS);
    irq_global_off();
    { const double r2 = (double)bg_run_seconds(2.0) / 2.0; printf("\r\nReferencia de CPU final (sin interrupciones): %.3e iteraciones/s (inicial %.3e, cociente %.3f)\r\n", r2, g_bg_ref, r2 / g_bg_ref); }
    printf("\r\nFin. Copia las tres tablas.\r\n");
    return 0;
}
