/* main.c -- LINEA BASE EN SOFTWARE DE EXTREMO A EXTREMO (Zybo Z7-10, Vitis 2023.2, bare-metal)
 *
 * El ARM hace TODO: lee por SPI los LSM9DS1 (un AXI Quad SPI por Pmod NAV, modo SPI 3, SCK 3,125 MHz), calibra, convierte,
 * ejecuta el filtro Madgwick (el MISMO codigo que Software_Baseline_ARM/sw_baseline.c) y, opcionalmente, saca el resultado por la UART.
 * Mide, por IMU y con el temporizador global del Cortex-A9:
 *   spi    = tiempo de las lecturas SPI de una trama (giro+acel+mag)
 *   filtro = tiempo de una actualizacion del filtro
 *   lat    = desde que se detecta el dato nuevo (GDA) hasta tener el cuaternion
 *   int    = intervalo entre tramas de la misma IMU (jitter) y tramas perdidas (intervalo > 1,5 periodos)
 * en tres fases: sin salida, con una linea por la UART cada 12 tramas (10 Hz, como el sistema de Galindo) y con una linea por trama.
 *
 * El sondeo es por lectura del registro de estado del A/G (bit GDA), igual que la maquina de estados de nav_spi_ctrl,
 * y los registros del AXI Quad SPI se manejan directamente (sin el driver XSpi), que es el limite inferior de tiempo del software.
 * Mismo orden de lectura que la PL: estado A/G -> giro -> acel -> estado mag -> mag (si hay dato nuevo).
 * Ejes y signos: AXIS_CFG 0x039 de la cadena en hardware (giro negado en x,y,z; mag x negado).
 *
 * Uso: programa el bitstream zybo_swspi, coloca las placas quietas, abre la UART a 115200 8N1 y ejecuta. Al terminar imprime las tablas.
 * Para la comparacion con la cadena en hardware usa las mismas placas, la misma ODR (119 Hz) y N_IMU = 2 (o los que tengas). */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#ifdef HOST
  #include <math.h>
  #include "host_sim.h"
#else
  #include "xil_io.h"
  #include "sleep.h"
  #define GT_BASE 0xF8F00200u
  static uint64_t gt_now(void) {
      volatile uint32_t *g = (volatile uint32_t *)GT_BASE; uint32_t hi, lo, hi2;
      do { hi = g[1]; lo = g[0]; hi2 = g[1]; } while (hi != hi2);
      return ((uint64_t)hi << 32) | lo;
  }
  static double gt_hz_calibrate(void) {
      volatile uint32_t *ctl = (volatile uint32_t *)(GT_BASE + 8u); if (!(*ctl & 1u)) *ctl |= 1u;
      uint64_t a = gt_now(); usleep(500000); uint64_t b = gt_now(); return (double)(b - a) * 2.0;
  }
#endif

#define N_IMU          2            /* IMUs conectadas (1..4) = NIMU del create_project.tcl */
#define QSPI_BASE(i)   (0x40000000u + 0x10000u * (uint32_t)(i))
#ifndef PHASE_SECONDS
#define PHASE_SECONDS  30           /* duracion de cada fase de medida */
#endif
#define BETA_START     1.0f
#define BETA_RUN       0.1f
#define BETA_START_MS  3000
#define FS_G_DPS_LSB   17.5e-3f     /* 500 dps */
static const float DT_IMU[4] = { 8.280e-3f, 8.307e-3f, 8.480e-3f, 8.057e-3f };   /* periodos medidos en la cadena en hardware */
static const int32_t MAG_PRESET[4][3] = {{1537, 1372, -2025}, {-1952, 2124, -1233}, {-2188, 2152, -1273}, {1508, 1242, -1704}};
#define GYRO_BIAS_FRAMES 240        /* ~2 s por IMU con la placa quieta */

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

/* Configuracion identica a CFG_PROGRAM de nav_pkg (ODR 119 Hz g/xl, 80 Hz mag, 500 dps, +-2 g, bajo consumo del mag) */
static int imu_config(int i) {
    uint8_t w;
    if (reg_read1(i, CS_AG, 0x0F, &w) || w != 0x68) { printf("IMU%d: WHO_AM_I A/G = 0x%02X (esperado 0x68)\r\n", i, w); return -1; }
    if (reg_read1(i, CS_MAG, 0x0F, &w) || w != 0x3D) { printf("IMU%d: WHO_AM_I mag = 0x%02X (esperado 0x3D)\r\n", i, w); return -1; }
    reg_write(i, CS_AG, 0x10, (3u << 5) | (1u << 3));    /* CTRL_REG1_G: 119 Hz, 500 dps */
    reg_write(i, CS_AG, 0x20, (3u << 5));                /* CTRL_REG6_XL: 119 Hz, +-2 g */
    reg_write(i, CS_AG, 0x22, 0x44);                     /* BDU, IF_ADD_INC */
    reg_write(i, CS_MAG, 0x20, (7u << 2));               /* CTRL_REG1_M: ODR 80 Hz */
    reg_write(i, CS_MAG, 0x22, 0x00);                    /* modo continuo */
    reg_write(i, CS_MAG, 0x24, 0x40);                    /* BDU_M */
    reg_write(i, CS_MAG, 0x23, 0x00);                    /* CTRL_REG4_M: OMZ bajo consumo */
    return 0;
}

/* ---- estado y estadisticas por IMU ---- */
typedef struct { uint64_t n; double sum; double mx; double mn; } stat_t;
static void st_clear(stat_t *s) { s->n = 0; s->sum = 0; s->mx = 0; s->mn = 1e30; }
static void st_add(stat_t *s, double v) { s->n++; s->sum += v; if (v > s->mx) s->mx = v; if (v < s->mn) s->mn = v; }

typedef struct {
    float q[4]; int16_t m[3];  int16_t goff[3];
    uint64_t t_start, t_prev; int have_prev;
    stat_t spi, flt, lat, itv; uint64_t lost, frames, polls, sat;
    uint64_t busy_ticks;
} imu_t;
static imu_t G[4];
static double g_hz;     /* frecuencia del temporizador */
static double us(uint64_t ticks) { return (double)ticks * 1e6 / g_hz; }

/* Una vuelta de sondeo de una IMU: devuelve 1 si ha procesado una trama. print_q: sacar la linea por la UART. */
static int imu_step(int i, int print_every, uint32_t *printed) {
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
    /* ejes como AXIS_CFG 0x039: giro negado (x,y,z) y menos el sesgo medido; mag con x negado y menos el hard-iron */
    const float gx = -(float)(g[0] - s->goff[0]) * FS_G_DPS_LSB * 0.0174532925f;
    const float gy = -(float)(g[1] - s->goff[1]) * FS_G_DPS_LSB * 0.0174532925f;
    const float gz = -(float)(g[2] - s->goff[2]) * FS_G_DPS_LSB * 0.0174532925f;
    const float mx = -(float)(s->m[0] - MAG_PRESET[i][0]), my = (float)(s->m[1] - MAG_PRESET[i][1]), mz = (float)(s->m[2] - MAG_PRESET[i][2]);
    const float beta = (us(t0 - s->t_start) < BETA_START_MS * 1000.0) ? BETA_START : BETA_RUN;
    madgwick_ahrs(s->q, gx, gy, gz, (float)a[0], (float)a[1], (float)a[2], mx, my, mz, beta, DT_IMU[i]);
    const uint64_t t2 = gt_now();
    st_add(&s->spi, us(t1 - t0)); st_add(&s->flt, us(t2 - t1)); st_add(&s->lat, us(t2 - t0));
    s->busy_ticks += t2 - t0;
    if (s->have_prev && s->frames > 2) {   /* se descartan las primeras tramas: al arrancar la fase hay un dato acumulado */
        const double itv = us(t0 - s->t_prev); st_add(&s->itv, itv);
        const double per = DT_IMU[i] * 1e6;
        if (itv > 1.5 * per) s->lost += (uint64_t)(itv / per + 0.5) - 1;
    }
    s->t_prev = t0; s->have_prev = 1; s->frames++;
    if (print_every > 0 && i == 0 && (s->frames % (uint32_t)print_every) == 0) {
        printf("%lu,%d,%d,%d,%d\r\n", (unsigned long)s->frames, (int)(s->q[0] * 10000.0f), (int)(s->q[1] * 10000.0f), (int)(s->q[2] * 10000.0f), (int)(s->q[3] * 10000.0f));
        (*printed)++;
    }
    return 1;
}

static void clear_stats(void) {
    for (int i = 0; i < N_IMU; i++) {
        st_clear(&G[i].spi); st_clear(&G[i].flt); st_clear(&G[i].lat); st_clear(&G[i].itv);
        G[i].lost = G[i].frames = G[i].polls = G[i].sat = 0; G[i].busy_ticks = 0; G[i].have_prev = 0;
    }
}
static void print_stats(const char *name, double secs, uint32_t printed) {
    printf("\r\n--- %s: %.1f s, %lu lineas impresas ---\r\n", name, secs, (unsigned long)printed);
    for (int i = 0; i < N_IMU; i++) {
        imu_t *s = &G[i];
        printf("IMU%d: %lu tramas (%.2f Hz), perdidas %lu, sondeos %lu (errores SPI %lu)\r\n", i, (unsigned long)s->frames, s->frames / secs, (unsigned long)s->lost, (unsigned long)s->polls, (unsigned long)s->sat);
        printf("   lectura SPI  media %.1f  max %.1f us | filtro  media %.2f  max %.2f us | deteccion->cuaternion  media %.1f  max %.1f us\r\n",
               s->spi.n ? s->spi.sum / s->spi.n : 0, s->spi.mx, s->flt.n ? s->flt.sum / s->flt.n : 0, s->flt.mx, s->lat.n ? s->lat.sum / s->lat.n : 0, s->lat.mx);
        printf("   intervalo entre tramas  media %.1f  min %.1f  max %.1f us (nominal %.1f)\r\n", s->itv.n ? s->itv.sum / s->itv.n : 0, s->itv.n ? s->itv.mn : 0, s->itv.mx, DT_IMU[i] * 1e6);
    }
    double busy = 0; for (int i = 0; i < N_IMU; i++) busy += us(G[i].busy_ticks);
    printf("Tiempo de SPI+filtro: %.2f %% de la CPU (el sondeo del estado ocupa el resto: el procesador esta al 100 %% mientras dura la adquisicion)\r\n", busy / (secs * 1e6) * 100.0);
}

static int run_phase(const char *name, int print_every) {
    clear_stats();
    uint32_t printed = 0;
    const uint64_t t_begin = gt_now(); const uint64_t t_end = t_begin + (uint64_t)(PHASE_SECONDS * g_hz);
    while (gt_now() < t_end) for (int i = 0; i < N_IMU; i++) imu_step(i, print_every, &printed);
    const double secs = (double)(gt_now() - t_begin) / g_hz;
    print_stats(name, secs, printed);
    return 0;
}

int main(void) {
#ifdef HOST
    g_hz = host_gt_hz();
#else
    g_hz = gt_hz_calibrate();
#endif
    printf("\r\n=== Linea base en software de extremo a extremo: %d IMU, SPI %.3f MHz, temporizador %.3f MHz ===\r\n", N_IMU, 100.0 / 32.0, g_hz / 1e6);
    for (int i = 0; i < N_IMU; i++) {
        qspi_init(i);
        if (imu_config(i)) { printf("fallo al configurar la IMU %d: se detiene\r\n", i); return -1; }
        G[i].q[0] = -1.0f; G[i].q[1] = 0; G[i].q[2] = 0; G[i].q[3] = 0.2f;     /* misma semilla que la cadena en hardware */
        G[i].m[0] = G[i].m[1] = G[i].m[2] = 0;
    }
    /* sesgo del giroscopio con las placas QUIETAS (media de GYRO_BIAS_FRAMES tramas, como GYRO_AT_BOOT en la cadena en hardware) */
    printf("Midiendo el sesgo del giroscopio (~%d s, placas quietas)...\r\n", GYRO_BIAS_FRAMES / 119);
    for (int i = 0; i < N_IMU; i++) {
        int32_t acc[3] = {0, 0, 0}; int n = 0;
        while (n < GYRO_BIAS_FRAMES) {
            uint8_t st; int16_t g[3];
            if (reg_read1(i, CS_AG, 0x17, &st) || !(st & 0x02u)) continue;
            reg_burst6(i, CS_AG, 0x18, g); for (int k = 0; k < 3; k++) acc[k] += g[k]; n++;
            int16_t a[3]; reg_burst6(i, CS_AG, 0x28, a);              /* vacia el acelerometro para no perder el siguiente GDA */
        }
        for (int k = 0; k < 3; k++) G[i].goff[k] = (int16_t)(acc[k] / n);
        printf("IMU%d: sesgo giro = (%d, %d, %d) cuentas\r\n", i, G[i].goff[0], G[i].goff[1], G[i].goff[2]);
    }
    const uint64_t t0 = gt_now(); for (int i = 0; i < N_IMU; i++) G[i].t_start = t0;
    run_phase("FASE A: sin salida por la UART", 0);
    run_phase("FASE B: una linea cada 12 tramas (10 Hz) por la UART", 12);
    run_phase("FASE C: una linea por trama por la UART", 1);
    printf("\r\nFin. Copia las tres tablas.\r\n");
    return 0;
}
