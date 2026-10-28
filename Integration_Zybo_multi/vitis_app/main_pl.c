/* main_pl.c -- BASELINE DE LA CADENA EN LA PL (diseno Integration_Zybo_multi, SIN CAMBIAR EL BITSTREAM)
 *
 * Es el equivalente de main.c del diseno swspi (ARM hace SPI + filtro): aqui el ARM solo CONFIGURA y LEE RESULTADOS.
 * La adquisicion SPI, el arbitraje, la conversion y el filtro Madgwick los hace la PL; el ARM no interviene en la ruta de datos.
 * Mismas tres fases (A sin salida, B una linea cada ~100 ms, C una linea por trama), misma UART (115 200 baud), mismas placas y ODR.
 *
 * Que mide (por fase y por IMU):
 *   - tramas adquiridas por la PL (SAMPLE_COUNT de nav_spi_ctrl) y procesadas por el nucleo (frame_cnt de mad_multi) -> tramas perdidas = diferencia
 *   - latencia de adquisicion en hardware: TS_END - TS_GDA (contador de 100 MHz de nav_spi_ctrl), muestreada a ~10 Hz en la fase B;
 *     la deteccion->cuaternion es esa cifra mas el tiempo del nucleo (<= 29,4 us, peor caso del programa en ROM, no medido por trama)
 *   - maximo de espera en el estado de streaming (STREAM_MAX, acumulado desde el arranque)
 *   - CPU del ARM: (i) tiempo en leer el cuaternion (2 IMU) y (ii) CPU LIBRE = iteraciones de un bucle de fondo / iteraciones sin la PL activa.
 *     En la baseline en software no hay CPU libre con sondeo (100 % ocupada); aqui el bucle de fondo representa "otra tarea" (SO, ROS...).
 * Uso: programa el bitstream zybo_multi, placas QUIETAS, UART a 115200 8N1; ODR_HZ = 119 o 952 igual que en la baseline en software. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "xil_io.h"
#include "xil_types.h"
#include "sleep.h"

#define GT_BASE 0xF8F00200u
static uint64_t gt_now(void) {
    uint32_t hi, lo;
    do { hi = Xil_In32(GT_BASE + 4u); lo = Xil_In32(GT_BASE + 0u); } while (hi != Xil_In32(GT_BASE + 4u));
    return ((uint64_t)hi << 32) | lo;
}
static double g_hz, g_hz_cal;
static void gt_init(void) {
    if (!(Xil_In32(GT_BASE + 8u) & 1u)) Xil_Out32(GT_BASE + 8u, Xil_In32(GT_BASE + 8u) | 1u);
    uint64_t a = gt_now(); usleep(500000); uint64_t b = gt_now(); g_hz_cal = (double)(b - a) * 2.0;
    g_hz = 333.333333e6;   /* temporizador global = CPU/2 = 333,33 MHz (CPU a 666,67 MHz); la calibracion con usleep varia ~1 % entre arranques */
}
static double us(uint64_t t) { return (double)t * 1e6 / g_hz; }

#define N_IMU 2
static const uint32_t NAV_BASES[4] = { 0x40000000u, 0x40010000u, 0x40020000u, 0x40030000u };
#define MAD_BASE 0x40040000u
#define R2F_BASE 0x40050000u
#define NAV_CTRL 0x00
#define NAV_ODR_CFG 0x04
#define NAV_RANGE_CFG 0x08
#define NAV_STATUS 0x40
#define NAV_SAMPLE_CNT 0x44
#define NAV_RAW0 0x50
#define NAV_OFF_G 0x20
#define NAV_OFF_M 0x2C
#define NAV_AXIS_CFG 0x38
#define NAV_SNAP_SEQ 0x74
#define NAV_SNAP_TS_GDA 0x78
#define NAV_SNAP_TS_END 0x7C
#define NAV_STREAM_MAX 0xA8
#define NAV_CTRL_ENABLE 1u
#define ST_CONFIG_DONE (1u << 1)
#define ST_ERROR (1u << 4)
#define HLS_AP_CTRL 0x00
#define AP_START 1u
#define AP_AUTORESTART 0x80u
#define R2F_FS_G_REG 0x10
#define MAD_BETA 0x10
#define MAD_RESET 0x20
#define MAD_Q_OUT 0x40
#define MAD_FRAME_CNT 0x80
#define MAD_DT_IMU 0x90
#define FS_G_SELECTED 1u            /* 500 dps */
#define AXIS_CFG_PRESET 0x039u      /* igual que la baseline en software */
#define BETA_START 1.0f
#define BETA_RUN 0.1f
#define BETA_START_MS 3000
static const int32_t MAG_PRESET[4][3] = {{1537,1372,-2025},{-1952,2124,-1233},{-2188,2152,-1273},{1508,1242,-1704}};

/* ODR: 119 Hz (3) o 952 Hz (6); el magnetometro sigue a 80 Hz. Igual que ODR_HZ de la baseline en software. */
#ifndef ODR_HZ
#define ODR_HZ 119
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
#define PRINT_10HZ ((ODR_HZ + 5) / 10)
#ifndef PHASE_SECONDS
#define PHASE_SECONDS 30
#endif
#ifndef PHASE_C_SECONDS
#define PHASE_C_SECONDS 5
#endif
static const float DT_IMU[4] = { 8.280e-3f / DT_DIV, 8.307e-3f / DT_DIV, 8.480e-3f / DT_DIV, 8.057e-3f / DT_DIV };

static inline void wr(uint32_t b, uint32_t o, uint32_t v) { Xil_Out32(b + o, v); }
static inline uint32_t rd(uint32_t b, uint32_t o) { return Xil_In32(b + o); }
static inline uint32_t f2u(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
static inline float u2f(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }

static int read_q(int i, float q[4], uint32_t *fc) {   /* cuaternion coherente con frame_cnt */
    uint32_t c1 = rd(MAD_BASE, MAD_FRAME_CNT + 4u * (uint32_t)i);
    for (int k = 0; k < 4; k++) q[k] = u2f(rd(MAD_BASE, MAD_Q_OUT + 16u * (uint32_t)i + 4u * (uint32_t)k));
    uint32_t c2 = rd(MAD_BASE, MAD_FRAME_CNT + 4u * (uint32_t)i);
    if (fc) *fc = c1;
    return c1 == c2;
}
static void snap(uint32_t b, uint32_t *seq, uint32_t *tg, uint32_t *te) {   /* instantanea atomica (SEQ antes y despues) */
    uint32_t s1, s2;
    do { s1 = rd(b, NAV_SNAP_SEQ); *tg = rd(b, NAV_SNAP_TS_GDA); *te = rd(b, NAV_SNAP_TS_END); s2 = rd(b, NAV_SNAP_SEQ); } while (s1 != s2);
    *seq = s1;
}

/* Tarea de fondo: cuenta iteraciones durante el tiempo que se le da; representa "otro trabajo" de la CPU. */
static volatile uint32_t g_sink;
__attribute__((noinline)) static uint64_t bg_chunk(void) { uint32_t x = g_sink; for (int k = 0; k < 4096; k++) x = x * 1664525u + 1013904223u; g_sink = x; return 4096u; }
__attribute__((noinline)) static uint64_t bg_run_until(uint64_t t_end) {   /* UNICA funcion de fondo: referencia, fase A y fase B */
    uint64_t n = 0;
    while (gt_now() < t_end) n += bg_chunk();
    return n;
}
static uint64_t bg_run_seconds(double secs) { return bg_run_until(gt_now() + (uint64_t)(secs * g_hz)); }
static double g_bg_ref;   /* iteraciones/s con la PL parada */

typedef struct { double n, sum, mn, mx; } st_t;
static void st_clear(st_t *s) { s->n = s->sum = 0; s->mn = 1e30; s->mx = 0; }
static void st_add(st_t *s, double v) { s->n++; s->sum += v; if (v > s->mx) s->mx = v; if (v < s->mn) s->mn = v; }

static void measure_dt(int seconds) {
    uint32_t s0[4], a0[4], b0[4];
    for (int i = 0; i < N_IMU; i++) snap(NAV_BASES[i], &s0[i], &a0[i], &b0[i]);
    usleep((uint32_t)seconds * 1000000u);
    for (int i = 0; i < N_IMU; i++) {
        uint32_t s1, a1, b1; snap(NAV_BASES[i], &s1, &a1, &b1);
        uint32_t ds = s1 - s0[i], dc = a1 - a0[i];
        float dt = DT_IMU[i];
        if (ds >= 100u) dt = (float)((double)dc / (double)ds * 1.0e-8);
        wr(MAD_BASE, MAD_DT_IMU + 4u * (uint32_t)i, f2u(dt));
        printf("IMU%d: periodo medido %.3f us (%.2f Hz)\r\n", i, (double)dt * 1e6, 1.0 / (double)dt);
    }
}

static uint32_t cnt_acq[4], cnt_core[4];
static void counters_mark(void) { for (int i = 0; i < N_IMU; i++) { cnt_acq[i] = rd(NAV_BASES[i], NAV_SAMPLE_CNT); cnt_core[i] = rd(MAD_BASE, MAD_FRAME_CNT + 4u * (uint32_t)i); } }

static void print_line(int i, uint32_t fc, const float q[4]) {
    printf("%lu,%d,%d,%d,%d\r\n", (unsigned long)fc, (int)(q[0] * 10000.0f), (int)(q[1] * 10000.0f), (int)(q[2] * 10000.0f), (int)(q[3] * 10000.0f));
    (void)i;
}

/* mode: 0 = A (nada), 1 = B (una lectura+linea cada ~100 ms), 2 = C (una linea por trama nueva de la IMU0) */
static void run_phase(const char *name, int mode, double secs) {
    st_t rdq, acq; st_clear(&rdq); st_clear(&acq);
    uint64_t bg_iters = 0, read_ticks = 0, handler_ticks = 0; uint32_t printed = 0, incoh = 0;
    counters_mark();
    const uint64_t t0 = gt_now(), t_end = t0 + (uint64_t)(secs * g_hz);
    uint64_t next_b = t0; uint32_t last_fc0 = cnt_core[0];
    while (gt_now() < t_end) {
        if (mode == 0) { bg_iters = bg_run_until(t_end); break; }   /* misma funcion que la referencia */
        if (mode == 1) {
            {
                bg_iters += bg_run_until(next_b < t_end ? next_b : t_end);   /* mismo bucle que la referencia hasta el siguiente instante de lectura */
                if (gt_now() >= t_end) break;
                const uint64_t h0 = gt_now();
                next_b += (uint64_t)(0.1 * g_hz);
                float q[2][4]; uint32_t fc[2];
                uint64_t a = gt_now();
                for (int i = 0; i < N_IMU; i++) if (!read_q(i, q[i], &fc[i])) incoh++;
                uint64_t b = gt_now(); st_add(&rdq, us(b - a)); read_ticks += b - a;
                for (int i = 0; i < N_IMU; i++) {      /* latencia de adquisicion en hardware, muestreada */
                    uint32_t s, tg, te; snap(NAV_BASES[i], &s, &tg, &te);
                    if (i == 0) st_add(&acq, (double)(uint32_t)(te - tg) * 0.01);
                }
                print_line(0, fc[0], q[0]); printed++;
                handler_ticks += gt_now() - h0;       /* lectura + instantaneas + impresion */
            }
            continue;
        }
        { /* C: sondeo del contador de tramas de la IMU0 y una linea por trama nueva */
            uint32_t c = rd(MAD_BASE, MAD_FRAME_CNT);
            if (c == last_fc0) continue;
            last_fc0 = c;
            float q[2][4]; uint32_t fc[2];
            uint64_t a = gt_now();
            for (int i = 0; i < N_IMU; i++) if (!read_q(i, q[i], &fc[i])) incoh++;
            uint64_t b = gt_now(); st_add(&rdq, us(b - a)); read_ticks += b - a;
            print_line(0, fc[0], q[0]); printed++;
        }
    }
    const double s = (double)(gt_now() - t0) / g_hz;
    printf("\r\n--- %s: %.1f s, %lu lineas impresas ---\r\n", name, s, (unsigned long)printed);
    for (int i = 0; i < N_IMU; i++) {
        const uint32_t da = rd(NAV_BASES[i], NAV_SAMPLE_CNT) - cnt_acq[i];
        const uint32_t dc = rd(MAD_BASE, MAD_FRAME_CNT + 4u * (uint32_t)i) - cnt_core[i];
        const int32_t lost = (int32_t)(da - dc);       /* adquiridas por la PL y no procesadas (+-1 en vuelo) */
        printf("IMU%d: %lu tramas adquiridas, %lu procesadas por el nucleo (%.2f Hz), perdidas %ld, espera max en streaming %.2f us (desde el arranque)\r\n",
               i, (unsigned long)da, (unsigned long)dc, (double)dc / s, (long)lost, (double)rd(NAV_BASES[i], NAV_STREAM_MAX) * 0.01);
    }
    if (rdq.n) printf("   lectura del cuaternion de %d IMU por AXI-Lite: media %.2f  max %.2f us (%lu lecturas, %lu no coherentes)\r\n", N_IMU, rdq.sum / rdq.n, rdq.mx, (unsigned long)rdq.n, (unsigned long)incoh);
    if (acq.n) printf("   latencia de adquisicion en hardware (TS_END-TS_GDA, IMU0, %lu muestras): media %.1f  min %.1f  max %.1f us; + nucleo <= 29.4 us\r\n", (unsigned long)acq.n, acq.sum / acq.n, acq.mn, acq.mx);
    printf("   CPU ocupada leyendo el cuaternion: %.4f %% | ", (double)us(read_ticks) / (s * 1e6) * 100.0);
    if (mode == 1) printf("\r\n   CPU ocupada en la comunicacion completa (lecturas + instantaneas + impresion): %.3f %%, por ciclo de 100 ms media %.1f us\r\n   ", (double)us(handler_ticks) / (s * 1e6) * 100.0, us(handler_ticks) / (double)(printed ? printed : 1));
    if (mode == 2) printf("sin bucle de fondo (la CPU imprime sin parar)\r\n");
    else printf("CPU libre para otra tarea (bucle de fondo / referencia sin PL): %.1f %%\r\n", (double)bg_iters / s / g_bg_ref * 100.0);
}

int main(void) {
    gt_init();
    printf("\r\n=== Cadena en la PL (un solo nucleo): %d IMU, ODR %d Hz, temporizador %.3f MHz (calibrado con usleep: %.3f) ===\r\n", N_IMU, ODR_HZ, g_hz / 1e6, g_hz_cal / 1e6);
    for (int i = 0; i < N_IMU; i++) {
        const uint32_t b = NAV_BASES[i];
        wr(b, NAV_ODR_CFG, (7u << 6) | (ODR_CODE << 3) | ODR_CODE);
        wr(b, NAV_RANGE_CFG, FS_G_SELECTED & 3u);
        wr(b, NAV_AXIS_CFG, AXIS_CFG_PRESET);
        for (int k = 0; k < 3; k++) { wr(b, NAV_OFF_G + 4u * (uint32_t)k, 0); wr(b, NAV_OFF_M + 4u * (uint32_t)k, (uint32_t)MAG_PRESET[i][k] & 0xFFFFu); }
    }
    wr(R2F_BASE, R2F_FS_G_REG, FS_G_SELECTED & 3u);
    wr(R2F_BASE, HLS_AP_CTRL, AP_START | AP_AUTORESTART);
    wr(MAD_BASE, MAD_BETA, f2u(BETA_START));
    for (int i = 0; i < 4; i++) wr(MAD_BASE, MAD_DT_IMU + 4u * (uint32_t)i, f2u(DT_IMU[i]));
    wr(MAD_BASE, MAD_RESET, (1u << N_IMU) - 1u);
    wr(MAD_BASE, HLS_AP_CTRL, AP_START | AP_AUTORESTART);

    /* referencia de CPU libre: el mismo bucle de fondo con la PL todavia sin adquirir */
    g_bg_ref = (double)bg_run_seconds(2.0) / 2.0;
    printf("Referencia de CPU (PL parada): %.3e iteraciones/s\r\n", g_bg_ref);

    for (int i = 0; i < N_IMU; i++) wr(NAV_BASES[i], NAV_CTRL, NAV_CTRL_ENABLE);
    for (int i = 0; i < N_IMU; i++) {
        int t = 0;
        for (; t < 1000; t++) { uint32_t st = rd(NAV_BASES[i], NAV_STATUS); if (st & (ST_CONFIG_DONE | ST_ERROR)) break; usleep(1000); }
        if (rd(NAV_BASES[i], NAV_STATUS) & ST_ERROR) { printf("IMU%d: ERROR de configuracion (STATUS=0x%08lx)\r\n", i, (unsigned long)rd(NAV_BASES[i], NAV_STATUS)); return -1; }
    }
    for (int t = 0; t < 2000; t++) {
        int all = 1; for (int i = 0; i < N_IMU; i++) if (rd(MAD_BASE, MAD_FRAME_CNT + 4u * (uint32_t)i) == 0) all = 0;
        if (all) break;
        usleep(1000);
    }
    measure_dt(3);
    /* sesgo del giroscopio con las placas QUIETAS: media de 2 s de tramas (RAW0..2 antes de offsets) */
    printf("Midiendo el sesgo del giroscopio (~2 s, placas quietas)...\r\n");
    for (int i = 0; i < N_IMU; i++) {
        const uint32_t b = NAV_BASES[i]; int64_t acc[3] = {0, 0, 0}; int n = 0; uint32_t last = rd(b, NAV_SAMPLE_CNT);
        while (n < 2 * ODR_HZ) {
            uint32_t c = rd(b, NAV_SAMPLE_CNT); if (c == last) continue; last = c;
            for (int k = 0; k < 3; k++) acc[k] += (int16_t)rd(b, NAV_RAW0 + 4u * (uint32_t)k);
            n++;
        }
        int32_t off[3];
        for (int k = 0; k < 3; k++) { off[k] = (int32_t)(acc[k] / n); wr(b, NAV_OFF_G + 4u * (uint32_t)k, (uint32_t)off[k] & 0xFFFFu); }
        printf("IMU%d: sesgo giro = (%ld, %ld, %ld) cuentas\r\n", i, (long)off[0], (long)off[1], (long)off[2]);
    }
    wr(MAD_BASE, MAD_RESET, 0); wr(MAD_BASE, MAD_BETA, f2u(BETA_START));
    { const uint64_t t_b = gt_now(); while (us(gt_now() - t_b) < BETA_START_MS * 1000.0) (void)0; }
    wr(MAD_BASE, MAD_BETA, f2u(BETA_RUN));

    run_phase("FASE A: sin salida por la UART (la CPU solo ejecuta el bucle de fondo)", 0, PHASE_SECONDS);
    run_phase("FASE B: una linea cada ~100 ms (10 Hz) por la UART", 1, PHASE_SECONDS);
    run_phase("FASE C: una linea por trama por la UART", 2, PHASE_C_SECONDS);
    /* referencia final con la PL parada: si coincide con la inicial, la PL no resta CPU al ARM */
    for (int i = 0; i < N_IMU; i++) wr(NAV_BASES[i], NAV_CTRL, 0);
    usleep(100000);
    { const double r2 = (double)bg_run_seconds(2.0) / 2.0; printf("\r\nReferencia de CPU final (PL parada): %.3e iteraciones/s (inicial %.3e, cociente %.3f)\r\n", r2, g_bg_ref, r2 / g_bg_ref); }
    printf("\r\nFin. Copia las tres tablas.\r\n");
    return 0;
}
