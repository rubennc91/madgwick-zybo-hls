/* main_freertos_pl.c -- PASO 1 de la guia micro-ROS: FreeRTOS en el A9 + cadena en la PL (diseno Integration_Zybo_multi, bitstream SIN CAMBIAR)
 *
 * Una tarea de inicio configura la PL igual que main_pl.c (ODR, offsets, dt, sesgo del giroscopio), mide la referencia de CPU libre
 * (solo la tarea de inactividad) y arranca dos tareas:
 *   - lector: lee los cuaterniones de las N_IMU IMU por AXI-Lite cada READ_MS ms (lectura coherente con frame_cnt);
 *   - estadistica: cada STAT_S s imprime un cuaternion por IMU, las lecturas incoherentes y la CPU LIBRE
 *     (iteraciones/s del gancho de inactividad frente a la referencia).
 * Aqui todavia NO hay micro-ROS: el paso 2 sustituye el lector por la publicacion del mensaje.
 *
 * BSP (Vitis, plataforma freertos10_xilinx): usar_idle_hook = true, total_heap_size >= 128 KB, baudios de la UART = el de la consola.
 * Placas QUIETAS al arrancar (sesgo del giroscopio). ODR_HZ = 119 o 952, N_IMU = 1..4 (por defecto 4). */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "xil_io.h"
#include "xil_types.h"
#include "sleep.h"

#ifndef N_IMU
#define N_IMU 4
#endif
#ifndef READ_MS
#define READ_MS 20          /* periodo del lector: 50 Hz */
#endif
#ifndef STAT_S
#define STAT_S 2
#endif

#define GT_BASE 0xF8F00200u
static uint64_t gt_now(void) {
    uint32_t hi, lo;
    do { hi = Xil_In32(GT_BASE + 4u); lo = Xil_In32(GT_BASE + 0u); } while (hi != Xil_In32(GT_BASE + 4u));
    return ((uint64_t)hi << 32) | lo;
}
static const double g_hz = 333.333333e6;   /* temporizador global = CPU/2 */
static void gt_init(void) { if (!(Xil_In32(GT_BASE + 8u) & 1u)) Xil_Out32(GT_BASE + 8u, Xil_In32(GT_BASE + 8u) | 1u); }
static double us(uint64_t t) { return (double)t * 1e6 / g_hz; }

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


/* ---- CPU libre: contador del gancho de inactividad (requiere configUSE_IDLE_HOOK = 1) ---- */
static volatile uint32_t g_idle;
void vApplicationIdleHook(void) { g_idle++; }

static double g_idle_ref;                 /* iteraciones/s con solo la tarea de inactividad */
static volatile uint32_t g_incoh, g_reads;
static float g_q[4][4];
static volatile uint32_t g_fc[4];

static void pl_setup(void) {
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
}

static int pl_start(void) {
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
    return 0;
}

static void reader_task(void *arg) {                    /* sustituira a la publicacion de micro-ROS en el paso 2 */
    (void)arg;
    TickType_t t0 = xTaskGetTickCount();
    for (;;) {
        for (int i = 0; i < N_IMU; i++) {
            float q[4]; uint32_t fc;
            if (read_q(i, q, &fc)) { memcpy(g_q[i], q, sizeof q); g_fc[i] = fc; } else g_incoh++;
        }
        g_reads++;
        vTaskDelayUntil(&t0, pdMS_TO_TICKS(READ_MS));
    }
}

static void stat_task(void *arg) {
    (void)arg;
    TickType_t t0 = xTaskGetTickCount();
    uint32_t i0 = g_idle; uint64_t g0 = gt_now();
    for (;;) {
        vTaskDelayUntil(&t0, pdMS_TO_TICKS(STAT_S * 1000));
        const uint32_t i1 = g_idle; const uint64_t g1 = gt_now();
        const double rate = (double)(uint32_t)(i1 - i0) / (((double)(g1 - g0)) / g_hz);
        i0 = i1; g0 = g1;
        printf("CPU libre %.1f %% (inactividad %.3e it/s, ref %.3e) | lecturas %lu, incoherentes %lu\r\n", 100.0 * rate / g_idle_ref, rate, g_idle_ref, (unsigned long)g_reads, (unsigned long)g_incoh);
        for (int i = 0; i < N_IMU; i++)
            printf("  IMU%d fc=%lu q=(%.4f, %.4f, %.4f, %.4f)\r\n", i, (unsigned long)g_fc[i], (double)g_q[i][0], (double)g_q[i][1], (double)g_q[i][2], (double)g_q[i][3]);
    }
}

static void init_task(void *arg) {
    (void)arg;
    gt_init();
    printf("\r\n=== FreeRTOS + cadena en la PL: %d IMU, ODR %d Hz, lector %d ms ===\r\n", N_IMU, ODR_HZ, READ_MS);
    pl_setup();
    /* referencia de CPU libre: solo la tarea de inactividad, con la PL todavia sin adquirir */
    { uint32_t a = g_idle; uint64_t t = gt_now(); vTaskDelay(pdMS_TO_TICKS(2000)); g_idle_ref = (double)(uint32_t)(g_idle - a) / (((double)(gt_now() - t)) / g_hz); }
    printf("Referencia de CPU (inactividad): %.3e iteraciones/s\r\n", g_idle_ref);
    if (pl_start() == 0) {
        xTaskCreate(reader_task, "leer", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
        xTaskCreate(stat_task,   "stat", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    }
    vTaskDelete(NULL);
}

int main(void) {
    xTaskCreate(init_task, "init", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);
    vTaskStartScheduler();
    for (;;) {}
}
