// main.c -- Varias IMUs (hasta 4) servidas por UN solo nucleo Madgwick (Zybo Z7-10)
//
// Cadena en la PL:
//   4 x { Pmod NAV (SPI) -> nav_spi_ctrl_i }  -> r2f_mux (raw2float + round-robin, TID = IMU)
//   -> mad_multi (un estado q por IMU)  -> q_out[4*i..4*i+3], frame_cnt[i]  (AXI-Lite)
//
// Mapa de direcciones (fijado en create_project.tcl):
//   nav_spi_ctrl_0..3 0x40000000 / 0x40010000 / 0x40020000 / 0x40030000
//   mad_multi 0x40040000     r2f_mux 0x40050000
// Offsets del nucleo multi: fijados con "offset=" en madgwick_multi.cpp (ver abajo).
// Teclas: 0..3 selecciona IMU, g gyro (todas), m mag (la seleccionada), c borrar offsets,
//         r reset filtros, a/b ejes 0x018/0x039, l/L captura CSV, d volcar de nuevo.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "xil_io.h"
#include "xil_types.h"
#include "sleep.h"
#include "xparameters.h"
#if defined(STDIN_BASEADDRESS)
  #define CON_BASE STDIN_BASEADDRESS
#elif defined(XPAR_XUARTPS_0_BASEADDR)
  #define CON_BASE XPAR_XUARTPS_0_BASEADDR
#endif
#ifdef CON_BASE
  #include "xuartps_hw.h"
#endif


// ---- Temporizador global del Cortex-A9 (lectura directa; xtime_l.h no existe en el flujo SDT) ----
#define GT_BASE 0xF8F00200u
static uint64_t gt_now(void) {
    u32 hi, lo;
    do { hi = Xil_In32(GT_BASE + 4u); lo = Xil_In32(GT_BASE + 0u); } while (hi != Xil_In32(GT_BASE + 4u));
    return ((uint64_t)hi << 32) | lo;
}
static uint64_t g_gt_hz = 0;              // frecuencia del temporizador (CPU/2)
static void gt_init(void) {
    if (!(Xil_In32(GT_BASE + 8u) & 1u)) Xil_Out32(GT_BASE + 8u, Xil_In32(GT_BASE + 8u) | 1u);
    // Las macros XPAR_*CLK_FREQ de este BSP no son fiables: se calibra el temporizador contra usleep (0,5 s).
    uint64_t a = gt_now(); usleep(500000); uint64_t b = gt_now();
    g_gt_hz = (b - a) * 2u;
}

// ---- Direcciones base (fijas, ver cabecera) -------------------------------
#define N_IMU     4                     // IMUs conectadas (1..4); las demas no se tocan
static const u32 NAV_BASES[4] = { 0x40000000u, 0x40010000u, 0x40020000u, 0x40030000u };
static u32 g_nav = 0x40000000u;         // nav_spi_ctrl de la IMU seleccionada
#define NAV_BASE  (g_nav)
#define MAD_BASE  0x40040000u           // mad_multi
#define R2F_BASE  0x40050000u           // r2f_mux

// ---- nav_spi_ctrl (nav_pkg.vhd) -------------------------------------------
#define NAV_CTRL        0x00
#define NAV_ODR_CFG     0x04
#define NAV_RANGE_CFG   0x08
#define NAV_APPLY_CFG   0x0C
#define NAV_STATUS      0x40
#define NAV_SAMPLE_CNT  0x44
#define NAV_RAW0        0x50   // 9 palabras: gx gy gz ax ay az mx my mz (int16 extendido)
#define NAV_OFF_G       0x20   // sesgo gyro:  gx 0x20, gy 0x24, gz 0x28 (int16, se RESTA)
#define NAV_OFF_M       0x2C   // hard-iron:   mx 0x2C, my 0x30, mz 0x34 (int16, se RESTA)
#define NAV_AXIS_CFG    0x38   // orientacion de ejes (ver nav_spi_ctrl.vhd): [2:0] signos mag,
                               // [5:3] signos gyro, [8:6] permutacion mag, [11:9] signos accel
#define NAV_DBG_WHOAMI  0x48   // [7:0] WHO_AM_I leido del A/G, [15:8] del Mag

#define NAV_CTRL_ENABLE     (1u << 0)
#define NAV_CTRL_SOFT_RESET (1u << 1)
#define NAV_CTRL_IRQ_EN     (1u << 2)
#define NAV_CTRL_CLEAR_ERR  (1u << 3)

#define ST_BUSY          (1u << 0)
#define ST_CONFIG_DONE   (1u << 1)
#define ST_AG_DETECTED   (1u << 2)
#define ST_MAG_DETECTED  (1u << 3)
#define ST_ERROR         (1u << 4)
#define ST_ERR_AG_WHOAMI (1u << 5)
#define ST_ERR_MAG_WHOAMI (1u << 6)
#define ST_ERR_TIMEOUT   (1u << 7)
#define ST_ERR_OVERRUN   (1u << 8)

// ---- Nucleos HLS (offsets de los *_hw.h) ----------------------------------
#define HLS_AP_CTRL   0x00   // bit0 ap_start, bit1 ap_done, bit2 ap_idle, bit7 auto_restart
#define AP_START        (1u << 0)
#define AP_DONE         (1u << 1)
#define AP_AUTORESTART  (1u << 7)

#define R2F_FS_G_REG  0x10   // [1:0]

// mad_multi: offsets fijados con #pragma HLS INTERFACE s_axilite ... offset=
#define MAD_BETA      0x10
#define MAD_DT        0x18
#define MAD_RESET     0x20   // MASCARA: bit i = reiniciar la IMU i
#define MAD_Q_OUT     0x40   // 16 palabras: IMU i -> q0..q3 en 0x40 + 16*i + 4*k
#define MAD_FRAME_CNT 0x80   // 4 palabras: tramas procesadas por IMU

// Rango del giroscopio: 0=245dps 1=500dps 2/3=2000dps.
// Debe ser el MISMO valor en nav_spi_ctrl (RANGE_CFG[1:0]) y en raw2float.
#define FS_G_SELECTED  1u

// nav_spi_ctrl entrega una trama por cada dato nuevo del giroscopio/acelerometro
// (ODR 119 Hz) y reutiliza la ultima muestra del magnetometro (ODR 80 Hz).
// ODR_CFG: odr_g=3 (119 Hz), odr_xl=3 (119 Hz), odr_m=7 (80 Hz).
#define DT_SECONDS    (1.0f / 119.0f)

// beta alta al arrancar (convergencia rapida) y baja en regimen (menos ruido).
#define BETA_START    1.0f
#define BETA_RUN      0.1f
#define BETA_START_MS 3000

// ---- Calibracion ------------------------------------------------------------
#define GYRO_CAL_SAMPLES   256     // ~2,2 s con la placa QUIETA
#define GYRO_CAL_MAX_RANGE 900     // cuentas (~10 dps): si se mueve mas, repite
#define MAG_CAL_SECONDS    0       // girar la placa en todas direcciones; 0 = no calibrar
#define MAG_CAL_MAX_FRAMES 4200    // tamano del buffer (35 s a 119 Hz)
#define MAG_CAL_MIN_COVER  1.2     // rango de cada eje >= 1,2 * radio (60 % del diametro)
#define MAG_CAL_MAX_RESID  0.08    // desviacion del ajuste / radio: mas = descartar

// LED de usuario del PS (Zybo Z7: LD4 en MIO7 segun el manual; si no es ese, no pasa nada).
// Se maneja por registros del GPIO del PS: no hace falta driver ni cambiar el bitstream.
#define USE_PS_LED         1
#define PS_LED_MIO         7
// Si MAG_CAL_SECONDS = 0 se usan estos valores (los que imprime la calibracion):
// Orientacion de ejes: -1 = detectarla en la calibracion del magnetometro ('m');
// otro valor = se usa tal cual. Por defecto del hardware: 0x018 (gyro x,y negados).
#define AXIS_CFG_PRESET  (0x039)
#define AXIS_CFG_DEFAULT 0x018
#define MAG_PRESET_X  0
#define MAG_PRESET_Y  0
#define MAG_PRESET_Z  0

static inline void wr(u32 base, u32 off, u32 v) { Xil_Out32(base + off, v); }
static inline u32  rd(u32 base, u32 off)        { return Xil_In32(base + off); }

static inline u32 f2u(float f) { u32 u; memcpy(&u, &f, 4); return u; }
static inline float u2f(u32 u) { float f; memcpy(&f, &u, 4); return f; }

// Imprime un float como "+d.dddd" usando solo enteros: asi no hace falta
// activar el soporte de float en printf (-u _printf_float) en el linker.
static void print_f4(float v) {
    long x = (long)(v * 10000.0f + (v >= 0.0f ? 0.5f : -0.5f));
    char sg = '+';
    if (x < 0) { sg = '-'; x = -x; }
    printf("%c%ld.%04ld", sg, x / 10000, x % 10000);
}

// "+ddd.d" (un decimal) usando solo enteros
static void print_f1(float v) {
    long x = (long)(v * 10.0f + (v >= 0.0f ? 0.5f : -0.5f));
    char sg = '+';
    if (x < 0) { sg = '-'; x = -x; }
    printf("%c%ld.%01ld", sg, x / 10, x % 10);
}

// Pitido en el PC: caracter BEL (0x07) por la UART. Tera Term lo reproduce como
// pitido (Setup > Additional settings > Beep). n pitidos separados 250 ms.
static void led_set(int on) {
#if USE_PS_LED
    // GPIO PS (Zynq): DIRM_0 0x204, OEN_0 0x208, MASK_DATA_0_LSW 0x000 (MIO0..15)
    const u32 gpio = 0xE000A000u;
    const u32 bit = 1u << PS_LED_MIO;
    Xil_Out32(gpio + 0x204, Xil_In32(gpio + 0x204) | bit);
    Xil_Out32(gpio + 0x208, Xil_In32(gpio + 0x208) | bit);
    Xil_Out32(gpio + 0x000, ((~bit & 0xFFFFu) << 16) | (on ? bit : 0u));
#else
    (void)on;
#endif
}

// Aviso al usuario: n pitidos (BEL por UART) y n parpadeos del LED.
static void beep(int n) {
    for (int i = 0; i < n; i++) {
        led_set(1);
        printf("\a*** AVISO %d/%d ***\r\n", i + 1, n);
        fflush(stdout);
        usleep(150000);
        led_set(0);
        if (i + 1 < n) usleep(250000);
    }
}

static double g_goff[3] = {0, 0, 0};   // sesgo del giroscopio (dominio crudo)
static float g_moff[3] = {0, 0, 0};   // offset del magnetometro vigente (para mostrar |m| compensado)

static inline int32_t raw_i(int i) { return (int32_t)rd(NAV_BASE, NAV_RAW0 + 4 * i); }
static inline void set_off(u32 reg, int32_t v) { wr(NAV_BASE, reg, (u32)v & 0xFFFFu); }

// Espera a una trama nueva (SAMPLE_COUNT cambia). Devuelve 0 si hay timeout.
static int wait_new_sample(u32 *last) {
    for (int i = 0; i < 200; i++) {
        u32 c = rd(NAV_BASE, NAV_SAMPLE_CNT);
        if (c != *last) { *last = c; return 1; }
        usleep(500);
    }
    return 0;
}

static void mad_set_beta(float b) { wr(MAD_BASE, MAD_BETA, f2u(b)); }

#define IMU_MASK ((1u << N_IMU) - 1u)
static void filter_reset(void) {
    mad_set_beta(BETA_START);
    wr(MAD_BASE, MAD_RESET, IMU_MASK);
    usleep(100000);                 // varias tramas con reset de cada IMU
    wr(MAD_BASE, MAD_RESET, 0);
}

// q de la IMU i, leida de forma coherente con frame_cnt (cuenta antes y despues). Devuelve 1 si coherente.
static int read_q(int i, float q[4], u32 *fc) {
    u32 c1 = rd(MAD_BASE, MAD_FRAME_CNT + 4 * i);
    for (int k = 0; k < 4; k++) q[k] = u2f(rd(MAD_BASE, MAD_Q_OUT + 16 * i + 4 * k));
    u32 c2 = rd(MAD_BASE, MAD_FRAME_CNT + 4 * i);
    if (fc) *fc = c1;
    return c1 == c2;
}

// Offsets de calibracion por IMU (g_goff/g_moff son los de la IMU seleccionada)
static double g_goff_all[4][3];
static float  g_moff_all[4][3];
static int    g_sel = 0;
static void select_imu(int i) { g_sel = i; g_nav = NAV_BASES[i]; }
static void save_cal(void) { for (int k = 0; k < 3; k++) { g_goff_all[g_sel][k] = g_goff[k]; g_moff_all[g_sel][k] = g_moff[k]; } }
static void load_cal(void) { for (int k = 0; k < 3; k++) { g_goff[k] = g_goff_all[g_sel][k]; g_moff[k] = g_moff_all[g_sel][k]; } }

// Mediana-recortada de v[0..n-1] (ordena v). Devuelve la media entre p10 y p90 y el rango p5..p95.
static double trimmed_mean(int16_t *v, int n, int *spread) {
    for (int i = 1; i < n; i++) {                 // insercion (n ~ 400, una sola vez)
        int16_t x = v[i]; int j = i - 1;
        while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
        v[j + 1] = x;
    }
    int lo = n / 10, hi = n - n / 10;
    double s = 0;
    for (int i = lo; i < hi; i++) s += v[i];
    *spread = v[n - 1 - n / 20] - v[n / 20];
    return s / (hi - lo);
}

// Sesgo del giroscopio con la placa quieta. El resultado se escribe en NAV_OFF_G.
#define GYRO_CAL_N          400
#define GYRO_CAL_MAX_SPREAD 250
static int calibrate_gyro(void) {
    static int16_t buf[3][GYRO_CAL_N];
    for (int intento = 0; intento < 3; intento++) {
        printf("Calibrando gyro: NO MUEVAS la placa...\r\n");
        beep(1);
        led_set(1);
        set_off(NAV_OFF_G + 0, 0); set_off(NAV_OFF_G + 4, 0); set_off(NAV_OFF_G + 8, 0);
        usleep(50000);
        u32 last = rd(NAV_BASE, NAV_SAMPLE_CNT);
        for (int n = 0; n < GYRO_CAL_N; n++) {
            if (!wait_new_sample(&last)) { led_set(0); printf("  sin tramas: no se calibra\r\n"); return -1; }
            for (int k = 0; k < 3; k++) buf[k][n] = (int16_t)raw_i(k);
        }
        led_set(0);
        int32_t off[3]; int ok = 1, sp[3];
        for (int k = 0; k < 3; k++) {
            off[k] = (int32_t)lrint(trimmed_mean(buf[k], GYRO_CAL_N, &sp[k]));
            if (sp[k] > GYRO_CAL_MAX_SPREAD) ok = 0;
        }
        printf("  dispersion p5-p95: (%d, %d, %d)\r\n", sp[0], sp[1], sp[2]);
        if (!ok) { printf("  la placa se ha movido, repito\r\n"); continue; }
        for (int k = 0; k < 3; k++) {
            set_off(NAV_OFF_G + 4 * k, off[k]);
            g_goff[k] = off[k];
        }
        printf("  gyro offset = (%ld, %ld, %ld) cuentas\r\n", (long)off[0], (long)off[1], (long)off[2]);
        beep(2);                    // fin: ya puedes mover la placa
        return 0;
    }
    led_set(0);
    printf("  no se pudo calibrar el gyro (placa en movimiento)\r\n");
    return -1;
}

// Resuelve A x = b (4x4) por Gauss con pivote parcial. Devuelve 0 si es singular.
static int solve4(double A[4][5], double x[4]) {
    for (int c = 0; c < 4; c++) {
        int p = c;
        for (int r = c + 1; r < 4; r++) if (fabs(A[r][c]) > fabs(A[p][c])) p = r;
        if (fabs(A[p][c]) < 1e-9) return 0;
        if (p != c) for (int k = 0; k < 5; k++) { double t = A[c][k]; A[c][k] = A[p][k]; A[p][k] = t; }
        for (int r = c + 1; r < 4; r++) {
            double f = A[r][c] / A[c][c];
            for (int k = c; k < 5; k++) A[r][k] -= f * A[c][k];
        }
    }
    for (int r = 3; r >= 0; r--) {
        double v = A[r][4];
        for (int k = r + 1; k < 4; k++) v -= A[r][k] * x[k];
        x[r] = v / A[r][r];
    }
    return 1;
}

// ---- Datos de la calibracion (crudos del sensor, ejes fisicos) -----------------------
typedef struct { int16_t g[3], a[3], m[3]; } cal_frame_t;
static cal_frame_t g_cal[MAG_CAL_MAX_FRAMES];

static const int PERM_TAB[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};

static double dot3(const double *a, const double *b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

// Ajuste de esfera por minimos cuadrados con un subconjunto (use[n] != 0). Devuelve 0 si falla.
static int sphere_fit(u32 frames, const unsigned char *use, double c[3], double *R) {
    double N[4][5];
    memset(N, 0, sizeof(N));
    for (u32 n = 0; n < frames; n++) {
        if (!use[n]) continue;
        double m0 = g_cal[n].m[0], m1 = g_cal[n].m[1], m2 = g_cal[n].m[2];
        double r[4] = {2.0 * m0, 2.0 * m1, 2.0 * m2, 1.0};
        double y = m0*m0 + m1*m1 + m2*m2;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) N[i][j] += r[i] * r[j];
            N[i][4] += r[i] * y;
        }
    }
    double sol[4];
    if (!solve4(N, sol)) return 0;
    double R2 = sol[3] + sol[0]*sol[0] + sol[1]*sol[1] + sol[2]*sol[2];
    if (R2 <= 0) return 0;
    c[0] = sol[0]; c[1] = sol[1]; c[2] = sol[2]; *R = sqrt(R2);
    return 1;
}

// Con el centro ya conocido, busca la permutacion/signos del magnetometro que hacen
// constante el angulo con la vertical (a . m / |m|, dip), y los signos del giroscopio que
// hacen coherente el giro del campo horizontal con la velocidad angular. Devuelve el
// valor de AXIS_CFG elegido (o -1 si no es concluyente).
static int detect_axes(u32 frames, const double c[3]) {
    const double K = (FS_G_SELECTED == 0 ? 8.75 : (FS_G_SELECTED == 1 ? 17.5 : 70.0)) * 1e-3 * 3.14159265358979 / 180.0;
    // ---- 1) magnetometro: 6 permutaciones x 8 signos ----
    double best_std = 1e9, second_std = 1e9, best_mean = 0;
    int best_code = -1, best_sg = 0;
    for (int code = 0; code < 6; code++) {
        for (int sg = 0; sg < 8; sg++) {
            double s1 = 0, s2 = 0; int cnt = 0;
            for (u32 n = 0; n < frames; n++) {
                double a[3] = {g_cal[n].a[0], g_cal[n].a[1], g_cal[n].a[2]};
                double an = sqrt(dot3(a, a));
                if (an < 0.9 * 16384.0 || an > 1.1 * 16384.0) continue;   // solo casi estaticas
                double v[3];
                for (int i = 0; i < 3; i++) {
                    v[i] = (double)g_cal[n].m[PERM_TAB[code][i]] - c[PERM_TAB[code][i]];
                    if (sg & (1 << i)) v[i] = -v[i];
                }
                double vn = sqrt(dot3(v, v));
                if (vn < 1.0) continue;
                double d = dot3(a, v) / (an * vn);
                s1 += d; s2 += d * d; cnt++;
            }
            if (cnt < 200) continue;
            double mean = s1 / cnt, sd = sqrt(fmax(0.0, s2 / cnt - mean * mean));
            if (mean >= 0) continue;                 // hemisferio norte: campo hacia abajo, a hacia arriba
            if (sd < best_std) {
                second_std = best_std;
                best_std = sd; best_mean = mean; best_code = code; best_sg = sg;
            } else if (sd < second_std) second_std = sd;
        }
    }
    if (best_code < 0) { printf("  ejes: datos insuficientes\r\n"); return -1; }
    printf("  ejes mag: mejor perm=%d signos=0x%x  dip medio=", best_code, best_sg);
    print_f4((float)best_mean); printf(" desv="); print_f4((float)best_std);
    printf(" (2o mejor desv="); print_f4((float)second_std); printf(")\r\n");
    int mag_ok = (best_std < 0.25 && second_std > 1.5 * best_std);

    // ---- 2) giroscopio: 8 combinaciones de signo ----
    double Sxy[8], Sxx[8], Syy[8];
    for (int i = 0; i < 8; i++) Sxy[i] = Sxx[i] = Syy[i] = 0;
    const int S = 24;                    // ventana de 0,2 s: con giros lentos la de 50 ms queda ahogada por el ruido
    const int STEP = 8;
    const double dt = 1.0 / 119.0;
    for (u32 n = 0; n + S < frames; n += STEP) {
        double h[2][3], an2[3] = {0, 0, 0};
        int bad = 0;
        double w[3] = {0, 0, 0};
        for (int e = 0; e < 2; e++) {
            u32 f = (e == 0) ? n : n + S;
            double a[3] = {g_cal[f].a[0], g_cal[f].a[1], g_cal[f].a[2]};
            double an = sqrt(dot3(a, a));
            if (an < 0.85 * 16384.0 || an > 1.15 * 16384.0) { bad = 1; break; }
            for (int i = 0; i < 3; i++) an2[i] += a[i] / an;
            double v[3];
            for (int i = 0; i < 3; i++) {
                v[i] = (double)g_cal[f].m[PERM_TAB[best_code][i]] - c[PERM_TAB[best_code][i]];
                if (best_sg & (1 << i)) v[i] = -v[i];
            }
            double up[3] = {a[0]/an, a[1]/an, a[2]/an};
            double vu = dot3(v, up);
            for (int i = 0; i < 3; i++) h[e][i] = v[i] - vu * up[i];
        }
        if (bad) continue;
        double nn = sqrt(dot3(an2, an2));
        if (nn < 1e-6) continue;
        for (int i = 0; i < 3; i++) an2[i] /= nn;
        double h1n = sqrt(dot3(h[0], h[0])), h2n = sqrt(dot3(h[1], h[1]));
        if (h1n < 500.0 || h2n < 500.0) continue;       // campo casi vertical: sin rumbo
        double cr[3] = {h[0][1]*h[1][2] - h[0][2]*h[1][1],
                        h[0][2]*h[1][0] - h[0][0]*h[1][2],
                        h[0][0]*h[1][1] - h[0][1]*h[1][0]};
        double dpsi = atan2(dot3(an2, cr), dot3(h[0], h[1]));
        for (int j = 0; j < S; j++)
            for (int i = 0; i < 3; i++) w[i] += ((double)g_cal[n + j].g[i] - g_goff[i]) * K * dt;
        for (int s = 0; s < 8; s++) {
            double x = 0;
            for (int i = 0; i < 3; i++) x -= ((s & (1 << i)) ? -w[i] : w[i]) * an2[i];
            Sxy[s] += x * dpsi; Sxx[s] += x * x; Syy[s] += dpsi * dpsi;
        }
    }
    double bc = -2, sc = -2; int bs = -1;
    for (int s = 0; s < 8; s++) {
        double den = sqrt(Sxx[s] * Syy[s]);
        double cc = den > 0 ? Sxy[s] / den : 0;
        if (cc > bc) { sc = bc; bc = cc; bs = s; } else if (cc > sc) sc = cc;
    }
    double slope = (bs >= 0 && Sxx[bs] > 0) ? Sxy[bs] / Sxx[bs] : 0;
    printf("  ejes gyro: mejor signos=0x%x corr=", bs);
    print_f4((float)bc); printf(" (2o="); print_f4((float)sc); printf(") pendiente="); print_f4((float)slope);
    printf(" (ideal 1)\r\n");
    int gyro_ok = (bs >= 0 && bc > 0.4 && bc - sc > 0.15);
    if (!mag_ok || !gyro_ok) {
        printf("  ejes: resultado NO concluyente (%s%s): se mantiene AXIS_CFG actual. Repite con mas giros.\r\n",
               mag_ok ? "" : "mag ", gyro_ok ? "" : "gyro");
        return -1;
    }
    return (best_sg & 7) | ((bs & 7) << 3) | ((best_code & 7) << 6);
}

// Hard-iron: ajuste de esfera por minimos cuadrados |m - c|^2 = R^2 mientras se gira
// la placa; despues un 2o ajuste sin los valores atipicos (>2 sigma). Luego se deduce
// la orientacion de ejes (detect_axes). Es mucho mas robusto que (max+min)/2.
static int calibrate_mag(int seconds) {
    u32 frames = (u32)seconds * 119u;
    if (frames > MAG_CAL_MAX_FRAMES) frames = MAG_CAL_MAX_FRAMES;
    printf("Calibrando magnetometro %lu s: gira la placa despacio por TODAS las orientaciones\r\n",
           (unsigned long)(frames / 119u));
    printf("  (cada eje hacia arriba y hacia abajo, y giros en 8). LEJOS de moviles, auriculares, altavoces, portatil.\r\n");
    set_off(NAV_OFF_M + 0, 0); set_off(NAV_OFF_M + 4, 0); set_off(NAV_OFF_M + 8, 0);
    g_moff[0] = g_moff[1] = g_moff[2] = 0;
    usleep(50000);
    beep(1);
    led_set(1);                     // LED fijo = gira la placa
    int32_t mn[3] = { 100000,  100000,  100000};
    int32_t mx[3] = {-100000, -100000, -100000};
    u32 last = rd(NAV_BASE, NAV_SAMPLE_CNT);
    for (u32 n = 0; n < frames; n++) {
        if (!wait_new_sample(&last)) { led_set(0); printf("  sin tramas: no se calibra\r\n"); return -1; }
        for (int k = 0; k < 3; k++) {
            g_cal[n].g[k] = (int16_t)raw_i(k);
            g_cal[n].a[k] = (int16_t)raw_i(3 + k);
            int32_t v = raw_i(6 + k);
            g_cal[n].m[k] = (int16_t)v;
            if (v < mn[k]) mn[k] = v;
            if (v > mx[k]) mx[k] = v;
        }
        if (n % 119u == 118u) printf("  %lu s restantes\r\n", (unsigned long)((frames - n - 1) / 119u));
    }
    led_set(0);
    beep(3);                        // fin: para de girar
    printf("  rango mag: x[%ld,%ld] y[%ld,%ld] z[%ld,%ld]\r\n",
           (long)mn[0], (long)mx[0], (long)mn[1], (long)mx[1], (long)mn[2], (long)mx[2]);

    static unsigned char use[MAG_CAL_MAX_FRAMES];
    static unsigned char spike[MAG_CAL_MAX_FRAMES];
    double c[3], R = 0, resid = 1;
    memset(use, 1, sizeof(use));
    memset(spike, 0, sizeof(spike));
    // Picos aislados (glitch SPI / cable): una muestra que se aparta >1500 cuentas de
    // sus dos vecinas, que a su vez coinciden entre si.
    u32 nspk = 0;
    for (u32 n = 1; n + 1 < frames; n++)
        for (int k = 0; k < 3; k++) {
            int dp = g_cal[n].m[k] - g_cal[n - 1].m[k], dn = g_cal[n].m[k] - g_cal[n + 1].m[k];
            int dv = g_cal[n - 1].m[k] - g_cal[n + 1].m[k];
            if (abs(dp) > 1500 && abs(dn) > 1500 && abs(dv) < 1500 && (dp > 0) == (dn > 0)) { spike[n] = 1; }
        }
    for (u32 n = 0; n < frames; n++) nspk += spike[n];
    printf("  picos aislados descartados: %lu\r\n", (unsigned long)nspk);
    for (int k = 0; k < 3; k++) {       // rango sin picos, para la comprobacion de cobertura
        mn[k] = 100000; mx[k] = -100000;
        for (u32 n = 0; n < frames; n++) if (!spike[n]) {
            if (g_cal[n].m[k] < mn[k]) mn[k] = g_cal[n].m[k];
            if (g_cal[n].m[k] > mx[k]) mx[k] = g_cal[n].m[k];
        }
    }
    for (u32 n = 0; n < frames; n++) use[n] = !spike[n];
    for (int pass = 0; pass < 2; pass++) {
        if (!sphere_fit(frames, use, c, &R)) { printf("  ajuste singular: gira mas la placa. Offset = 0\r\n"); return -1; }
        double se = 0; u32 cnt = 0;
        static float err[MAG_CAL_MAX_FRAMES];
        for (u32 n = 0; n < frames; n++) {
            double dx = g_cal[n].m[0] - c[0], dy = g_cal[n].m[1] - c[1], dz = g_cal[n].m[2] - c[2];
            err[n] = (float)(sqrt(dx*dx + dy*dy + dz*dz) - R);
            if (use[n]) { se += (double)err[n] * err[n]; cnt++; }
        }
        double rms = sqrt(se / (cnt ? cnt : 1));
        resid = rms / R;
        if (pass == 0) {            // recorta atipicos y reajusta
            u32 kept = 0;
            for (u32 n = 0; n < frames; n++) { use[n] = (!spike[n] && fabs(err[n]) < 2.0 * rms); kept += use[n]; }
            printf("  1er ajuste: radio=%ld residuo=%ld%%  (atipicos descartados: %lu de %lu)\r\n",
                   (long)R, (long)(resid * 100.0), (unsigned long)(frames - kept), (unsigned long)frames);
            if (kept < frames / 2) { printf("  demasiados atipicos: perturbacion magnetica. Offset = 0\r\n"); return -1; }
        }
    }
    int cover_ok = 1;
    for (int k = 0; k < 3; k++) if ((double)(mx[k] - mn[k]) < MAG_CAL_MIN_COVER * R) cover_ok = 0;
    printf("  ajuste: centro=(%ld, %ld, %ld) radio=%ld  residuo=", (long)c[0], (long)c[1], (long)c[2], (long)R);
    print_f1((float)(resid * 100.0)); printf("%%\r\n");
    if (!cover_ok || resid > MAG_CAL_MAX_RESID) {
        printf("  CALIBRACION DESCARTADA (%s). Offset del mag = 0. Repite con 'm'.\r\n",
               !cover_ok ? "cobertura insuficiente: algun eje no ha recorrido su rango" : "residuo alto: perturbacion magnetica cerca");
        return -1;
    }
    for (int k = 0; k < 3; k++) {
        g_moff[k] = (float)c[k];
        set_off(NAV_OFF_M + 4 * k, (int32_t)c[k]);
    }
    printf("  mag offset = (%ld, %ld, %ld)  -> copialos a MAG_PRESET_X/Y/Z para fijarlos\r\n",
           (long)c[0], (long)c[1], (long)c[2]);

    if (AXIS_CFG_PRESET < 0) {
        int cfg = detect_axes(frames, c);
        if (cfg >= 0) {
            wr(NAV_BASE, NAV_AXIS_CFG, (u32)cfg);
            printf("  AXIS_CFG = 0x%03x aplicado  -> copialo a AXIS_CFG_PRESET para fijarlo\r\n", cfg);
        }
    }
    return 0;
}

static void print_status(u32 st) {
    printf("NAV STATUS=0x%08lx  busy=%d cfg_done=%d ag=%d mag=%d err=%d "
           "[whoami_ag=%d whoami_mag=%d timeout=%d overrun=%d]\r\n",
           (unsigned long)st,
           !!(st & ST_BUSY), !!(st & ST_CONFIG_DONE),
           !!(st & ST_AG_DETECTED), !!(st & ST_MAG_DETECTED), !!(st & ST_ERROR),
           !!(st & ST_ERR_AG_WHOAMI), !!(st & ST_ERR_MAG_WHOAMI),
           !!(st & ST_ERR_TIMEOUT), !!(st & ST_ERR_OVERRUN));
    u32 w = rd(NAV_BASE, NAV_DBG_WHOAMI);
    printf("WHO_AM_I leido: A/G=0x%02lx (esperado 0x68)  Mag=0x%02lx (esperado 0x3D)\r\n",
           (unsigned long)(w & 0xFF), (unsigned long)((w >> 8) & 0xFF));
}


// ---------------------------------------------------------------------------
// Registro de datos de las N IMUs
// ---------------------------------------------------------------------------
// Una fila por trama de cualquier IMU (cada una muestrea por su cuenta a ~118 Hz).
// Se vuelca al final como CSV por la UART (~125 s para 4 IMUs y 30 s).
#define LOG_SECONDS 30
#define LOG_MAX     (4 * LOG_SECONDS * 120 + 64)
typedef struct {
    u32     cnt;        // SAMPLE_COUNT de nav_spi_ctrl de esa IMU
    u32     fc;         // frame_cnt del nucleo para esa IMU (tramas ya procesadas)
    u32     t_us;       // microsegundos desde la primera fila
    int32_t raw[9];     // gx gy gz ax ay az mx my mz (crudos, antes de offset/ejes)
    u32     q[4];       // q de esa IMU (bits de float)
    u8      imu;
    u8      ok;         // 1 si ni SAMPLE_COUNT ni frame_cnt cambiaron durante la lectura
} logrec_t;
static logrec_t g_log[LOG_MAX];
static int      g_log_n = 0;
static int      g_log_beta_row = -1;       // fila en la que se cambio beta de arranque -> regimen
static u32      g_log_cnt0[4];

static void log_dump(void) {
    printf("\r\n#HDR,fs_g=%u,dt_hz=119,beta_start=1.0,beta_run=0.1,beta_start_ms=%d,nimu=%d,n=%d,"
           "beta_row=%d,gt_hz=%lu\r\n",
           (unsigned)FS_G_SELECTED, (int)BETA_START_MS, N_IMU, g_log_n, g_log_beta_row, (unsigned long)g_gt_hz);
    for (int i = 0; i < N_IMU; i++) {
        const u32 b = NAV_BASES[i];
        printf("#IMU,%d,axis_cfg=0x%03lx,goff=%ld/%ld/%ld,moff=%ld/%ld/%ld,cnt0=%lu\r\n", i,
               (unsigned long)(rd(b, NAV_AXIS_CFG) & 0xFFFu),
               (long)(int32_t)rd(b, NAV_OFF_G + 0), (long)(int32_t)rd(b, NAV_OFF_G + 4), (long)(int32_t)rd(b, NAV_OFF_G + 8),
               (long)(int32_t)rd(b, NAV_OFF_M + 0), (long)(int32_t)rd(b, NAV_OFF_M + 4), (long)(int32_t)rd(b, NAV_OFF_M + 8),
               (unsigned long)g_log_cnt0[i]);
    }
    printf("#COLS,imu,cnt,fc,t_us,gx,gy,gz,ax,ay,az,mx,my,mz,q0,q1,q2,q3,ok  (q en hex de float32)\r\n");
    for (int i = 0; i < g_log_n; i++) {
        const logrec_t *r = &g_log[i];
        printf("%u,%lu,%lu,%lu", (unsigned)r->imu, (unsigned long)r->cnt, (unsigned long)r->fc, (unsigned long)r->t_us);
        for (int k = 0; k < 9; k++) printf(",%ld", (long)r->raw[k]);
        for (int k = 0; k < 4; k++) printf(",%08lx", (unsigned long)r->q[k]);
        printf(",%u\r\n", (unsigned)r->ok);
    }
    printf("#END\r\n");
}

// Avisos por terminal durante la captura (a partir del giroscopio de la IMU 0):
//   MOV = empieza un movimiento   QUIETO = vuelve el reposo   LISTO = pausa cumplida, puedes mover
#define MOVE_DPS_ON   10.0f
#define MOVE_DPS_OFF   6.0f
#define HOLD_FIRST_MS 8000
#define HOLD_POSE_MS  3000
static void say_t(const char *msg, u32 t_ms) {
    printf("t=%lu.%lu %s\r\n", (unsigned long)(t_ms / 1000u), (unsigned long)((t_ms / 100u) % 10u), msg);
}

// do_reset=1: reinicia los filtros y captura desde la semilla.
static void log_capture(int do_reset, int *beta_low) {
    if (do_reset) { filter_reset(); *beta_low = 0; }
    printf("Capturando %d s de %d IMUs... (avisos: MOV / QUIETO / LISTO segun la IMU 0)\r\n", LOG_SECONDS, N_IMU);
    u32 last[4];
    for (int i = 0; i < N_IMU; i++) { last[i] = rd(NAV_BASES[i], NAV_SAMPLE_CNT); g_log_cnt0[i] = last[i]; }
    const float gs = (FS_G_SELECTED == 0 ? 8.75e-3f : (FS_G_SELECTED == 1 ? 17.5e-3f : 70.0e-3f));
    float go[3];
    for (int k = 0; k < 3; k++) go[k] = (float)(int16_t)rd(NAV_BASES[0], NAV_OFF_G + 4 * k);
    uint64_t t0 = 0;
    float ws = 0.0f; int moving = 0, ready = 0, first = 1; u32 still_since = 0;
    int n = 0; g_log_beta_row = -1;
    u32 idle = 0;
    while (n < LOG_MAX) {
        int got = 0;
        for (int i = 0; i < N_IMU; i++) {
            const u32 b = NAV_BASES[i];
            if (rd(b, NAV_SAMPLE_CNT) == last[i]) continue;
            usleep(500);                           // da tiempo al mux y al nucleo
            logrec_t *r = &g_log[n];
            u32 c1 = rd(b, NAV_SAMPLE_CNT);
            for (int k = 0; k < 9; k++) r->raw[k] = (int32_t)rd(b, NAV_RAW0 + 4 * k);
            float qq[4]; u32 fc;
            int coh = read_q(i, qq, &fc);
            for (int k = 0; k < 4; k++) r->q[k] = f2u(qq[k]);
            u32 c2 = rd(b, NAV_SAMPLE_CNT);
            r->cnt = c1; r->fc = fc; r->imu = (u8)i;
            r->ok = (c1 == c2 && coh) ? 1u : 0u;
            last[i] = c1;                          // si llegaron 2 tramas seguidas se vera como hueco en cnt
            uint64_t tn = gt_now();
            if (n == 0) t0 = tn;
            r->t_us = (u32)(((tn - t0) * 1000000ull) / g_gt_hz);
            u32 t_ms = r->t_us / 1000u;
            if (i == 0) {                          // avisos: solo con la IMU 0
                float gx = ((float)r->raw[0] - go[0]) * gs, gy = ((float)r->raw[1] - go[1]) * gs, gz = ((float)r->raw[2] - go[2]) * gs;
                ws += 0.1f * (sqrtf(gx * gx + gy * gy + gz * gz) - ws);
                if (!moving && ws > MOVE_DPS_ON) { moving = 1; ready = 0; say_t("MOV", t_ms); }
                else if (moving && ws < MOVE_DPS_OFF) { moving = 0; still_since = t_ms; say_t("QUIETO", t_ms); }
                if (!moving && !ready && (t_ms - still_since) >= (first ? HOLD_FIRST_MS : HOLD_POSE_MS)) {
                    ready = 1; first = 0; say_t("LISTO - mueve", t_ms);
                }
            }
            n++; got = 1; idle = 0;
            if (!*beta_low && t_ms >= BETA_START_MS) { mad_set_beta(BETA_RUN); *beta_low = 1; g_log_beta_row = n; }
            if (r->t_us >= (u32)LOG_SECONDS * 1000000u) { g_log_n = n; goto done; }
            if (n >= LOG_MAX) break;
        }
        if (!got) {
            usleep(100);
            if (++idle > 3000) { printf("sin tramas: captura cortada\r\n"); break; }   // ~0,3 s sin datos
        }
    }
    g_log_n = n;
done:
    printf("Captura terminada: %d filas, %lu ms. Volcando CSV...\r\n", g_log_n,
           g_log_n ? (unsigned long)(g_log[g_log_n - 1].t_us / 1000u) : 0ul);
    log_dump();
}

static void show_imu(int i) {
    float q[4]; u32 fc;
    read_q(i, q, &fc);
    float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    const float R2D = 57.29578f;
    float roll  = atan2f(2.0f * (q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2])) * R2D;
    float sp    = 2.0f * (q[0]*q[2] - q[3]*q[1]);
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    float pitch = asinf(sp) * R2D;
    float yaw   = atan2f(2.0f * (q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3])) * R2D;
    printf("IMU%d%s q=[", i, i == g_sel ? "*" : " ");
    for (int k = 0; k < 4; k++) { print_f4(q[k]); printf(k < 3 ? " " : ""); }
    printf("] |q|^2="); print_f4(n2);
    printf(" r="); print_f1(roll); printf(" p="); print_f1(pitch); printf(" y="); print_f1(yaw);
    printf(" fc=%lu nav=%lu err=%d\r\n", (unsigned long)fc, (unsigned long)rd(NAV_BASES[i], NAV_SAMPLE_CNT),
           !!(rd(NAV_BASES[i], NAV_STATUS) & ST_ERROR));
}

static const int32_t MAG_PRESET[4][3] = { {-2107, 2132, -1331}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };

int main(void) {
    printf("\r\n=== Madgwick en FPGA: %d IMUs, un solo nucleo ===\r\n", N_IMU);
    gt_init();

    // Orden: primero los bloques HLS (para que esten listos cuando llegue la primera
    // trama) y al final se habilitan los nav_spi_ctrl.
    // 1) Cada nav_spi_ctrl: ODR, rango y ejes (aun sin habilitar)
    for (int i = 0; i < N_IMU; i++) {
        const u32 b = NAV_BASES[i];
        wr(b, NAV_ODR_CFG, (7u << 6) | (3u << 3) | 3u);
        wr(b, NAV_RANGE_CFG, (0u << 2) | (FS_G_SELECTED & 0x3u));
        wr(b, NAV_AXIS_CFG, (AXIS_CFG_PRESET >= 0) ? (u32)AXIS_CFG_PRESET : (u32)AXIS_CFG_DEFAULT);
    }
    // 2) r2f_mux: mismo rango de giroscopio; arranca en modo libre
    wr(R2F_BASE, R2F_FS_G_REG, FS_G_SELECTED & 0x3u);
    wr(R2F_BASE, HLS_AP_CTRL, AP_START | AP_AUTORESTART);
    // 3) Nucleo multi: beta, dt, reset de todas las IMUs y modo libre
    wr(MAD_BASE, MAD_BETA, f2u(BETA_START));
    wr(MAD_BASE, MAD_DT, f2u(DT_SECONDS));
    wr(MAD_BASE, MAD_RESET, IMU_MASK);
    wr(MAD_BASE, HLS_AP_CTRL, AP_START | AP_AUTORESTART);

    // 4) Habilita los controladores SPI y comprueba cada sensor
    for (int i = 0; i < N_IMU; i++) wr(NAV_BASES[i], NAV_CTRL, NAV_CTRL_ENABLE);
    int bad = 0;
    for (int i = 0; i < N_IMU; i++) {
        select_imu(i);
        for (int t = 0; t < 500; t++) {
            if (rd(NAV_BASE, NAV_STATUS) & (ST_CONFIG_DONE | ST_ERROR)) break;
            usleep(1000);
        }
        u32 st = rd(NAV_BASE, NAV_STATUS);
        printf("IMU%d: ", i);
        print_status(st);
        if (st & ST_ERROR) { printf("  -> ERROR en IMU%d: revisa el cableado de ese Pmod NAV y el XDC\r\n", i); bad = 1; }
    }
    if (bad) return -1;

    // 5) Comprueba que el nucleo recibe tramas de cada IMU (frame_cnt > 0)
    for (int t = 0; t < 1000; t++) {
        int all = 1;
        for (int i = 0; i < N_IMU; i++) if (rd(MAD_BASE, MAD_FRAME_CNT + 4 * i) == 0) all = 0;
        if (all) break;
        usleep(1000);
    }
    for (int i = 0; i < N_IMU; i++) {
        u32 fc = rd(MAD_BASE, MAD_FRAME_CNT + 4 * i);
        printf("IMU%d: frame_cnt=%lu%s\r\n", i, (unsigned long)fc, fc ? "" : "   <-- SIN TRAMAS (cable, XDC u offsets de mad_multi)");
        if (!fc) bad = 1;
    }
    if (bad) return -2;
    wr(MAD_BASE, MAD_RESET, 0);

    // 6) Calibracion: gyro de cada IMU (placas quietas) y mag (preset o, si MAG_CAL_SECONDS>0, giro de cada una)
    for (int i = 0; i < N_IMU; i++) {
        select_imu(i);
        printf("--- IMU%d ---\r\n", i);
        calibrate_gyro();
        save_cal();
    }
    for (int i = 0; i < N_IMU; i++) {
        select_imu(i);
        if (MAG_CAL_SECONDS > 0) {
            printf("IMU%d: tienes 3 s para coger la placa (pitido = empieza)...\r\n", i);
            usleep(3000000);
            calibrate_mag(MAG_CAL_SECONDS);
        } else {
            set_off(NAV_OFF_M + 0, MAG_PRESET[i][0]);
            set_off(NAV_OFF_M + 4, MAG_PRESET[i][1]);
            set_off(NAV_OFF_M + 8, MAG_PRESET[i][2]);
            for (int k = 0; k < 3; k++) g_moff[k] = (float)MAG_PRESET[i][k];
        }
        save_cal();
    }
    select_imu(0); load_cal();
    filter_reset();
    u32 t_start_beta = 0;
    int beta_low = 0;

    printf("Teclas: 0..3 selecciona IMU  g=gyro(todas, quietas)  m=mag(seleccionada)  c=borrar offsets  r=reset  a/b=ejes 0x018/0x039  l/L=captura CSV %d s (L reinicia antes)  d=volcar de nuevo\r\n", LOG_SECONDS);
    for (;;) {
#ifdef CON_BASE
        while (XUartPs_IsReceiveData(CON_BASE)) {
            char ch = (char)XUartPs_ReadReg(CON_BASE, XUARTPS_FIFO_OFFSET);
            if (ch >= '0' && ch < '0' + N_IMU) { select_imu(ch - '0'); load_cal(); printf("IMU seleccionada: %d\r\n", g_sel); }
            else if (ch == 'g') {
                for (int i = 0; i < N_IMU; i++) { select_imu(i); calibrate_gyro(); save_cal(); }
                select_imu(0); load_cal(); filter_reset(); t_start_beta = 0; beta_low = 0;
            }
            else if (ch == 'm') { calibrate_mag(MAG_CAL_SECONDS > 0 ? MAG_CAL_SECONDS : 30); save_cal(); filter_reset(); t_start_beta = 0; beta_low = 0; }
            else if (ch == 'c') {
                for (int i = 0; i < N_IMU; i++) {
                    for (int k = 0; k < 3; k++) { wr(NAV_BASES[i], NAV_OFF_G + 4*k, 0); wr(NAV_BASES[i], NAV_OFF_M + 4*k, 0); g_goff_all[i][k] = 0; g_moff_all[i][k] = 0; }
                }
                load_cal(); printf("offsets a 0 (todas las IMUs)\r\n"); filter_reset(); t_start_beta = 0; beta_low = 0;
            }
            else if (ch == 'r') { filter_reset(); t_start_beta = 0; beta_low = 0; }
            else if (ch == 'l') { log_capture(0, &beta_low); }
            else if (ch == 'L') { log_capture(1, &beta_low); t_start_beta = 0; }
            else if (ch == 'd') { log_dump(); }
            else if (ch == 'a' || ch == 'b') {
                u32 cfg = (ch == 'a') ? 0x018u : 0x039u;
                for (int i = 0; i < N_IMU; i++) wr(NAV_BASES[i], NAV_AXIS_CFG, cfg);
                printf("AXIS_CFG = 0x%03lx (todas)\r\n", (unsigned long)cfg);
                filter_reset(); t_start_beta = 0; beta_low = 0;
            }
        }
#endif
        if (!beta_low && ++t_start_beta * 200u >= BETA_START_MS) { mad_set_beta(BETA_RUN); beta_low = 1; }
        for (int i = 0; i < N_IMU; i++) show_imu(i);
        printf("\r\n");
        usleep(200000);
    }
    return 0;
}
