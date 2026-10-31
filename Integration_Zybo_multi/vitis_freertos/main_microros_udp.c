/* main_microros_udp.c -- PASO 4b de la guia micro-ROS: FreeRTOS + PL + micro-ROS por UDP/Ethernet (lwIP, modo socket).
 *
 * Igual que main_microros_pl.c, pero el transporte es UDP sobre el Ethernet del PS (ENET0) y no la UART del FTDI:
 *   - la UART queda libre: se imprime el estado por la consola (printf) y el JTAG sigue disponible;
 *   - la red se inicia como en la plantilla "lwIP Echo Server" de Vitis (xemac_add + xemacif_input_thread), IP estatica;
 *   - el transporte de micro-ROS son sockets UDP de lwIP (rmw_uros_set_custom_transport con framing = false), sin cambiar libmicroros.a.
 *
 * Topics: /madgwick/q (geometry_msgs/QuaternionStamped, frame_id imu0..imu3) y /madgwick/cpu_free (std_msgs/Float32).
 * Agente (WSL con red "mirrored"): ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 *
 * BSP: use_idle_hook = true, total_heap_size >= 1 MB, lwip213 (api_mode SOCKET_API, como la plantilla), stdout = UART del FTDI.
 * Hay que dejar los mismos includes de micro-ROS y -lmicroros que en main_microros_pl.c.
 * Placas QUIETAS al arrancar (sesgo del giroscopio). SIN VERIFICAR: firmas de los callbacks, sockets con MSG_DONTWAIT, orden del cuaternion. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "xil_io.h"
#include "xil_types.h"
#include "xparameters.h"
#include "netif/xadapter.h"
#include "platform_config.h"
#include "xil_printf.h"
#include "lwip/init.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "sleep.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>
#include <geometry_msgs/msg/quaternion_stamped.h>
#include <std_msgs/msg/float32.h>

#ifndef N_IMU
#define N_IMU 4
#endif
#ifndef PUB_MS
#define PUB_MS 100          /* periodo de publicacion: 10 Hz (prueba despues con 20 -> 50 Hz) */
#endif
#ifndef STAT_S
#define STAT_S 2
#endif
#ifndef LOG_ENABLE
#define LOG_ENABLE 1        /* la consola es independiente del transporte: se puede imprimir */
#endif
#ifndef BOARD_IP
#define BOARD_IP  192, 168, 50, 2
#define BOARD_NM  255, 255, 255, 0
#define BOARD_GW  192, 168, 50, 1
#endif
#ifndef AGENT_IP
#define AGENT_IP   "192.168.50.1"    /* el PC (adaptador Ethernet con IP fija) */
#endif
#ifndef AGENT_PORT
#define AGENT_PORT 8888
#endif
#define NET_STACK 1024
#define IP4_FROM(p, ...) IP4_ADDR(p, __VA_ARGS__)   /* expande BOARD_IP antes de pasar los 4 octetos */
#ifndef Q_W_FIRST
#define Q_W_FIRST 1         /* 1: q[0..3] = (w,x,y,z) (Madgwick estandar); 0: (x,y,z,w) */
#endif

#if LOG_ENABLE
  #define LOG(...) printf(__VA_ARGS__)
#else
  #define LOG(...) ((void)0)
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

/* ---------------- PL (igual que main_freertos_pl.c) ---------------- */
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
#define FS_G_SELECTED 1u
#define AXIS_CFG_PRESET 0x039u
#define BETA_START 1.0f
#define BETA_RUN 0.1f
#define BETA_START_MS 3000
static const int32_t MAG_PRESET[4][3] = {{1537,1372,-2025},{-1952,2124,-1233},{-2188,2152,-1273},{1508,1242,-1704}};

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
static const float DT_IMU[4] = { 8.280e-3f / DT_DIV, 8.307e-3f / DT_DIV, 8.480e-3f / DT_DIV, 8.057e-3f / DT_DIV };
static inline void wr(uint32_t b, uint32_t o, uint32_t v) { Xil_Out32(b + o, v); }
static inline uint32_t rd(uint32_t b, uint32_t o) { return Xil_In32(b + o); }
static inline uint32_t f2u(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
static inline float u2f(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }

static int read_q(int i, float q[4], uint32_t *fc) {
    uint32_t c1 = rd(MAD_BASE, MAD_FRAME_CNT + 4u * (uint32_t)i);
    for (int k = 0; k < 4; k++) q[k] = u2f(rd(MAD_BASE, MAD_Q_OUT + 16u * (uint32_t)i + 4u * (uint32_t)k));
    uint32_t c2 = rd(MAD_BASE, MAD_FRAME_CNT + 4u * (uint32_t)i);
    if (fc) *fc = c1;
    return c1 == c2;
}
static void snap(uint32_t b, uint32_t *seq, uint32_t *tg, uint32_t *te) {
    uint32_t s1, s2;
    do { s1 = rd(b, NAV_SNAP_SEQ); *tg = rd(b, NAV_SNAP_TS_GDA); *te = rd(b, NAV_SNAP_TS_END); s2 = rd(b, NAV_SNAP_SEQ); } while (s1 != s2);
    *seq = s1;
}
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
        LOG("IMU%d: periodo medido %.3f us\r\n", i, (double)dt * 1e6);
    }
}

static volatile uint32_t g_idle;
void vApplicationIdleHook(void) { g_idle++; }
static double g_idle_ref;
static volatile uint32_t g_incoh, g_pubs, g_pub_err;

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
        for (int t = 0; t < 1000; t++) { uint32_t st = rd(NAV_BASES[i], NAV_STATUS); if (st & (ST_CONFIG_DONE | ST_ERROR)) break; usleep(1000); }
        if (rd(NAV_BASES[i], NAV_STATUS) & ST_ERROR) { LOG("IMU%d: ERROR de configuracion\r\n", i); return -1; }
    }
    for (int t = 0; t < 2000; t++) {
        int all = 1; for (int i = 0; i < N_IMU; i++) if (rd(MAD_BASE, MAD_FRAME_CNT + 4u * (uint32_t)i) == 0) all = 0;
        if (all) break;
        usleep(1000);
    }
    measure_dt(3);
    for (int i = 0; i < N_IMU; i++) {
        const uint32_t b = NAV_BASES[i]; int64_t acc[3] = {0, 0, 0}; int n = 0; uint32_t last = rd(b, NAV_SAMPLE_CNT);
        while (n < 2 * ODR_HZ) {
            uint32_t c = rd(b, NAV_SAMPLE_CNT); if (c == last) continue; last = c;
            for (int k = 0; k < 3; k++) acc[k] += (int16_t)rd(b, NAV_RAW0 + 4u * (uint32_t)k);
            n++;
        }
        for (int k = 0; k < 3; k++) { int32_t off = (int32_t)(acc[k] / n); wr(b, NAV_OFF_G + 4u * (uint32_t)k, (uint32_t)off & 0xFFFFu); }
    }
    wr(MAD_BASE, MAD_RESET, 0); wr(MAD_BASE, MAD_BETA, f2u(BETA_START));
    { const uint64_t t_b = gt_now(); while (us(gt_now() - t_b) < BETA_START_MS * 1000.0) (void)0; }
    wr(MAD_BASE, MAD_BETA, f2u(BETA_RUN));
    return 0;
}


/* ================= micro-ROS sobre UDP (lwIP) ================= */

int clock_gettime(clockid_t clk, struct timespec *tp) {      /* rcutils pide el reloj; lo servimos con el temporizador global */
    (void)clk;
    const uint64_t ns = gt_now() * 3ull;
    tp->tv_sec = (time_t)(ns / 1000000000ull);
    tp->tv_nsec = (long)(ns % 1000000000ull);
    return 0;
}

static void *uros_alloc(size_t size, void *state) { (void)state; size_t *p = (size_t *)pvPortMalloc(size + 8u); if (!p) return NULL; p[0] = size; return (uint8_t *)p + 8u; }
static void uros_free(void *ptr, void *state) { (void)state; if (ptr) vPortFree((uint8_t *)ptr - 8u); }
static void *uros_realloc(void *ptr, size_t size, void *state) {
    if (!ptr) return uros_alloc(size, state);
    const size_t old = ((size_t *)((uint8_t *)ptr - 8u))[0];
    void *n = uros_alloc(size, state);
    if (n) { memcpy(n, ptr, old < size ? old : size); uros_free(ptr, state); }
    return n;
}
static void *uros_zalloc(size_t nmemb, size_t size, void *state) { void *p = uros_alloc(nmemb * size, state); if (p) memset(p, 0, nmemb * size); return p; }

static int g_sock = -1;

static bool uros_open(struct uxrCustomTransport *t) {
    (void)t;
    g_sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0) return false;
    struct sockaddr_in a; memset(&a, 0, sizeof a);
    a.sin_family = AF_INET; a.sin_port = htons(AGENT_PORT); a.sin_addr.s_addr = inet_addr(AGENT_IP);
    if (lwip_connect(g_sock, (struct sockaddr *)&a, sizeof a) != 0) { lwip_close(g_sock); g_sock = -1; return false; }
    return true;
}
static bool uros_close(struct uxrCustomTransport *t) { (void)t; if (g_sock >= 0) { lwip_close(g_sock); g_sock = -1; } return true; }

static size_t uros_write(struct uxrCustomTransport *t, const uint8_t *buf, size_t len, uint8_t *err) {
    (void)t;
    int n = lwip_send(g_sock, buf, len, 0);
    if (n < 0) { if (err) *err = 1; return 0; }
    return (size_t)n;
}
static size_t uros_read(struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout, uint8_t *err) {
    (void)t;
    const uint64_t t0 = gt_now();
    for (;;) {
        int n = lwip_recv(g_sock, buf, len, MSG_DONTWAIT);
        if (n > 0) return (size_t)n;
        if (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN) { if (err) *err = 1; return 0; }
        if (us(gt_now() - t0) >= (double)timeout * 1000.0) return 0;
        vTaskDelay(1);
    }
}

#define RCCHECK(fn, what) do { if ((fn) != RCL_RET_OK) { LOG("micro-ROS: fallo en %s\r\n", what); vTaskDelete(NULL); } } while (0)

static void uros_task(void *arg) {
    (void)arg;
    rcutils_allocator_t al = rcutils_get_zero_initialized_allocator();
    al.allocate = uros_alloc; al.deallocate = uros_free; al.reallocate = uros_realloc; al.zero_allocate = uros_zalloc; al.state = NULL;
    if (!rcutils_set_default_allocator(&al)) vTaskDelete(NULL);

    rmw_uros_set_custom_transport(false /* paquetes UDP, sin framing */, NULL, uros_open, uros_close, uros_write, uros_read);

    LOG("Esperando al agente en %s:%d ...\r\n", AGENT_IP, AGENT_PORT);
    while (rmw_uros_ping_agent(200, 1) != RMW_RET_OK) vTaskDelay(pdMS_TO_TICKS(500));
    LOG("Agente encontrado.\r\n");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support; rcl_node_t node; rcl_publisher_t pub_q, pub_cpu;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator), "support");
    RCCHECK(rclc_node_init_default(&node, "madgwick_zynq", "", &support), "node");
    RCCHECK(rclc_publisher_init_best_effort(&pub_q, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, QuaternionStamped), "madgwick/q"), "pub_q");
    RCCHECK(rclc_publisher_init_best_effort(&pub_cpu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "madgwick/cpu_free"), "pub_cpu");
    rmw_uros_sync_session(1000);
    LOG("Publicando en /madgwick/q cada %d ms.\r\n", PUB_MS);

    static char frame[4][8];
    geometry_msgs__msg__QuaternionStamped m; memset(&m, 0, sizeof m);
    std_msgs__msg__Float32 c; c.data = 0.0f;

    TickType_t t0 = xTaskGetTickCount();
    uint32_t i0 = g_idle; uint64_t g0 = gt_now(); uint32_t n_stat = 0;
    for (;;) {
        int64_t ns = rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos() : (int64_t)(gt_now() * 3ull);
        for (int i = 0; i < N_IMU; i++) {
            float q[4]; uint32_t fc;
            if (!read_q(i, q, &fc) && !read_q(i, q, &fc)) { g_incoh++; continue; }   /* una lectura que coincide con una escritura del nucleo se repite una vez */
            snprintf(frame[i], sizeof frame[i], "imu%d", i);
            m.header.frame_id.data = frame[i]; m.header.frame_id.size = strlen(frame[i]); m.header.frame_id.capacity = sizeof frame[i];
            m.header.stamp.sec = (int32_t)(ns / 1000000000ll); m.header.stamp.nanosec = (uint32_t)(ns % 1000000000ll);
#if Q_W_FIRST
            m.quaternion.w = q[0]; m.quaternion.x = q[1]; m.quaternion.y = q[2]; m.quaternion.z = q[3];
#else
            m.quaternion.x = q[0]; m.quaternion.y = q[1]; m.quaternion.z = q[2]; m.quaternion.w = q[3];
#endif
            if (rcl_publish(&pub_q, &m, NULL) == RCL_RET_OK) g_pubs++; else g_pub_err++;
        }
        n_stat += PUB_MS;
        if (n_stat >= STAT_S * 1000u) {
            n_stat = 0;
            const uint32_t i1 = g_idle; const uint64_t g1 = gt_now();
            const double rate = (double)(uint32_t)(i1 - i0) / (((double)(g1 - g0)) / g_hz);
            i0 = i1; g0 = g1;
            c.data = (float)(100.0 * rate / g_idle_ref);
            (void)rcl_publish(&pub_cpu, &c, NULL);
            LOG("CPU libre %.1f %% | publicados %lu, errores %lu, incoherentes %lu\r\n", (double)c.data, (unsigned long)g_pubs, (unsigned long)g_pub_err, (unsigned long)g_incoh);
        }
        vTaskDelayUntil(&t0, pdMS_TO_TICKS(PUB_MS));
    }
}

/* ---- red (como la plantilla lwIP Echo Server, con IP estatica) ---- */
static struct netif g_netif;

static void network_thread(void *p) {
    (void)p;
    unsigned char mac[] = { 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };
    ip_addr_t ipaddr, netmask, gw;
    IP4_FROM(&ipaddr, BOARD_IP);
    IP4_FROM(&netmask, BOARD_NM);
    IP4_FROM(&gw, BOARD_GW);
    LOG("Red: IP %d.%d.%d.%d, agente %s:%d\r\n", ip4_addr1(&ipaddr), ip4_addr2(&ipaddr), ip4_addr3(&ipaddr), ip4_addr4(&ipaddr), AGENT_IP, AGENT_PORT);
    if (!xemac_add(&g_netif, &ipaddr, &netmask, &gw, mac, PLATFORM_EMAC_BASEADDR)) { LOG("Error al anadir la interfaz de red\r\n"); vTaskDelete(NULL); return; }
    netif_set_default(&g_netif);
    netif_set_up(&g_netif);
    sys_thread_new("xemacif_input_thread", (void (*)(void *))xemacif_input_thread, &g_netif, NET_STACK, DEFAULT_THREAD_PRIO);
    xTaskCreate(uros_task, "uros", 8192, NULL, tskIDLE_PRIORITY + 2, NULL);      /* 8192 palabras = 32 KB de pila */
    vTaskDelete(NULL);
}

static void main_thread(void *p) {
    (void)p;
    lwip_init();
    sys_thread_new("NW_THRD", network_thread, NULL, NET_STACK, DEFAULT_THREAD_PRIO);
    vTaskDelete(NULL);
}

static void init_task(void *arg) {
    (void)arg;
    gt_init();
    LOG("\r\n=== FreeRTOS + PL + micro-ROS (UDP): %d IMU, ODR %d Hz ===\r\n", N_IMU, ODR_HZ);
    pl_setup();
    { uint32_t a = g_idle; uint64_t t = gt_now(); vTaskDelay(pdMS_TO_TICKS(2000)); g_idle_ref = (double)(uint32_t)(g_idle - a) / (((double)(gt_now() - t)) / g_hz); }
    LOG("Referencia de CPU (inactividad): %.3e it/s\r\n", g_idle_ref);
    if (pl_start() == 0) {
        LOG("PL lista; arrancando la red.\r\n");
        sys_thread_new("main_thrd", main_thread, NULL, NET_STACK, DEFAULT_THREAD_PRIO);
    }
    vTaskDelete(NULL);
}

int main(void) {
    xTaskCreate(init_task, "init", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);
    vTaskStartScheduler();
    for (;;) {}
}
