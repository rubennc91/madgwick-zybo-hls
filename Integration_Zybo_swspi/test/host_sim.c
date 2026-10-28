#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "host_sim.h"

static uint64_t now_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (uint64_t)t.tv_sec * 1000000000ull + t.tv_nsec; }
uint64_t gt_now(void) { return now_ns(); }
double host_gt_hz(void) { return 1.0e9; }

typedef struct {   /* un LSM9DS1 */
    uint8_t ag_reg[64], m_reg[64];
    uint64_t gda_ready, mag_ready; int period_ns;
} lsm_t;
typedef struct { uint32_t cr, ssr; uint8_t tx[32]; int ntx; uint8_t rx[32]; int nrx; lsm_t dev; } qspi_t;
static qspi_t Q[4];
static int inited;
static const int period_ns[4] = { 8280000, 8307000, 8480000, 8057000 };
long sim_transactions;

static void init_all(void) {
    if (inited) return;
    inited = 1;
    uint64_t t = now_ns();
    for (int i = 0; i < 4; i++) {
        memset(&Q[i], 0, sizeof Q[i]); Q[i].ssr = 3;
        Q[i].dev.ag_reg[0x0F] = 0x68; Q[i].dev.m_reg[0x0F] = 0x3D;
        Q[i].dev.period_ns = period_ns[i]; Q[i].dev.gda_ready = t + period_ns[i]; Q[i].dev.mag_ready = t + 12500000;
    }
}
static void spin_ns(uint64_t d) { uint64_t t = now_ns() + d; while (now_ns() < t) { } }

static void slave(qspi_t *q, int cs, int ntx) {
    lsm_t *d = &q->dev; uint64_t t = now_ns();
    uint8_t addr = q->tx[0] & 0x3F; int rd = q->tx[0] & 0x80;
    q->nrx = ntx; memset(q->rx, 0, sizeof q->rx);
    if (!rd) { (cs == 0 ? d->ag_reg : d->m_reg)[addr] = q->tx[1]; return; }
    if (cs == 0 && addr == 0x17) { q->rx[1] = (t >= d->gda_ready) ? 0x03 : 0x00; return; }
    if (cs == 1 && addr == 0x27) { q->rx[1] = (t >= d->mag_ready) ? 0x0F : 0x00; return; }
    if (cs == 0 && addr == 0x18) { int16_t g[3] = { 100, -50, 30 }; for (int k = 0; k < 3; k++) { q->rx[1 + 2 * k] = g[k] & 0xFF; q->rx[2 + 2 * k] = (g[k] >> 8) & 0xFF; }
        while (d->gda_ready <= t) d->gda_ready += d->period_ns;
        return; }
    if (cs == 0 && addr == 0x28) { int16_t a[3] = { 0, 0, 16384 }; for (int k = 0; k < 3; k++) { q->rx[1 + 2 * k] = a[k] & 0xFF; q->rx[2 + 2 * k] = (a[k] >> 8) & 0xFF; } return; }
    if (cs == 1 && addr == 0x28) { int16_t m[3] = { 1500, 1400, -1500 }; for (int k = 0; k < 3; k++) { q->rx[1 + 2 * k] = m[k] & 0xFF; q->rx[2 + 2 * k] = (m[k] >> 8) & 0xFF; }
        while (d->mag_ready <= t) d->mag_ready += 12500000;
        return; }
    q->rx[1] = (cs == 0 ? d->ag_reg : d->m_reg)[addr];
}

uint32_t Xil_In32(uint32_t a) {
    init_all(); int i = (a - 0x40000000u) >> 16; uint32_t r = a & 0xFFFFu; qspi_t *q = &Q[i];
    if (r == 0x64) return (q->nrx ? 0 : 1u) | 4u;               /* RX_EMPTY, TX_EMPTY */
    if (r == 0x78) return q->nrx ? (uint32_t)(q->nrx - 1) : 0;
    if (r == 0x6C) { uint8_t v = q->rx[0]; memmove(q->rx, q->rx + 1, sizeof q->rx - 1); if (q->nrx) q->nrx--; return v; }
    if (r == 0x60) return q->cr;
    return 0;
}
void Xil_Out32(uint32_t a, uint32_t v) {
    init_all(); int i = (a - 0x40000000u) >> 16; uint32_t r = a & 0xFFFFu; qspi_t *q = &Q[i];
    if (r == 0x40) { q->ntx = q->nrx = 0; return; }
    if (r == 0x68) { if (q->ntx < 32) q->tx[q->ntx++] = (uint8_t)v; return; }
    if (r == 0x70) { q->ssr = v; return; }
    if (r == 0x60) {
        uint32_t was_inhibit = q->cr & (1u << 8); q->cr = v;
        if (v & (1u << 5)) q->ntx = 0;
        if (v & (1u << 6)) q->nrx = 0;
        if (!(v & (1u << 8)) && was_inhibit && q->ntx > 0) {
            int cs = !(q->ssr & 1u) ? 0 : (!(q->ssr & 2u) ? 1 : -1);
            if (cs < 0) { q->ntx = 0; return; }
            if ((v & 0x18) != 0x18) fprintf(stderr, "AVISO: el SPI no esta en modo 3 (CR=0x%X)\n", v);
            spin_ns((uint64_t)q->ntx * 8ull * 320ull);          /* 3,125 MHz: 320 ns por bit */
            slave(q, cs, q->ntx); q->ntx = 0; sim_transactions++;
        }
        return;
    }
}
