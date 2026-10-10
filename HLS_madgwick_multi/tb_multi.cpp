// Testbench (csim): 4 IMUs sinteticas -> r2f_mux_top -> madgwick_multi_top, y comparacion
// bit a bit con el mismo nucleo ejecutado con UNA sola IMU (madgwick_single, NIMU=1).
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <vector>
#include "madgwick_multi.h"
#include "r2f_mux.h"

void madgwick_single(hls::stream<ap_axis_id_t>&, float q_out[4], unsigned int frame_cnt[1], float, float, int);

static unsigned f2u(float x) { unsigned u; memcpy(&u, &x, 4); return u; }
enum { NF = 600 };

static void make_frame(int imu, int k, int raw[9]) {
    double t = k / 119.0, ph = 0.9 * imu;
    raw[0] = (int)(2000 * sin(0.8 * t + ph)) + 150 * imu;
    raw[1] = (int)(3000 * sin(0.5 * t + 2 * ph));
    raw[2] = (int)(1500 * cos(1.1 * t + ph));
    raw[3] = (int)(1500 * sin(0.3 * t + ph));
    raw[4] = (int)(1200 * cos(0.4 * t + ph));
    raw[5] = 16000 + 100 * imu;
    raw[6] = 4000 - 300 * imu; raw[7] = 800 * sin(0.2 * t); raw[8] = -3000;
    if (imu == 3 && k % 50 >= 40) { raw[6] = raw[7] = raw[8] = 0; }       // rama IMU (sin magnetometro)
    if (imu == 2 && k % 97 >= 90) { raw[3] = raw[4] = raw[5] = 0; }       // rama GYRO (sin acelerometro)
}

int main() {
    hls::stream<ap_axis_raw_t> in[4];
    hls::stream<ap_axis_fid_t> mid;
    hls::stream<ap_axis_id_t>  mad;
    hls::stream<ap_axis_id_t>  one[4];
    std::vector<unsigned> order;                    // IMU de cada trama, en el orden en que llegan al nucleo
    // las 4 IMUs envian tramas con distinta cadencia (reparto desigual)
    int sent[4] = {0, 0, 0, 0};
    int rawf[4][NF][9];
    for (int i = 0; i < 4; i++) for (int k = 0; k < NF; k++) make_frame(i, k, rawf[i][k]);
    unsigned rng = 12345;
    int total = 0;
    while (total < 4 * NF) {
        rng = rng * 1664525u + 1013904223u;
        int i = (rng >> 24) & 3;
        if (i == 1 && (rng & 0x100)) continue;          // la IMU 1 va mas lenta
        if (sent[i] < NF) {
            for (int w = 0; w < 9; w++) {
                ap_axis_raw_t a; a.data = rawf[i][sent[i]][w]; a.last = (w == 8); a.keep = -1;
                in[i].write(a);
            }
            sent[i]++; total++;
        }
        // el nucleo procesa a ratos (para que haya tramas apiladas en varias entradas)
        if ((rng >> 8) % 3 == 0) {
            r2f_mux_top(in[0], in[1], in[2], in[3], mid, 1);
            while (!mid.empty()) {
                // trasvase de la salida del mux a la entrada del nucleo multi
                ap_axis_fid_t f = mid.read(); ap_axis_id_t g;
                g.data = f.data; g.last = f.last; g.id = f.id; g.keep = f.keep; mad.write(g); if (f.last) order.push_back((unsigned)f.id);
                { ap_axis_id_t h = g; h.id = 0; one[f.id].write(h); }   // mismo flujo separado por IMU (id=0) para la referencia
            }
        }
    }
    for (int r = 0; r < 4 * NF; r++) {                   // vaciar lo que quede en las entradas
        r2f_mux_top(in[0], in[1], in[2], in[3], mid, 1);
        while (!mid.empty()) {
            ap_axis_fid_t f = mid.read(); ap_axis_id_t g;
            g.data = f.data; g.last = f.last; g.id = f.id; g.keep = f.keep; mad.write(g); if (f.last) order.push_back((unsigned)f.id);
            { ap_axis_id_t h = g; h.id = 0; one[f.id].write(h); }
        }
    }
    float qm[16] = {0}; unsigned cnt[4] = {0};
    // El nucleo multi: primero un reset de las 4 IMUs (mascara 0xF) con 1 trama de cada una, luego beta=0.1
    // (como hace main.c). Aqui aplicamos el reset a la primera trama de cada IMU.
    std::vector<float> trace[4];
    int nm = 0; int seen[4] = {0, 0, 0, 0};
    while (!mad.empty()) {
        // mascara de reset: bit i activo hasta que la IMU i ha procesado su primera trama
        int mask = 0; for (int i = 0; i < 4; i++) if (seen[i] == 0) mask |= (1 << i);
        // para saber a que IMU pertenece la siguiente trama miramos su TID (solo en el banco de pruebas)
        unsigned id = order[nm];
        madgwick_multi_top(mad, qm, cnt, 0.1f, 1.0f / 119.0f, mask);
        seen[id]++;
        for (int j = 0; j < 4; j++) trace[id].push_back(qm[4 * id + j]);
        nm++;
    }
    // referencia: cada IMU sola, con el nucleo de una IMU
    int bad = 0; long checked = 0;
    for (int i = 0; i < 4; i++) {
        float q1[4] = {0}; unsigned c1[1] = {0}; int first = 1; size_t pos = 0;
        while (!one[i].empty()) {
            madgwick_single(one[i], q1, c1, 0.1f, 1.0f / 119.0f, first ? 1 : 0);   // primera trama: reset
            first = 0;
            for (int j = 0; j < 4; j++) {
                if (f2u(trace[i][pos + j]) != f2u(q1[j])) bad++;
                checked++;
            }
            pos += 4;
        }
        printf("IMU %d: %d tramas (frame_cnt=%u)  q_final = %+.5f %+.5f %+.5f %+.5f\n", i, sent[i], cnt[i], qm[4*i], qm[4*i+1], qm[4*i+2], qm[4*i+3]);
        if ((int)cnt[i] != NF) { printf("ERROR: frame_cnt[%d]=%u != %d\n", i, cnt[i], NF); bad++; }
    }
    printf("tramas procesadas por el nucleo multi: %d (esperadas %d)\n", nm, 4 * NF);
    printf("comparacion bit a bit con 4 nucleos de una IMU: %ld valores, %d diferencias\n", checked, bad);
    printf(bad == 0 && nm == 4 * NF ? "PASS\n" : "FAIL\n");
    return (bad == 0 && nm == 4 * NF) ? 0 : 1;
}
