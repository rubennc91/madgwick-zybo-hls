#!/usr/bin/env python3
"""Genera sw_input.h para el filtro Madgwick en el ARM con LA MISMA ENTRADA que el nucleo de la FPGA.
Uso:  python gen_sw_input.py deriva_raw.csv --imu 0 --frames 12000 --out sw_input.h
Toma las palabras crudas (las mismas que recibio el nucleo), las convierte a float32 con la misma escala que r2f_tid
y guarda tambien el cuaternion que dio el nucleo (SW_QCORE) para comparar. 12000 tramas ~ 100 s ~ 620 KB en DDR."""
import sys, argparse, numpy as np
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'analysis'))   # replay.py vive en analysis/
from replay import pl_inputs

ap = argparse.ArgumentParser(); ap.add_argument('csv'); ap.add_argument('--imu', type=int, default=0)
ap.add_argument('--frames', type=int, default=12000); ap.add_argument('--out', default='sw_input.h')
ap.add_argument('--skip', type=int, default=0, help='tramas a saltar al inicio'); a = ap.parse_args()
with open(a.csv) as f: first = f.readline()
hdr = dict(kv.split('=', 1) for kv in first.lstrip('# ').strip().split(',')[1:] if '=' in kv)
fs = int(hdr.get('fs_g') or 1); beta_run = float(hdr.get('beta_run', 0.1)); beta_start = float(hdr.get('beta_start', 1.0)); tsw = int(hdr.get('beta_start_ms', 3000)) / 1000.0
D = np.loadtxt(a.csv, delimiter=',', skiprows=2)
S = D[D[:, 1] == a.imu][a.skip:a.skip + a.frames]
t = S[:, 4] - S[0, 4]
dts = (hdr.get('dt_ns') or '').split('/'); dt = int(dts[a.imu]) * 1e-9 if len(dts) > a.imu and dts[a.imu].strip().lstrip('-').isdigit() and int(dts[a.imu]) > 0 else float(np.median(np.diff(t)))
inp = pl_inputs(S[:, 5:14], 0, 0, 0, fs=fs, words=True).astype(np.float32)
qc = S[:, 14:18].astype(np.float32)
k_sw = int(np.searchsorted(t, tsw))
def arr(name, A, w):
    rows = ',\n'.join('  {' + ','.join('%.9ef' % v for v in r) + '}' for r in A)
    return 'static const float %s[%d][%d] = {\n%s\n};\n' % (name, len(A), w, rows)
with open(a.out, 'w') as f:
    f.write('/* generado por gen_sw_input.py: %s imu=%d frames=%d */\n#define SW_N %d\n#define SW_DT %.9ef\n#define SW_BETA_START %.6ef\n#define SW_BETA_RUN %.6ef\n#define SW_BETA_SWITCH_K %d\n#define SW_RATE_HZ %.4f\n'
            % (a.csv, a.imu, len(S), len(S), dt, beta_start, beta_run, k_sw, 1.0 / dt))
    f.write(arr('SW_IN', inp, 9)); f.write(arr('SW_QCORE', qc, 4))
np.savetxt(a.out.replace('.h', '_qcore.csv'), qc, delimiter=',', fmt='%.9g')
print('%s: %d tramas, dt=%.3f us (%.2f Hz), beta %g->%g en la trama %d' % (a.out, len(S), dt * 1e6, 1 / dt, beta_start, beta_run, k_sw))
