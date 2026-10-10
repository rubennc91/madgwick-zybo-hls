#!/usr/bin/env python3
"""Analiza una captura de Integration_Zybo_multi (tecla 'L'/'l'): varias IMUs servidas por UN nucleo.

Uso:  python analyze_multi.py captura_multi.log [--rigid] [--png fig.png]

Comprueba, para cada IMU:
  * integridad: huecos en el contador de muestras, filas con ok=0, desfase constante entre el contador de
    tramas del nucleo (fc) y el de nav_spi_ctrl (cnt), tasa real de muestreo (a partir de t_us);
  * exactitud numerica: el cuaternion que dio el nucleo frente a la repeticion offline de esa IMU
    en float32 (modelo del nucleo) y en float64 (referencia), con las mismas entradas;
  * --rigid: si las IMUs estan fijadas a un mismo cuerpo rigido, deriva de la rotacion relativa IMU0-IMUj
    (desviacion respecto a su valor medio): mide la coherencia entre sensores, no la exactitud absoluta.
"""
import argparse, struct
import numpy as np
from replay import pl_inputs, run, ang

def f_(h): return struct.unpack('<f', struct.pack('<I', int(h, 16)))[0]

def load_multi(path):
    hdr, imus, rows = {}, {}, []
    for l in open(path, errors='ignore'):
        l = l.strip()
        if l.startswith('#HDR'):
            for kv in l.split(',')[1:]:
                k, v = kv.split('=', 1); hdr[k] = v
        elif l.startswith('#IMU'):
            p = l.split(',')
            d = dict(kv.split('=', 1) for kv in p[2:])
            imus[int(p[1])] = d
        elif l and l[0].isdigit() and l.count(',') == 17:
            p = l.split(',')
            rows.append([int(p[0]), int(p[1]), int(p[2]), int(p[3])] + [int(x) for x in p[4:13]] + [f_(x) for x in p[13:17]] + [int(p[17])])
    return hdr, imus, np.array(rows, dtype=np.float64)

def qmean(Q):
    Q = Q.copy(); Q[np.sum(Q * Q[0], axis=1) < 0] *= -1
    m = Q.mean(axis=0); return m / np.linalg.norm(m)

def qconj_mul(a, b):  # conj(a) * b, fila a fila
    a0, a1, a2, a3 = a.T; b0, b1, b2, b3 = b.T
    return np.stack([a0*b0 + a1*b1 + a2*b2 + a3*b3,
                     a0*b1 - a1*b0 - a2*b3 + a3*b2,
                     a0*b2 + a1*b3 - a2*b0 - a3*b1,
                     a0*b3 - a1*b2 + a2*b1 - a3*b0], axis=1)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('log'); ap.add_argument('--rigid', action='store_true'); ap.add_argument('--png')
    a = ap.parse_args()
    hdr, imus, D = load_multi(a.log)
    nimu = int(hdr['nimu']); beta_row = int(hdr['beta_row']); fs = int(hdr['fs_g'])
    SEED = np.array([-1, 0, 0, 0.2], dtype=np.float32)
    print('filas %d, IMUs %d, beta de arranque -> regimen en la fila %d, gt_hz=%s' % (len(D), nimu, beta_row, hdr.get('gt_hz')))
    ser, err = {}, {}
    for i in range(nimu):
        sel = D[:, 0] == i
        idx = np.where(sel)[0]
        S = D[sel]; cnt, fc, t, raw, Q, ok = S[:, 1], S[:, 2], S[:, 3] * 1e-6, S[:, 4:13].astype(np.int64), S[:, 13:17], S[:, 17]
        info = imus[i]
        cfg = int(info['axis_cfg'], 16)
        goff = [int(x) for x in info['goff'].split('/')]; moff = [int(x) for x in info['moff'].split('/')]
        rate = (len(S) - 1) / (t[-1] - t[0])
        dfc = fc - cnt
        print('\nIMU%d: %d filas, %.2f Hz, huecos en cnt: %d, filas ok=0: %d, desfase fc-cnt: %s' %
              (i, len(S), rate, int((np.diff(cnt) != 1).sum()), int((ok == 0).sum()),
               'constante (%d)' % dfc[0] if (dfc == dfc[0]).all() else 'VARIABLE (min %d, max %d)' % (dfc.min(), dfc.max())))
        inp = pl_inputs(raw, cfg, goff, moff, fs=fs)
        reset_mode = np.allclose(Q[0], SEED, atol=1e-6)
        seed = tuple(SEED) if reset_mode else tuple(Q[0])
        n_before = int((idx < beta_row).sum())          # tramas de esta IMU anteriores al cambio de beta
        best = None
        cands = [n_before + lat for lat in (0, 1, 2, 3)]      # el valor de beta se captura al arrancar cada invocacion
        if not reset_mode: cands = [-1] + cands               # sin reset: beta ya en regimen o cambia durante la captura
        for sw_c in cands:
            R32 = run(np.float32, inp, sw_c, seed=seed)
            m = ang(R32, Q).max()
            if best is None or m < best[0]: best = (m, sw_c, R32)
        m32, sw, R32 = best
        R64 = run(np.float64, inp, sw, seed=seed)
        e32, e64 = ang(R32, Q), ang(R64, Q)
        print('   modo %s, beta cambia en la trama %d de esta IMU' % ('reset (semilla)' if reset_mode else 'sin reset', sw))
        print('   nucleo vs modelo float32: media %.2e  max %.2e deg' % (e32.mean(), e32.max()))
        print('   nucleo vs float64       : media %.2e  max %.2e deg' % (e64.mean(), e64.max()))
        ser[i] = dict(t=t, Q=Q); err[i] = e64
    if a.rigid and nimu > 1:
        print('\nCoherencia entre IMUs (cuerpo rigido): desviacion de la rotacion relativa IMU0->IMUj respecto a su media')
        t0, Q0 = ser[0]['t'], ser[0]['Q']
        rel = {}
        for j in range(1, nimu):
            tj, Qj = ser[j]['t'], ser[j]['Q']
            k = np.clip(np.searchsorted(tj, t0), 1, len(tj) - 1)
            k = np.where(np.abs(tj[k - 1] - t0) < np.abs(tj[k] - t0), k - 1, k)     # muestra mas cercana en el tiempo
            r = qconj_mul(Q0, Qj[k]); r /= np.linalg.norm(r, axis=1)[:, None]
            r0 = qmean(r[int(3 * 118):])                                           # sin el arranque (beta alta)
            dev = ang(r, np.tile(r0, (len(r), 1)))
            rel[j] = dev
            d = dev[int(3 * 118):]
            print('   IMU0-IMU%d: rotacion media %.1f deg ; desviacion media %.2f  p95 %.2f  max %.2f deg' %
                  (j, np.degrees(2 * np.arctan2(np.linalg.norm(r0[1:]), abs(r0[0]))), d.mean(), np.percentile(d, 95), d.max()))
    if a.png:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        cols = ['#2a78d6', '#eb6834', '#2f9e6e', '#8a63d2']
        fig, ax = plt.subplots(2 if a.rigid else 1, 1, figsize=(8, 6 if a.rigid else 3.4), squeeze=False, facecolor='#fcfcfb')
        for i in range(nimu):
            ax[0, 0].semilogy(ser[i]['t'] - ser[i]['t'][0], np.maximum(err[i], 1e-9), lw=0.8, color=cols[i], label='IMU%d' % i)
        ax[0, 0].set_ylabel('error vs float64 (deg)'); ax[0, 0].legend(frameon=False, ncol=nimu)
        if a.rigid and nimu > 1:
            for j in rel: ax[1, 0].plot(ser[0]['t'] - ser[0]['t'][0], rel[j], lw=0.8, color=cols[j], label='IMU0-IMU%d' % j)
            ax[1, 0].set_ylabel('desviacion relativa (deg)'); ax[1, 0].set_xlabel('t (s)'); ax[1, 0].legend(frameon=False, ncol=3)
        for x in ax.ravel(): x.set_facecolor('#fcfcfb'); x.spines[['top', 'right']].set_visible(False)
        fig.tight_layout(); fig.savefig(a.png, dpi=160); print('figura:', a.png)

if __name__ == '__main__':
    main()
