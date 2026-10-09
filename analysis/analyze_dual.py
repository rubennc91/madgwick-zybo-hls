#!/usr/bin/env python3
"""Analiza un volcado CSV de Integration_Zybo_dual (capturas 'l'/'L' del main.c).

Uso:  python analyze_dual.py teraterm.log [salida.png]

1. Lee la cabecera #HDR y las filas del log (cnt, 9 datos crudos, q original, q opt2).
2. Reconstruye la entrada exacta de los nucleos (offsets, ejes y escalas de nav_spi_ctrl/raw2float).
3. Repite el filtro en doble precision (referencia) y en float32 con rsqrt exacto.
4. Calcula el error angular de cada nucleo respecto a la referencia y entre ellos.

Nota sobre la captura: con reset, el primer registro es la propia semilla (un nucleo HLS en
modo auto-restart toma 'reset' al empezar la invocacion, antes de que el PS lo cambie) y el
cambio de beta llega con dos tramas de retraso; BETA_SWITCH_FRAME se calcula como en main.c.
"""
import re, struct, sys
import numpy as np
from replay import pl_inputs, run, ang

def load(path):
    hdr, rows = {}, []
    for l in open(path, errors='ignore'):
        l = l.strip()
        if l.startswith('#HDR'):
            for kv in l.split(',')[1:]:
                k, v = kv.split('=', 1); hdr[k] = v
        elif l and l[0].isdigit() and l.count(',') == 18:
            p = l.split(',')
            f = lambda h: struct.unpack('<f', struct.pack('<I', int(h, 16)))[0]
            rows.append([int(p[0])] + [int(x) for x in p[1:10]] + [f(x) for x in p[10:18]] + [int(p[18])])
    return hdr, np.array(rows, dtype=np.float64)

def main():
    hdr, D = load(sys.argv[1])
    cnt, raw, QA, QB, ok = D[:, 0], D[:, 1:10].astype(np.int64), D[:, 10:14], D[:, 14:18], D[:, 18]
    cfg = int(hdr['axis_cfg'], 16)
    goff = [int(x) for x in hdr['goff'].split('/')]; moff = [int(x) for x in hdr['moff'].split('/')]
    beta_ms = int(hdr['beta_start_ms'])
    n_sw = -(-beta_ms * 119 // 1000)          # primera fila con n*1000/119 >= beta_ms (n = filas capturadas)
    sw = n_sw                                # ultima trama con beta de arranque (ver nota)
    print('filas %d  huecos en cnt: %d  filas con ok=0: %d' % (len(D), int((np.diff(cnt) != 1).sum()), int((ok == 0).sum())))
    inp = pl_inputs(raw, cfg, goff, moff, fs=int(hdr['fs_g']))
    R64 = run(np.float64, inp, sw)
    R32 = run(np.float32, inp, sw)
    st = lambda n, e: print('%-34s media %.2e  p99 %.2e  max %.2e  (grados)' % (n, e.mean(), np.percentile(e, 99), e.max()))
    eA, eB = ang(QA, R64), ang(QB, R64)
    st('original   vs doble', eA); st('opt2       vs doble', eB)
    st('float32 exacto vs doble', ang(R32, R64)); st('original   vs opt2', ang(QA, QB))
    print('control: modelo float32 vs nucleo original max %.2e deg (valida la reproduccion)' % ang(R32, QA).max())
    if len(sys.argv) > 2:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        t = np.arange(len(D)) / 119.0
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.semilogy(t, np.maximum(eA, 1e-7), lw=.4, label='original'); ax.semilogy(t, np.maximum(eB, 1e-7), lw=.4, label='opt2')
        ax.set_xlabel('t (s)'); ax.set_ylabel('error vs doble (grados)'); ax.legend(); fig.savefig(sys.argv[2], dpi=150)

if __name__ == '__main__':
    main()
