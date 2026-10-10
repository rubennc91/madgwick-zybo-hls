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
        elif l and l[0].isdigit() and l.count(',') in (18, 19):
            p = l.split(',')
            f = lambda h: struct.unpack('<f', struct.pack('<I', int(h, 16)))[0]
            rows.append([int(p[0])] + [int(x) for x in p[1:10]] + [f(x) for x in p[10:18]] + [int(p[18])] + [int(p[19]) if len(p) > 19 else float('nan')])
    return hdr, np.array(rows, dtype=np.float64)

def main():
    hdr, D = load(sys.argv[1])
    cnt, raw, QA, QB, ok = D[:, 0], D[:, 1:10].astype(np.int64), D[:, 10:14], D[:, 14:18], D[:, 18]
    cfg = int(hdr['axis_cfg'], 16)
    goff = [int(x) for x in hdr['goff'].split('/')]; moff = [int(x) for x in hdr['moff'].split('/')]
    beta_ms = int(hdr['beta_start_ms'])
    n_sw = -(-beta_ms * 119 // 1000)          # primera fila con n*1000/119 >= beta_ms (n = filas capturadas)
    print('filas %d  huecos en cnt: %d  filas con ok=0: %d' % (len(D), int((np.diff(cnt) != 1).sum()), int((ok == 0).sum())))
    inp = pl_inputs(raw, cfg, goff, moff, fs=int(hdr['fs_g']))
    SEED = np.array([-1, 0, 0, 0.2], dtype=np.float32)
    if np.allclose(QA[0], SEED, atol=1e-6):
        # captura con reinicio ('L'): empieza en la semilla; beta de arranque durante beta_start_ms
        print('modo: captura con reinicio (semilla en la primera fila)')
        cands = [(tuple(SEED), n_sw)]
    else:
        # captura sin reinicio ('l'): el estado inicial es el de la primera fila; beta ya en regimen
        # (o, si se pulso poco despues de un reinicio, con beta de arranque hasta beta_start_ms)
        print('modo: captura sin reinicio (estado inicial = primera fila)')
        cands = [(tuple(QA[0]), -1), (tuple(QA[0]), n_sw)]
    best = None
    for seed, sw in cands:
        R32 = run(np.float32, inp, sw, seed=seed)
        m = ang(R32, QA).max()
        if best is None or m < best[0]: best = (m, seed, sw, R32)
    _, seed, sw, R32 = best
    print('beta: %s' % ('arranque hasta la trama %d' % sw if sw >= 0 else 'en regimen todo el tramo'))
    R64 = run(np.float64, inp, sw, seed=seed)
    st = lambda n, e: print('%-34s media %.2e  p99 %.2e  max %.2e  (grados)' % (n, e.mean(), np.percentile(e, 99), e.max()))
    eA, eB = ang(QA, R64), ang(QB, R64)
    st('original   vs doble', eA); st('opt2       vs doble', eB)
    st('float32 exacto vs doble', ang(R32, R64)); st('original   vs opt2', ang(QA, QB))
    print('control: modelo float32 vs nucleo original max %.2e deg (valida la reproduccion)' % ang(R32, QA).max())
    if len(sys.argv) > 2:
        figure(sys.argv[2], raw, goff, eA, eB)

def figure(path, raw, goff, eA, eB):
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
    t = np.arange(len(eA)) / 119.0
    w = np.linalg.norm(raw[:, :3] - np.array(goff), axis=1) * 17.5e-3
    S, TXT, T2, GR, B, O = '#fcfcfb', '#0b0b0b', '#52514e', '#e3e2dd', '#2a78d6', '#eb6834'
    plt.rcParams.update({'font.size': 9, 'axes.edgecolor': GR, 'axes.labelcolor': T2, 'xtick.color': T2, 'ytick.color': T2})
    fig = plt.figure(figsize=(10, 6), facecolor=S)
    gs = GridSpec(2, 2, height_ratios=[1, 2], width_ratios=[2.2, 1], hspace=0.22, wspace=0.28, left=0.07, right=0.97, top=0.90, bottom=0.1)
    a1 = fig.add_subplot(gs[0, 0]); a2 = fig.add_subplot(gs[1, 0], sharex=a1); a3 = fig.add_subplot(gs[:, 1])
    for a in (a1, a2, a3):
        a.set_facecolor(S); a.grid(True, color=GR, lw=0.6); a.set_axisbelow(True)
        for sp in ('top', 'right'): a.spines[sp].set_visible(False)
    a1.plot(t, w, color=T2, lw=0.8); a1.set_ylabel('|w| (deg/s)'); a1.tick_params(labelbottom=False)
    a1.set_title('Entrada: velocidad angular', loc='left', color=TXT, fontsize=10)
    sm = lambda x, k=60: np.convolve(x, np.ones(k) / k, mode='same')
    for e, c, l in ((eA, B, 'original'), (eB, O, 'opt2')):
        a2.semilogy(t, np.maximum(e, 1e-7), color=c, lw=0.35, alpha=0.3); a2.semilogy(t, sm(e), color=c, lw=1.6, label=l)
        x = np.sort(e); a3.plot(x * 1e5, np.arange(1, len(x) + 1) / len(x), color=c, lw=1.8, label=l)
    a2.set_ylim(1e-6, 3e-4); a2.set_xlabel('t (s)'); a2.set_ylabel('error angular vs. doble precision (deg)')
    a2.set_title('Error de calculo con la misma entrada (linea gruesa: media movil 0,5 s)', loc='left', color=TXT, fontsize=10)
    a3.set_xlabel('error angular (1e-5 deg)'); a3.set_ylabel('fraccion de muestras'); a3.set_ylim(0, 1.02)
    a3.set_title('Distribucion acumulada', loc='left', color=TXT, fontsize=10)
    a2.legend(frameon=False, loc='upper left', labelcolor=TXT); a3.legend(frameon=False, loc='lower right', labelcolor=TXT)
    fig.text(0.07, 0.955, 'Original frente a opt2: error de calculo respecto a doble precision', color=TXT, fontsize=11, weight='bold')
    fig.savefig(path, dpi=160, facecolor=S)

if __name__ == '__main__':
    main()

def sample_rate(D):
    """Tasa real de muestreo (Hz) a partir de la columna t_us (indice 19), o None si el log no la trae."""
    t = D[:, 19]
    if not np.isfinite(t).all() or len(t) < 2: return None
    return (len(t) - 1) / ((t[-1] - t[0]) * 1e-6)
