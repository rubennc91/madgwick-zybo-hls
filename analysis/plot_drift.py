#!/usr/bin/env python3
"""Analiza el CSV de viewer_live.py: deriva del angulo relativo con las IMUs en reposo relativo.
Uso: python plot_drift.py captura.csv [--png deriva.png] [--skip 10]
--skip S: ignora los primeros S segundos (el filtro converge / se pone la tara)."""
import sys, numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

def opt(n, d): return sys.argv[sys.argv.index(n) + 1] if n in sys.argv else d
path = sys.argv[1]; png = opt('--png', 'deriva.png'); skip = float(opt('--skip', 10))
d = np.genfromtxt(path, delimiter=',', skip_header=2, usecols=(0, 1, 2, 3, 5, 6, 7))
host, seq, tus, ang, tare, lost, bad = d.T
# t_us es u32 y da la vuelta a los 71.6 min: se desenrolla
wraps = np.concatenate([[0], np.cumsum(np.diff(tus) < 0)])
t = (tus + wraps * 2**32) / 1e6; m = (t >= skip) & (tare > 0)
t, ang = t[m], ang[m]
if len(t) < 10: sys.exit('muy pocos datos tras la tara')
slope, icpt = np.polyfit(t, ang, 1)
n_ok = len(seq); n_exp = int(seq[-1] - seq[0] + 1)
print(f'duracion tras la tara: {t[-1]-t[0]:.0f} s ({(t[-1]-t[0])/3600:.2f} h), tramas {n_ok}')
print(f'angulo: media {ang.mean():.3f}  desv.tipica {ang.std():.3f}  min {ang.min():.3f}  max {ang.max():.3f} grados')
print(f'tendencia lineal: {slope*3600:+.3f} grados/hora')
print(f'tramas esperadas {n_exp}, perdidas {int(lost[-1])} ({100*lost[-1]/max(n_exp,1):.3f} %), CRC malos {int(bad[-1])}')
fig, ax = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
ax[0].plot(t / 60, ang, lw=0.6); ax[0].plot(t / 60, icpt + slope * t, 'r--', lw=1, label=f'tendencia {slope*3600:+.2f} deg/h')
ax[0].set_ylabel('angulo relativo (deg)'); ax[0].legend(); ax[0].grid(alpha=.3)
w = max(1, int(len(t) / 400)); r = ang[:len(ang)//w*w].reshape(-1, w)
ax[1].plot(t[:len(r)*w:w] / 60, r.max(1) - r.min(1), lw=0.8); ax[1].set_ylabel('ruido pico a pico (deg)')
ax[1].set_xlabel('tiempo (min)'); ax[1].grid(alpha=.3)
fig.tight_layout(); fig.savefig(png, dpi=200); print('figura:', png)
