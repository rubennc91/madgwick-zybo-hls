#!/usr/bin/env python3
"""Prueba de poses con una caja: mide el error en las pausas de una captura de Integration_Zybo_dual.

Uso:  python analyze_poses.py captura.log [--nominal 90,90,90,90,90,90,90,90] [--thr 6] [--min 2.0]

La captura (tecla 'L') contiene: reposo inicial, una secuencia de giros de 90 grados con una pausa
en cada posicion, y reposo final. El script:
  1. Detecta las pausas (giroscopio casi parado durante >= --min segundos).
  2. Toma el cuaternion medio de cada nucleo en el ultimo segundo de cada pausa.
  3. Imprime, para cada pausa: el angulo girado desde la pausa anterior (original y opt2) frente al nominal,
     el error de inclinacion (gravedad estimada por el filtro frente al acelerometro; no depende del magnetometro),
     y la diferencia original-opt2.
  4. Cierre: angulo entre la primera pausa y cada pausa en la que deberias haber vuelto a la posicion de partida.
--nominal: angulos nominales (grados) entre pausas consecutivas, separados por comas. Si se omite solo se listan los angulos.
"""
import argparse, sys
import numpy as np
from analyze_dual import load, sample_rate
from replay import pl_inputs, ang

def qmean(Q):
    Q = Q.copy()
    Q[np.sum(Q * Q[0], axis=1) < 0] *= -1
    m = Q.mean(axis=0)
    return m / np.linalg.norm(m)

def qangle(a, b):
    return float(ang(a[None], b[None])[0])

def gravity_body(q):
    q0, q1, q2, q3 = q / np.linalg.norm(q)
    return np.array([2 * (q1 * q3 - q0 * q2), 2 * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3])

def dwells(raw, goff, thr, min_s, fs=119.0):
    w = np.linalg.norm(raw[:, :3] - np.array(goff), axis=1) * 17.5e-3
    k = int(0.25 * fs)
    ws = np.convolve(w, np.ones(k) / k, mode='same')
    quiet = ws < thr
    out, i, n = [], 0, len(quiet)
    while i < n:
        if quiet[i]:
            j = i
            while j < n and quiet[j]: j += 1
            if (j - i) / fs >= min_s: out.append((i, j))
            i = j
        else:
            i += 1
    return out


def qmul(a, b):
    return np.array([a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3], a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
                     a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1], a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]])

def gyro_angles(inp, ds, t, fs=119.0):
    """Angulo (grados) girado entre pausas consecutivas, integrando el giroscopio (marco del cuerpo, rad/s) con el
    sesgo medido en el ultimo segundo de cada pausa (interpolado). No depende del filtro ni del acelerometro."""
    bias = [inp[max(i, j - int(fs)):j, :3].mean(axis=0) for i, j in ds]
    out = []
    h = int(fs // 2)
    for k in range(1, len(ds)):
        s, e = ds[k - 1][1] - h, ds[k][1] - h
        q = np.array([1.0, 0, 0, 0])
        for n in range(s, e):
            f = (n - s) / (e - s)
            w = inp[n, :3] - (bias[k - 1] * (1 - f) + bias[k] * f)
            dt = (t[n + 1] - t[n]) if n + 1 < len(t) else 1 / fs
            th = np.linalg.norm(w) * dt
            dq = np.array([1.0, 0, 0, 0]) if th < 1e-12 else np.concatenate([[np.cos(th / 2)], np.sin(th / 2) * w / np.linalg.norm(w)])
            q = qmul(q, dq)
        out.append(np.degrees(2 * np.arctan2(np.linalg.norm(q[1:]), abs(q[0]))))
    return np.array(out)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log'); ap.add_argument('--nominal', default='')
    ap.add_argument('--thr', type=float, default=6.0, help='umbral de velocidad angular de reposo (deg/s)')
    ap.add_argument('--min', type=float, default=2.0, help='duracion minima de una pausa (s)')
    ap.add_argument('--settle', type=float, default=1.0, help='segundos finales de la pausa que se promedian')
    ap.add_argument('--gyro', action='store_true', help='ademas, angulo girado por integracion del giroscopio (escala del giroscopio)')
    a = ap.parse_args()
    hdr, D = load(a.log)
    raw = D[:, 1:10].astype(np.int64); QA, QB = D[:, 10:14], D[:, 14:18]
    fr = sample_rate(D)
    print('tasa real de muestreo: ' + ('%.2f Hz (nominal 119; el filtro integra con dt=1/119)' % fr if fr else 'desconocida (log sin t_us)'))
    goff = [int(x) for x in hdr['goff'].split('/')]; moff = [int(x) for x in hdr['moff'].split('/')]
    inp = pl_inputs(raw, int(hdr['axis_cfg'], 16), goff, moff, fs=int(hdr['fs_g']))
    acc = inp[:, 3:6].astype(np.float64)
    ds = dwells(raw, goff, a.thr, a.min)
    fs = 119.0
    print('pausas detectadas: %d' % len(ds))
    nom = [float(x) for x in a.nominal.split(',')] if a.nominal else []
    if nom and len(nom) != len(ds) - 1:
        print('AVISO: %d angulos nominales para %d pausas (deberian ser %d). Ajusta --thr/--min o la secuencia.' % (len(nom), len(ds), len(ds) - 1))
    P = []
    for (i, j) in ds:
        s = max(i, j - int(a.settle * fs))
        qa, qb = qmean(QA[s:j]), qmean(QB[s:j])
        am = acc[s:j].mean(axis=0); am /= np.linalg.norm(am)
        ta = np.degrees(np.arccos(np.clip(np.dot(gravity_body(qa), am), -1, 1)))
        tb = np.degrees(np.arccos(np.clip(np.dot(gravity_body(qb), am), -1, 1)))
        P.append((i / fs, (j - i) / fs, qa, qb, ta, tb))
    print('\n #   t_ini  dur | giro desde anterior: orig   opt2  nominal  err_orig  err_opt2 | inclin.err orig  opt2 | orig-opt2')
    for k, (t0, dur, qa, qb, ta, tb) in enumerate(P):
        if k == 0:
            print('%2d %6.1f %4.1f |      (referencia)                                       |           %5.2f  %5.2f | %7.4f' % (k, t0, dur, ta, tb, qangle(qa, qb)))
            continue
        ga, gb = qangle(P[k - 1][2], qa), qangle(P[k - 1][3], qb)
        nm = nom[k - 1] if k - 1 < len(nom) else float('nan')
        print('%2d %6.1f %4.1f |        %7.2f %7.2f %7.1f %9.2f %9.2f |           %5.2f  %5.2f | %7.4f' % (k, t0, dur, ga, gb, nm, ga - nm, gb - nm, ta, tb, qangle(qa, qb)))
    if len(P) > 1:
        print('\nCierre respecto a la primera pausa (angulo entre la pausa 0 y la k; 0 = vuelta exacta):')
        for k in range(1, len(P)):
            print('  pausa %2d: orig %6.2f deg   opt2 %6.2f deg' % (k, qangle(P[0][2], P[k][2]), qangle(P[0][3], P[k][3])))
    if a.gyro and len(P) > 1:
        t = D[:, 19] * 1e-6 if np.isfinite(D[:, 19]).all() else np.arange(len(D)) / 119.0
        g = gyro_angles(inp.astype(np.float64), ds, t)
        print('\nAngulo girado por el giroscopio (integrado, sin filtro):')
        for k, v in enumerate(g, 1):
            print('  paso %2d: %7.2f deg%s' % (k, v, ('   (nominal %.0f -> escala %.3f)' % (nom[k - 1], v / nom[k - 1])) if k - 1 < len(nom) else ''))
        if len(nom) == len(g):
            print('  total %.1f deg de %.1f nominales -> escala del giroscopio %.3f' % (g.sum(), sum(nom), g.sum() / sum(nom)))
    if nom:
        e = np.array([[qangle(P[k - 1][2], P[k][2]) - nom[k - 1], qangle(P[k - 1][3], P[k][3]) - nom[k - 1]] for k in range(1, min(len(P), len(nom) + 1))])
        if len(e):
            print('\nError del angulo girado vs nominal: orig media |e| %.2f max %.2f  |  opt2 media |e| %.2f max %.2f (grados)' % (np.abs(e[:, 0]).mean(), np.abs(e[:, 0]).max(), np.abs(e[:, 1]).mean(), np.abs(e[:, 1]).max()))

if __name__ == '__main__':
    main()
