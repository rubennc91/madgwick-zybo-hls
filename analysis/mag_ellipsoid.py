#!/usr/bin/env python3
"""Calibracion del magnetometro por elipsoide (hard-iron + soft-iron), fuera de linea.
Uso:   python mag_ellipsoid.py captura_raw.csv --imu 1 [--plot]
       python mag_ellipsoid.py --selftest
Entrada: CSV de capture_raw.py (tecla 'w'): host_s,imu,flags,cnt,t_s,w0..w8,q0..q3 ; el campo magnetico son w6..w8 (cuentas).
Gira la IMU SUELTA por todas las orientaciones (30-60 s). No sirve montada en el brazo: no cubre la esfera.
Modelo:  m_corr = W (m - c)   con c = offset (hard-iron, cuentas) y W = matriz 3x3 simetrica (soft-iron + escala)
         |m_corr| = r0 (cuentas) constante: W se normaliza para que la esfera resultante tenga el radio medio r0.
Salida:  c y W, residuo antes (solo esfera/offset) y despues (elipsoide), validacion cruzada (ajuste con la mitad, prueba en la otra),
         cobertura de orientaciones, y c entero listo para MAG_PRESET."""
import sys, argparse
import numpy as np

def load(path, imu):
    rows = []
    for l in open(path):
        if l.startswith('#') or not l.strip() or l[0].isalpha(): continue
        p = l.strip().split(',')
        if len(p) < 14 or int(p[1]) != imu: continue
        rows.append([float(x) for x in p[10:13]])          # w6,w7,w8 = columnas 5+6..5+8
    return np.array(rows)

def fit_sphere(m):
    A = np.c_[2 * m, np.ones(len(m))]; b = (m ** 2).sum(1)
    s = np.linalg.lstsq(A, b, rcond=None)[0]; c = s[:3]; r = np.sqrt(s[3] + c @ c); return c, r

def fit_ellipsoid(m):
    mu = m.mean(0); sc = np.linalg.norm(m - mu, axis=1).mean()      # normaliza: las cuentas^2 mal condicionan el ajuste
    c, S, ev = _fit_ellipsoid_n((m - mu) / sc)
    return c * sc + mu, S / sc, ev / sc ** 2

def _fit_ellipsoid_n(m):
    x, y, z = m.T
    D = np.c_[x*x, y*y, z*z, 2*y*z, 2*x*z, 2*x*y, 2*x, 2*y, 2*z]
    u = np.linalg.lstsq(D, np.ones(len(m)), rcond=None)[0]
    Q = np.array([[u[0], u[5], u[4]], [u[5], u[1], u[3]], [u[4], u[3], u[2]]]); w = u[6:9]
    if np.min(np.linalg.eigvalsh(Q)) <= 0: raise ValueError('el ajuste no es un elipsoide (datos poco variados)')
    c = -np.linalg.solve(Q, w)
    M = Q / (1 + w @ np.linalg.solve(Q, w))                 # (p-c)^T M (p-c) = 1
    ev, V = np.linalg.eigh(M); S = V @ np.diag(np.sqrt(ev)) @ V.T      # raiz simetrica: |S (p-c)| = 1
    return c, S, ev

def residual(m, c, W=None):
    d = m - c
    if W is not None: d = d @ W.T
    r = np.linalg.norm(d, axis=1); return r.std() / r.mean() * 100

def coverage(m, c):
    u = m - c; u = u / np.linalg.norm(u, axis=1)[:, None]
    az = np.floor((np.arctan2(u[:, 1], u[:, 0]) + np.pi) / (2*np.pi) * 8).astype(int).clip(0, 7)
    el = np.floor((np.arcsin(u[:, 2].clip(-1, 1)) + np.pi/2) / np.pi * 4).astype(int).clip(0, 3)
    return len(set(zip(az, el))), 32

def calibrate(m, verbose=True):
    c0, r0 = fit_sphere(m)
    c, S, ev = fit_ellipsoid(m)
    r = float(np.mean(np.linalg.norm(m - c, axis=1)))
    W = S * r                                                 # misma unidad (cuentas) que la entrada
    rng = np.random.default_rng(0); idx = rng.permutation(len(m)); a, b = idx[: len(m)//2], idx[len(m)//2:]
    ca, Sa, _ = fit_ellipsoid(m[a]); cv = residual(m[b], ca, Sa); cv0 = residual(m[b], fit_sphere(m[a])[0])
    ax = 1 / np.sqrt(ev)
    if verbose:
        n_ok, n_all = coverage(m, c)
        print(f'muestras {len(m)}   cobertura {n_ok}/{n_all} sectores de la esfera' + ('   <-- POCA: gira mas por todas las orientaciones' if n_ok < 20 else ''))
        print(f'esfera  (solo offset): c = [{c0[0]:.0f} {c0[1]:.0f} {c0[2]:.0f}]  radio {r0:.0f}  residuo {residual(m, c0):.2f} %')
        print(f'elipsoide: c = [{c[0]:.0f} {c[1]:.0f} {c[2]:.0f}]  semiejes {ax[0]:.0f} {ax[1]:.0f} {ax[2]:.0f}  (cociente max/min {ax.max()/ax.min():.3f})  residuo {residual(m, c, W):.2f} %')
        print(f'validacion cruzada (ajusto con una mitad, evaluo en la otra): esfera {cv0:.2f} %   elipsoide {cv:.2f} %')
        print('W (m_corr = W (m - c)):'); print(np.array2string(W, precision=4) + f'   (adimensional; el radio resultante es {r:.0f} cuentas)')
        print('MAG_PRESET fila:  {%d, %d, %d}' % tuple(int(round(v)) for v in c))
        print('Mejora real solo si la validacion cruzada del elipsoide baja claramente respecto a la esfera.')
    return c, W, r

def selftest():
    rng = np.random.default_rng(1)
    u = rng.normal(size=(6000, 3)); u /= np.linalg.norm(u, axis=1)[:, None]
    T = np.array([[1.00, 0.04, -0.02], [0.04, 0.90, 0.03], [-0.02, 0.03, 1.08]]); c_true = np.array([1500., -2000., 800.])
    m = 2900 * u @ T.T + c_true + rng.normal(0, 8, (6000, 3))
    c, W, r = calibrate(m)
    err = np.abs(c - c_true).max(); print('error de offset', round(err, 1), 'cuentas')
    assert err < 10 and residual(m, c, W) < 0.6; print('selftest OK')

if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('csv', nargs='?'); ap.add_argument('--imu', type=int, default=0); ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest or not a.csv: selftest(); sys.exit()
    m = load(a.csv, a.imu)
    if len(m) < 500: sys.exit(f'solo {len(m)} muestras de la IMU {a.imu}: captura al menos 30 s girando')
    calibrate(m)
