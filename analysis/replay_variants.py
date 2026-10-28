#!/usr/bin/env python3
"""Repite el filtro Madgwick fuera de linea sobre una captura cruda (capture_raw.py) con distintas variantes
y diagnostica de donde viene la oscilacion lenta de la guinada entre dos IMUs quietas.
Uso:  python replay_variants.py deriva_raw.csv [--png out.png] [--tare-at 30] [--max-min 60] [--betas 0.03,0.3]
      python replay_variants.py --selftest
Modelo: replay.py (mismo que analysis/replay.py del repo; float32, bit-comparable con el nucleo a ~1e-4 grados).
Variantes: 'nucleo' (AHRS con beta de regimen: debe coincidir con la q grabada), 'sin mag' (IMU: solo giroscopio+acelerometro),
           'beta X' (otras ganancias).  Se parte de la q grabada en la primera fila de cada IMU.
Diagnostico sin filtro: sesgo del giroscopio y rumbo magnetico de cada IMU, medias de 1 min."""
import sys, argparse, math
import numpy as np
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from replay import make, pl_inputs

def load(path, max_min=0):
    hdr = {}
    with open(path) as f:
        first = f.readline()
    for kv in first.lstrip('# ').strip().split(',')[1:]:
        if '=' in kv: k, v = kv.split('=', 1); hdr[k] = v
    D = np.loadtxt(path, delimiter=',', skiprows=2)
    if max_min: D = D[D[:, 4] <= max_min * 60 + 1]
    return hdr, D

def qmul(a, b):
    a0, a1, a2, a3 = a.T; b0, b1, b2, b3 = b.T
    return np.stack([a0*b0-a1*b1-a2*b2-a3*b3, a0*b1+a1*b0+a2*b3-a3*b2, a0*b2-a1*b3+a2*b0+a3*b1, a0*b3+a1*b2-a2*b1+a3*b0], 1)
def qconj(a): return a * np.array([1, -1, -1, -1])
def avg_quat(Q):
    Q = Q / np.linalg.norm(Q, axis=1, keepdims=True); v = np.linalg.eigh(Q.T @ Q)[1][:, -1]
    return v if v[0] >= 0 else -v
def ang_between(a, b):
    r = qmul(qconj(a), b); r = r / np.linalg.norm(r, axis=1, keepdims=True)
    return 2 * np.degrees(np.arctan2(np.linalg.norm(r[:, 1:], axis=1), np.abs(r[:, 0])))

def replay(inp, t, q_start, dt, beta_run, beta_start, tsw, use_mag):
    ahrs = make(np.float32); f32 = np.float32
    q = [f32(v) for v in q_start]; out = np.empty((len(inp), 4)); out[0] = q_start
    dtf = f32(dt); b0 = f32(beta_start); b1 = f32(beta_run)
    L = inp.tolist()
    for k in range(1, len(L)):
        x = [f32(v) for v in L[k]]
        if not use_mag: x[6] = x[7] = x[8] = f32(0)
        q = ahrs(q, *x, b0 if t[k] < tsw else b1, dtf)
        out[k] = q
    return out

def relative(t0, Q0, t1, Q1, tare_at):
    j = np.clip(np.searchsorted(t1, t0, side='right') - 1, 0, len(t1) - 1)
    R = qmul(qconj(Q0), Q1[j])
    m = (t0 >= tare_at) & (t0 < tare_at + 1.0)
    tare = avg_quat(R[m]) if m.any() else R[0]
    Rt = qmul(np.tile(qconj(tare), (len(R), 1)), R); Rt *= np.where(Rt[:, [0]] < 0, -1, 1)
    return Rt

def stats(t, Rt, tare_at, label):
    m = t >= tare_at + 10; tt = t[m]; ang = 2 * np.degrees(np.arctan2(np.linalg.norm(Rt[m, 1:], axis=1), Rt[m, 0]))
    rv = np.degrees(2 * Rt[m, 1:]); tr = np.polyfit(tt, ang, 1)[0] * 3600
    nb = int((tt[-1] - tt[0]) // 60); k = 7440 if nb < 1 else None
    blk = [rv[(tt - tt[0] >= i * 60) & (tt - tt[0] < (i + 1) * 60)].mean(0) for i in range(nb)]
    blk = np.array(blk) if blk else np.zeros((1, 3))
    print('%-12s angulo: media %.3f  std %.3f  max %.3f  tend %+.3f deg/h | std ejes x/y/z %.3f/%.3f/%.3f | medias de 1 min en z: std %.3f, rango [%.2f, %.2f]' %
          (label, ang.mean(), ang.std(), ang.max(), tr, *rv.std(0), blk[:, 2].std(), blk[:, 2].min(), blk[:, 2].max()))
    return tt, blk

def heading(a, m):
    u = a / np.linalg.norm(a); n = m - np.dot(m, u) * u; x = np.array([1., 0, 0]); xh = x - u[0] * u
    return math.degrees(math.atan2(np.dot(u, np.cross(xh, n)), np.dot(xh, n)))

def main():
    if '--selftest' in sys.argv: return selftest()
    ap = argparse.ArgumentParser(); ap.add_argument('csv'); ap.add_argument('--png'); ap.add_argument('--tare-at', type=float, default=30.0)
    ap.add_argument('--max-min', type=float, default=0); ap.add_argument('--betas', default='0.03,0.3'); ap.add_argument('--no-variants', action='store_true')
    a = ap.parse_args()
    hdr, D = load(a.csv, a.max_min)
    fs = int(hdr.get('fs_g') or 1)
    dts = []
    for x in (hdr.get('dt_ns') or '').split('/'): dts.append(int(x) * 1e-9 if x.strip().lstrip('-').isdigit() else 0.0)
    dts += [0.0] * (2 - len(dts))
    b_run = float(hdr.get('beta_run', 0.1)); b_start = float(hdr.get('beta_start', 1.0)); tsw = int(hdr.get('beta_start_ms', 3000)) / 1000.0
    gs = {0: 8.75e-3, 1: 17.5e-3}.get(fs, 70e-3)
    I = {}
    for i in (0, 1):
        S = D[D[:, 1] == i]; cnt = S[:, 3]; gaps = int((np.diff(cnt) != 1).sum()); lost = int((np.diff(cnt)[np.diff(cnt) != 1] - 1).sum())
        bad = int((S[:, 2] == 0).sum())
        if dts[i] <= 0:                    # cabecera incompleta: se usa el intervalo mediano entre tramas (mismo reloj de hardware)
            dts[i] = float(np.median(np.diff(S[:, 4]))); print('AVISO: dt de la IMU%d no esta en la cabecera; uso la mediana de los intervalos: %.1f us' % (i, dts[i] * 1e6))
        I[i] = dict(t=S[:, 4], w=S[:, 5:14].astype(np.int64), q=S[:, 14:18], inp=pl_inputs(S[:, 5:14], 0, 0, 0, fs=fs, words=True))
        print('IMU%d: %d registros, %.1f min, %.2f Hz, huecos en cnt %d (tramas no leidas %d), flags=0: %d' % (i, len(S), (S[-1, 4] - S[0, 4]) / 60, (len(S) - 1) / (S[-1, 4] - S[0, 4]), gaps, lost, bad))
    if gaps or lost: print('AVISO: con huecos el filtro repetido pierde tramas; usa tramas contiguas o re-inicia en cada hueco')
    variants = [('nucleo', b_run, True)]
    if not a.no_variants:
        variants += [('sin mag', b_run, False)] + [('beta %s' % b, float(b), True) for b in a.betas.split(',') if b]
    res = {}
    for name, beta, mag in variants:
        Q = {}
        for i in (0, 1):
            Q[i] = replay(I[i]['inp'], I[i]['t'], I[i]['q'][0], dts[i], beta, b_start, tsw, mag)
        if name == 'nucleo':
            for i in (0, 1):
                m = I[i]['t'] > tsw + 10; e = ang_between(Q[i][m], I[i]['q'][m])
                print('  validacion IMU%d: modelo repetido vs q del nucleo: media %.2e  max %.2e grados (t > %.0f s)' % (i, e.mean(), e.max(), tsw + 10))
        Rt = relative(I[0]['t'], Q[0], I[1]['t'], Q[1], a.tare_at)
        res[name] = stats(I[0]['t'], Rt, a.tare_at, name)
    # ---- diagnostico sin filtro ----
    t = I[0]['t']; T0 = a.tare_at + 10; nb = int((t[-1] - T0) // 60)      # mismos bloques de 1 min que stats() (que empieza en tare_at + 10)
    if nb >= 3:
        print('\nDiagnostico por minuto (desde t=%.0f s): sesgo del giroscopio y rumbo magnetico, sin filtro' % T0)
        G = {i: [] for i in (0, 1)}; H = {i: [] for i in (0, 1)}; MM = {i: [] for i in (0, 1)}
        for i in (0, 1):
            ti = I[i]['t']
            for b in range(nb):
                m = (ti >= T0 + 60 * b) & (ti < T0 + 60 * (b + 1)); w = I[i]['w'][m].mean(0)
                G[i].append(w[:3] * gs); acc = w[3:6]; mag = w[6:9]; H[i].append(heading(acc, mag)); MM[i].append(np.linalg.norm(mag))
        G = {i: np.array(v) for i, v in G.items()}; H = {i: np.array(v) for i, v in H.items()}; MM = {i: np.array(v) for i, v in MM.items()}
        dh = H[1] - H[0]; dh = (dh - dh[0] + 180) % 360 - 180
        gz = (G[1][:, 2] - G[0][:, 2]) * 60                   # grados de guinada acumulados por minuto por el sesgo relativo
        print('  rumbo IMU1-IMU0 (grados, respecto al minuto 0): media %.3f  std %.3f  rango [%.2f, %.2f]' % (dh.mean(), dh.std(), dh.min(), dh.max()))
        print('  |mag| (cuentas): IMU0 std %.1f  IMU1 std %.1f' % (MM[0].std(), MM[1].std()))
        for i in (0, 1): print('  sesgo gz IMU%d: media %+.4f dps, variacion entre minutos (std) %.4f dps' % (i, G[i][:, 2].mean(), G[i][:, 2].std()))
        print('  sesgo relativo de gz acumula %.3f grados/min (std de la variacion %.3f)' % (gz.mean(), gz.std()))
        # ---- temperatura (columna 18; solo con el firmware/bitstream nuevos) ----
        if D.shape[1] > 18 and np.isfinite(D[:, 18]).any():
            TT = {}
            for i in (0, 1):
                ti = I[i]['t']; Ti = D[D[:, 1] == i][:, 18]; v = []
                for b in range(nb):
                    m = (ti >= T0 + 60 * b) & (ti < T0 + 60 * (b + 1)) & np.isfinite(Ti); v.append(Ti[m].mean() if m.any() else np.nan)
                TT[i] = np.array(v)
            print('\nTemperatura (OUT_TEMP, +-1 LSB = 0.0625 C; resolucion util ~0.5 C, offset absoluto sin calibrar):')
            for i in (0, 1): print('  IMU%d: inicio %.2f C  fin %.2f C  variacion %+.2f C  (rango %.2f..%.2f)' % (i, TT[i][0], TT[i][-1], TT[i][-1] - TT[i][0], np.nanmin(TT[i]), np.nanmax(TT[i])))
            dT = TT[1] - TT[0]; print('  diferencia IMU1-IMU0: media %+.2f C  variacion %+.2f C' % (np.nanmean(dT), dT[-1] - dT[0]))
            x = np.arange(nb) * 1.0
            def rep(name, y, T):
                ok = np.isfinite(T) & np.isfinite(y)
                if ok.sum() < 4 or np.ptp(T[ok]) < 0.3 or np.ptp(y[ok]) == 0: print('  %-40s (temperatura casi constante: sin informacion)' % name); return
                r = np.corrcoef(T[ok], y[ok])[0, 1]; p = np.polyfit(T[ok], y[ok], 1); print('  %-40s r = %+.2f   pendiente %+.4g por grado C' % (name, r, p[0]))
            Tm = (TT[0] + TT[1]) / 2
            print('Correlacion con la temperatura media de las dos IMUs (por minuto):')
            rep('rumbo magnetico IMU1-IMU0 (deg)', dh, Tm)
            rep('guinada del nucleo (deg)', res['nucleo'][1][:nb, 2], Tm)
            rep('sesgo relativo gz (dps)', G[1][:, 2] - G[0][:, 2], dT if np.ptp(dT) > 0.3 else Tm)
            for i in (0, 1): rep('sesgo gz IMU%d (dps)' % i, G[i][:, 2], TT[i]); rep('|mag| IMU%d (cuentas)' % i, MM[i], TT[i])
            # el tiempo tambien correlaciona con todo: se compara contra una rampa lineal para avisar
            print('  (aviso: la temperatura casi siempre sube con el tiempo; una correlacion alta no distingue causa de simple deriva monotona.\n   Lo informativo es que cambie la pendiente al calentar/enfriar a proposito)')
        zc = res['nucleo'][1][:, 2][:nb]; n = min(len(zc), len(dh))
        if n > 3: print('  correlacion (guinada del nucleo, 1 min) vs (rumbo magnetico IMU1-IMU0): r = %+.2f (el signo depende de la convencion de ejes; importa |r|)' % np.corrcoef(zc[:n], dh[:n])[0, 1])
    if a.png:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
        for name, (tt, blk) in res.items(): ax[0].plot((tt[0] - a.tare_at + 60 * np.arange(len(blk)) + 30) / 60, blk[:, 2], label=name, lw=1.2)
        ax[0].set_ylabel('guinada relativa, media 1 min (deg)'); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
        if nb >= 3:
            x = np.arange(nb) + .5; ax[1].plot(x, dh, label='rumbo magnetico IMU1-IMU0'); ax[1].plot(x, res['nucleo'][1][:nb, 2] - res['nucleo'][1][0, 2], label='guinada del nucleo (rel. al min 0)')
            ax[1].set_ylabel('deg'); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
        ax[1].set_xlabel('minutos'); fig.tight_layout(); fig.savefig(a.png, dpi=160); print('figura:', a.png)

def selftest():
    """Genera un registro sintetico (IMUs quietas con ruido + campo magnetico), repite el filtro y comprueba la validacion."""
    import tempfile, os
    rng = np.random.default_rng(3); n = 36000; fs = 1; gs = 17.5e-3 * 0.017453293
    # 'nucleo' sintetico = el mismo modelo con las mismas entradas => la validacion debe dar ~0
    path = os.path.join(tempfile.mkdtemp(), 'syn.csv'); rows = []
    ahrs = make(np.float32); f32 = np.float32; q = {0: [f32(v) for v in (-1, 0, 0, .2)], 1: [f32(v) for v in (-1, 0, 0, .2)]}
    for k in range(n):
        for imu in (0, 1):
            w = np.array([rng.normal(15, 6), rng.normal(10, 6), rng.normal(60, 6), rng.normal(0, 30), rng.normal(0, 30), rng.normal(16384, 30),
                          rng.normal(3000 + (60 * math.sin(k / 3000.0) if imu == 1 else 0), 40), rng.normal(1000, 40), rng.normal(-4000, 40)]).astype(int)
            x = [f32(w[j] * (gs if j < 3 else 1 / 16384)) for j in range(9)]
            q[imu] = ahrs(q[imu], *x, f32(1.0 if k * .0083 < 3 else .1), f32(.0083))
            rows.append('%.4f,%d,1,%d,%.6f,%s,%s' % (k * .0083, imu, k + 1000, k * .0083 + imu * .002, ','.join(map(str, w)), ','.join('%.8g' % float(v) for v in q[imu])))
    with open(path, 'w') as f:
        f.write('# RAW,fmt=A55B,n=2,odr_hz=119,fs_g=1,dt_ns=8300000/8300000/0/0,beta_start=1.0,beta_run=0.1,beta_start_ms=3000\nhost_s,imu,flags,cnt,t_s,w0,w1,w2,w3,w4,w5,w6,w7,w8,q0,q1,q2,q3\n' + '\n'.join(rows) + '\n')
    sys.argv = ['x', path, '--tare-at', '6', '--betas', '0.3']; main()

if __name__ == "__main__": main()
