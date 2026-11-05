#!/usr/bin/env python3
"""PC: resume el CSV de bench_sub.py. Parte la captura en ejecuciones por los silencios (> 8 s) y las etiqueta con el mismo orden
que bench_matrix.sh (repeticion, condicion, modo). Los valores por defecto coinciden con los de bench_matrix.sh.
Uso: python3 bench_pc_summary.py bench_sub.csv [--conds "952/10 952/50 119/10 119/50"] [--reps 2] [--modes "A B"] [--n-imu 4]"""
import argparse, csv, statistics

ap = argparse.ArgumentParser()
ap.add_argument('csv'); ap.add_argument('--conds', default='952/10 952/50 119/10 119/50')
ap.add_argument('--reps', type=int, default=2); ap.add_argument('--modes', default='A B')
ap.add_argument('--n-imu', type=int, default=4); ap.add_argument('--gap', type=float, default=8.0)
ap.add_argument('--skip', type=float, default=10.0)
a = ap.parse_args()

rows = []
with open(a.csv) as f:
    for r in csv.DictReader(f):
        rows.append((int(r['epoch']), int(r['count']), float(r['max_gap_ms']), [int(r['n%d' % i]) for i in range(4)]))
segs, cur = [], []
for r in rows:
    if cur and r[0] - cur[-1][0] > a.gap:
        segs.append(cur); cur = []
    cur.append(r)
if cur: segs.append(cur)
plan = [(r + 1, c, m) for r in range(a.reps) for c in a.conds.split() for m in a.modes.split()]
print('Ejecuciones encontradas: %d; esperadas: %d%s\n' % (len(segs), len(plan), '' if len(segs) == len(plan) else '  <-- NO COINCIDEN: revisa las etiquetas'))
print('| # | rep | ODR/pub_hz | modo | s | msg/s medios | esperado | pérdida % | min-max msg/s | hueco máx (ms) | msg/s por IMU |'); print('|' + '---|' * 11)
agg = {}
for i, s in enumerate(segs):
    lab = plan[i] if i < len(plan) else ('?', '?/?', '?')
    t0, t1 = s[0][0] + a.skip, s[-1][0] - 1
    w = [r for r in s if t0 <= r[0] <= t1]
    if len(w) < 5:
        print('| %d | %s | %s | %s | %d | pocas filas | | | | | |' % (i + 1, lab[0], lab[1], lab[2], len(s))); continue
    secs = w[-1][0] - w[0][0] + 1
    mean = sum(r[1] for r in w) / secs
    hz = float(lab[1].split('/')[1]) if '/' in lab[1] and lab[1] != '?/?' else 0
    exp = a.n_imu * hz
    per = [sum(r[3][k] for r in w) / secs for k in range(4)]
    loss = (1 - mean / exp) * 100 if exp else 0.0
    print('| %d | %s | %s | %s | %d | %.1f | %.0f | %.2f | %d-%d | %.0f | %s |' % (
        i + 1, lab[0], lab[1], lab[2], secs, mean, exp, loss, min(r[1] for r in w), max(r[1] for r in w),
        max(r[2] for r in w), '/'.join('%.1f' % p for p in per)))
    agg.setdefault(lab[1], []).append((mean, loss, max(r[2] for r in w)))
print('\nPromedio por condición:')
print('| ODR/pub_hz | msg/s medios | pérdida % | hueco máx (ms) |'); print('|---|---|---|---|')
for k, v in agg.items():
    print('| %s | %.1f | %.2f | %.0f |' % (k, statistics.mean(x[0] for x in v), statistics.mean(x[1] for x in v), max(x[2] for x in v)))
