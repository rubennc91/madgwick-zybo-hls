#!/usr/bin/env python3
"""Resume la carpeta de salida de bench_matrix.sh: CPU libre (/proc/stat del nodo y bucle de fondo), mensajes/s publicados y lecturas incoherentes.
Uso: python3 bench_summary.py CARPETA"""
import csv, os, re, statistics, sys

SKIP = 10.0    # s desde "Publicando" que se descartan
RE_PUB = re.compile(r'\[(\d+\.\d+)\] \[pl_madgwick\]: Publicando')
RE_STAT = re.compile(r'\[(\d+\.\d+)\] \[pl_madgwick\]: CPU libre ([\d.]+) % \| publicados (\d+), incoherentes (\d+)')


def read_bg(path):
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.setdefault(int(r['core']), []).append((float(r['epoch']), float(r['iters_per_s'])))
    return rows


def mean_rate(rows, t_from=None, skip_first=0):
    per = {}
    for core, v in rows.items():
        if t_from is None:
            t_from_c = v[0][0] + skip_first
        else:
            t_from_c = t_from
        xs = [x for t, x in v if t >= t_from_c]
        if xs:
            per[core] = statistics.mean(xs)
    return per


def main():
    d = sys.argv[1]
    base = []
    for f in ('base_start.csv', 'base_end.csv'):
        p = os.path.join(d, f)
        if os.path.exists(p):
            base.append(mean_rate(read_bg(p), skip_first=3))
    if not base:
        print('Falta la referencia base_*.csv'); sys.exit(1)
    cores = sorted(set.intersection(*[set(b) for b in base]))
    base_sum = [sum(b[c] for c in cores) for b in base]
    base_ref = statistics.mean(base_sum)
    drift = (max(base_sum) - min(base_sum)) / base_ref * 100 if len(base_sum) > 1 else 0.0
    print('Referencia sin nodo: %.0f it/s en %d nucleos (deriva inicio-fin %.1f %%)\n' % (base_ref, len(cores), drift))
    out = []
    with open(os.path.join(d, 'runs.csv')) as f:
        for r in csv.DictReader(f):
            log = open(os.path.join(d, r['node_log']), errors='replace').read()
            m = RE_PUB.search(log)
            res = dict(r)
            if not m:
                res.update(ok=False, note='el nodo no llego a publicar')
                out.append(res); continue
            t_pub = float(m.group(1)); t_from = t_pub + SKIP
            st = [(float(a), float(b), int(c), int(e)) for a, b, c, e in RE_STAT.findall(log) if float(a) >= t_from]
            res['ok'] = len(st) >= 2
            if len(st) >= 2:
                if r['mode'] == 'A':      # con el bucle de fondo (modo B) /proc/stat no mide CPU libre: el bucle se come todo
                    res['cpu_proc'] = statistics.mean(s[1] for s in st)
                res['msg_s'] = (st[-1][2] - st[0][2]) / (st[-1][0] - st[0][0])
                res['incoh'] = st[-1][3]
            else:
                res['note'] = 'pocas lineas de estadistica'
            if r['bg_csv'] and os.path.exists(os.path.join(d, r['bg_csv'])):
                per = mean_rate(read_bg(os.path.join(d, r['bg_csv'])), t_from=t_from)
                if all(c in per for c in cores):
                    res['cpu_bg'] = 100.0 * sum(per[c] for c in cores) / base_ref
            out.append(res)
    hdr = '| run | ODR | pub_hz | modo | CPU libre /proc/stat (%) | CPU libre bucle (%) | msg/s publicados | incoherentes | nota |'
    print(hdr); print('|' + '---|' * 9)
    f2 = lambda x, n=1: ('%.*f' % (n, x)) if x is not None else '-'
    for r in out:
        print('| %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
            r['run'], r['odr'], r['hz'], r['mode'], f2(r.get('cpu_proc')), f2(r.get('cpu_bg')),
            f2(r.get('msg_s')), r.get('incoh', '-'), r.get('note', '')))
    print('\nPromedios por condicion (sobre las repeticiones):')
    print('| ODR | pub_hz | CPU libre /proc/stat (%) | CPU libre bucle (%) | msg/s publicados | incoherentes (suma) |'); print('|' + '---|' * 6)
    keys = sorted({(r['odr'], r['hz']) for r in out}, key=lambda k: (-int(k[0]), int(k[1])))
    for k in keys:
        rs = [r for r in out if (r['odr'], r['hz']) == k and r.get('ok')]
        av = lambda key: f2(statistics.mean(r[key] for r in rs if key in r)) if any(key in r for r in rs) else '-'
        print('| %s | %s | %s | %s | %s | %d |' % (k[0], k[1], av('cpu_proc'), av('cpu_bg'), av('msg_s'), sum(r.get('incoh', 0) for r in rs)))
    with open(os.path.join(d, 'summary.csv'), 'w') as f:
        w = csv.writer(f)
        w.writerow(['run', 'odr', 'hz', 'mode', 'cpu_proc', 'cpu_bg', 'msg_s', 'incoh', 'note'])
        for r in out:
            w.writerow([r['run'], r['odr'], r['hz'], r['mode'], f2(r.get('cpu_proc'), 2), f2(r.get('cpu_bg'), 2), f2(r.get('msg_s'), 2), r.get('incoh', ''), r.get('note', '')])


if __name__ == '__main__':
    main()
