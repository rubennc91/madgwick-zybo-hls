#!/usr/bin/env python3
"""Bucle de fondo para medir la CPU que queda libre, con la definicion del paper (VII-H):
velocidad de un bucle de baja prioridad con el nodo en marcha, relativa a la del mismo bucle sin el nodo.
Lanza un proceso por nucleo (afinidad fija, nice 19) que cuenta iteraciones y escribe una fila por segundo y nucleo:
  epoch,core,iters_per_s
Uso: python3 bg_loop.py --secs 35 --out base_start.csv"""
import argparse, multiprocessing as mp, os, time


def worker(core, secs, q):
    os.sched_setaffinity(0, {core})
    os.nice(19)
    t0 = last = time.time()
    n = 0
    while True:
        for _ in range(5000):
            n += 1
        now = time.time()
        if now - last >= 1.0:
            q.put((core, now, n / (now - last)))
            n, last = 0, now
            if now - t0 >= secs:
                break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=float, default=35)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    cores = sorted(os.sched_getaffinity(0))
    q = mp.Queue()
    ps = [mp.Process(target=worker, args=(c, a.secs, q)) for c in cores]
    for p in ps: p.start()
    rows = []
    while any(p.is_alive() for p in ps) or not q.empty():
        try:
            rows.append(q.get(timeout=0.5))
        except Exception:
            pass
    for p in ps: p.join()
    rows.sort(key=lambda r: (r[1], r[0]))
    with open(a.out, 'w') as f:
        f.write('epoch,core,iters_per_s\n')
        for c, t, r in rows:
            f.write('%.3f,%d,%.1f\n' % (t, c, r))
    print('bg_loop: %d filas en %s (%d nucleos)' % (len(rows), a.out, len(cores)))


if __name__ == '__main__':
    main()
