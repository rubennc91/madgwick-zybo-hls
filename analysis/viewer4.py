#!/usr/bin/env python3
"""Visor 3D de 4 IMUs (cuaterniones del nucleo Madgwick en FPGA).
Modo replay (sin placa):  python viewer4.py --log data/multi_3vueltas.log [--speed 1] [--conj]
Requiere: numpy, pyqtgraph, PyOpenGL, PyQt5/PySide6.  --check: solo parsea y valida (sin ventana).
Convencion: q = (w,x,y,z). Si el cubo gira al reves, usar --conj.
"""
import argparse, struct, sys, time
import numpy as np

def f32(h): return struct.unpack('>f', bytes.fromhex(h.zfill(8)))[0]

def load(path):
    rows = []
    for ln in open(path, errors='ignore'):
        if not ln or not ln[0].isdigit(): continue
        p = ln.strip().split(',')
        if len(p) < 18: continue
        try:
            i = int(p[0]); t = int(p[3]); q = [f32(h) for h in p[13:17]]
        except ValueError: continue
        rows.append((t, i, q))
    return rows

def qmat(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

def euler(q):
    w, x, y, z = q / np.linalg.norm(q)
    r = np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
    p = np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))
    ya = np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    return r, p, ya

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', required=True); ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--conj', action='store_true'); ap.add_argument('--check', action='store_true')
    a = ap.parse_args()
    rows = load(a.log)
    n = len(rows); print(f'{n} filas, IMUs {sorted(set(r[1] for r in rows))}')
    if a.check:
        last = {}
        for t, i, q in rows: last[i] = q
        for i, q in sorted(last.items()):
            print(i, 'norma=%.6f' % np.linalg.norm(q), 'rpy=%.1f %.1f %.1f' % euler(np.array(q)))
        return
    from pyqtgraph.Qt import QtWidgets, QtCore
    import pyqtgraph as pg, pyqtgraph.opengl as gl
    app = pg.mkQApp('4 IMUs')
    w = gl.GLViewWidget(); w.setWindowTitle('Madgwick FPGA - 4 IMUs'); w.resize(1100, 450)
    w.setCameraPosition(distance=14); w.show()
    verts = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-.6, .6) for sz in (-.2, .2)], float)
    faces = np.array([[0,1,3],[0,3,2],[4,5,7],[4,7,6],[0,1,5],[0,5,4],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,3,7],[1,7,5]])
    cols = np.array([[.2,.5,1,1]]*12)
    items = []
    for k in range(4):
        m = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=cols, smooth=False, drawEdges=True)
        ax = gl.GLAxisItem(); ax.setSize(1.6, 1.6, 1.6)
        items.append((m, ax)); w.addItem(m); w.addItem(ax)
    cur = [np.array([1., 0, 0, 0])]*4
    def place(k, q):
        R = qmat(q)
        if a.conj: R = R.T
        T = np.eye(4); T[:3, :3] = R; T[0, 3] = (k - 1.5) * 3.2
        tr = pg.Transform3D(T.tolist())
        for it in items[k]: it.setTransform(tr)
    for k in range(4): place(k, cur[k])
    state = {'i': 0, 't0': time.time(), 'ts0': rows[0][0]}
    def tick():
        el = (time.time() - state['t0']) * a.speed * 1e6 + state['ts0']
        while state['i'] < n and rows[state['i']][0] <= el:
            t, k, q = rows[state['i']]; cur[k] = np.array(q); state['i'] += 1
        if state['i'] >= n: state['i'] = 0; state['t0'] = time.time()
        for k in range(4): place(k, cur[k])
        w.setWindowTitle('  '.join('IMU%d R%.0f P%.0f Y%.0f' % ((k,) + euler(cur[k])) for k in range(4)))
    tm = QtCore.QTimer(); tm.timeout.connect(tick); tm.start(16)
    app.exec_() if hasattr(app, 'exec_') else app.exec()
main()
