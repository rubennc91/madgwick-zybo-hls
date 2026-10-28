#!/usr/bin/env python3
"""Visor en vivo de 2 IMUs (bisagra / angulo relativo) con el streaming binario de main.c (tecla 's').
En vivo:   python viewer_live.py --port COM5 --baud 921600 [--send s]
Sin placa: python viewer_live.py --demo           (movimiento sintetico para ensayar la ventana)
Con registro CSV: añade --csv salida.csv (una fila por trama; ver plot_drift.py para la grafica)
Prueba del parser: python viewer_live.py --selftest
Teclas en la ventana:  T = tara (angulo 0 con el libro cerrado)   A = calibra el eje de la bisagra: pulsa A, abre y cierra el libro 2-3 veces, pulsa A otra vez
                       R = quita el eje (vuelve a mostrar el angulo total)   Q = salir
Trama: A5 5A | n | seq(u32) | t_us(u32) | n*4 float32 (q0..q3, w primero) | crc8(poly 0x07)
"""
import argparse, struct, sys, time, math
import numpy as np

def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8): c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c

class Parser:
    def __init__(s): s.buf = bytearray(); s.good = 0; s.bad = 0; s.seq_last = None; s.lost = 0
    def feed(s, d):
        s.buf += d; out = []
        while True:
            i = s.buf.find(b'\xA5\x5A')
            if i < 0: s.buf = s.buf[-1:]; break
            if i: del s.buf[:i]
            if len(s.buf) < 3: break
            n = s.buf[2]
            if n < 1 or n > 4: del s.buf[:2]; continue
            L = 2 + 1 + 8 + 16 * n + 1
            if len(s.buf) < L: break
            body = bytes(s.buf[2:L-1])
            if crc8(body) != s.buf[L-1]: s.bad += 1; del s.buf[:2]; continue
            seq, t = struct.unpack('<II', body[1:9])
            q = np.frombuffer(body[9:], '<f4').reshape(n, 4).astype(float)
            if s.seq_last is not None and seq != s.seq_last + 1: s.lost += seq - s.seq_last - 1
            s.seq_last = seq; s.good += 1; del s.buf[:L]
            out.append((seq, t, q))
        return out

def qconj(q): return np.array([q[0], -q[1], -q[2], -q[3]])
def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])
def rel_angle_deg(q0, q1, tare=None):
    """Angulo de rotacion de conj(q0)*q1 respecto a la tara (grados, 0..180)."""
    r = qmul(qconj(q0 / np.linalg.norm(q0)), q1 / np.linalg.norm(q1))
    if tare is not None: r = qmul(qconj(tare), r)
    return math.degrees(2 * math.acos(min(1.0, abs(r[0]))))
def hinge_angle_deg(q0, q1, tare, axis):
    """Angulo (con signo) de la rotacion relativa tarada alrededor del eje de la bisagra (en el sistema de la IMU0)."""
    r = qmul(qconj(tare), rel_q(q0, q1))
    if r[0] < 0: r = -r
    return math.degrees(2 * math.atan2(float(np.dot(r[1:], axis)), r[0]))
def axis_from_samples(vs):
    """Eje de la bisagra: direccion dominante (SVD) de los vectores de rotacion relativa tarada."""
    V = np.array(vs); V = V / np.linalg.norm(V, axis=1, keepdims=True)
    ax = np.linalg.svd(V, full_matrices=False)[2][0]
    return ax if np.dot(ax, V.sum(0)) >= 0 else -ax
TARE_N = 120      # tramas promediadas para la tara (~1 s a 120 Hz)
def avg_quat(qs):
    """Cuaternion medio (autovector dominante de sum q q^T); insensible al signo de cada q."""
    Q = np.array(qs, float); Q = Q / np.linalg.norm(Q, axis=1, keepdims=True)
    v = np.linalg.eigh(Q.T @ Q)[1][:, -1]
    return v if v[0] >= 0 else -v
def rel_q(q0, q1): return qmul(qconj(q0 / np.linalg.norm(q0)), q1 / np.linalg.norm(q1))
def qmat(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

def make_frame(seq, t_us, qs):
    body = bytes([len(qs)]) + struct.pack('<II', seq, t_us) + b''.join(struct.pack('<4f', *q) for q in qs)
    return b'\xA5\x5A' + body + bytes([crc8(body)])

def selftest_axis():
    ax = np.array([0.0, 1.0, 0.0]); tare = np.array([1., 0, 0, 0]); q0 = np.array([1., 0, 0, 0])
    def rot(axv, deg): h = math.radians(deg) / 2; return np.array([math.cos(h), *(math.sin(h) * axv)])
    a1 = hinge_angle_deg(q0, rot(ax, 40), tare, ax)
    a2 = hinge_angle_deg(q0, qmul(rot(np.array([0., 0, 1.]), 50), rot(ax, 40)), tare, ax)   # + yaw espurio de 50 grados
    print(f'bisagra pura {a1:.1f} (40) ; con yaw de 50 grados añadido {a2:.1f} (angulo total daria {rel_angle_deg(q0, qmul(rot(np.array([0.,0,1.]),50), rot(ax,40))):.1f})')
    assert abs(a1 - 40) < 1e-6 and abs(a2 - 40) < 15

def selftest():
    selftest_axis()
    rng = np.random.default_rng(1); q_true = np.array([0.9, 0.1, -0.2, 0.3]); q_true /= np.linalg.norm(q_true)
    qs = [(q_true + rng.normal(0, 0.005, 4)) * rng.choice([-1, 1]) for _ in range(120)]      # ruido y signo aleatorio
    qm = avg_quat(qs); err = math.degrees(2 * math.acos(min(1, abs(float(np.dot(qm, q_true))))))
    print(f'tara promediada: error {err:.3f} grados (una muestra suelta daria ~{math.degrees(2*0.005):.2f})'); assert err < 0.2
    qa = np.array([1., 0, 0, 0]); a = math.radians(70); qb = np.array([math.cos(a/2), math.sin(a/2), 0, 0])
    data = b''.join(make_frame(i, i * 8400, [qa, qb]) for i in range(100))
    data = bytearray(data); data[500] ^= 0xFF; data = bytes(data)      # un byte corrupto
    p = Parser(); fr = []
    for i in range(0, len(data), 37): fr += p.feed(data[i:i+37])         # trozos arbitrarios
    ang = rel_angle_deg(fr[0][2][0], fr[0][2][1])
    print(f'tramas buenas={p.good} malas={p.bad} perdidas={p.lost} angulo={ang:.2f} (esperado 70.00)')
    assert p.good == 99 and p.bad >= 1 and abs(ang - 70) < 0.01

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port'); ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--send', default='s'); ap.add_argument('--demo', action='store_true')
    ap.add_argument('--selftest', action='store_true'); ap.add_argument('--csv'); ap.add_argument('--conj', action='store_true')
    a = ap.parse_args()
    if a.selftest: return selftest()
    from pyqtgraph.Qt import QtCore
    import pyqtgraph as pg, pyqtgraph.opengl as gl
    app = pg.mkQApp('Bisagra 2 IMUs')
    from pyqtgraph.Qt import QtWidgets
    win = QtWidgets.QWidget(); win.setWindowTitle('Madgwick en FPGA - 2 IMUs'); lay = QtWidgets.QGridLayout(win)
    view = gl.GLViewWidget(); view.setCameraPosition(distance=9, elevation=20, azimuth=-70)
    big = QtWidgets.QLabel('-- deg'); big.setStyleSheet('font-size:64px;font-weight:bold')
    info = QtWidgets.QLabel('esperando datos...'); info.setStyleSheet('font-size:14px')
    plot = pg.PlotWidget(); plot.setYRange(0, 180); plot.setLabel('left', 'angulo (deg)'); plot.setLabel('bottom', 't (s)')
    curve = plot.plot(pen=pg.mkPen(width=3))
    lay.addWidget(view, 0, 0, 3, 2); lay.addWidget(big, 0, 2); lay.addWidget(info, 1, 2); lay.addWidget(plot, 2, 2)
    lay.setColumnStretch(0, 3); lay.setColumnStretch(2, 2); win.resize(1300, 560); win.show()
    # que las teclas T/A/R/Q las reciba siempre la ventana (el visor 3D y el grafico no capturan el foco)
    _nf = getattr(QtCore.Qt, 'NoFocus', None) or QtCore.Qt.FocusPolicy.NoFocus
    _sf = getattr(QtCore.Qt, 'StrongFocus', None) or QtCore.Qt.FocusPolicy.StrongFocus
    for w_ in (view, plot, big, info): w_.setFocusPolicy(_nf)
    win.setFocusPolicy(_sf); win.setFocus()
    verts = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-.7, .7) for sz in (-.12, .12)], float)
    faces = np.array([[0,1,3],[0,3,2],[4,5,7],[4,7,6],[0,1,5],[0,5,4],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,3,7],[1,7,5]])
    cols = [np.array([[.2,.5,1,1]]*12), np.array([[1,.5,.2,1]]*12)]
    items = []
    for k in range(2):
        m = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=cols[k], smooth=False, drawEdges=True)
        ax = gl.GLAxisItem(); ax.setSize(1.5, 1.5, 1.5); items.append((m, ax)); view.addItem(m); view.addItem(ax)
    state = {'q': [np.array([1., 0, 0, 0])] * 2, 'tare': None, 'tare_acc': None, 'axis': None, 'cal': None, 'msg': '', 't': [], 'ang': [], 'frames': 0, 't0': time.time()}
    def place(k, q):
        R = qmat(q); R = R.T if a.conj else R
        T = np.eye(4); T[:3, :3] = R; T[0, 3] = (k - 0.5) * 3.4
        tr = pg.Transform3D(T.tolist())
        for it in items[k]: it.setTransform(tr)
    import threading, collections
    ser = None; parser = Parser()
    newq = collections.deque(maxlen=50000)      # (t_s, angulo) producidos por el hilo lector
    stop = threading.Event()
    csvf = None
    if a.csv:
        csvf = open(a.csv, 'w', buffering=1 << 16)
        csvf.write('# viewer_live.py; angle_deg = angulo mostrado (total o bisagra segun mode); tare=1 si hay tara; mode=hinge/total\n')
        csvf.write('host_s,seq,t_us,angle_deg,mode,tare,lost_cum,crc_bad_cum,q0_0,q0_1,q0_2,q0_3,q1_0,q1_1,q1_2,q1_3\n')
    if a.port:
        import serial
        ser = serial.Serial(a.port, a.baud, timeout=0.05)
        try: ser.set_buffer_size(rx_size=4 << 20)
        except Exception: pass
        if a.send: ser.write(a.send.encode())

    def process(seq, t, q):
        """Calcula el angulo y registra la trama. Se ejecuta en el hilo lector (no depende del dibujado)."""
        state['frames'] += 1; state['q'] = [q[0], q[1]]
        if state['tare_acc'] is not None:
            state['tare_acc'].append(rel_q(q[0], q[1]))
            if len(state['tare_acc']) >= TARE_N:
                state['tare'] = avg_quat(state['tare_acc']); state['tare_acc'] = None
                state['msg'] = 'Tara hecha (media de %d tramas)' % TARE_N
        tare, axis = state['tare'], state['axis']
        ang = abs(hinge_angle_deg(q[0], q[1], tare, axis)) if (axis is not None and tare is not None) \
            else rel_angle_deg(q[0], q[1], tare)
        if state['cal'] is not None and tare is not None:
            r = qmul(qconj(tare), rel_q(q[0], q[1]))
            if r[0] < 0: r = -r
            if np.linalg.norm(r[1:]) > math.sin(math.radians(6) / 2): state['cal'].append(r[1:].copy())
        newq.append((t / 1e6, ang))
        if csvf:
            csvf.write('%.4f,%d,%d,%.4f,%s,%d,%d,%d,%s\n' % (time.time() - state['t0'], seq, t, ang,
                       'hinge' if axis is not None else 'total', int(tare is not None),
                       parser.lost, parser.bad, ','.join('%.6f' % v for v in q.flatten())))

    def reader():
        while not stop.is_set():
            d = ser.read(max(1, ser.in_waiting))       # bloquea hasta 50 ms; no depende de la GUI
            if d:
                for seq, t, q in parser.feed(d): process(seq, t, q)
    th = None
    if ser is not None:
        th = threading.Thread(target=reader, daemon=True); th.start()

    def on_key(ev):
        if ev.key() == ord("T") and state['q']:
            state['tare'] = None; state['tare_acc'] = []; state['axis'] = None; state['cal'] = None; state['msg'] = 'Tomando tara (1 s): no muevas las IMUs'
        elif ev.key() == ord("A"):
            if state['tare'] is None: state['msg'] = 'Primero pulsa T con el libro cerrado'
            elif state['cal'] is None:
                state['cal'] = []; state['msg'] = 'CALIBRANDO EJE: abre y cierra el libro 2-3 veces y pulsa A otra vez'
            else:
                n = len(state['cal'])
                if n < 100: state['msg'] = 'Muy poco movimiento (%d muestras): repite, abriendo mas de 20 grados' % n
                else:
                    state['axis'] = axis_from_samples(state['cal']); state['msg'] = 'Eje fijado (%d muestras): %s' % (n, np.round(state['axis'], 2))
                state['cal'] = None
        elif ev.key() == ord("R"): state['axis'] = None; state['cal'] = None; state['msg'] = 'Eje quitado'
        elif ev.key() == ord("Q"): app.quit()
    win.keyPressEvent = on_key

    def tick():
        if ser is None and a.demo:
            tt = time.time() - state['t0']; ang = math.radians(50 + 50 * math.sin(tt * 0.8))
            process(state['frames'], int(tt * 1e6), np.array([[1, 0, 0, 0], [math.cos(ang/2), math.sin(ang/2), 0, 0]], float))
        n = 0
        while newq:
            t, ang = newq.popleft(); state['t'].append(t); state['ang'].append(ang); n += 1
        if n:
            state['t'] = state['t'][-600:]; state['ang'] = state['ang'][-600:]
            big.setText(f'{state["ang"][-1]:.1f} deg'); curve.setData(state['t'], state['ang'])
            for k in range(2): place(k, state['q'][k])
            info.setText(f'{state["msg"]}\ntramas {parser.good if ser else state["frames"]}  perdidas {parser.lost}  CRC malos {parser.bad}\nT = tara  A = eje bisagra  R = sin eje  Q = salir' + ('\nMODO BISAGRA (solo el eje)' if state['axis'] is not None else '\nangulo total (sin eje fijado)'))
    tm = QtCore.QTimer(); tm.timeout.connect(tick); tm.start(33)      # el dibujado va a ~30 Hz; la lectura, aparte
    for k in range(2): place(k, state['q'][k])
    app.exec() if hasattr(app, 'exec') else app.exec_()
    stop.set()
    if th: th.join(1.0)
    if csvf: csvf.close()
if __name__ == '__main__': main()
