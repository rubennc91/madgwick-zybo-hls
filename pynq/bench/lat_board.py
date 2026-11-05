#!/usr/bin/env python3
"""Placa: latencia de ida y vuelta de /madgwick/q por el PC (lat_echo.py la devuelve en /bench/echo) y coste de leer q de la PL.
La marca de tiempo del mensaje la pone el nodo de la placa con su propio reloj y el RTT se calcula con ese mismo reloj, asi que no
hace falta sincronizar el reloj del PC. Latencia de ida estimada = RTT / 2 (supone un camino simetrico).
Uso (como root, con el nodo en marcha y lat_echo.py en el PC):  python3 lat_board.py --secs 55 --out lat.csv --label "952 Hz, 50 Hz"
Con --no-read-bench se omite la medida del coste de lectura de la PL."""
import argparse, statistics, struct, time

import rclpy
from geometry_msgs.msg import QuaternionStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def pct(v, p):
    v = sorted(v); k = (len(v) - 1) * p / 100.0; i = int(k); j = min(i + 1, len(v) - 1)
    return v[i] + (v[j] - v[i]) * (k - i)


def read_bench(n_imu=4, reps=2000):
    from pynq import MMIO
    mad = MMIO(0x40040000, 0x10000)
    ts = []
    for _ in range(reps):
        t = time.perf_counter()
        for i in range(n_imu):
            mad.read(0x80 + 4 * i)
            q = [struct.unpack('<f', struct.pack('<I', mad.read(0x40 + 16 * i + 4 * k)))[0] for k in range(4)]
            mad.read(0x80 + 4 * i)
        ts.append((time.perf_counter() - t) * 1e6)
    return statistics.mean(ts), pct(ts, 99), max(ts)


class Lat(Node):
    def __init__(self):
        super().__init__('bench_lat')
        self.t0 = None; self.rows = []
        self.create_subscription(QuaternionStamped, '/bench/echo', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        now = self.get_clock().now().nanoseconds
        st = m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec
        if self.t0 is None:
            self.t0 = now
        self.rows.append(((now - self.t0) / 1e9, m.header.frame_id, (now - st) / 1e6))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=float, default=55); ap.add_argument('--skip', type=float, default=3)
    ap.add_argument('--out', default='lat.csv'); ap.add_argument('--label', default='')
    ap.add_argument('--no-read-bench', action='store_true')
    a = ap.parse_args()
    rb = None
    if not a.no_read_bench:
        try:
            rb = read_bench()
        except Exception as e:
            print('(read-bench omitido: %s)' % e)
    rclpy.init()
    node = Lat()
    end = time.time() + a.secs
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    rows = [r for r in node.rows if r[0] >= a.skip]
    with open(a.out, 'w') as f:
        f.write('t_s,frame_id,rtt_ms\n')
        for t, fid, v in node.rows:
            f.write('%.4f,%s,%.3f\n' % (t, fid, v))
    if not rows:
        print('Sin mensajes en /bench/echo: ¿esta lat_echo.py corriendo en el PC?'); return
    v = [r[2] for r in rows]
    print('== %s ==' % a.label)
    print('mensajes %d | RTT ms: media %.2f, mediana %.2f, p95 %.2f, p99 %.2f, max %.2f | ida estimada (RTT/2) mediana %.2f ms' % (
        len(v), statistics.mean(v), statistics.median(v), pct(v, 95), pct(v, 99), max(v), statistics.median(v) / 2))
    for fid in sorted({r[1] for r in rows}):
        w = [r[2] for r in rows if r[1] == fid]
        print('  %s: n %d, mediana %.2f ms, p99 %.2f ms' % (fid, len(w), statistics.median(w), pct(w, 99)))
    if rb:
        print('lectura de q de la PL (4 IMU, con conversion): media %.0f us, p99 %.0f us, max %.0f us' % rb)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
