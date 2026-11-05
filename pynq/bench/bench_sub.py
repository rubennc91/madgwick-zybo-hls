#!/usr/bin/env python3
"""PC (WSL2): cuenta los mensajes de /madgwick/q por segundo y el mayor hueco entre mensajes; escribe un CSV mientras corre.
Dejalo corriendo durante todo bench_matrix.sh y paralo con Ctrl-C al terminar. Luego: python3 bench_pc_summary.py bench_sub.csv
Uso: python3 bench_sub.py [salida.csv]    (con RMW_IMPLEMENTATION=rmw_cyclonedds_cpp y CYCLONEDDS_URI, como en la demo)"""
import sys
import time

import rclpy
from geometry_msgs.msg import QuaternionStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Sub(Node):
    def __init__(self, path):
        super().__init__('bench_sub')
        self.f = open(path, 'w', buffering=1)
        self.f.write('epoch,count,max_gap_ms,n0,n1,n2,n3\n')
        self.sec = None; self.cnt = 0; self.per = [0, 0, 0, 0]; self.gap = 0.0; self.last = None
        self.create_subscription(QuaternionStamped, '/madgwick/q', self.cb, qos_profile_sensor_data)

    def flush(self):
        if self.sec is not None and self.cnt:
            self.f.write('%d,%d,%.1f,%d,%d,%d,%d\n' % (self.sec, self.cnt, self.gap * 1e3, *self.per))

    def cb(self, m):
        now = time.time(); s = int(now)
        if s != self.sec:
            self.flush(); self.sec = s; self.cnt = 0; self.per = [0, 0, 0, 0]; self.gap = 0.0
        if self.last is not None:
            self.gap = max(self.gap, now - self.last)
        self.last = now; self.cnt += 1
        try:
            self.per[int(m.header.frame_id[3:])] += 1
        except (ValueError, IndexError):
            pass


def main():
    node = Sub(sys.argv[1] if len(sys.argv) > 1 else 'bench_sub.csv')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.flush()
    node.f.close()


if __name__ == '__main__':
    rclpy.init()
    main()
