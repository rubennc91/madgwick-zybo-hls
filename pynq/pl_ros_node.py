#!/usr/bin/env python3
"""Nodo ROS 2 (rclpy) nativo en la Zybo con PYNQ: lee los cuaterniones de la cadena en la PL (Integration_Zybo_multi)
y publica lo mismo que la version micro-ROS:
  /madgwick/q         geometry_msgs/QuaternionStamped, frame_id imu0..imuN-1, QoS best effort
  /madgwick/cpu_free  std_msgs/Float32 (% de CPU libre de la placa, de /proc/stat; no es el mismo metodo que el bucle de reposo de FreeRTOS)

Uso (como root, con las placas QUIETAS los primeros ~8 s, que es lo que tarda el arranque del filtro):
  python3 pl_ros_node.py --ros-args -p n_imu:=2 -p odr_hz:=119 -p pub_hz:=10.0
Necesita zybo_multi.bit y zybo_multi.hwh en la misma carpeta (parametro 'bit').
"""
import struct
import time

import numpy as np
import rclpy
from geometry_msgs.msg import QuaternionStamped
from pynq import MMIO, Overlay
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32

NAV_BASES = [0x40000000 + 0x10000 * i for i in range(4)]
MAD_BASE, R2F_BASE = 0x40040000, 0x40050000
NAV_CTRL, NAV_ODR_CFG, NAV_RANGE_CFG, NAV_STATUS, NAV_SAMPLE_CNT, NAV_RAW0 = 0x00, 0x04, 0x08, 0x40, 0x44, 0x50
NAV_OFF_G, NAV_OFF_M, NAV_AXIS_CFG = 0x20, 0x2C, 0x38
NAV_SNAP_SEQ, NAV_SNAP_TS_GDA = 0x74, 0x78
HLS_AP_CTRL, AP_START, AP_AUTORESTART = 0x00, 1, 0x80
R2F_FS_G_REG = 0x10
MAD_BETA, MAD_RESET, MAD_Q_OUT, MAD_FRAME_CNT, MAD_DT_IMU = 0x10, 0x20, 0x40, 0x80, 0x90
ST_CONFIG_DONE, ST_ERROR = 1 << 1, 1 << 4
FS_G_SELECTED, AXIS_CFG_PRESET = 1, 0x039
BETA_START, BETA_RUN, BETA_START_MS = 1.0, 0.1, 3000
MAG_PRESET = [[1537, 1372, -2025], [-1952, 2124, -1233], [-2188, 2152, -1273], [1508, 1242, -1704]]
DT_NOM = [8.280e-3, 8.307e-3, 8.480e-3, 8.057e-3]      # a 119 Hz; se dividen por 8 a 952 Hz


def f2u(f): return struct.unpack('<I', struct.pack('<f', f))[0]
def u2f(u): return struct.unpack('<f', struct.pack('<I', u & 0xFFFFFFFF))[0]
def s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


class PlChain:
    """Acceso a la cadena en la PL por registros AXI-Lite (equivalente a main_pl.c)."""

    def __init__(self, bit, n_imu, odr_hz, log):
        self.n, self.odr_hz, self.log = n_imu, odr_hz, log
        self.odr_code = {119: 3, 952: 6}[odr_hz]
        self.dt_div = {119: 1.0, 952: 8.0}[odr_hz]
        self.ol = Overlay(bit)
        self.nav = [MMIO(NAV_BASES[i], 0x1000) for i in range(4)]
        self.mad = MMIO(MAD_BASE, 0x10000)
        self.r2f = MMIO(R2F_BASE, 0x10000)

    def snap(self, i):
        while True:
            s1 = self.nav[i].read(NAV_SNAP_SEQ); tg = self.nav[i].read(NAV_SNAP_TS_GDA)
            if self.nav[i].read(NAV_SNAP_SEQ) == s1:
                return s1, tg

    def start(self):
        n, nav, mad, r2f = self.n, self.nav, self.mad, self.r2f
        for i in range(n):
            nav[i].write(NAV_ODR_CFG, (7 << 6) | (self.odr_code << 3) | self.odr_code)
            nav[i].write(NAV_RANGE_CFG, FS_G_SELECTED & 3)
            nav[i].write(NAV_AXIS_CFG, AXIS_CFG_PRESET)
            for k in range(3):
                nav[i].write(NAV_OFF_G + 4 * k, 0)
                nav[i].write(NAV_OFF_M + 4 * k, MAG_PRESET[i][k] & 0xFFFF)
        r2f.write(R2F_FS_G_REG, FS_G_SELECTED & 3)
        r2f.write(HLS_AP_CTRL, AP_START | AP_AUTORESTART)
        mad.write(MAD_BETA, f2u(BETA_START))
        for i in range(4):
            mad.write(MAD_DT_IMU + 4 * i, f2u(DT_NOM[i] / self.dt_div))
        mad.write(MAD_RESET, (1 << n) - 1)
        mad.write(HLS_AP_CTRL, AP_START | AP_AUTORESTART)
        for i in range(n):
            nav[i].write(NAV_CTRL, 1)
        for i in range(n):
            t0 = time.time()
            while not (nav[i].read(NAV_STATUS) & (ST_CONFIG_DONE | ST_ERROR)):
                if time.time() - t0 > 2:
                    raise RuntimeError('IMU%d: la configuracion no termina' % i)
                time.sleep(0.001)
            if nav[i].read(NAV_STATUS) & ST_ERROR:
                raise RuntimeError('IMU%d: error de configuracion (STATUS=0x%08x)' % (i, nav[i].read(NAV_STATUS)))
        t0 = time.time()
        while not all(mad.read(MAD_FRAME_CNT + 4 * i) for i in range(n)):
            if time.time() - t0 > 2:
                raise RuntimeError('el nucleo no recibe tramas de todas las IMU')
            time.sleep(0.001)
        # periodo real de cada IMU (3 s)
        s0 = [self.snap(i) for i in range(n)]
        time.sleep(3)
        for i in range(n):
            s1 = self.snap(i)
            ds, dc = (s1[0] - s0[i][0]) & 0xFFFFFFFF, (s1[1] - s0[i][1]) & 0xFFFFFFFF
            dt = dc / ds * 1e-8 if ds >= 100 else DT_NOM[i] / self.dt_div
            mad.write(MAD_DT_IMU + 4 * i, f2u(dt))
            self.log('IMU%d: periodo %.3f us (%.1f Hz)' % (i, dt * 1e6, 1 / dt))
        # sesgo del giroscopio con las placas quietas (~2 s)
        for i in range(n):
            acc = np.zeros(3, np.int64); cnt = 0; last = nav[i].read(NAV_SAMPLE_CNT)
            while cnt < 2 * self.odr_hz:
                c = nav[i].read(NAV_SAMPLE_CNT)
                if c == last:
                    continue
                last = c
                acc += [s16(nav[i].read(NAV_RAW0 + 4 * k)) for k in range(3)]
                cnt += 1
            off = (acc // cnt).tolist()
            for k in range(3):
                nav[i].write(NAV_OFF_G + 4 * k, off[k] & 0xFFFF)
            self.log('IMU%d: sesgo giro %s' % (i, off))
        mad.write(MAD_RESET, 0); mad.write(MAD_BETA, f2u(BETA_START))
        time.sleep(BETA_START_MS / 1000.0)
        mad.write(MAD_BETA, f2u(BETA_RUN))

    def read_q(self, i):
        """(w, x, y, z) coherente con frame_cnt; una lectura que choca con una escritura del nucleo se repite."""
        for _ in range(5):
            c1 = self.mad.read(MAD_FRAME_CNT + 4 * i)
            q = [u2f(self.mad.read(MAD_Q_OUT + 16 * i + 4 * k)) for k in range(4)]
            if self.mad.read(MAD_FRAME_CNT + 4 * i) == c1:
                return q
        return None

    def stop(self):
        for i in range(self.n):
            self.nav[i].write(NAV_CTRL, 0)


def cpu_times():
    with open('/proc/stat') as f:
        v = [int(x) for x in f.readline().split()[1:]]
    return v[3] + (v[4] if len(v) > 4 else 0), sum(v)     # idle + iowait, total


class PlRosNode(Node):
    def __init__(self):
        super().__init__('pl_madgwick')
        self.declare_parameter('bit', '/home/xilinx/zybo_multi.bit')
        self.declare_parameter('n_imu', 2)
        self.declare_parameter('odr_hz', 119)
        self.declare_parameter('pub_hz', 10.0)
        self.declare_parameter('stat_s', 5.0)
        n = int(self.get_parameter('n_imu').value)
        self.n = n
        self.get_logger().info('Arrancando la cadena en la PL (placas quietas ~8 s)...')
        self.pl = PlChain(str(self.get_parameter('bit').value), n, int(self.get_parameter('odr_hz').value),
                          self.get_logger().info)
        self.pl.start()
        self.pub_q = self.create_publisher(QuaternionStamped, 'madgwick/q', qos_profile_sensor_data)
        self.pub_cpu = self.create_publisher(Float32, 'madgwick/cpu_free', 10)
        self.n_pub = self.n_bad = 0
        self.idle0, self.tot0 = cpu_times()
        self.create_timer(1.0 / float(self.get_parameter('pub_hz').value), self.on_pub)
        self.create_timer(float(self.get_parameter('stat_s').value), self.on_stat)
        self.get_logger().info('Publicando /madgwick/q y /madgwick/cpu_free')

    def on_pub(self):
        stamp = self.get_clock().now().to_msg()
        for i in range(self.n):
            q = self.pl.read_q(i)
            if q is None:
                self.n_bad += 1
                continue
            m = QuaternionStamped()
            m.header.stamp = stamp
            m.header.frame_id = 'imu%d' % i
            m.quaternion.w, m.quaternion.x, m.quaternion.y, m.quaternion.z = q    # el nucleo entrega (w, x, y, z)
            self.pub_q.publish(m)
            self.n_pub += 1

    def on_stat(self):
        idle, tot = cpu_times()
        d = tot - self.tot0
        free = 100.0 * (idle - self.idle0) / d if d > 0 else 0.0
        self.idle0, self.tot0 = idle, tot
        self.pub_cpu.publish(Float32(data=float(free)))
        self.get_logger().info('CPU libre %.1f %% | publicados %d, incoherentes %d' % (free, self.n_pub, self.n_bad))


def main():
    rclpy.init()
    node = PlRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pl.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
