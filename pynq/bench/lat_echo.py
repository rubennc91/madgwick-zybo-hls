#!/usr/bin/env python3
"""PC (WSL2): devuelve cada mensaje de /madgwick/q por /bench/echo con la misma cabecera (misma marca de tiempo de la placa).
Para la medida de latencia (ver lat_board.py). Con RMW_IMPLEMENTATION=rmw_cyclonedds_cpp y CYCLONEDDS_URI exportadas."""
import time

import rclpy
from geometry_msgs.msg import QuaternionStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Echo(Node):
    def __init__(self):
        super().__init__('bench_echo')
        self.n = 0; self.t = time.time()
        self.pub = self.create_publisher(QuaternionStamped, '/bench/echo', qos_profile_sensor_data)
        self.create_subscription(QuaternionStamped, '/madgwick/q', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.pub.publish(m)
        self.n += 1
        if time.time() - self.t >= 10:
            self.get_logger().info('devueltos %d mensajes' % self.n)
            self.t = time.time()


if __name__ == '__main__':
    rclpy.init()
    node = Echo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
