#!/usr/bin/env python3
"""Visualiza en RViz2 los cuaterniones de /madgwick/q (geometry_msgs/QuaternionStamped, frame_id imu0..imu3).

Publica una transformacion world -> imuN por cada mensaje (la rotacion es el cuaternion recibido; las IMU se separan
'spacing' metros en X solo para verlas) y, si se piden dos IMU, el angulo relativo entre ellas en /hinge/angle_deg.

  python3 imu_tf.py --ros-args -p imu_a:=imu0 -p imu_b:=imu1 -p hinge_axis:=y

El angulo relativo es la rotacion de imu_b respecto a imu_a proyectada sobre hinge_axis (2*atan2(componente, w)),
en grados y en (-180, 180]. Comprueba que coincide con la definicion del angulo de la bisagra que uses en el analisis.
El orden de los componentes es el del mensaje (x, y, z, w); si la placa gira al reves, vuelve a compilar la aplicacion
con -DQ_W_FIRST=0. Requiere rclpy, tf2_ros y geometry_msgs (incluidos en ros-humble-ros-base).
"""
import math

import rclpy
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


def conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def mul(a, b):  # producto de Hamilton, (x, y, z, w)
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


class ImuTf(Node):
    def __init__(self):
        super().__init__('imu_tf')
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('spacing', 0.25)
        self.declare_parameter('imu_a', 'imu0')
        self.declare_parameter('imu_b', 'imu1')
        self.declare_parameter('hinge_axis', 'y')
        self.parent = self.get_parameter('parent_frame').value
        self.spacing = float(self.get_parameter('spacing').value)
        self.imu_a = self.get_parameter('imu_a').value
        self.imu_b = self.get_parameter('imu_b').value
        self.axis = {'x': 0, 'y': 1, 'z': 2}[self.get_parameter('hinge_axis').value]
        self.q = {}
        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(Float64, 'hinge/angle_deg', 10)
        self.create_subscription(QuaternionStamped, 'madgwick/q', self.on_q, qos_profile_sensor_data)

    def on_q(self, msg):
        name = msg.header.frame_id
        q = (msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w)
        n = math.sqrt(sum(c * c for c in q))
        if n < 1e-6:
            return
        q = tuple(c / n for c in q)
        self.q[name] = q
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()  # reloj local: evita depender de la sincronizacion con el agente
        t.header.frame_id = self.parent
        t.child_frame_id = name
        try:
            t.transform.translation.x = self.spacing * int(name[3:])
        except ValueError:
            pass
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.br.sendTransform(t)
        if name == self.imu_b and self.imu_a in self.q:
            r = mul(conj(self.q[self.imu_a]), q)
            ang = math.degrees(2.0 * math.atan2(r[self.axis], r[3]))
            ang = (ang + 180.0) % 360.0 - 180.0
            self.pub.publish(Float64(data=ang))


def main():
    rclpy.init()
    node = ImuTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
