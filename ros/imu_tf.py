#!/usr/bin/env python3
"""Visualiza en RViz2 los cuaterniones de /madgwick/q (geometry_msgs/QuaternionStamped, frame_id imu0..imu3).

Publica una transformacion world -> imuN por cada mensaje (la rotacion es el cuaternion recibido; las IMU se separan
'spacing' metros en X solo para verlas) y, si se piden dos IMU, el angulo relativo entre ellas en /hinge/angle_deg (y como texto en /hinge/marker, para RViz2).

  python3 imu_tf.py --ros-args -p imu_a:=imu0 -p imu_b:=imu1 -p hinge_axis:=y

El angulo relativo es la rotacion de imu_b respecto a imu_a proyectada sobre hinge_axis (2*atan2(componente, w)),
en grados y en (-180, 180]. Comprueba que coincide con la definicion del angulo de la bisagra que uses en el analisis.
El orden de los componentes es el del mensaje (x, y, z, w); si la placa gira al reves, vuelve a compilar la aplicacion
con -DQ_W_FIRST=0. Requiere rclpy, tf2_ros y geometry_msgs (incluidos en ros-humble-ros-base).
"""
import math

import rclpy
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


# Caja con una cara de color por eje: +x rojo, -x rojo oscuro, +y verde, -y verde oscuro, +z azul, -z azul oscuro.
FACES = [  # (normal, color)
    ((1, 0, 0), (1.0, 0.1, 0.1)), ((-1, 0, 0), (0.45, 0.0, 0.0)),
    ((0, 1, 0), (0.1, 0.9, 0.1)), ((0, -1, 0), (0.0, 0.4, 0.0)),
    ((0, 0, 1), (0.2, 0.3, 1.0)), ((0, 0, -1), (0.0, 0.0, 0.45)),
]


def box_marker(name, idx, dims, stamp):
    hx, hy, hz = (d / 2.0 for d in dims)
    m = Marker()
    m.header.frame_id = name
    m.header.stamp = stamp
    m.ns = 'imu_box'
    m.id = idx
    m.type = Marker.TRIANGLE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = 1.0
    for n, col in FACES:
        ax = [i for i in range(3) if n[i] != 0][0]
        u, v = [i for i in range(3) if i != ax]
        h = (hx, hy, hz)
        corners = []
        for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            p = [0.0, 0.0, 0.0]
            p[ax] = n[ax] * h[ax]
            p[u] = su * h[u]
            p[v] = sv * h[v]
            corners.append(p)
        for tri in ((0, 1, 2), (0, 2, 3), (0, 2, 1), (0, 3, 2)):  # ambos sentidos: visible desde las dos caras
            for k in tri:
                m.points.append(Point(x=corners[k][0], y=corners[k][1], z=corners[k][2]))
                m.colors.append(ColorRGBA(r=col[0], g=col[1], b=col[2], a=1.0))
    return m


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
        # dynamic_typing: ROS lee 'y' como booleano (YAML 1.1); se acepta y se interpreta como el eje y
        self.declare_parameter('hinge_axis', 'y', ParameterDescriptor(dynamic_typing=True))
        self.parent = self.get_parameter('parent_frame').value
        self.spacing = float(self.get_parameter('spacing').value)
        self.imu_a = self.get_parameter('imu_a').value
        self.imu_b = self.get_parameter('imu_b').value
        axis = self.get_parameter('hinge_axis').value
        axis = 'y' if axis is True else str(axis).lower()
        self.axis = {'x': 0, 'y': 1, 'z': 2}[axis]
        self.declare_parameter('mode', 'axis')   # axis: proyectado sobre hinge_axis; total: angulo total de giro relativo
        self.declare_parameter('tare', False)    # True: el angulo es 0 en el primer par de muestras (pon la bisagra en 0 al arrancar)
        self.mode = str(self.get_parameter('mode').value).lower()
        self.tare = bool(self.get_parameter('tare').value)
        self.declare_parameter('yaw_tare', False)  # True: iguala el rumbo (yaw) de todas las IMU en la primera muestra de cada una
        self.yaw_tare = bool(self.get_parameter('yaw_tare').value)
        self.yaw0 = {}
        self.r0 = None
        self.last_log = 0.0
        self.declare_parameter('box_size', [0.12, 0.08, 0.03])  # largo x, ancho y, alto z de la caja (m)
        self.box_dims = [float(v) for v in self.get_parameter('box_size').value]
        self.bpub = self.create_publisher(Marker, 'imu/box', 10)
        self.last_box = {}
        self.q = {}
        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(Float64, 'hinge/angle_deg', 10)
        self.mpub = self.create_publisher(Marker, 'hinge/marker', 10)   # texto con el angulo para RViz2
        self.create_subscription(QuaternionStamped, 'madgwick/q', self.on_q, qos_profile_sensor_data)

    def on_q(self, msg):
        name = msg.header.frame_id
        q = (msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w)
        n = math.sqrt(sum(c * c for c in q))
        if n < 1e-6:
            return
        q = tuple(c / n for c in q)
        if self.yaw_tare:
            if name not in self.yaw0:
                x, y, z, w = q
                self.yaw0[name] = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            h = -0.5 * self.yaw0[name]
            q = mul((0.0, 0.0, math.sin(h), math.cos(h)), q)
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
        nowb = self.get_clock().now().nanoseconds * 1e-9
        if nowb - self.last_box.get(name, -10.0) > 1.0:   # la caja va fijada al frame imuN: basta reenviarla cada segundo
            self.last_box[name] = nowb
            try:
                idx = int(name[3:])
            except ValueError:
                idx = 0
            self.bpub.publish(box_marker(name, idx, self.box_dims, t.header.stamp))
        if name == self.imu_b and self.imu_a in self.q:
            r = mul(conj(self.q[self.imu_a]), q)
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_log > 2.0:   # diagnostico: cuaternion relativo (x, y, z, w)
                self.last_log = now
                self.get_logger().info('relativo b respecto a a: x=%.3f y=%.3f z=%.3f w=%.3f' % r)
            if self.tare:
                if self.r0 is None:
                    self.r0 = r
                r = mul(conj(self.r0), r)
            if r[3] < 0:
                r = tuple(-c for c in r)
            if self.mode == 'total':
                v = math.sqrt(r[0] ** 2 + r[1] ** 2 + r[2] ** 2)
                ang = math.degrees(2.0 * math.atan2(v, r[3]))
                if r[self.axis] < 0:
                    ang = -ang
            else:
                ang = math.degrees(2.0 * math.atan2(r[self.axis], r[3]))
            ang = (ang + 180.0) % 360.0 - 180.0
            self.pub.publish(Float64(data=ang))
            m = Marker()
            m.header.frame_id = self.parent
            m.header.stamp = t.header.stamp
            m.ns = 'hinge'
            m.id = 0
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = self.spacing * 0.5
            m.pose.position.z = 0.25
            m.pose.orientation.w = 1.0
            m.scale.z = 0.09
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            m.text = '%.1f deg' % ang
            self.mpub.publish(m)


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
