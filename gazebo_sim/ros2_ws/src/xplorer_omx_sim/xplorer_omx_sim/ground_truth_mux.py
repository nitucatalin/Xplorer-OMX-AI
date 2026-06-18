#!/usr/bin/env python3
"""
ground_truth_mux.py — reasambleaza pozele ground-truth in
/sim/dynamic_poses (TFMessage) cu child_frame_id corecte.

De ce exista: in Gazebo Harmonic, /world/.../dynamic_pose/info publica
pozele DOAR cu ID-uri numerice (numele entitatilor lipsesc), deci puntea
gz->ROS livreaza TFMessage cu child_frame_id gol — perceptia nu poate
distinge robotul de obiecte. In schimb, fiecare model are un
OdometryPublisher pe topic cu NUME FIX (/model/<nume>/ground_truth),
bridge-uite in /sim/gt/*. Acest nod le aduna si publica /sim/dynamic_poses
exact in formatul asteptat de go_collect / manipulation_infer_node /
mission_multi_poi / sim_doctor.

Calibrare automata: OdometryPublisher poate raporta poza absoluta in world
sau relativa la poza de spawn (difera intre versiuni). La primul mesaj al
fiecarei surse, daca citirea difera de pozitia initiala CUNOSCUTA (robotul
spawneaza la POI A; obiectele la pozitiile din pois.yaml), se calculeaza
offset-ul SE(2) o singura data si se aplica ulterior.
"""
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def se2_mul(a, b):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (a[0] + ca * b[0] - sa * b[1],
            a[1] + sa * b[0] + ca * b[1],
            a[2] + b[2])


def se2_inv(a):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (-ca * a[0] - sa * a[1], sa * a[0] - ca * a[1], -a[2])


class GroundTruthMux(Node):
    def __init__(self):
        super().__init__(
            'ground_truth_mux',
            parameter_overrides=[Parameter('use_sim_time', value=True)])

        # pozitiile initiale cunoscute (pentru calibrarea offset-ului)
        self.initial = {'xplorer_omx': (1.0, 0.0, 0.0)}   # spawn = POI A
        try:
            import yaml
            from ament_index_python.packages import get_package_share_directory
            p = os.path.join(get_package_share_directory('xplorer_omx_sim'),
                             'config', 'pois.yaml')
            d = yaml.safe_load(open(p))
            a = d['poi_a']
            self.initial['xplorer_omx'] = (a['x'], a['y'], a['yaw'])
            for i, poi in enumerate(d['pois']):
                self.initial[f'obj_{i}'] = (poi['obj_x'], poi['obj_y'], 0.0)
        except Exception as e:
            self.get_logger().warn(f'pois.yaml indisponibil ({e})')

        self.offset = {}   # nume -> SE2 offset (calibrat la primul mesaj)
        self.latest = {}   # nume -> (x, y, z, yaw)

        sources = (['robot'] + [f'obj_{i}' for i in range(6)] + ['obj_spawn'])
        for src in sources:
            name = 'xplorer_omx' if src == 'robot' else src
            self.create_subscription(
                Odometry, f'/sim/gt/{src}',
                lambda m, n=name: self._on_odom(n, m), 10)

        self.pub = self.create_publisher(TFMessage, '/sim/dynamic_poses', 10)
        self.create_timer(1.0 / 15.0, self._publish)
        self.get_logger().info('ground_truth_mux pornit — /sim/dynamic_poses')

    def _on_odom(self, name, msg):
        p = msg.pose.pose.position
        reading = (p.x, p.y, yaw_of(msg.pose.pose.orientation))
        if name not in self.offset:
            init = self.initial.get(name)
            if init is not None and (math.hypot(reading[0] - init[0],
                                                reading[1] - init[1]) > 0.25):
                # raportare RELATIVA la spawn -> offset = init * reading^-1
                self.offset[name] = init
                self.get_logger().info(
                    f'{name}: ground-truth relativ la spawn — '
                    f'calibrez cu offset {init}')
            else:
                self.offset[name] = (0.0, 0.0, 0.0)
        x, y, yaw = se2_mul(self.offset[name], reading)
        self.latest[name] = (x, y, p.z, yaw)

    def _publish(self):
        if not self.latest:
            return
        out = TFMessage()
        now = self.get_clock().now().to_msg()
        for name, (x, y, z, yaw) in self.latest.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = name
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.z = math.sin(yaw / 2)
            t.transform.rotation.w = math.cos(yaw / 2)
            out.transforms.append(t)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
