#!/usr/bin/env python3
"""
capture_pose.py
Capturează poza curentă a robotului din /amcl_pose și o salvează în ~/mission_poses.yaml
sub numele dat (poi sau home).

Utilizare:
  source ~/setup_manip_bridge.bash

  # 1. Conduci robotul cu teleop_twist_keyboard până la locul de pick:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

  # 2. În alt terminal (cu env sursat), capturezi poza ca POI:
  python3 capture_pose.py poi

  # 3. Conduci la HOME, capturezi:
  python3 capture_pose.py home

  # 4. Pornești orchestratorul cu pozele capturate:
  python3 mission_orchestrator.py --from-file ~/mission_poses.yaml --runs 5

Output:
  ~/mission_poses.yaml are mereu cea mai recentă poză capturată pentru fiecare nume.
"""
import argparse
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)


class PoseCapture(Node):
    def __init__(self):
        super().__init__('pose_capture')
        self.last_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._on_pose, RELIABLE_QOS
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.last_pose = (p.x, p.y, yaw)


def parse_args():
    p = argparse.ArgumentParser(
        description='Capturează poza curentă din /amcl_pose în ~/mission_poses.yaml'
    )
    p.add_argument(
        'name',
        choices=['poi', 'home'],
        help='Cum se salvează poza ("poi" sau "home")'
    )
    p.add_argument(
        '--file', type=str,
        default=str(Path.home() / 'mission_poses.yaml'),
        help='Fișier yaml destinație (default ~/mission_poses.yaml)'
    )
    p.add_argument(
        '--timeout', type=float, default=10.0,
        help='Cât așteaptă pentru primul mesaj /amcl_pose (default 10s)'
    )
    return p.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    cap = PoseCapture()

    print(f'Aștept /amcl_pose (până la {args.timeout:.0f}s)...')
    end = time.time() + args.timeout
    while rclpy.ok() and time.time() < end and cap.last_pose is None:
        rclpy.spin_once(cap, timeout_sec=0.2)

    if cap.last_pose is None:
        print('Eroare: niciun mesaj pe /amcl_pose primit.')
        print('Verifică că Nav2 rulează și AMCL are pose inițială (rviz 2D Pose Estimate).')
        cap.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    x, y, yaw = cap.last_pose
    print()
    print(f'Captură [{args.name.upper()}]:')
    print(f'  x   = {x:.4f}')
    print(f'  y   = {y:.4f}')
    print(f'  yaw = {yaw:.4f} rad  ({math.degrees(yaw):.1f}°)')

    # Încarcă fișierul existent dacă există
    try:
        import yaml
    except ImportError:
        print('Eroare: pip install pyyaml')
        sys.exit(1)

    data = {}
    if os.path.exists(args.file):
        with open(args.file) as f:
            data = yaml.safe_load(f) or {}

    data[args.name] = {'x': x, 'y': y, 'yaw': yaw}

    with open(args.file, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False)

    print(f'\nSalvat în {args.file}')

    if 'poi' in data and 'home' in data:
        print('\n✓ Ambele POI și HOME sunt salvate. Poți rula orchestratorul:')
        print(f'  python3 mission_orchestrator.py --from-file {args.file} --runs 5')
    else:
        lipsa = 'home' if 'poi' in data else 'poi'
        print(f'\nMai trebuie capturat: {lipsa.upper()}')
        print(f'  python3 capture_pose.py {lipsa}')

    cap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
