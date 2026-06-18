#!/usr/bin/env python3
"""
sim_doctor.py — verifica IN CONTAINER ca rulezi versiunea corecta si ca
fluxul de pozitii (perceptia) functioneaza. Ruleaza cu simularea pornita:

  ros2 run xplorer_omx_sim sim_doctor

Daca ceva pica, rezolvarea standard e REBUILD CURAT:
  cd ~/ros2_ws && rm -rf build install log
  colcon build --symlink-install && source install/setup.bash
  (apoi reporneste sim + nav2 + manip)
"""
import math
import os
import re
import sys
import time

OK = '\033[92m[OK]\033[0m'
FAIL = '\033[91m[FAIL]\033[0m'
problems = []


def check(name, cond, fix=''):
    print(f'  {OK if cond else FAIL} {name}' + ('' if cond else f'  -> {fix}'))
    if not cond:
        problems.append(name)
    return cond


def main():
    print('\n=== SIM DOCTOR — verificare versiune + flux de date ===\n')

    # ── 1. constantele din pachetul INSTALAT (cel care chiar ruleaza) ──
    from xplorer_omx_sim import go_collect as gc
    from xplorer_omx_sim import manipulation_infer_node_sim as mn
    print(f'  go_collect PICK_OFFSET   = {gc.PICK_OFFSET}')
    print(f'  manip NOMINAL_PICK       = {tuple(mn.NOMINAL_PICK)}')
    print(f'  manip GRIP_OPEN          = {mn.GRIP_OPEN}')
    check('PICK_OFFSET actualizat (-0.3302, -0.0643)',
          abs(gc.PICK_OFFSET[0] + 0.3302) < 1e-3
          and abs(gc.PICK_OFFSET[1] + 0.0643) < 1e-3,
          'rulezi cod VECHI — rebuild curat')
    check('NOMINAL_PICK == PICK_OFFSET',
          abs(mn.NOMINAL_PICK[0] - gc.PICK_OFFSET[0]) < 1e-3
          and abs(mn.NOMINAL_PICK[1] - gc.PICK_OFFSET[1]) < 1e-3,
          'versiuni mixte — rebuild curat')
    check('GRIP_OPEN = 0.95', abs(mn.GRIP_OPEN - 0.95) < 1e-6,
          'nod vechi — rebuild curat')
    check('go_collect are aliniere fina', hasattr(gc.GoCollect, 'fine_align'),
          'cod vechi — rebuild curat')

    # ── 2. fisierele instalate in share ──
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('xplorer_omx_sim')
    urdf = open(os.path.join(share, 'urdf', 'xplorer_omx_real.urdf')).read()
    check('URDF instalat: lamele degete 12 mm',
          urdf.count('<box size="0.012') >= 2, 'URDF vechi — rebuild curat')
    m = re.findall(r'wheel_\w+_joint" type="continuous">.*?<axis xyz="([^"]+)"',
                   urdf, re.S)
    import yaml
    pois = yaml.safe_load(open(os.path.join(share, 'config', 'pois.yaml')))
    poi = pois['pois'][0]
    ox = poi['x'] + (gc.PICK_OFFSET[0] * math.cos(poi['yaw'])
                     - gc.PICK_OFFSET[1] * math.sin(poi['yaw']))
    oy = poi['y'] + (gc.PICK_OFFSET[0] * math.sin(poi['yaw'])
                     + gc.PICK_OFFSET[1] * math.cos(poi['yaw']))
    err = math.hypot(ox - poi['obj_x'], oy - poi['obj_y'])
    print(f'  pois.yaml: POI ({poi["x"]}, {poi["y"]}) obiect '
          f'({poi["obj_x"]}, {poi["obj_y"]}) -> consistenta {err*1000:.1f} mm')
    check('pois.yaml consistent cu PICK_OFFSET (sub 5 mm)', err < 0.005,
          'pois.yaml vechi in install — rebuild curat')

    # ── 3. fluxul de pozitii din Gazebo (perceptia) ──
    print('\n  ... ascult /sim/dynamic_poses timp de 6 s (simularea pe Play!)')
    import rclpy
    from rclpy.node import Node
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    node = Node('sim_doctor')
    seen = set()

    def cb(msg):
        for t in msg.transforms:
            seen.add(t.child_frame_id)
    node.create_subscription(TFMessage, '/sim/dynamic_poses', cb, 10)
    end = time.time() + 6.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    objs = sorted(n for n in seen if n.startswith('obj_'))
    print(f'  entitati vazute: {sorted(seen) if seen else "NIMIC"}')
    check('robotul (xplorer_omx) publica pozitia', 'xplorer_omx' in seen,
          'simularea nu ruleaza / bridge-ul nu e pornit (sim.launch.py)')
    check('obiectul/obiectele publica pozitia', len(objs) >= 1,
          'world vechi fara obiect sau simularea pe pauza')
    node.destroy_node()
    rclpy.shutdown()

    print()
    if problems:
        print(f'{FAIL} {len(problems)} probleme: {problems}')
        print('\nREZOLVARE STANDARD (build curat):')
        print('  cd ~/ros2_ws && rm -rf build install log')
        print('  colcon build --symlink-install && source install/setup.bash')
        print('  apoi reporneste T1 (sim), T2 (nav2), T3 (manip)')
        sys.exit(1)
    print(f'{OK} TOTUL E LA ZI — ruleaza: ros2 run xplorer_omx_sim go_collect')
    sys.exit(0)


if __name__ == '__main__':
    main()
