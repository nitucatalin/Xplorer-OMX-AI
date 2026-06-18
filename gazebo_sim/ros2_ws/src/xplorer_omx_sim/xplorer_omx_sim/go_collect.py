#!/usr/bin/env python3
"""
go_collect.py — comanda SIMPLA pentru scenariul de baza:

  1. ii dai pozitia OBIECTULUI (sau direct poza de parcare)
  2. platforma navigheaza cu Nav2 si PARCHEAZA CU SPATELE la obiect
     (bratul, montat cu fata in spate, ramane cu obiectul in raza lui)
  3. la atingerea goal-ului: mesaj "AM AJUNS LA POI" + /manip_trigger
  4. nodul de manipulare vede obiectul (pozele din Gazebo = camera),
     porneste episodul, il apuca si il pune in cutia de LANGA brat
  5. /manip_done -> verdict; optional revine la --home

Utilizare (dupa sim + nav2 + manip pornite):
  # pozitia obiectului de pe harta (ia-o din config/pois.yaml sau Gazebo):
  ros2 run xplorer_omx_sim go_collect --obiect 2.79 0.90

  # sau direct poza de parcare (x y yaw):
  ros2 run xplorer_omx_sim go_collect --poi 2.71 0.53 -2.0

  # cu revenire la punctul de start dupa colectare:
  ros2 run xplorer_omx_sim go_collect --obiect 2.79 0.90 --home 1.0 0.0 0.0

Masina de stari: NAV_TO_POI -> AT_POI -> TRIGGER_MANIP -> WAIT_MANIP_DONE
                 -> (optional NAV_TO_HOME) -> COMPLETE / FAILED
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from action_msgs.msg import GoalStatus
from tf2_msgs.msg import TFMessage

# offsetul punctului de pick in frame-ul robotului (bratul e in spate)
# centrul real dintre clesti la poza PICK (identic cu gen_scene si
# manipulation_infer_node_sim — obiectul pica exact intre lamele)
PICK_OFFSET = (-0.3302, -0.0643)

CUBE_SDF = """<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{name}">
    <pose>{x} {y} 0.05 0 0 0</pose>
    <link name="link">
      <inertial><mass>0.05</mass>
        <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="c"><geometry><box><size>0.028 0.028 0.028</size></box></geometry>
        <surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><box><size>0.028 0.028 0.028</size></box></geometry>
        <material><ambient>0.85 0.1 0.1 1</ambient><diffuse>0.85 0.1 0.1 1</diffuse></material></visual>
    </link>
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_topic>/model/{name}/ground_truth</odom_topic>
      <odom_frame>gt_world</odom_frame>
      <robot_base_frame>{name}_link</robot_base_frame>
      <odom_publish_frequency>10</odom_publish_frequency>
    </plugin>
  </model>
</sdf>
"""


def spawn_cube(x, y):
    """Spawneaza un cub rosu la (x, y). Nume FIX (obj_spawn) ca topicul
    lui de ground-truth sa fie pre-bridge-uit; daca exista deja, il
    teleporteaza la noile coordonate."""
    name = 'obj_spawn'
    sdf = CUBE_SDF.format(name=name, x=x, y=y)
    with tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False) as f:
        f.write(sdf)
        path = f.name
    cmd = ['gz', 'service', '-s', '/world/lab_world/create',
           '--reqtype', 'gz.msgs.EntityFactory',
           '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
           '--req', f'sdf_filename: "{path}", allow_renaming: false']
    r = subprocess.run(cmd, capture_output=True, text=True)
    os.unlink(path)
    ok = 'data: true' in (r.stdout or '')
    if not ok:
        # probabil exista deja dintr-o rulare anterioara -> teleport
        ok = teleport(name, x, y, z=0.05)
    return ok, name


def teleport(name, x, y, z=0.04):
    """Repune un obiect la (x, y) — folosit intre episoadele campaniei."""
    req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{w: 1.0}}')
    cmd = ['gz', 'service', '-s', '/world/lab_world/set_pose',
           '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
           '--timeout', '2000', '--req', req]
    r = subprocess.run(cmd, capture_output=True, text=True)
    return 'data: true' in (r.stdout or '')

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=5)


class GoCollect(Node):
    def __init__(self):
        super().__init__(
            'go_collect',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.manip_ready = False
        self.manip_done_flag = False
        self.manip_done_success = False
        self.manip_result = {}
        self.amcl = None
        self.true_pose = None

        self.objects = {}
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_trigger = self.create_publisher(Bool, '/manip_trigger', RELIABLE_QOS)
        self.pub_n_episodes = self.create_publisher(Int32, '/manip_n_episodes', RELIABLE_QOS)
        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(Bool, '/manip_ready',
                                 lambda m: setattr(self, 'manip_ready', m.data),
                                 RELIABLE_QOS)
        self.create_subscription(Bool, '/manip_done', self._on_done, RELIABLE_QOS)
        self.create_subscription(String, '/manip_result', self._on_result, RELIABLE_QOS)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_amcl, RELIABLE_QOS)
        self.create_subscription(TFMessage, '/sim/dynamic_poses',
                                 self._on_poses, 10)
        self.clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _on_done(self, m):
        self.manip_done_flag = True
        self.manip_done_success = m.data

    def _on_result(self, m):
        try:
            self.manip_result = json.loads(m.data)
        except json.JSONDecodeError:
            pass

    def _on_amcl(self, m):
        p = m.pose.pose.position
        self.amcl = (p.x, p.y)

    def _on_poses(self, m):
        for t in m.transforms:
            if t.child_frame_id == 'xplorer_omx':
                tr = t.transform.translation
                q = t.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                self.true_pose = (tr.x, tr.y, yaw)
            elif t.child_frame_id.startswith('obj_'):
                tr = t.transform.translation
                self.objects[t.child_frame_id] = (tr.x, tr.y)

    # ── alinierea fina (segmentul "aliniere" din teza, cap. 3.5) ─────
    def _rel_object(self):
        """obiectul cel mai apropiat de punctul de pick, in frame-ul
        robotului (din ground-truth). Returneaza (eroare, rel, nume)."""
        if self.true_pose is None or not self.objects:
            return None
        rx, ry, ryaw = self.true_pose
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        best = None
        for n, (ox, oy) in self.objects.items():
            dx, dy = ox - rx, oy - ry
            rel = (c * dx - s * dy, s * dx + c * dy)
            d = math.hypot(rel[0] - PICK_OFFSET[0], rel[1] - PICK_OFFSET[1])
            if best is None or d < best[0]:
                best = (d, rel, n)
        return best

    def fine_align(self, timeout=30.0):
        """Dupa Nav2: corecteaza pozitia bazei din cmd_vel pana cand
        obiectul pica EXACT in centrul clestilor (punctul de pick).
        Roteste pe loc pentru unghiul corect, apoi avanseaza/retrage
        pentru distanta. Returneaza (durata, eroare_inainte, eroare_dupa)."""
        t0 = time.time()
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._rel_object() is not None:
                break
        first = self._rel_object()
        if first is None:
            self.get_logger().warn('Aliniere: nu vad obiectul — sar peste')
            return 0.0, None, None
        err0 = first[0]
        b_nom = math.atan2(PICK_OFFSET[1], PICK_OFFSET[0])
        r_nom = math.hypot(*PICK_OFFSET)
        self.get_logger().info(
            f'Aliniere fina: eroare initiala {err0 * 100:.1f} cm '
            f'(obiect {first[2]})')
        err = err0
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            res = self._rel_object()
            if res is None:
                break
            err, rel, _ = res
            if err < 0.008:
                break
            e_b = math.atan2(math.sin(math.atan2(rel[1], rel[0]) - b_nom),
                             math.cos(math.atan2(rel[1], rel[0]) - b_nom))
            e_r = math.hypot(*rel) - r_nom
            tw = Twist()
            if abs(e_b) > 0.015:
                # rotire pe loc: aduce obiectul la unghiul nominal
                tw.angular.z = max(-0.30, min(0.30, 1.2 * e_b))
            else:
                # obiectul e in spate: prea departe -> mers inapoi
                tw.linear.x = max(-0.06, min(0.06, -0.8 * e_r))
            self.pub_cmd.publish(tw)
        self.pub_cmd.publish(Twist())   # stop
        end = time.time() + 0.8         # asezare
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        res = self._rel_object()
        err1 = res[0] if res else err
        dt = time.time() - t0
        self.get_logger().info(
            f'Aliniere fina terminata in {dt:.1f}s: '
            f'{err0 * 100:.1f} cm -> {err1 * 100:.1f} cm '
            f'(obiect centrat intre clesti)')
        return dt, err0, err1

    # ── robustete (invizibile pentru utilizator) ─────────────────────
    def _prepare_nav(self):
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.true_pose and self.amcl:
            err = math.hypot(self.true_pose[0] - self.amcl[0],
                             self.true_pose[1] - self.amcl[1])
            if err > 0.35:
                x, y, yaw = self.true_pose
                self.get_logger().warn(
                    f'AMCL deviat {err:.2f} m — re-initializez localizarea')
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.pose.position.x = x
                msg.pose.pose.position.y = y
                msg.pose.pose.orientation.z = math.sin(yaw / 2)
                msg.pose.pose.orientation.w = math.cos(yaw / 2)
                cov = [0.0] * 36
                cov[0] = cov[7] = 0.04
                cov[35] = 0.02
                msg.pose.covariance = cov
                self.pub_initialpose.publish(msg)
                end = time.time() + 3
                while time.time() < end:
                    rclpy.spin_once(self, timeout_sec=0.2)
                for cli in (self.clear_global, self.clear_local):
                    if cli.wait_for_service(timeout_sec=1.0):
                        f = cli.call_async(ClearEntireCostmap.Request())
                        rclpy.spin_until_future_complete(self, f, timeout_sec=2.0)

    # ── pasii misiunii ───────────────────────────────────────────────
    def navigate(self, x, y, yaw, label, timeout=120.0):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.get_logger().info(
            f'[{label}] Nav2 -> x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}°')
        fut = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=timeout)
        if not rfut.done():
            gh.cancel_goal_async()
            return False
        return rfut.result().status == GoalStatus.STATUS_SUCCEEDED

    def run(self, poi, home):
        self.get_logger().info('Astept Nav2 si nodul de manipulare...')
        if not self.nav.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('Nav2 indisponibil — porneste nav2.launch.py')
            return {'success': False, 'reason': 'nav2_unavailable'}
        end = time.time() + 30
        while time.time() < end and not self.manip_ready:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not self.manip_ready:
            self.get_logger().error('/manip_ready absent — porneste manip.launch.py')
            return {'success': False, 'reason': 'manip_unavailable'}

        # 1. navigare la POI (parcat cu spatele la obiect)
        self._prepare_nav()
        t0 = time.time()
        if not self.navigate(*poi, 'NAV_TO_POI'):
            self.get_logger().error('Navigarea la POI a esuat')
            return {'success': False, 'reason': 'nav_to_poi_failed',
                    'nav_to_poi_s': round(time.time() - t0, 2)}
        nav_s = time.time() - t0
        self.get_logger().info(f'=== AM AJUNS LA POI in {nav_s:.1f}s ===')

        # 1b. ALINIEREA FINA: corecteaza baza pana cand obiectul e centrat
        #     exact intre clestii gripperului (segmentul "aliniere" din
        #     cap. 3.5), abia apoi declanseaza inferenta
        align_s, align_err0, align_err1 = self.fine_align()

        # 2. trigger inferenta
        self.manip_done_flag = False
        self.manip_result = {}
        self.pub_n_episodes.publish(Int32(data=1))
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.pub_trigger.publish(Bool(data=True))

        # 3. asteapta finalizarea episodului
        t0 = time.time()
        end = time.time() + 90
        while time.time() < end and not self.manip_done_flag:
            rclpy.spin_once(self, timeout_sec=0.2)
        manip_s = time.time() - t0
        if not self.manip_done_flag:
            self.get_logger().error('Timeout manipulare')
            return {'success': False, 'reason': 'manip_timeout',
                    'nav_to_poi_s': round(nav_s, 2),
                    'manip_s': round(manip_s, 2)}
        ok = self.manip_done_success
        eps = self.manip_result.get('episodes') or [{}]
        reason = eps[-1].get('reason', '')
        placed = eps[-1].get('placed_in_box')
        self.get_logger().info(
            f'=== MANIPULARE {"COMPLETA" if ok else "ESUATA"} ({reason}'
            + (f', obiect in cutie: {"DA" if placed else "NU"}'
               if placed is not None else '')
            + f') in {manip_s:.1f}s ===')

        # 4. optional: revenire la home
        home_s = None
        if home is not None:
            self._prepare_nav()
            t0 = time.time()
            if self.navigate(*home, 'NAV_TO_HOME'):
                home_s = time.time() - t0
                self.get_logger().info(
                    f'Revenit la punctul de start in {home_s:.1f}s')

        # 5. log JSON pentru teza (cap. 3.4/3.5: cronometrare segmente)
        from pathlib import Path
        from datetime import datetime
        rec = {
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'poi': list(poi),
            'nav_to_poi_s': round(nav_s, 2),
            'align_s': round(align_s, 2),
            'align_err_before_m': (round(align_err0, 4)
                                   if align_err0 is not None else None),
            'align_err_after_m': (round(align_err1, 4)
                                  if align_err1 is not None else None),
            'manip_s': round(manip_s, 2),
            'nav_to_home_s': round(home_s, 2) if home_s else None,
            'success': ok,
            'placed_in_box': placed,
            'reason': reason,
            'pose_at_poi_amcl': self.amcl,
            'pose_at_poi_true': (list(self.true_pose[:2])
                                 if self.true_pose else None),
            'manip_result': self.manip_result,
        }
        log_dir = Path.home() / 'mission_logs'
        log_dir.mkdir(exist_ok=True)
        out = log_dir / ('go_collect_'
                         + datetime.now().strftime('%Y%m%d_%H%M%S') + '.json')
        with open(out, 'w') as f:
            json.dump(rec, f, indent=2)
        self.get_logger().info(f'Log: {out}')
        return rec


def _load_scene_default():
    """POI-ul, HOME-ul si obiectul din scena generata (config/pois.yaml)."""
    import yaml
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('xplorer_omx_sim'),
                            'config', 'pois.yaml')
    except Exception:
        path = os.path.join(os.path.dirname(__file__), '..',
                            'config', 'pois.yaml')
    with open(path) as f:
        d = yaml.safe_load(f)
    p = d['pois'][0]
    a = d['poi_a']
    return ((p['x'], p['y'], p['yaw']), (a['x'], a['y'], a['yaw']), p)


def main():
    ap = argparse.ArgumentParser(
        description='Scenariul de baza: POI-A -> POI-B (parcat cu spatele '
                    'la obiect) -> goal succeeded -> inferenta -> obiect in '
                    'cutie -> POI-A. Fara argumente foloseste obiectul din '
                    'scena (config/pois.yaml). --runs N = campanie.')
    ap.add_argument('--obiect', nargs=2, type=float, metavar=('X', 'Y'),
                    help='pozitia obiectului pe harta — poza de parcare se '
                         'calculeaza automat (cu spatele la obiect)')
    ap.add_argument('--poi', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                    help='poza de parcare data direct')
    ap.add_argument('--home', nargs=3, type=float, default=None,
                    metavar=('X', 'Y', 'YAW'),
                    help='punct de revenire (default: POI A)')
    ap.add_argument('--spawn', action='store_true',
                    help='spawneaza un cub la coordonatele --obiect inainte '
                         'de a pleca (nu mai depinzi de scena generata)')
    ap.add_argument('--runs', type=int, default=1,
                    help='campanie: cate episoade (default 1); obiectul e '
                         'repozitionat automat intre episoade')
    ap.add_argument('--pause', type=float, default=6.0,
                    help='pauza intre episoade [s] (default 6)')
    ap.add_argument('--label', type=str, default='campania_sim')
    args = ap.parse_args()

    rclpy.init()
    node = GoCollect()

    # scena default (pentru POI/home implicite + resetul obiectului)
    try:
        scene_poi, scene_home, scene_obj = _load_scene_default()
    except Exception:
        scene_poi, scene_home, scene_obj = None, (1.0, 0.0, 0.0), None

    # obiectul: spawnat la cerere / din scena / dat cu --obiect
    obj_name, obj_xy = None, None
    if args.spawn:
        if args.obiect is None:
            print('--spawn cere --obiect X Y')
            sys.exit(1)
        ok, obj_name = spawn_cube(args.obiect[0], args.obiect[1])
        if not ok:
            node.get_logger().error('Spawn esuat')
            sys.exit(1)
        node.get_logger().info(
            f'Cub spawnat la ({args.obiect[0]}, {args.obiect[1]}) [{obj_name}]')
        obj_xy = (args.obiect[0], args.obiect[1])
        time.sleep(1.0)
    elif args.obiect is None and scene_obj is not None:
        obj_name = 'obj_0'
        obj_xy = (scene_obj['obj_x'], scene_obj['obj_y'])

    # POI-ul de parcare
    if args.poi is not None:
        poi = tuple(args.poi)
    elif args.obiect is not None:
        ox, oy = args.obiect
        for _ in range(25):
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.true_pose or node.amcl:
                break
        if node.true_pose:
            ref = node.true_pose
        elif node.amcl:
            ref = (node.amcl[0], node.amcl[1], 0.0)
        else:
            ref = (1.0, 0.0, 0.0)
        psi = math.atan2(ref[1] - oy, ref[0] - ox)
        px = ox - (PICK_OFFSET[0] * math.cos(psi) - PICK_OFFSET[1] * math.sin(psi))
        py = oy - (PICK_OFFSET[0] * math.sin(psi) + PICK_OFFSET[1] * math.cos(psi))
        poi = (px, py, psi)
        obj_xy = (ox, oy)
        node.get_logger().info(
            f'Obiect la ({ox:.2f},{oy:.2f}) -> parchez cu spatele la el, '
            f'la ({px:.2f},{py:.2f}), yaw={math.degrees(psi):.0f}°')
    elif scene_poi is not None:
        poi = scene_poi
        node.get_logger().info(
            f"Obiectul din scena: {scene_obj['object']} la "
            f"({scene_obj['obj_x']}, {scene_obj['obj_y']}) — POI "
            f"({poi[0]}, {poi[1]}, yaw={math.degrees(poi[2]):.0f} grade)")
    else:
        print('Nu gasesc pois.yaml — da --obiect X Y sau --poi X Y YAW')
        sys.exit(1)

    home = tuple(args.home) if args.home else scene_home

    # ── campania: POI -> inferenta -> HOME, repetat --runs ori ──
    from datetime import datetime
    from pathlib import Path
    runs = []
    try:
        for i in range(1, args.runs + 1):
            if args.runs > 1:
                node.get_logger().info(
                    f'=============== EPISODUL {i}/{args.runs} ===============')
            rec = node.run(poi, home)
            rec['run'] = i
            runs.append(rec)
            if i < args.runs:
                if obj_name is not None and obj_xy is not None:
                    ok = teleport(obj_name, obj_xy[0], obj_xy[1])
                    node.get_logger().info(
                        'Obiect repozitionat la POI pentru episodul urmator: '
                        + ('OK' if ok else 'ESEC'))
                node.get_logger().info(f'Pauza {args.pause}s...')
                time.sleep(args.pause)
    except KeyboardInterrupt:
        node.get_logger().warn('Ctrl+C — opresc campania')
    finally:
        if runs and args.runs > 1:
            n_ok = sum(1 for r in runs if r.get('success'))
            n_box = sum(1 for r in runs if r.get('placed_in_box'))
            avg = lambda k: round(
                sum(r.get(k) or 0 for r in runs) / max(len(runs), 1), 2)
            summary = {
                'label': args.label,
                'timestamp': datetime.now().isoformat(timespec='seconds'),
                'poi': list(poi), 'home': list(home),
                'object': obj_name, 'n_runs': len(runs), 'runs': runs,
                'totals': {
                    'n_success': n_ok,
                    'n_placed_in_box': n_box,
                    'success_rate': round(n_ok / max(len(runs), 1), 3),
                    'placed_rate': round(n_box / max(len(runs), 1), 3),
                    'avg_nav_to_poi_s': avg('nav_to_poi_s'),
                    'avg_align_s': avg('align_s'),
                    'avg_manip_s': avg('manip_s'),
                    'avg_nav_to_home_s': avg('nav_to_home_s'),
                },
            }
            log_dir = Path.home() / 'mission_logs'
            log_dir.mkdir(exist_ok=True)
            out = log_dir / (f'campania_{args.label}_'
                             + datetime.now().strftime('%Y%m%d_%H%M%S')
                             + '.json')
            with open(out, 'w') as f:
                json.dump(summary, f, indent=2)
            t = summary['totals']
            node.get_logger().info('=' * 60)
            node.get_logger().info(
                f"CAMPANIE TERMINATA — {t['n_success']}/{len(runs)} episoade "
                f"complete, {t['n_placed_in_box']}/{len(runs)} obiecte in cutie")
            node.get_logger().info(
                f"Medii: nav={t['avg_nav_to_poi_s']}s "
                f"manip={t['avg_manip_s']}s home={t['avg_nav_to_home_s']}s")
            node.get_logger().info(f'Raport: {out}')
            node.get_logger().info('=' * 60)
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if runs and all(r.get('success') for r in runs) else 1)


if __name__ == '__main__':
    main()
