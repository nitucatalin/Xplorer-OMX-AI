#!/usr/bin/env python3
"""
mission_multi_poi.py — orchestrator pentru scenariul multi-POI cu colectare.

Scenariu (mirror al misiunii reale, extins la mai multe obiecte):
  Robotul pleaca din POI A (start). Pentru fiecare POI din lant (B, C, D...):
    1. Nav2 -> POI-ul obiectului (robotul opreste cu obiectul in punctul
       de pick al bratului, care e montat cu fata spre spate)
    2. /manip_trigger -> bratul culege obiectul si il pune in CUTIA de pe
       platforma; asteapta /manip_done
    3. SUCCES  -> merge mai departe la urmatorul POI
       ESEC    -> se INTOARCE LA POI A si reincearca acelasi POI
                  (pana la --max-retries reincercari; apoi treci mai departe)
  La final revine la POI A cu obiectele colectate in cutie.

Masina de stari per POI:
  NAV_TO_POI -> AT_POI -> TRIGGER_MANIP -> WAIT_MANIP_DONE
     -> (succes)  NEXT_POI
     -> (esec)    NAV_TO_A -> RETRY

Toate datele (timpi per faza, tentative, retry-uri, rata de succes) se
salveaza in JSON pentru capitolele 3.5/3.6.

Utilizare (POI-urile vin din config/pois.yaml, generat de tools/gen_scene.py):
  ros2 run xplorer_omx_sim mission_multi_poi --label campanie_sim
  ros2 run xplorer_omx_sim mission_multi_poi --pois /cale/pois.yaml --max-retries 2
"""
import argparse
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from action_msgs.msg import GoalStatus
from tf2_msgs.msg import TFMessage

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=5)
BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)


class MultiPoiMission(Node):
    def __init__(self, args):
        super().__init__(
            'mission_multi_poi',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args

        self.manip_ready = False
        self.manip_done_flag = False
        self.manip_done_success = False
        self.manip_state = 'UNKNOWN'
        self.manip_result = {}
        self.current_pose = None

        self.pub_trigger = self.create_publisher(Bool, '/manip_trigger', RELIABLE_QOS)
        self.pub_n_episodes = self.create_publisher(Int32, '/manip_n_episodes', RELIABLE_QOS)
        self.pub_abort = self.create_publisher(Bool, '/manip_abort', RELIABLE_QOS)

        self.create_subscription(Bool, '/manip_ready', self._on_ready, RELIABLE_QOS)
        self.create_subscription(Bool, '/manip_done', self._on_done, RELIABLE_QOS)
        self.create_subscription(String, '/manip_status', self._on_status, BEST_EFFORT_QOS)
        self.create_subscription(String, '/manip_result', self._on_result, RELIABLE_QOS)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_amcl, RELIABLE_QOS)

        # ground-truth-ul robotului (supervizor de re-localizare in sim)
        self.true_pose = None
        self.create_subscription(TFMessage, '/sim/dynamic_poses',
                                 self._on_world_poses, 10)
        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # servicii de curatare a costmap-urilor (inainte de reincercari)
        self.clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = Path(args.output_dir) / f'multi_poi_{ts}_{args.label}'
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.records = []

    # ── callbacks ────────────────────────────────────────────────────
    def _on_ready(self, msg):
        self.manip_ready = msg.data

    def _on_done(self, msg):
        self.manip_done_flag = True
        self.manip_done_success = msg.data
        self.get_logger().info(f'[manip] done success={msg.data}')

    def _on_status(self, msg):
        try:
            self.manip_state = json.loads(msg.data).get('state', '?')
        except json.JSONDecodeError:
            pass

    def _on_result(self, msg):
        try:
            self.manip_result = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_amcl(self, msg):
        p = msg.pose.pose.position
        self.current_pose = (round(p.x, 3), round(p.y, 3))

    def _on_world_poses(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == 'xplorer_omx':
                tr = t.transform.translation
                q = t.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                self.true_pose = (tr.x, tr.y, yaw)

    # ── robustete navigare ───────────────────────────────────────────
    def clear_costmaps(self):
        """Curata costmap-urile (sterge obstacolele-fantoma ramase)."""
        for cli, name in ((self.clear_global, 'global'),
                          (self.clear_local, 'local')):
            if cli.wait_for_service(timeout_sec=2.0):
                fut = cli.call_async(ClearEntireCostmap.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        self.get_logger().info('  costmap-uri curatate')

    def relocalize_if_lost(self, threshold=0.35):
        """Supervizor sim: daca AMCL a deviat de la pozitia reala
        (ground-truth Gazebo), republica /initialpose si asteapta
        convergenta. Echivalentul re-initializarii manuale din rviz
        de pe robotul real."""
        for _ in range(10):
            if self.true_pose is not None and self.current_pose is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.3)
        if self.true_pose is None or self.current_pose is None:
            return False
        err = math.hypot(self.true_pose[0] - self.current_pose[0],
                         self.true_pose[1] - self.current_pose[1])
        if err < threshold:
            return False
        x, y, yaw = self.true_pose
        self.get_logger().warn(
            f'  AMCL deviat cu {err:.2f} m de pozitia reala — '
            f're-initializez la ({x:.2f},{y:.2f})')
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
        end = time.time() + 3.0
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
        return True

    # ── actiuni ──────────────────────────────────────────────────────
    def navigate_to(self, x, y, yaw, label=''):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.get_logger().info(
            f'  Nav2 [{label}]: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}°')
        t0 = time.time()
        fut = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, time.time() - t0, 'REJECTED'
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut,
                                         timeout_sec=self.args.nav_timeout)
        if not rfut.done():
            cf = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cf, timeout_sec=5.0)
            return False, time.time() - t0, 'TIMEOUT'
        st = rfut.result().status
        ok = st == GoalStatus.STATUS_SUCCEEDED
        return ok, time.time() - t0, ('SUCCEEDED' if ok else f'STATUS_{st}')

    def run_manipulation(self):
        self.manip_done_flag = False
        self.manip_done_success = False
        self.manip_result = {}
        self.pub_n_episodes.publish(Int32(data=1))
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.pub_trigger.publish(Bool(data=True))
        self.get_logger().info('  -> /manip_trigger = True')
        t0 = time.time()
        end = t0 + self.args.manip_timeout
        while rclpy.ok() and time.time() < end:
            if self.manip_done_flag:
                return self.manip_done_success, time.time() - t0
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().warn('  manip timeout — trimit abort')
        self.pub_abort.publish(Bool(data=True))
        return False, time.time() - t0

    def wait_manip_ready(self, timeout=30.0):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            if self.manip_ready:
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    # ── misiunea ─────────────────────────────────────────────────────
    def run(self, poi_a, pois):
        t_mission = time.time()
        if not self.wait_manip_ready():
            self.get_logger().error('Timeout /manip_ready')
            return
        self.get_logger().info('Astept Nav2...')
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Nav2 indisponibil')
            return
        self.get_logger().info(
            f'Misiune multi-POI: {len(pois)} obiecte, '
            f'max {self.args.max_retries} reincercari/POI')

        collected = 0
        for poi in pois:
            name = poi['name']
            attempt = 0
            success = False
            while attempt <= self.args.max_retries and not success:
                attempt += 1
                rec = {'poi': name, 'object': poi.get('object', '?'),
                       'attempt': attempt,
                       'started_at': datetime.now().isoformat(timespec='seconds')}
                self.get_logger().info(
                    f'=== POI {name} ({poi.get("object","?")}) — '
                    f'tentativa {attempt}/{self.args.max_retries + 1} ===')

                # robustete: re-localizare daca AMCL a deviat + costmap-uri
                # curate inainte de fiecare tentativa
                relocalized = self.relocalize_if_lost()
                if attempt > 1 or relocalized:
                    self.clear_costmaps()
                rec['relocalized'] = relocalized

                self.manip_result = {}
                nav_ok, nav_s, nav_st = self.navigate_to(
                    poi['x'], poi['y'], poi['yaw'], f'POI-{name}')
                rec['nav_to_poi'] = {'ok': nav_ok, 'duration_s': round(nav_s, 2),
                                     'status': nav_st}
                rec['pose_at_poi'] = self.current_pose
                rec['true_pose_at_poi'] = (
                    [round(v, 3) for v in self.true_pose[:2]]
                    if self.true_pose else None)

                manip_ok, manip_s = (False, 0.0)
                if nav_ok:
                    manip_ok, manip_s = self.run_manipulation()
                rec['manipulation'] = {'ok': manip_ok,
                                       'duration_s': round(manip_s, 2),
                                       'result': (dict(self.manip_result)
                                                  if nav_ok else {})}
                success = nav_ok and manip_ok
                rec['success'] = success

                if not success and attempt <= self.args.max_retries:
                    self.get_logger().warn(
                        f'  POI {name} esuat — ma intorc la POI A si reincerc')
                    self.relocalize_if_lost()
                    self.clear_costmaps()
                    a_ok, a_s, a_st = self.navigate_to(
                        poi_a['x'], poi_a['y'], poi_a['yaw'], 'POI-A(retry)')
                    rec['return_to_a'] = {'ok': a_ok,
                                          'duration_s': round(a_s, 2),
                                          'status': a_st}
                self.records.append(rec)
                self._save(t_mission, collected, final=False)

            if success:
                collected += 1
                self.get_logger().info(
                    f'  POI {name}: obiect colectat in cutie '
                    f'({collected}/{len(pois)})')
            else:
                self.get_logger().error(
                    f'  POI {name}: abandonat dupa {attempt} tentative')

        self.get_logger().info('Toate POI-urile parcurse — revin la POI A')
        self.relocalize_if_lost()
        self.clear_costmaps()
        a_ok, a_s, a_st = self.navigate_to(
            poi_a['x'], poi_a['y'], poi_a['yaw'], 'POI-A(final)')
        self.records.append({'poi': 'A', 'final_return': True,
                             'nav': {'ok': a_ok, 'duration_s': round(a_s, 2),
                                     'status': a_st}})
        self._save(t_mission, collected, final=True)

    def _save(self, t_mission, collected, final):
        attempts = [r for r in self.records if 'attempt' in r]
        n_pois = len({r['poi'] for r in attempts})
        ok_pois = {r['poi'] for r in attempts if r.get('success')}
        navs = [r['nav_to_poi']['duration_s'] for r in attempts
                if r['nav_to_poi']['ok']]
        manips = [r['manipulation']['duration_s'] for r in attempts
                  if r['manipulation']['ok']]
        summary = {
            'label': self.args.label,
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'config': {'max_retries': self.args.max_retries,
                       'nav_timeout_s': self.args.nav_timeout,
                       'manip_timeout_s': self.args.manip_timeout,
                       'pois_file': self.args.pois},
            'attempts': self.records,
            'totals': {
                'n_pois': n_pois,
                'n_collected': collected,
                'n_attempts': len(attempts),
                'n_retries': len(attempts) - n_pois,
                'success_rate_poi': round(len(ok_pois) / max(n_pois, 1), 3),
                'success_rate_attempts': round(
                    sum(1 for r in attempts if r.get('success'))
                    / max(len(attempts), 1), 3),
                'avg_nav_s': round(sum(navs) / max(len(navs), 1), 2),
                'avg_manip_s': round(sum(manips) / max(len(manips), 1), 2),
                'mission_total_s': round(time.time() - t_mission, 2),
                'complete': final,
            },
        }
        out = self.log_dir / 'multi_poi_summary.json'
        with open(out, 'w') as f:
            json.dump(summary, f, indent=2)
        if final:
            t = summary['totals']
            self.get_logger().info('=' * 60)
            self.get_logger().info(
                f"MISIUNE TERMINATA — {t['n_collected']}/{t['n_pois']} obiecte "
                f"colectate, {t['n_retries']} reincercari, "
                f"{t['mission_total_s']}s total")
            self.get_logger().info(f'Raport: {out}')
            self.get_logger().info('=' * 60)


def load_pois(path):
    import yaml
    with open(path) as f:
        d = yaml.safe_load(f)
    return d['poi_a'], d['pois']


def default_pois_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('xplorer_omx_sim'),
                            'config', 'pois.yaml')
    except Exception:
        return str(Path(__file__).resolve().parents[1] / 'config' / 'pois.yaml')


def main():
    ap = argparse.ArgumentParser(description='Misiune multi-POI cu colectare')
    ap.add_argument('--pois', type=str, default=None,
                    help='pois.yaml (default: cel din pachet, generat de gen_scene)')
    ap.add_argument('--max-retries', type=int, default=2,
                    help='reincercari per POI dupa intoarcerea la A (default 2)')
    ap.add_argument('--nav-timeout', type=float, default=120.0)
    ap.add_argument('--manip-timeout', type=float, default=90.0)
    ap.add_argument('--label', type=str, default='multi_poi')
    ap.add_argument('--output-dir', type=str,
                    default=str(Path.home() / 'mission_logs'))
    args = ap.parse_args()
    if args.pois is None:
        args.pois = default_pois_path()
    if not os.path.exists(args.pois):
        print(f'Eroare: nu gasesc {args.pois} — ruleaza tools/gen_scene.py')
        sys.exit(1)
    poi_a, pois = load_pois(args.pois)

    rclpy.init()
    node = MultiPoiMission(args)
    try:
        node.run(poi_a, pois)
    except KeyboardInterrupt:
        node.get_logger().warn('Ctrl+C — abort manipulare')
        node.pub_abort.publish(Bool(data=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
