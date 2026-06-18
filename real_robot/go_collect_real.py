#!/usr/bin/env python3
"""
go_collect_real.py — comanda de campanie END-TO-END pe ROBOTUL REAL (RPi5).

Echivalentul comenzii go_collect din simulare, pentru sistemul fizic:
  POI-A (HOME) -> Nav2 -> POI-B (goal succeeded) -> /manip_trigger ->
  inferenta ACT pe Jetson -> /manip_done -> Nav2 inapoi la HOME,
repetat de --runs ori, cu JSON per rulare + sumar de campanie in acelasi
format ca in simulare (campurile tabelului 3.5: nav_to_poi_s, manip_s,
nav_to_home_s, success, totals cu rate si medii).

Inainte: pe RPi5 ruleaza ./start_all.sh, pe Jetson ./start_jetson.sh,
AMCL initializat (vezi RUNBOOK_J5). Apoi:

  source ~/setup_manip_bridge.bash && source ~/saim_xplorer/install/setup.bash
  python3 ~/go_collect_real.py --poi 2.5 0.5 0.0 --home 1.0 0.0 0.0 \\
          --runs 5 --pause 15 --label campania_reala

  # sau cu pozele salvate de capture_pose.py / --teach anterior:
  python3 ~/go_collect_real.py --from-file ~/mission_poses.yaml --runs 5

INTRE RULARI (pauza --pause): repozitioneaza obiectul la POI, ca in
RUNBOOK_J5 pasul 7.2. Scriptul iti aminteste in terminal.
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
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=5)
BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)


class GoCollectReal(Node):
    def __init__(self, args):
        super().__init__('go_collect_real')
        self.args = args
        self.manip_ready = False
        self.manip_done_flag = False
        self.manip_done_success = False
        self.manip_result = {}
        self.manip_state = 'UNKNOWN'
        self.amcl = None

        self.pub_trigger = self.create_publisher(Bool, '/manip_trigger', RELIABLE_QOS)
        self.pub_n_episodes = self.create_publisher(Int32, '/manip_n_episodes', RELIABLE_QOS)
        self.pub_abort = self.create_publisher(Bool, '/manip_abort', RELIABLE_QOS)
        self.create_subscription(Bool, '/manip_ready',
                                 lambda m: setattr(self, 'manip_ready', m.data),
                                 RELIABLE_QOS)
        self.create_subscription(Bool, '/manip_done', self._on_done, RELIABLE_QOS)
        self.create_subscription(String, '/manip_status', self._on_status,
                                 BEST_EFFORT_QOS)
        self.create_subscription(String, '/manip_result', self._on_result,
                                 RELIABLE_QOS)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_amcl, RELIABLE_QOS)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _on_done(self, m):
        self.manip_done_flag = True
        self.manip_done_success = m.data
        self.get_logger().info(f'[manip] done success={m.data}')

    def _on_status(self, m):
        try:
            d = json.loads(m.data)
            st = d.get('state', '?')
            if st != self.manip_state:
                self.get_logger().info(
                    f'[manip] {self.manip_state} -> {st}')
                self.manip_state = st
        except json.JSONDecodeError:
            pass

    def _on_result(self, m):
        try:
            self.manip_result = json.loads(m.data)
        except json.JSONDecodeError:
            pass

    def _on_amcl(self, m):
        p = m.pose.pose.position
        o = m.pose.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y),
                         1 - 2 * (o.y * o.y + o.z * o.z))
        self.amcl = (round(p.x, 3), round(p.y, 3), round(yaw, 3))

    # ── primitive ───────────────────────────────────────────────────
    def navigate(self, x, y, yaw, label):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.get_logger().info(
            f'[{label}] Nav2 -> x={x:.2f} y={y:.2f} '
            f'yaw={math.degrees(yaw):.0f} grade')
        fut = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, 'REJECTED'
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut,
                                         timeout_sec=self.args.nav_timeout)
        if not rfut.done():
            gh.cancel_goal_async()
            return False, 'TIMEOUT'
        st = rfut.result().status
        return (st == GoalStatus.STATUS_SUCCEEDED,
                {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                 GoalStatus.STATUS_ABORTED: 'ABORTED',
                 GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(st, f'ST_{st}'))

    def run_manipulation(self):
        self.manip_done_flag = False
        self.manip_done_success = False
        self.manip_result = {}
        self.pub_n_episodes.publish(Int32(data=int(self.args.episodes)))
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
        self.get_logger().warn('  timeout manipulare — trimit abort')
        self.pub_abort.publish(Bool(data=True))
        return False, time.time() - t0

    # ── o rulare completa ───────────────────────────────────────────
    def run_once(self, poi, home):
        rec = {'timestamp': datetime.now().isoformat(timespec='seconds'),
               'poi': list(poi), 'home': list(home)}
        t0 = time.time()
        nav_ok, nav_st = self.navigate(*poi, 'NAV_TO_POI')
        rec['nav_to_poi_s'] = round(time.time() - t0, 2)
        rec['nav_to_poi_status'] = nav_st
        # poza AMCL exact la atingerea POI-ului (precizia de pozitionare 3.4)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        rec['pose_at_poi_amcl'] = self.amcl
        if not nav_ok:
            rec.update(success=False, reason='nav_to_poi_failed')
            return rec
        self.get_logger().info(
            f'=== AM AJUNS LA POI in {rec["nav_to_poi_s"]}s '
            f'(AMCL: {self.amcl}) — declansez inferenta ===')

        manip_ok, manip_s = self.run_manipulation()
        rec['manip_s'] = round(manip_s, 2)
        rec['manip_result'] = dict(self.manip_result)
        self.get_logger().info(
            f'=== MANIPULARE {"COMPLETA" if manip_ok else "ESUATA"} '
            f'in {rec["manip_s"]}s ===')

        t0 = time.time()
        home_ok, home_st = self.navigate(*home, 'NAV_TO_HOME')
        rec['nav_to_home_s'] = round(time.time() - t0, 2)
        rec['nav_to_home_status'] = home_st

        rec['success'] = bool(nav_ok and manip_ok and home_ok)
        rec['reason'] = 'completed' if rec['success'] else 'partial'
        return rec


def load_poses_yaml(path):
    import yaml
    with open(path) as f:
        d = yaml.safe_load(f)
    return ([d['poi']['x'], d['poi']['y'], d['poi']['yaw']],
            [d['home']['x'], d['home']['y'], d['home']['yaw']])


def main():
    ap = argparse.ArgumentParser(
        description='Campanie end-to-end pe robotul real: '
                    'POI -> inferenta -> HOME, repetat --runs ori')
    ap.add_argument('--poi', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--home', nargs=3, type=float, default=[1.0, 0.0, 0.0],
                    metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--from-file', type=str, default=None,
                    help='yaml cu poi/home (de la capture_pose.py)')
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--episodes', type=int, default=1)
    ap.add_argument('--pause', type=float, default=15.0,
                    help='pauza intre rulari [s] — REPOZITIONEAZA OBIECTUL!')
    ap.add_argument('--nav-timeout', type=float, default=120.0)
    ap.add_argument('--manip-timeout', type=float, default=90.0)
    ap.add_argument('--label', type=str, default='campania_reala')
    ap.add_argument('--output-dir', type=str,
                    default=str(Path.home() / 'mission_logs'))
    args = ap.parse_args()

    if args.from_file:
        args.poi, args.home = load_poses_yaml(os.path.expanduser(args.from_file))
    if args.poi is None:
        print('Da --poi X Y YAW (sau --from-file ~/mission_poses.yaml)')
        sys.exit(1)

    rclpy.init()
    node = GoCollectReal(args)

    # asteptari initiale
    node.get_logger().info('Astept Nav2 si nodul de manipulare (Jetson)...')
    if not node.nav.wait_for_server(timeout_sec=20.0):
        node.get_logger().error('Nav2 indisponibil — ruleaza start_all.sh')
        sys.exit(1)
    end = time.time() + 40
    while time.time() < end and not node.manip_ready:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.manip_ready:
        node.get_logger().error(
            '/manip_ready absent — ruleaza start_jetson.sh pe Jetson '
            '(sau asteapta discovery-ul DDS ~15s si reia)')
        sys.exit(1)
    node.get_logger().info('Sistem gata — pornesc campania')

    runs = []
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f'campania_{args.label}_{ts}.json'

    def save(final=False):
        n = len(runs)
        ok = sum(1 for r in runs if r.get('success'))
        avg = lambda k: round(sum(r.get(k) or 0 for r in runs) / max(n, 1), 2)
        summary = {
            'label': args.label,
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'platform': 'real_robot',
            'config': {'poi': args.poi, 'home': args.home,
                       'runs': args.runs, 'episodes': args.episodes,
                       'nav_timeout_s': args.nav_timeout,
                       'manip_timeout_s': args.manip_timeout},
            'runs': runs,
            'totals': {'n_runs': n, 'n_success': ok,
                       'success_rate': round(ok / max(n, 1), 3),
                       'avg_nav_to_poi_s': avg('nav_to_poi_s'),
                       'avg_manip_s': avg('manip_s'),
                       'avg_nav_to_home_s': avg('nav_to_home_s'),
                       'complete': final},
        }
        with open(out, 'w') as f:
            json.dump(summary, f, indent=2)
        return summary

    try:
        for i in range(1, args.runs + 1):
            node.get_logger().info(
                f'=============== RULAREA {i}/{args.runs} ===============')
            rec = node.run_once(tuple(args.poi), tuple(args.home))
            rec['run'] = i
            runs.append(rec)
            save()
            if i < args.runs:
                node.get_logger().info(
                    f'>>> PAUZA {args.pause}s — REPOZITIONEAZA OBIECTUL '
                    f'LA POI (ca in RUNBOOK_J5 pasul 7.2) <<<')
                end = time.time() + args.pause
                while time.time() < end:
                    rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        node.get_logger().warn('Ctrl+C — abort manipulare + salvez ce am')
        node.pub_abort.publish(Bool(data=True))
    finally:
        s = save(final=True)
        t = s['totals']
        node.get_logger().info('=' * 60)
        node.get_logger().info(
            f"CAMPANIE: {t['n_success']}/{t['n_runs']} reusite "
            f"(rata {t['success_rate'] * 100:.0f}%)")
        node.get_logger().info(
            f"Medii: nav_POI={t['avg_nav_to_poi_s']}s "
            f"manip={t['avg_manip_s']}s nav_HOME={t['avg_nav_to_home_s']}s")
        node.get_logger().info(f'Raport: {out}')
        node.get_logger().info('=' * 60)
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if runs and all(r.get('success') for r in runs) else 1)


if __name__ == '__main__':
    main()
