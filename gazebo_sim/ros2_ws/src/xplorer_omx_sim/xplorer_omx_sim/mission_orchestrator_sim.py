#!/usr/bin/env python3
"""
mission_orchestrator_sim.py — VARIANTA PENTRU SIMULAREA GAZEBO
Identic cu mission_orchestrator.py de pe RPi5, cu O SINGURĂ diferență:
nodul rulează cu use_sim_time=True (ceasul /clock din Gazebo), altfel
timestamp-urile goal-urilor Nav2 (timp-perete) nu s-ar potrivi cu TF-ul
simulat și navigarea ar eșua.

Orchestrator end-to-end: Nav2 -> POI -> trigger manipulare -> Nav2 -> HOME.
Comunică cu manipulation_infer_node (în sim: manipulation_infer_node_sim)
prin topicele /manip_*.

Mașina de stări:
  IDLE -> WAIT_MANIP_READY -> NAV_TO_POI -> AT_POI -> TRIGGER_MANIP
       -> WAIT_MANIP_DONE  -> NAV_TO_HOME -> AT_HOME -> COMPLETE
       -> orice -> FAILED / ABORTED (la eroare sau Ctrl+C)

Comunicație cu mașina de stări a manipulation_infer_node:
  Publicare:
    /manip_trigger    Bool   trigger sesiune
    /manip_n_episodes Int32  câte episoade pe sesiune
    /manip_abort      Bool   abort din afară
  Abonare:
    /manip_ready      Bool   braț în home, gata
    /manip_done       Bool   episod terminat
    /manip_status     String JSON heartbeat 2 Hz
    /manip_result     String JSON raport final

Nav2 prin ActionClient pe /navigate_to_pose.

Utilizare:
  source ~/setup_manip_bridge.bash

  # MODUL A — pose-uri date din CLI:
  python3 mission_orchestrator.py \\
      --poi 2.0 1.5 0.0 \\
      --home 0.5 0.0 0.0 \\
      --runs 5

  # MODUL B — pose-uri din click-uri pe rviz (2D Goal Pose):
  #   Pornești cu --teach, faci 2 click-uri pe rviz (POI apoi HOME).
  #   Robotul VA naviga la fiecare click — folosește asta să verifici pozele.
  python3 mission_orchestrator.py --teach --runs 5

  # MODUL C — pose-uri salvate într-un yaml de capture_pose.py:
  python3 mission_orchestrator.py --from-file ~/mission_poses.yaml --runs 5
"""
import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ---------------------------------------------------------------------------
# QoS care matchează manipulation_infer_node (RELIABLE + VOLATILE + depth=5)
# ---------------------------------------------------------------------------
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


# ---------------------------------------------------------------------------
# Stările orchestratorului
# ---------------------------------------------------------------------------
class OState:
    IDLE              = 'IDLE'
    WAIT_MANIP_READY  = 'WAIT_MANIP_READY'
    NAV_TO_POI        = 'NAV_TO_POI'
    AT_POI            = 'AT_POI'
    TRIGGER_MANIP     = 'TRIGGER_MANIP'
    WAIT_MANIP_DONE   = 'WAIT_MANIP_DONE'
    NAV_TO_HOME       = 'NAV_TO_HOME'
    AT_HOME           = 'AT_HOME'
    COMPLETE          = 'COMPLETE'
    FAILED            = 'FAILED'
    ABORTED           = 'ABORTED'


# ---------------------------------------------------------------------------
# Structura pentru un run individual
# ---------------------------------------------------------------------------
@dataclass
class RunMetrics:
    run_id: int
    started_at: str = ''
    ended_at: str = ''
    nav_to_poi_s: float = 0.0
    manip_s: float = 0.0
    nav_to_home_s: float = 0.0
    total_s: float = 0.0
    nav_to_poi_status: str = ''
    manip_success: bool = False
    manip_result_json: dict = field(default_factory=dict)
    nav_to_home_status: str = ''
    final_state: str = ''
    error: str = ''


# ---------------------------------------------------------------------------
# Orchestratorul principal
# ---------------------------------------------------------------------------
class MissionOrchestrator(Node):
    def __init__(self, args):
        super().__init__(
            'mission_orchestrator',
            parameter_overrides=[Parameter('use_sim_time', value=True)])

        self.args = args
        self.state = OState.IDLE
        self.start_time = 0.0

        # ── Stare manipulation_infer_node, citită din /manip_status
        self.manip_state_remote = 'UNKNOWN'
        self.manip_ready = False
        self.manip_done_flag = False
        self.manip_done_success = False
        self.manip_result = {}
        self.last_manip_heartbeat = 0.0

        # ── Pose curentă (din /amcl_pose, pentru logare)
        self.current_pose = None

        # ── Publishers către Jetson
        self.pub_trigger = self.create_publisher(
            Bool, '/manip_trigger', RELIABLE_QOS)
        self.pub_n_episodes = self.create_publisher(
            Int32, '/manip_n_episodes', RELIABLE_QOS)
        self.pub_abort = self.create_publisher(
            Bool, '/manip_abort', RELIABLE_QOS)
        self.pub_go_home = self.create_publisher(
            Bool, '/manip_go_home', RELIABLE_QOS)

        # ── Subscribers de la Jetson
        self.create_subscription(
            Bool, '/manip_ready', self._on_manip_ready, RELIABLE_QOS)
        self.create_subscription(
            Bool, '/manip_done', self._on_manip_done, RELIABLE_QOS)
        self.create_subscription(
            String, '/manip_status', self._on_manip_status, BEST_EFFORT_QOS)
        self.create_subscription(
            String, '/manip_result', self._on_manip_result, RELIABLE_QOS)

        # ── Pose feedback (pentru logare)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._on_amcl_pose, RELIABLE_QOS)

        # ── ActionClient Nav2
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # ── Log directory
        self.session_label = args.label or 'session'
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = Path(args.output_dir) / f'mission_{ts}_{self.session_label}'
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.runs_metrics = []

        # ── Stats globale
        self.total_runs = args.runs
        self.current_run = 0

        self.get_logger().info(
            f'MissionOrchestrator pornit | '
            f'POI=({args.poi[0]:.2f},{args.poi[1]:.2f},{math.degrees(args.poi[2]):.0f}°) '
            f'HOME=({args.home[0]:.2f},{args.home[1]:.2f},{math.degrees(args.home[2]):.0f}°) '
            f'runs={args.runs} episodes={args.episodes} pause={args.pause}s'
        )
        self.get_logger().info(f'Log dir: {self.log_dir}')

    # ════════════════════════════════════════════════════════════════
    # Callbacks de la Jetson
    # ════════════════════════════════════════════════════════════════
    def _on_manip_ready(self, msg: Bool):
        self.manip_ready = msg.data
        self.get_logger().info(f'[manip] ready={msg.data}')

    def _on_manip_done(self, msg: Bool):
        self.manip_done_flag = True
        self.manip_done_success = msg.data
        self.get_logger().info(f'[manip] done success={msg.data}')

    def _on_manip_status(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.manip_state_remote = d.get('state', 'UNKNOWN')
            self.last_manip_heartbeat = time.time()
        except json.JSONDecodeError:
            pass

    def _on_manip_result(self, msg: String):
        try:
            self.manip_result = json.loads(msg.data)
            self.get_logger().info(f'[manip] result={self.manip_result}')
        except json.JSONDecodeError:
            pass

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = self._quat_to_yaw(o.x, o.y, o.z, o.w)
        self.current_pose = (p.x, p.y, yaw)

    # ════════════════════════════════════════════════════════════════
    # Mașina de stări — rulare
    # ════════════════════════════════════════════════════════════════
    def run_mission(self):
        """Bucla principală. Rulează args.runs misiuni complete."""
        self.start_time = time.time()

        # 1. Așteaptă manip_ready=True (Jetson a făcut home la startup)
        self._set_state(OState.WAIT_MANIP_READY)
        if not self._wait_manip_ready(timeout=30.0):
            self.get_logger().error('Timeout așteptând manip_ready de la Jetson')
            self._set_state(OState.FAILED)
            return

        # 2. Așteaptă Nav2 server (un singur dată)
        self.get_logger().info('Aștept Nav2 action server...')
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Nav2 /navigate_to_pose nu răspunde')
            self._set_state(OState.FAILED)
            return
        self.get_logger().info('Nav2 action server gata')

        # 3. Loop pe runuri
        for run_idx in range(self.total_runs):
            self.current_run = run_idx + 1
            ok = self._run_one_mission(run_idx)
            if not ok:
                self.get_logger().warn(
                    f'Run {run_idx + 1}/{self.total_runs} eșuat — continuă oricum'
                )

            # Pauză între rulări (nu după ultima)
            if run_idx < self.total_runs - 1:
                self.get_logger().info(
                    f'Pauză {self.args.pause}s între rulări...'
                )
                time.sleep(self.args.pause)

        self._set_state(OState.COMPLETE)
        self._save_summary()

    def _run_one_mission(self, run_idx: int) -> bool:
        """Un singur ciclu: POI -> manip -> HOME."""
        metrics = RunMetrics(run_id=run_idx + 1)
        metrics.started_at = datetime.now().isoformat()
        run_t0 = time.time()

        try:
            # 1. NAV_TO_POI
            self._set_state(OState.NAV_TO_POI)
            t0 = time.time()
            poi_ok, poi_status = self._navigate_to(
                self.args.poi[0], self.args.poi[1], self.args.poi[2]
            )
            metrics.nav_to_poi_s = round(time.time() - t0, 2)
            metrics.nav_to_poi_status = poi_status
            if not poi_ok:
                metrics.error = f'Nav2 -> POI eșuat: {poi_status}'
                metrics.final_state = OState.FAILED
                self._set_state(OState.FAILED)
                self.runs_metrics.append(metrics)
                return False

            self._set_state(OState.AT_POI)
            self.get_logger().info(f'  Run {run_idx + 1}: ajuns la POI în {metrics.nav_to_poi_s}s')

            # 2. TRIGGER_MANIP
            self._set_state(OState.TRIGGER_MANIP)
            self._send_manip_trigger(self.args.episodes)

            # 3. WAIT_MANIP_DONE
            self._set_state(OState.WAIT_MANIP_DONE)
            t0 = time.time()
            manip_ok = self._wait_manip_done(
                timeout=self.args.manip_timeout
            )
            metrics.manip_s = round(time.time() - t0, 2)
            metrics.manip_success = manip_ok and self.manip_done_success
            metrics.manip_result_json = self.manip_result.copy()

            if not metrics.manip_success:
                self.get_logger().warn(
                    f'  Run {run_idx + 1}: manipulare nereușită sau timeout'
                )

            # 4. NAV_TO_HOME (chiar dacă manipularea a eșuat — vrem să revenim)
            self._set_state(OState.NAV_TO_HOME)
            t0 = time.time()
            home_ok, home_status = self._navigate_to(
                self.args.home[0], self.args.home[1], self.args.home[2]
            )
            metrics.nav_to_home_s = round(time.time() - t0, 2)
            metrics.nav_to_home_status = home_status

            if home_ok:
                self._set_state(OState.AT_HOME)
                self.get_logger().info(
                    f'  Run {run_idx + 1}: revenit la HOME în {metrics.nav_to_home_s}s'
                )

            metrics.total_s = round(time.time() - run_t0, 2)
            metrics.ended_at = datetime.now().isoformat()
            metrics.final_state = OState.COMPLETE if (poi_ok and home_ok and metrics.manip_success) \
                                  else OState.FAILED

            self.runs_metrics.append(metrics)

            self.get_logger().info(
                f'  Run {run_idx + 1}/{self.total_runs} terminat: '
                f'nav_poi={metrics.nav_to_poi_s}s '
                f'manip={metrics.manip_s}s '
                f'nav_home={metrics.nav_to_home_s}s '
                f'total={metrics.total_s}s '
                f'success={metrics.manip_success}'
            )
            return metrics.manip_success and poi_ok and home_ok

        except Exception as e:
            metrics.error = str(e)
            metrics.final_state = OState.FAILED
            self.runs_metrics.append(metrics)
            self.get_logger().error(f'  Run {run_idx + 1} excepție: {e}')
            return False

    # ════════════════════════════════════════════════════════════════
    # Helpere acțiuni
    # ════════════════════════════════════════════════════════════════
    def _wait_manip_ready(self, timeout: float) -> bool:
        """Așteaptă /manip_ready=True până la timeout."""
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            if self.manip_ready:
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def _navigate_to(self, x: float, y: float, yaw: float):
        """Trimite goal la Nav2 și așteaptă SUCCEEDED/FAILED/ABORTED."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = self._yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'  Nav2 goal: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}°'
        )

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            return False, 'REJECTED'

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.args.nav_timeout
        )

        if not result_future.done():
            # Cancel-uim ca să nu rămână atârnat
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            return False, 'TIMEOUT'

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True, 'SUCCEEDED'
        elif status == GoalStatus.STATUS_ABORTED:
            return False, 'ABORTED'
        elif status == GoalStatus.STATUS_CANCELED:
            return False, 'CANCELED'
        else:
            return False, f'STATUS_{status}'

    def _send_manip_trigger(self, n_episodes: int):
        """Trimite n_episodes și trigger către Jetson."""
        # Reset flag
        self.manip_done_flag = False
        self.manip_done_success = False
        self.manip_result = {}

        # 1. n_episodes
        self.pub_n_episodes.publish(Int32(data=int(n_episodes)))
        self.get_logger().info(f'  -> /manip_n_episodes = {n_episodes}')

        # Pauză scurtă pentru a fi siguri că Jetson a procesat
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        # 2. trigger
        self.pub_trigger.publish(Bool(data=True))
        self.get_logger().info('  -> /manip_trigger = True')

    def _wait_manip_done(self, timeout: float) -> bool:
        """Așteaptă /manip_done până la timeout."""
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            if self.manip_done_flag:
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    # ════════════════════════════════════════════════════════════════
    # Save summary
    # ════════════════════════════════════════════════════════════════
    def _save_summary(self):
        n_total = len(self.runs_metrics)
        n_ok = sum(1 for m in self.runs_metrics if m.manip_success
                   and m.final_state == OState.COMPLETE)
        rate = round(n_ok / max(n_total, 1), 3)

        avg = lambda key: round(
            sum(getattr(m, key) for m in self.runs_metrics) / max(n_total, 1), 2
        )

        summary = {
            'session_label': self.session_label,
            'timestamp': datetime.now().isoformat(),
            'config': {
                'poi': list(self.args.poi),
                'home': list(self.args.home),
                'runs': self.args.runs,
                'episodes_per_run': self.args.episodes,
                'pause_between_runs_s': self.args.pause,
                'nav_timeout_s': self.args.nav_timeout,
                'manip_timeout_s': self.args.manip_timeout,
            },
            'runs': [asdict(m) for m in self.runs_metrics],
            'totals': {
                'n_total': n_total,
                'n_success': n_ok,
                'n_failed': n_total - n_ok,
                'success_rate': rate,
                'avg_nav_to_poi_s': avg('nav_to_poi_s'),
                'avg_manip_s': avg('manip_s'),
                'avg_nav_to_home_s': avg('nav_to_home_s'),
                'avg_total_s': avg('total_s'),
            }
        }

        out_path = self.log_dir / 'mission_summary.json'
        with open(out_path, 'w') as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'MISIUNE TERMINATĂ — {n_ok}/{n_total} cu succes '
            f'(rată {rate * 100:.0f}%)'
        )
        self.get_logger().info(
            f'Medii: nav_POI={avg("nav_to_poi_s")}s '
            f'manip={avg("manip_s")}s '
            f'nav_HOME={avg("nav_to_home_s")}s '
            f'total={avg("total_s")}s'
        )
        self.get_logger().info(f'Raport salvat: {out_path}')
        self.get_logger().info('=' * 60)

    # ════════════════════════════════════════════════════════════════
    # Helpere stare
    # ════════════════════════════════════════════════════════════════
    def _set_state(self, new_state: str):
        if self.state != new_state:
            self.get_logger().info(
                f'[orch] {self.state} -> {new_state} '
                f'(run {self.current_run}/{self.total_runs})'
            )
            self.state = new_state

    @staticmethod
    def _yaw_to_quat(yaw: float):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (0.0, 0.0, sy, cy)

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description='Orchestrator end-to-end Nav2 + manipulare'
    )
    # ── Sursă pose-uri: una din --poi/--home SAU --teach SAU --from-file
    p.add_argument(
        '--poi', nargs=3, type=float, default=None,
        metavar=('X', 'Y', 'YAW'),
        help='Coordonate POI în frame map (x m, y m, yaw rad). Necesar fără --teach/--from-file.'
    )
    p.add_argument(
        '--home', nargs=3, type=float, default=None,
        metavar=('X', 'Y', 'YAW'),
        help='Coordonate HOME în frame map'
    )
    p.add_argument(
        '--teach', action='store_true',
        help='Mod interactiv: așteaptă 2 click-uri pe rviz "2D Goal Pose". Primul=POI, al 2-lea=HOME.'
    )
    p.add_argument(
        '--from-file', type=str, default=None,
        metavar='YAML',
        help='Încarcă POI și HOME dintr-un yaml (vezi capture_pose.py).'
    )
    p.add_argument(
        '--runs', type=int, default=5,
        help='Câte misiuni POI->manip->HOME (default 5)'
    )
    p.add_argument(
        '--episodes', type=int, default=1,
        help='Câte episoade de manipulare pe vizită POI (default 1)'
    )
    p.add_argument(
        '--pause', type=float, default=5.0,
        help='Pauză între rulări [s] (default 5)'
    )
    p.add_argument(
        '--nav-timeout', type=float, default=120.0,
        help='Timeout Nav2 pe un singur goal [s] (default 120)'
    )
    p.add_argument(
        '--manip-timeout', type=float, default=90.0,
        help='Timeout manipulare per sesiune [s] (default 90)'
    )
    p.add_argument(
        '--label', type=str, default='',
        help='Etichetă pentru log dir (e.g. test_lab_09jun)'
    )
    p.add_argument(
        '--output-dir', type=str,
        default=str(Path.home() / 'mission_logs'),
        help='Director pentru logurile JSON (default ~/mission_logs)'
    )
    return p.parse_args()


def _load_poses_yaml(path: str):
    """Încarcă POI și HOME dintr-un fișier yaml de forma:
    poi:  {x: 2.0, y: 1.5, yaw: 0.0}
    home: {x: 0.5, y: 0.0, yaw: 0.0}
    """
    try:
        import yaml
    except ImportError:
        print("Eroare: pip install pyyaml")
        sys.exit(1)
    with open(path) as f:
        d = yaml.safe_load(f)
    return (
        [float(d['poi']['x']),  float(d['poi']['y']),  float(d['poi']['yaw'])],
        [float(d['home']['x']), float(d['home']['y']), float(d['home']['yaw'])],
    )


def _capture_poses_from_rviz():
    """Mod interactiv: ascultă /goal_pose. Primul click = POI, al 2-lea = HOME.
    Robotul VA naviga prin Nav2 la fiecare click (Nav2 se abonează la /goal_pose
    automat) — folosește asta să verifici vizual fiecare poza.
    Returnează ([x, y, yaw], [x, y, yaw]).
    """
    captured = []

    rclpy.init()
    capture_node = rclpy.create_node('mission_pose_capture')

    def cb(msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        captured.append([p.x, p.y, yaw])
        which = 'POI' if len(captured) == 1 else 'HOME'
        print(f'  [{which}] x={p.x:.3f}  y={p.y:.3f}  yaw={math.degrees(yaw):.1f}°')

    capture_node.create_subscription(PoseStamped, '/goal_pose', cb, 10)

    print()
    print('=' * 70)
    print('  TEACH MODE — Click pe rviz "2D Goal Pose" de 2 ori')
    print('    1. Primul click = POI (locul de pick & place)')
    print('    2. Al 2-lea click = HOME (locul de revenire)')
    print()
    print('  Robotul va naviga la fiecare click — folosește asta să verifici')
    print('  vizual că poza e ok. După 2 click-uri continuă automat.')
    print('  Ctrl+C ca să renunți.')
    print('=' * 70)
    print()

    while rclpy.ok() and len(captured) < 2:
        rclpy.spin_once(capture_node, timeout_sec=0.2)

    capture_node.destroy_node()
    rclpy.shutdown()

    poi, home = captured[0], captured[1]
    print()
    print('=' * 70)
    print(f'  POI  = ({poi[0]:.3f},  {poi[1]:.3f},  {math.degrees(poi[2]):.1f}°)')
    print(f'  HOME = ({home[0]:.3f}, {home[1]:.3f}, {math.degrees(home[2]):.1f}°)')
    print('=' * 70)

    # Salvează în yaml pentru reutilizare
    out = Path.home() / 'mission_poses.yaml'
    try:
        import yaml
        with open(out, 'w') as f:
            yaml.safe_dump({
                'poi':  {'x': poi[0],  'y': poi[1],  'yaw': poi[2]},
                'home': {'x': home[0], 'y': home[1], 'yaw': home[2]},
            }, f)
        print(f'  Salvat în {out}')
    except ImportError:
        pass
    print()

    input('  Apasă Enter ca să pornesc misiunea (sau Ctrl+C ca să renunți)... ')
    return poi, home


def main():
    args = parse_args()

    # Validare și rezolvare sursă pose-uri
    if args.teach:
        if args.poi or args.home:
            print('Warn: --teach ignoră --poi/--home, folosește click-urile pe rviz')
        args.poi, args.home = _capture_poses_from_rviz()
    elif args.from_file:
        if not os.path.exists(args.from_file):
            print(f'Eroare: fișier inexistent: {args.from_file}')
            sys.exit(1)
        args.poi, args.home = _load_poses_yaml(args.from_file)
        print(f'Încărcat din {args.from_file}:')
        print(f'  POI  = ({args.poi[0]:.3f},  {args.poi[1]:.3f},  '
              f'{math.degrees(args.poi[2]):.1f}°)')
        print(f'  HOME = ({args.home[0]:.3f}, {args.home[1]:.3f}, '
              f'{math.degrees(args.home[2]):.1f}°)')
    else:
        if args.poi is None or args.home is None:
            print('Eroare: dă fie --poi X Y YAW --home X Y YAW, fie --teach, '
                  'fie --from-file FILE')
            sys.exit(1)

    rclpy.init()
    orch = MissionOrchestrator(args)

    try:
        orch.run_mission()
    except KeyboardInterrupt:
        orch.get_logger().warn('Ctrl+C — trimit abort către Jetson...')
        orch.pub_abort.publish(Bool(data=True))
        orch._set_state(OState.ABORTED)
        orch._save_summary()
    finally:
        orch.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
