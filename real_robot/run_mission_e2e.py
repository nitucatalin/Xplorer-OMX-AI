#!/usr/bin/env python3
"""
run_mission_e2e.py — Orchestrator end-to-end pentru testul integrat
====================================================================
Secvența automată:
  1. Trimite Nav2 goal la POI-A (coordonate furnizate)
  2. Așteaptă atingerea goal-ului
  3. Declanșează episod de manipulare ACT (/manip_trigger)
  4. Așteaptă întoarcerea brațului în IDLE (sau /manip_done)
  5. Trimite Nav2 goal la HOME (coordonate furnizate sau salvate)
  6. Așteaptă întoarcere
  7. Loghează toate timpii și verdictul într-un JSON

Utilizare:
  # 1 rulare cu coordonate explicite
  python3 run_mission_e2e.py --poi 1.5 0.5 0.0 --home 0.0 0.0 0.0

  # 5 rulări (campanie)
  python3 run_mission_e2e.py --poi 1.5 0.5 0.0 --home 0.0 0.0 0.0 --runs 5

  # Cu nume custom pentru log
  python3 run_mission_e2e.py --poi 1.5 0.5 0.0 --home 0.0 0.0 0.0 --label "POI-A_chunk50"

Coordonatele sunt: x y yaw_rad (în map frame).
Pe RPi5, cu env Domain 50 sourcuit.
"""

import sys
import json
import time
import math
import argparse
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


# Timeout-uri (secunde)
NAV_TIMEOUT       = 120   # max pentru a ajunge la un goal
MANIP_TIMEOUT     = 90    # max pentru un episod (HOMING + RUNNING)
MANIP_TRIGGER_LAG = 15    # așteptare propagare DDS până apare HOMING


def yaw_to_quat(yaw):
    """Convertește yaw (rad) la quaternion (x,y,z,w)."""
    return (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))


def make_pose(x, y, yaw, frame='map'):
    """Construiește PoseStamped din x,y,yaw."""
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quat(yaw)
    p.pose.orientation.x = qx
    p.pose.orientation.y = qy
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p


class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')

        # Action client Nav2
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Publishers spre manipulare
        self.pub_trigger    = self.create_publisher(Bool, '/manip_trigger', 10)
        self.pub_n_episodes = self.create_publisher(Int32, '/manip_n_episodes', 10)
        self.pub_abort      = self.create_publisher(Bool, '/manip_abort', 10)

        # Subscriber pe manip_status (mașina de stări)
        self.manip_state = 'UNKNOWN'
        self.manip_busy  = False
        self.manip_state_history = []
        qos_status = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                depth=1)
        self.create_subscription(String, '/manip_status', self.on_status, qos_status)

        # Subscriber pe /manip_done
        self.manip_done = False
        self.manip_result = None
        self.create_subscription(Bool, '/manip_done', self.on_done, 10)
        self.create_subscription(String, '/manip_result', self.on_result, 10)

        self.get_logger().info('Mission executor pornit')

    def on_status(self, msg):
        try:
            data = json.loads(msg.data)
            new_state = data.get('state', '?')
            if new_state != self.manip_state:
                self.get_logger().info(f'  manip: {self.manip_state} → {new_state}')
                self.manip_state_history.append({
                    'state': new_state,
                    'ts': time.time(),
                    'ts_human': data.get('ts', '')
                })
            self.manip_state = new_state
            self.manip_busy  = data.get('busy', False)
        except Exception:
            pass

    def on_done(self, msg):
        if msg.data:
            self.get_logger().info('  manip: DONE primit')
            self.manip_done = True

    def on_result(self, msg):
        self.manip_result = msg.data
        self.get_logger().info(f'  manip: result = {msg.data[:80]}')

    # ----- helper-i navigare -----
    def send_nav_goal(self, x, y, yaw, label=''):
        """Trimite goal Nav2 și așteaptă goal_handle."""
        self.get_logger().info(f'>>> Nav2 goal [{label}]: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
        if not self.nav_client.wait_for_server(timeout_sec=5):
            self.get_logger().error('  Nav2 action server indisponibil')
            return None

        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw)
        send_fut = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10)
        if not send_fut.result() or not send_fut.result().accepted:
            self.get_logger().error('  Goal refuzat de Nav2')
            return None
        return send_fut.result()

    def wait_nav_complete(self, goal_handle, timeout=NAV_TIMEOUT):
        """Așteaptă finalizarea navigării. Returnează (success, duration_s, error_code)."""
        t0 = time.time()
        result_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut, timeout_sec=timeout)

        if not result_fut.done():
            self.get_logger().error(f'  Nav2 timeout după {timeout}s')
            goal_handle.cancel_goal_async()
            return False, time.time() - t0, -1

        result = result_fut.result()
        if result is None:
            return False, time.time() - t0, -2

        success = (result.status == 4)  # STATUS_SUCCEEDED
        return success, time.time() - t0, result.result.error_code

    # ----- manipulare -----
    def trigger_manipulation(self, n_episodes=1):
        """Trimite trigger și așteaptă tranziția HOMING/RUNNING."""
        # Publică n_episodes
        ne = Int32() ; ne.data = n_episodes
        self.pub_n_episodes.publish(ne)
        time.sleep(0.5)
        # Trigger
        bt = Bool() ; bt.data = True
        self.pub_trigger.publish(bt)
        self.get_logger().info(f'>>> Manipulare: trigger trimis ({n_episodes} episod)')

    def wait_manipulation_complete(self, timeout=MANIP_TIMEOUT):
        """
        Așteaptă finalizarea episodului:
          - Vede manip_state trece prin HOMING / RUNNING
          - Apoi revine la IDLE (sau primește /manip_done = true)
        """
        t0 = time.time()
        saw_running = False
        # Spin până când vedem RUNNING-ul cel puțin o dată
        while time.time() - t0 < MANIP_TRIGGER_LAG:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.manip_state in ('HOMING', 'RUNNING'):
                saw_running = True
                break

        if not saw_running:
            self.get_logger().warn('  manip nu a tranzitat HOMING/RUNNING în 15s')
            return False, time.time() - t0

        # Acum așteptăm să revină la IDLE sau să primim /manip_done
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.manip_done:
                return True, time.time() - t0
            if not self.manip_busy and self.manip_state == 'IDLE':
                return True, time.time() - t0

        # Timeout
        self.get_logger().error(f'  manip timeout după {timeout}s')
        return False, time.time() - t0

    def abort_manipulation(self):
        msg = Bool() ; msg.data = True
        self.pub_abort.publish(msg)


def run_one_mission(node, poi, home, run_idx):
    """Execută un ciclu complet și returnează dict cu metrici."""
    result = {
        'run': run_idx,
        'start_ts': datetime.now().isoformat(timespec='seconds'),
        'poi': list(poi),
        'home': list(home),
        'nav_to_poi':   {'ok': False, 'duration_s': None, 'error_code': None},
        'manipulation': {'ok': False, 'duration_s': None, 'states': []},
        'nav_to_home':  {'ok': False, 'duration_s': None, 'error_code': None},
        'total_duration_s': None,
    }

    t_total = time.time()

    # --- Faza 1: Nav2 la POI ---
    node.get_logger().info(f'\n=== RUN {run_idx} | FAZA 1: Navigare POI ===')
    node.manip_state_history.clear()
    gh = node.send_nav_goal(*poi, label=f'POI_run{run_idx}')
    if gh:
        ok, dur, err = node.wait_nav_complete(gh)
        result['nav_to_poi'] = {'ok': ok, 'duration_s': dur, 'error_code': err}
        if not ok:
            node.get_logger().error(f'  Nav2 POI eșuat (err={err}). Skip manipulare.')
            result['total_duration_s'] = time.time() - t_total
            return result
    else:
        result['total_duration_s'] = time.time() - t_total
        return result

    # --- Faza 2: Manipulare ---
    node.get_logger().info(f'\n=== RUN {run_idx} | FAZA 2: Manipulare ===')
    node.manip_done = False
    node.trigger_manipulation(n_episodes=1)
    ok, dur = node.wait_manipulation_complete()
    result['manipulation'] = {
        'ok': ok,
        'duration_s': dur,
        'states': list(node.manip_state_history),
        'result_payload': node.manip_result,
    }
    if not ok:
        node.get_logger().warn('  Manipulare eșuată/timeout — încerc abort și continuă')
        node.abort_manipulation()
        time.sleep(2)

    # --- Faza 3: Nav2 la HOME ---
    node.get_logger().info(f'\n=== RUN {run_idx} | FAZA 3: Navigare HOME ===')
    gh = node.send_nav_goal(*home, label=f'HOME_run{run_idx}')
    if gh:
        ok, dur, err = node.wait_nav_complete(gh)
        result['nav_to_home'] = {'ok': ok, 'duration_s': dur, 'error_code': err}

    result['total_duration_s'] = time.time() - t_total
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--poi',  nargs=3, type=float, required=True,
                        metavar=('X','Y','YAW'),
                        help='Coordonate POI (x y yaw_rad)')
    parser.add_argument('--home', nargs=3, type=float, required=True,
                        metavar=('X','Y','YAW'),
                        help='Coordonate HOME (x y yaw_rad)')
    parser.add_argument('--runs', type=int, default=1,
                        help='Câte rulări de campanie (default 1)')
    parser.add_argument('--label', type=str, default='mission',
                        help='Etichetă pentru fișierul de log')
    parser.add_argument('--pause', type=float, default=5.0,
                        help='Pauză între rulări (s)')
    args = parser.parse_args()

    rclpy.init()
    node = MissionExecutor()

    # Așteaptă conectarea la Nav2
    node.get_logger().info('Aștept Nav2 action server...')
    if not node.nav_client.wait_for_server(timeout_sec=15):
        node.get_logger().error('Nav2 action server nu e disponibil. Verifică start_all.sh')
        rclpy.shutdown()
        sys.exit(1)
    node.get_logger().info('✅ Nav2 conectat')

    # Pregătește fișier de log
    log_dir = Path.home() / 'mission_logs'
    log_dir.mkdir(exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = log_dir / f'{args.label}_{ts}.json'

    all_results = {
        'label': args.label,
        'started': datetime.now().isoformat(timespec='seconds'),
        'poi': args.poi,
        'home': args.home,
        'n_runs': args.runs,
        'runs': []
    }

    try:
        for i in range(1, args.runs + 1):
            r = run_one_mission(node, tuple(args.poi), tuple(args.home), i)
            all_results['runs'].append(r)
            # Scrie incremental după fiecare rulare
            with open(log_path, 'w') as f:
                json.dump(all_results, f, indent=2)
            node.get_logger().info(f'>>> Run {i}/{args.runs} terminat în {r["total_duration_s"]:.1f}s')
            if i < args.runs:
                node.get_logger().info(f'    Pauză {args.pause}s înainte de următoarea rulare...')
                time.sleep(args.pause)
    except KeyboardInterrupt:
        node.get_logger().warn('Întrerupt de utilizator')

    # Sumar final
    n_ok = sum(1 for r in all_results['runs']
               if r['nav_to_poi']['ok'] and r['manipulation']['ok'] and r['nav_to_home']['ok'])
    node.get_logger().info('\n' + '='*60)
    node.get_logger().info(f' SUMAR: {n_ok}/{len(all_results["runs"])} rulări complet reușite')
    node.get_logger().info(f' Log JSON: {log_path}')
    node.get_logger().info('='*60)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
