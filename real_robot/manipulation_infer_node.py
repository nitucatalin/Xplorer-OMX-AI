#!/usr/bin/env python3
"""
manipulation_infer_node.py — nodul de inferenta ACT de pe Jetson Orin Nano.

Versiune verificata 1:1 contra contractului orchestratorului de pe RPi5
(mission_orchestrator.py / go_collect_real.py) si construita PESTE clasele
deja validate din infer_loop_v5.py (I/O prin OmxFollower — cel care
manipuleaza corect, vezi Jurnal D2). Destinat caii ~/ros2_manip/ de pe
Jetson; lansat de start_jetson.sh:

  python3 manipulation_infer_node.py --ros-args -p stub_mode:=false

Contract (identic cu simularea — capitolul 3.4/3.5):
  Abonare:
    /manip_trigger     Bool    declanseaza o sesiune de inferenta
    /manip_n_episodes  Int32   cate episoade pe sesiune (default 1)
    /manip_abort       Bool    abort: opreste episodul, revine in home
    /manip_go_home     Bool    trimite bratul in home (cand e IDLE)
  Publicare (QoS RELIABLE/VOLATILE/depth5, ca orchestratorul):
    /manip_ready       Bool    gata de trigger (republicat la 1 Hz)
    /manip_done        Bool    sesiune terminata (True = toate episoadele
                               au rulat complet; reusita FIZICA se scoreaza
                               vizual, ca in teza)
    /manip_status      String  JSON heartbeat 2 Hz:
                               {"state","busy","episode","n_episodes","ts"}
    /manip_result      String  JSON raport final de sesiune

Masina de stari: IDLE -> HOMING -> RUNNING -> IDLE  (ABORTED la abort)

Parametri ROS:
  stub_mode        (bool,  false)  fara hardware — tranzitii + durate reale
  model_path       (str)   default /home/jnfiir/lerobot_models/model_act_licenta
  port             (str)   /dev/ttyACM0
  camera           (str)   /dev/video0
  fps              (int)   30   — OBLIGATORIU 30 (modelul e antrenat la 30)
  episode_time_s   (double) 26.0
  n_action_steps   (int)   0    — 0 = chunk_size-ul politicii (ex. 50 la pbn2)
  home_pose        (double[6])  pozitia home in unitati LeRobot; daca e
                   goala, se foloseste pozitia curenta a bratului la pornire
                   (ia valorile calibrate din omx_init_home.py!)
  idle_pose        (double[6])  pozitie intermediara SIGURA; daca e setata,
                   homing-ul trece intai prin ea si abia apoi spre home_pose
                   (evita sa loveasca ceva pe traseul direct spre HOME)
"""
import json
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from std_msgs.msg import Bool, Int32, String

# clasele VALIDATE din infer_loop_v5 (acelasi director ~/ros2_manip)
from infer_loop_v5 import (ACTChunkPolicy, OmxSystem, DrySystem,
                           TemporalEnsemble, EpisodeLogger)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=5)

HOMING_RAMP_S = 5.0      # durata deplasarii lente spre home


class MState:
    IDLE = 'IDLE'
    HOMING = 'HOMING'
    RUNNING = 'RUNNING'
    ABORTED = 'ABORTED'


class ManipulationInferNode(Node):
    def __init__(self):
        super().__init__('manipulation_infer_node')

        # ── parametri
        self.declare_parameter('stub_mode', False)
        self.declare_parameter('model_path',
                               '/home/jnfiir/lerobot_models/model_act_licenta')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('camera', '/dev/video0')
        self.declare_parameter('fps', 30)
        self.declare_parameter('episode_time_s', 26.0)
        self.declare_parameter('n_action_steps', 0)
        self.declare_parameter('home_pose', [0.0])
        self.declare_parameter('idle_pose', [0.0])
        # durate homing (secunde): rampa spre idle, pauza in idle, rampa spre home
        self.declare_parameter('idle_ramp_s', 5.0)
        self.declare_parameter('idle_settle_s', 1.0)
        self.declare_parameter('home_ramp_s', 5.0)

        gp = lambda n: self.get_parameter(n).value
        self.stub = bool(gp('stub_mode'))
        self.fps = int(gp('fps'))
        self.episode_time = float(gp('episode_time_s'))
        self.idle_ramp_s = float(gp('idle_ramp_s'))
        self.idle_settle_s = float(gp('idle_settle_s'))
        self.home_ramp_s = float(gp('home_ramp_s'))

        # ── stare
        self.state = MState.IDLE
        self.busy = False
        self.episode_idx = 0
        self.n_episodes = 1
        self.abort_requested = False
        self._session_results = []

        # ── interfata /manip_*
        self.pub_ready = self.create_publisher(Bool, '/manip_ready', RELIABLE_QOS)
        self.pub_done = self.create_publisher(Bool, '/manip_done', RELIABLE_QOS)
        self.pub_status = self.create_publisher(String, '/manip_status', RELIABLE_QOS)
        self.pub_result = self.create_publisher(String, '/manip_result', RELIABLE_QOS)
        self.create_subscription(Bool, '/manip_trigger', self._on_trigger, RELIABLE_QOS)
        self.create_subscription(Int32, '/manip_n_episodes', self._on_n_episodes, RELIABLE_QOS)
        self.create_subscription(Bool, '/manip_abort', self._on_abort, RELIABLE_QOS)
        self.create_subscription(Bool, '/manip_go_home', self._on_go_home, RELIABLE_QOS)

        # ── log dir (aceeasi structura ca infer_loop_v5 / simularea)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = Path.home() / 'infer_logs' / f'node_session_{ts}'
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # ── incarcare model + conectare hardware (o singura data, la start)
        self.get_logger().info(
            f'Pornire | stub_mode={self.stub} | model={gp("model_path")}')
        if self.stub:
            self.act = None
            self.sysm = DrySystem()
            self.sysm.connect()
            self.nas = 50
        else:
            self.act = ACTChunkPolicy(str(gp('model_path')))
            self.nas = int(gp('n_action_steps')) or self.act.chunk_size
            self.sysm = OmxSystem(str(gp('port')), str(gp('camera')),
                                  fps=self.fps)
            self.sysm.connect()   # configure(): moduri operare, PID, calibrare
        self.ensemble = TemporalEnsemble(None, 6)

        # pozitia home: prioritate parametrul home_pose; daca lipseste, foloseste
        # AUTOMAT media pozitiilor din antrenare (obs_mean din model) — o poza
        # prin care bratul a trecut real in dataset, deci sigura: fara coliziuni,
        # fara depasire de limite, cuplu de mentinere mic (nu se decupleaza).
        hp = list(gp('home_pose'))
        if len(hp) == 6:
            self.home_pose = np.array(hp, dtype=np.float32)
            self.get_logger().info(f'home_pose din parametru: {hp}')
        elif (not self.stub) and getattr(self.act, 'norm', None) is not None:
            self.home_pose = np.asarray(self.act.norm.obs_mean, dtype=np.float32)
            self.get_logger().info(
                'home_pose AUTO = media din model (obs_mean): '
                f'{np.round(self.home_pose, 2).tolist()} — poza sigura din antrenare')
        else:
            self.home_pose = self.sysm.get_state()
            self.get_logger().warn(
                'home_pose nesetat si fara model — folosesc pozitia curenta a '
                f'bratului ({np.round(self.home_pose, 1).tolist()}).')

        # pozitie de idle (intermediara, sigura): daca e setata, homing-ul
        # trece intai prin ea si abia apoi spre home_pose.
        ip = list(gp('idle_pose'))
        if len(ip) == 6:
            self.idle_pose = np.array(ip, dtype=np.float32)
            self.get_logger().info(f'idle_pose din parametru: {ip}')
        else:
            self.idle_pose = None

        # ── timere heartbeat
        self.create_timer(0.5, self._publish_status)
        self.create_timer(1.0, self._publish_ready)

        # homing initial in thread (nu bloca executor-ul)
        self._worker = threading.Thread(target=self._initial_homing,
                                        daemon=True)
        self._worker.start()
        self.get_logger().info(
            f'manipulation_infer_node pornit | stub_mode={self.stub} | '
            f'NAS={self.nas} fps={self.fps} episode_time={self.episode_time}s')

    # ════════════════════════════════════════════════════════════════
    # Callbacks /manip_*
    # ════════════════════════════════════════════════════════════════
    def _on_trigger(self, msg: Bool):
        if not msg.data:
            return
        if self.busy:
            self.get_logger().warn('Trigger ignorat — sesiune in curs')
            return
        self.abort_requested = False
        self.busy = True
        self._worker = threading.Thread(target=self._run_session, daemon=True)
        self._worker.start()

    def _on_n_episodes(self, msg: Int32):
        self.n_episodes = max(1, int(msg.data))
        self.get_logger().info(f'n_episodes = {self.n_episodes}')

    def _on_abort(self, msg: Bool):
        if msg.data and self.busy:
            self.get_logger().warn('ABORT primit')
            self.abort_requested = True

    def _on_go_home(self, msg: Bool):
        if msg.data and not self.busy:
            self.busy = True
            threading.Thread(target=self._go_home_session,
                             daemon=True).start()

    # ════════════════════════════════════════════════════════════════
    # Miscari
    # ════════════════════════════════════════════════════════════════
    def _ramp_to(self, target, duration):
        """Deplasare lenta, interpolata, spre o pozitie (home)."""
        start = self.sysm.get_state()
        steps = max(1, int(duration * self.fps))
        for i in range(1, steps + 1):
            if self.abort_requested:
                return False
            a = i / steps
            self.sysm.send(start + (np.asarray(target) - start) * a)
            time.sleep(1.0 / self.fps)
        return True

    def _settle(self, seconds):
        """Pauza intrerupibila (verifica abort la fiecare 50 ms)."""
        t_end = time.time() + max(0.0, seconds)
        while time.time() < t_end:
            if self.abort_requested:
                return False
            time.sleep(0.05)
        return True

    def _home_sequence(self):
        """Homing sigur in doua etape, cu timeri separati:
          1) rampa spre IDLE  (idle_ramp_s)
          2) pauza in IDLE     (idle_settle_s)
          3) rampa spre HOME  (home_ramp_s)
        Daca idle_pose nu e setat, merge direct la HOME.
        Returneaza False daca a fost intrerupt (abort)."""
        if self.idle_pose is not None:
            self.get_logger().info(f'Homing 1/2: spre IDLE ({self.idle_ramp_s}s)')
            if not self._ramp_to(self.idle_pose, self.idle_ramp_s):
                return False
            self.get_logger().info(f'Pauza in IDLE: {self.idle_settle_s}s')
            if not self._settle(self.idle_settle_s):
                return False
        self.get_logger().info(f'Homing 2/2: spre HOME ({self.home_ramp_s}s)')
        return self._ramp_to(self.home_pose, self.home_ramp_s)

    def _initial_homing(self):
        self.busy = True
        self.state = MState.HOMING
        try:
            if not self.stub:
                if self._home_sequence():
                    self.get_logger().info('Homing initial COMPLET (idle -> home)')
                else:
                    self.get_logger().warn('Homing initial INTRERUPT (abort)')
        except Exception as e:
            self.get_logger().error(
                f'Homing initial ESUAT: {e!r} — bratul ramane unde a ajuns. '
                'Cauza tipica: camera (frame read failed) sau busul motoarelor.')
        finally:
            self.state = MState.IDLE
            self.busy = False
            self.get_logger().info('Homing initial terminat — READY')

    def _go_home_session(self):
        self.state = MState.HOMING
        try:
            if not self.stub:
                self._home_sequence()
        except Exception as e:
            self.get_logger().error(f'Go-home ESUAT: {e!r}')
        finally:
            self.state = MState.IDLE
            self.busy = False

    # ════════════════════════════════════════════════════════════════
    # Sesiunea de inferenta (thread separat; heartbeat-ul continua)
    # ════════════════════════════════════════════════════════════════
    def _run_session(self):
        self._session_results = []
        self.episode_idx = 0
        t_session = time.time()
        try:
            for ep in range(self.n_episodes):
                if self.abort_requested:
                    break
                self.episode_idx = ep + 1

                # HOMING inainte de fiecare episod
                self.state = MState.HOMING
                if not self.stub:
                    if not self._ramp_to(self.home_pose, 2.0):
                        break

                # RUNNING: episodul de inferenta propriu-zis
                self.state = MState.RUNNING
                ok, detail = self._run_episode()
                self._session_results.append({
                    'episode': self.episode_idx, 'success': ok, **detail})
                self.get_logger().info(
                    f'Episod {self.episode_idx}/{self.n_episodes}: '
                    f'{"COMPLET" if ok else "INTRERUPT"} '
                    f'({detail.get("duration_s")}s, '
                    f'{detail.get("steps")} pasi)')

            # inapoi in home dupa sesiune (bratul ramane cu torque)
            self.state = MState.HOMING
            if not self.stub:
                self.abort_requested = False
                self._ramp_to(self.home_pose, 3.0)
        except Exception as e:
            self.get_logger().error(f'Exceptie in sesiune: {e}')
            self._session_results.append({
                'episode': self.episode_idx, 'success': False,
                'error': str(e)})
        finally:
            self._finish_session(time.time() - t_session)
            self.state = MState.IDLE
            self.busy = False

    def _run_episode(self):
        """Bucla de inferenta — logica din infer_loop_v5.run_episode, cu
        verificare de abort la fiecare pas (necesara pentru /manip_abort)."""
        t0 = time.time()
        if self.stub:
            # stub: doar durata reala a unui episod
            end = t0 + self.episode_time
            while time.time() < end:
                if self.abort_requested:
                    return False, {'duration_s': round(time.time() - t0, 2),
                                   'steps': 0, 'reason': 'aborted'}
                time.sleep(0.1)
            return True, {'duration_s': round(time.time() - t0, 2),
                          'steps': 0, 'reason': 'stub_completed'}

        dt = 1.0 / self.fps
        max_steps = int(self.episode_time * self.fps)
        self.ensemble.reset()
        action_buf, buf_idx, step, n_reinf = None, 0, 0, 0
        infer_ms_list = []
        import torch
        while step < max_steps:
            if self.abort_requested:
                return False, {'duration_s': round(time.time() - t0, 2),
                               'steps': step, 'reason': 'aborted'}
            t_loop = time.perf_counter()
            if action_buf is None or buf_idx >= self.nas:
                state, frame = self.sysm.get_obs()
                torch.cuda.synchronize()
                ti = time.perf_counter()
                chunk = self.act.get_chunk(frame, state)
                torch.cuda.synchronize()
                infer_ms_list.append((time.perf_counter() - ti) * 1000)
                action_buf = self.ensemble.apply(chunk, self.nas)
                buf_idx = 0
                n_reinf += 1
            self.sysm.send(action_buf[buf_idx])
            buf_idx += 1
            self.ensemble.advance()
            step += 1
            sl = dt - (time.perf_counter() - t_loop)
            if sl > 0:
                time.sleep(sl)
        return True, {
            'duration_s': round(time.time() - t0, 2),
            'steps': step,
            'reinferences': n_reinf,
            'infer_ms_median': round(float(np.median(infer_ms_list)), 1)
            if infer_ms_list else 0,
            'reason': 'completed'}

    def _finish_session(self, total_s):
        n = len(self._session_results)
        ok = sum(1 for r in self._session_results if r.get('success'))
        summary = {
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'mode': 'stub' if self.stub else 'real',
            'nas': self.nas, 'fps': self.fps,
            'episode_time_s': self.episode_time,
            'session_total_s': round(total_s, 2),
            'episodes': self._session_results,
            'totals': {'n_episodes': n, 'n_success': ok,
                       'n_failed': n - ok,
                       'rate': round(ok / max(n, 1), 3)},
        }
        out = self.log_dir / 'session_summary.json'
        with open(out, 'w') as f:
            json.dump(summary, f, indent=2)
        self.pub_result.publish(String(data=json.dumps(summary)))
        self.pub_done.publish(Bool(data=(ok == n and n > 0)))
        self.get_logger().info(
            f'Sesiune terminata: {ok}/{n} | /manip_done='
            f'{ok == n and n > 0} | {out}')

    # ════════════════════════════════════════════════════════════════
    def _publish_status(self):
        self.pub_status.publish(String(data=json.dumps({
            'state': self.state, 'busy': self.busy,
            'episode': self.episode_idx, 'n_episodes': self.n_episodes,
            'ts': datetime.now().isoformat(timespec='seconds')})))

    def _publish_ready(self):
        self.pub_ready.publish(Bool(data=not self.busy))


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationInferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn(
            'Ctrl+C — sustine bratul cu mana (torque off la disconnect)!')
    finally:
        try:
            node.sysm.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
