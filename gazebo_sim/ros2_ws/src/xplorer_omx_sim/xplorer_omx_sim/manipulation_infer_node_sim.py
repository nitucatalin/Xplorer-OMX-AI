#!/usr/bin/env python3
"""
manipulation_infer_node_sim.py — replica de simulare a nodului
manipulation_infer_node de pe Jetson Orin Nano, cu PICK ADAPTIV.

Contractul de comunicatie cu orchestratorul e identic cu sistemul real:
  Abonare:   /manip_trigger  /manip_n_episodes  /manip_abort  /manip_go_home
  Publicare: /manip_ready  /manip_done  /manip_status (2 Hz)  /manip_result

Masina de stari: IDLE -> HOMING -> RUNNING -> IDLE (ABORTED la abort).

PICK ADAPTIV (echivalentul perceptiei vizuale a politicii ACT):
  Nodul citeste pozele ground-truth ale modelelor dinamice din Gazebo
  (topic /sim/dynamic_poses, bridge peste /world/.../dynamic_pose/info).
  La trigger:
    1. gaseste obiectul obj_* cel mai apropiat de zona de lucru din
       spatele robotului;
    2. rezolva IK numeric (FK pe lantul CAD din config/fk_data.json)
       pentru pozitia REALA a obiectului -> compenseaza eroarea de
       parcare Nav2/AMCL (pana la ~20 cm);
    3. executa episodul: PREPICK -> PICK -> GRASP -> LIFT -> BOX_DROP ->
       RELEASE -> HOME;
    4. VERIFICA fizic rezultatul: obiectul este in cutia de colectare de
       pe sasiu? -> /manip_done True/False (orchestratorul reincearca
       la False).

Logging identic ca structura cu infer_loop_v5.py:
  ~/manip_sim_logs/session_<ts>/session_summary.json
"""
import json
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Int32, String, Float64, Empty
from tf2_msgs.msg import TFMessage

# obiectele care pot fi "sudate" de gripper la strangere (DetachableJoint)
GRASPABLE = [f'obj_{i}' for i in range(6)] + ['obj_spawn']

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex']
GRIPPER_JOINTS = ['gripper_left', 'gripper_right']

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=5)

# Degete: la gripper_left rotatia pozitiva DESCHIDE, la gripper_right
# INCHIDE -> comanda dreapta = -comanda stanga (inverseaza aici daca un
# deget se misca invers in Gazebo).
GRIP_SIGN = {'gripper_left': +1.0, 'gripper_right': -1.0}
# lamele distale: deschis 7.1 cm; contactul cu cubul de 2.8 cm e la ~0.34.
# Comanda de inchidere e DINCOLO de contact (-0.10): PID-ul nu isi atinge
# niciodata tinta cat timp cubul e intre lamele, deci apasa CONTINUU cu
# forta maxima — strangere ferma pe tot transportul. (Fara obiect, lamelele
# doar se suprapun vizual: self-collision e dezactivat intre degete.)
GRIP_OPEN = 0.95
GRIP_CLOSED = -0.10

# ---------------------------------------------------------------------------
# Pozitii nominale [pan, lift, elbow, wrist, grip] — FK numeric pe lantul
# CAD. Bratul e in coltul lui original, rotit 180 din baza: gripperul
# impachetat sta in exterior, peste marginea din spate.
# PREPICK/PICK sunt DOAR fallback — la rulare sunt inlocuite de IK-ul
# adaptiv pe pozitia reala a obiectului.
# ---------------------------------------------------------------------------
POSE_HOME     = [0.0,  0.0,   0.0,   0.0,  GRIP_OPEN]
# Secventa de coborare DREAPTA, pe verticala (fara efectul "cupa de
# excavator"): toate cele 3 poze tin gripperul orientat strict in jos
# (dir_z = -1.00) si sunt din ACEEASI ramura IK -> interpolarea coboara
# centrul clestilor pe o linie verticala (deviatie laterala < 4 mm).
# Bratul coboara PANA LA CAPAT din lift/elbow: tinta centrului lamelelor
# e 5 mm SUB nivelul solului — fizica opreste lamelele la contact, iar
# PID-ul le tine apasate jos; orice abatere de model sau lasare a PID-ului
# e absorbita, lamelele cuprind garantat obiectul la baza lui.
POSE_PREPICK  = [0.0, -0.054, -0.604, 0.922, GRIP_OPEN]   # +14 cm
POSE_MID      = [0.0, -0.506, -0.512, 0.562, GRIP_OPEN]   # +7 cm
POSE_PICK     = [0.0, -1.090, -0.100, 0.410, GRIP_OPEN]   # apasat la sol
POSE_LIFT     = [0.0,  0.67, -0.85,  1.12, GRIP_CLOSED]
# cutia e LANGA brat (cum a fost antrenat modelul real): plasarea se face
# prin rotirea pan spre stanga (~105 grade), nu peste crestet
POSE_BOX_OVER = [1.84, 0.66,  0.09,  1.9,  GRIP_CLOSED]
POSE_BOX_DROP = [1.84, 0.87, -0.7,   1.4,  GRIP_CLOSED]

HOMING_DURATION = 5.0
# punctul nominal de pick in frame base_link (robotul parcheaza cu spatele
# la obiect; pois.yaml e generat cu acelasi offset)
NOMINAL_PICK = np.array([-0.3302, -0.0643])
# cutia de colectare de pe sasiu, LANGA brat (frame base_link)
BOX_CENTER = np.array([-0.12, 0.10])
BOX_HALF = 0.09
BOX_TOP_Z = 0.145


class MState:
    IDLE = 'IDLE'
    HOMING = 'HOMING'
    RUNNING = 'RUNNING'
    ABORTED = 'ABORTED'


# ═══════════════════════════════════════════════════════════════════
# FK / IK pe lantul CAD (fk_data.json generat de flatten_onshape_urdf)
# ═══════════════════════════════════════════════════════════════════
class ArmKinematics:
    def __init__(self, fk_path):
        fk = json.loads(Path(fk_path).read_text())
        S = fk['segments']
        self.Trel = [np.array(S[j]['T_rel']) for j in ARM_JOINTS]
        self.axes = [np.array(S[j]['axis_local']) for j in ARM_JOINTS]
        self.axes = [a / np.linalg.norm(a) for a in self.axes]
        fl = np.array(S['gripper_left']['T_rel'])
        fr = np.array(S['gripper_right']['T_rel'])
        mid = (fl[:3, 3] + fr[:3, 3]) / 2
        self.pdir = mid / np.linalg.norm(mid)
        # punctul-tinta al IK-ului = CENTRUL REAL DINTRE CLESTI (mijlocul
        # centrelor de coliziune ale degetelor, cu gripperul inchis) —
        # acolo trebuie sa fie centrul obiectului apucat
        fcl = np.array(S['gripper_left'].get('collision_center')
                       or [0, 0, 0.05])
        fcr = np.array(S['gripper_right'].get('collision_center')
                       or [0, 0, 0.05])
        fla = np.array(S['gripper_left']['axis_local'])
        fra = np.array(S['gripper_right']['axis_local'])
        fla = fla / np.linalg.norm(fla)
        fra = fra / np.linalg.norm(fra)
        Tl = fl @ self._rot(fla, +0.02)   # GRIP_CLOSED
        Tr = fr @ self._rot(fra, -0.02)
        pl = Tl[:3, :3] @ fcl + Tl[:3, 3]
        pr = Tr[:3, :3] @ fcr + Tr[:3, 3]
        self.tip_local = (pl + pr) / 2
        self.pivot = np.array(S['shoulder_pan']['T_rel'])[:3, 3]
        # semnul pan: incotro muta tip-ul o rotatie pozitiva
        t0 = self.tip([0.3, *POSE_PICK[1:4]])
        t1 = self.tip([-0.3, *POSE_PICK[1:4]])
        b0 = math.atan2(t0[1] - self.pivot[1], t0[0] - self.pivot[0])
        b1 = math.atan2(t1[1] - self.pivot[1], t1[0] - self.pivot[0])
        d = math.atan2(math.sin(b0 - b1), math.cos(b0 - b1))
        self.pan_sign = 1.0 if d > 0 else -1.0

    @staticmethod
    def _rot(a, th):
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        T = np.eye(4)
        T[:3, :3] = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)
        return T

    def tip(self, q):
        T = np.eye(4)
        for Tr, a, th in zip(self.Trel, self.axes, q):
            T = T @ Tr @ self._rot(a, th)
        return (T[:3, :3] @ self.tip_local) + T[:3, 3]

    def tip_dir(self, q):
        T = np.eye(4)
        for Tr, a, th in zip(self.Trel, self.axes, q):
            T = T @ Tr @ self._rot(a, th)
        return ((T[:3, :3] @ self.tip_local) + T[:3, 3],
                T[:3, :3] @ self.pdir)

    def solve_pick(self, target, w_orient=0.5):
        """IK pentru tinta (x,y,z) in frame base_link: pan analitic din
        bearing + cautare grid/rafinare pe lift/elbow/wrist, cu gripperul
        tinut vertical (greutate w_orient). Daca tinta e la marginea
        anvelopei verticale, reincearca automat cu orientare relaxata.
        Returneaza (q, eroare_pozitie) sau (None, err)."""
        target = np.asarray(target, float)
        # pan din diferenta de bearing fata de punctul nominal de pick
        b_nom = math.atan2(NOMINAL_PICK[1] - self.pivot[1],
                           NOMINAL_PICK[0] - self.pivot[0])
        b_tgt = math.atan2(target[1] - self.pivot[1],
                           target[0] - self.pivot[0])
        dpan = math.atan2(math.sin(b_tgt - b_nom), math.cos(b_tgt - b_nom))
        pan = self.pan_sign * dpan

        def cost(q):
            p, d = self.tip_dir(q)
            # tinta + gripper orientat STRICT VERTICAL (coborare dreapta,
            # fara efectul "cupa de excavator")
            return float(np.linalg.norm(p - target)
                         + w_orient * np.linalg.norm(d - np.array([0, 0, -1.0])))

        best = (1e9, None)
        for L in np.arange(-1.5, 0.95, 0.12):
            for E in np.arange(-1.6, 0.65, 0.12):
                for W in np.arange(-1.5, 1.91, 0.18):
                    q = [pan, L, E, W]
                    c = cost(q)
                    if c < best[0]:
                        best = (c, q)
        q = list(best[1])
        for step in (0.05, 0.02, 0.008):
            improved = True
            while improved:
                improved = False
                for i in (0, 1, 2, 3):
                    lim = 3.14 if i == 0 else 1.9
                    for s in (-step, step):
                        q2 = list(q)
                        q2[i] = min(lim, max(-lim, q2[i] + s))
                        c = cost(q2)
                        if c < best[0]:
                            best = (c, q2)
                            q = q2
                            improved = True
        # acceptarea se face DOAR pe eroarea de pozitie; orientarea
        # verticala e o preferinta puternica in cost, nu un criteriu
        p, _ = self.tip_dir(best[1])
        pos_err = float(np.linalg.norm(p - target))
        if pos_err < 0.02:
            return best[1], pos_err
        if w_orient > 0.15:
            # marginea anvelopei verticale: relaxeaza orientarea si reia
            return self.solve_pick(target, w_orient=0.1)
        return None, pos_err

    def solve_near(self, q0, target):
        """Rafinare LOCALA pornind din q0 (aceeasi ramura IK) — folosita
        pentru punctele intermediare ale coborarii verticale, ca
        interpolarea sa ramana o linie dreapta."""
        target = np.asarray(target, float)

        def cost(q):
            p, d = self.tip_dir(q)
            return float(np.linalg.norm(p - target)
                         + 0.5 * np.linalg.norm(d - np.array([0, 0, -1.0])))

        q = list(q0)
        best = (cost(q), q)
        for step in (0.06, 0.02, 0.008):
            improved = True
            while improved:
                improved = False
                for i in (1, 2, 3):
                    for s in (-step, step):
                        q2 = list(best[1])
                        q2[i] = min(1.9, max(-1.9, q2[i] + s))
                        c = cost(q2)
                        if c < best[0]:
                            best = (c, q2)
                            improved = True
        return best[1]


class ManipulationInferNodeSim(Node):
    def __init__(self):
        super().__init__(
            'manipulation_infer_node',
            parameter_overrides=[Parameter('use_sim_time', value=True)])

        self.state = MState.IDLE
        self.busy = False
        self.episode_idx = 0
        self.n_episodes = 1
        self.abort_requested = False
        self.current_pose = list(POSE_HOME)
        self._plan = []
        self._seg_tick = 0
        self._session_results = []
        self._episode_t0 = 0.0
        self._pending_episodes = 0
        self._target_obj = None       # numele obiectului tintit

        # pozele ground-truth (model -> (x, y, z, yaw))
        self.world_poses = {}

        # ── cinematica (fk_data.json din share/config)
        fk_path = self._find_fk_data()
        self.kin = ArmKinematics(fk_path)
        self.get_logger().info(f'FK/IK incarcat din {fk_path}')

        # ── Publishers comenzi articulatii
        self.joint_pubs = {}
        for j in ARM_JOINTS + GRIPPER_JOINTS:
            self.joint_pubs[j] = self.create_publisher(
                Float64, f'/arm/{j}/cmd_pos', 10)

        # ── attach/detach (DetachableJoint): sudarea obiectului apucat
        self.attach_pubs, self.detach_pubs = {}, {}
        for obj in GRASPABLE:
            self.attach_pubs[obj] = self.create_publisher(
                Empty, f'/gripper/attach_{obj}', 10)
            self.detach_pubs[obj] = self.create_publisher(
                Empty, f'/gripper/detach_{obj}', 10)
        # pluginul ataseaza automat la pornire -> desfacem TOT imediat
        # si inca o data dupa ce bridge-ul s-a conectat sigur
        self._detach_all()
        self._startup_detach_timer = self.create_timer(
            2.0, self._startup_detach)

        # ── Interfata /manip_* (identica cu nodul real)
        self.pub_ready = self.create_publisher(Bool, '/manip_ready', RELIABLE_QOS)
        self.pub_done = self.create_publisher(Bool, '/manip_done', RELIABLE_QOS)
        self.pub_status = self.create_publisher(String, '/manip_status', RELIABLE_QOS)
        self.pub_result = self.create_publisher(String, '/manip_result', RELIABLE_QOS)
        self.create_subscription(Bool, '/manip_trigger', self._on_trigger, RELIABLE_QOS)
        self.create_subscription(Int32, '/manip_n_episodes', self._on_n_episodes, RELIABLE_QOS)
        self.create_subscription(Bool, '/manip_abort', self._on_abort, RELIABLE_QOS)
        self.create_subscription(Bool, '/manip_go_home', self._on_go_home, RELIABLE_QOS)

        # ── ground-truth (echivalentul perceptiei)
        self.create_subscription(TFMessage, '/sim/dynamic_poses',
                                 self._on_world_poses, 10)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = Path.home() / 'manip_sim_logs' / f'session_{ts}'
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.ctrl_hz = 20.0
        self.create_timer(1.0 / self.ctrl_hz, self._control_tick)
        self.create_timer(0.5, self._publish_status)
        self.create_timer(1.0, self._publish_ready)

        self._start_motion(MState.HOMING, [(HOMING_DURATION, POSE_HOME)])
        self.get_logger().info(
            'manipulation_infer_node (SIM Gazebo, pick adaptiv) pornit | '
            f'log={self.log_dir}')

    # ── DetachableJoint: sudarea/desfacerea obiectului apucat ─────────
    def _detach_all(self):
        for pub in self.detach_pubs.values():
            pub.publish(Empty())

    def _startup_detach(self):
        # pluginul DetachableJoint ataseaza automat la pornire; dupa ce
        # bridge-ul e sigur conectat, desfacem tot inca o data si oprim
        self._detach_all()
        self._startup_detach_timer.cancel()
        self.get_logger().info('DetachableJoint: toate obiectele desfacute')

    def _grip_action(self, action):
        if action == 'attach' and self._target_obj in self.attach_pubs:
            self.attach_pubs[self._target_obj].publish(Empty())
            self.get_logger().info(
                f'Obiect {self._target_obj} SUDAT de gripper (attach)')
        elif action == 'detach':
            self._detach_all()
            self.get_logger().info('Obiect eliberat in cutie (detach)')

    @staticmethod
    def _find_fk_data():
        try:
            from ament_index_python.packages import get_package_share_directory
            p = Path(get_package_share_directory('xplorer_omx_sim')) / 'config' / 'fk_data.json'
            if p.exists():
                return p
        except Exception:
            pass
        return Path(__file__).resolve().parents[1] / 'config' / 'fk_data.json'

    # ════════════════════════════════════════════════════════════════
    # Callbacks
    # ════════════════════════════════════════════════════════════════
    def _on_world_poses(self, msg: TFMessage):
        for t in msg.transforms:
            name = t.child_frame_id
            tr = t.transform.translation
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            self.world_poses[name] = (tr.x, tr.y, tr.z, yaw)

    def _robot_pose(self):
        return self.world_poses.get('xplorer_omx')

    def _objects(self):
        return {k: v for k, v in self.world_poses.items()
                if k.startswith('obj_')}

    def _to_base(self, wx, wy, wz):
        """world -> frame base_link (din ground-truth-ul robotului)."""
        rp = self._robot_pose()
        if rp is None:
            return None
        rx, ry, rz, ryaw = rp
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        dx, dy = wx - rx, wy - ry
        return np.array([c * dx - s * dy, s * dx + c * dy, wz - rz])

    def _on_trigger(self, msg: Bool):
        if not msg.data:
            return
        if self.busy:
            self.get_logger().warn('Trigger ignorat — sesiune in curs')
            return
        self.abort_requested = False
        self._session_results = []
        self._pending_episodes = self.n_episodes
        self.episode_idx = 0
        self.get_logger().info(
            f'Trigger primit — sesiune cu {self.n_episodes} episod(e)')
        self._begin_episode()

    def _on_n_episodes(self, msg: Int32):
        self.n_episodes = max(1, int(msg.data))

    def _on_abort(self, msg: Bool):
        if msg.data and self.busy:
            self.get_logger().warn('ABORT primit — revin in home')
            self.abort_requested = True

    def _on_go_home(self, msg: Bool):
        if msg.data and not self.busy:
            self._start_motion(MState.HOMING, [(HOMING_DURATION, POSE_HOME)])

    # ════════════════════════════════════════════════════════════════
    # Episod cu pick adaptiv
    # ════════════════════════════════════════════════════════════════
    def _find_target_object(self):
        """Obiectul cel mai apropiat de zona de lucru din spatele
        robotului; returneaza (nume, pozitie_in_base) sau (None, None)."""
        best = (None, None, 1e9)
        for name, (wx, wy, wz, _) in self._objects().items():
            rel = self._to_base(wx, wy, wz)
            if rel is None:
                continue
            d = float(np.linalg.norm(rel[:2] - NOMINAL_PICK))
            if d < best[2]:
                best = (name, rel, d)
        name, rel, d = best
        if name is None or d > 0.30:
            return None, None
        return name, rel

    def _begin_episode(self):
        """Episodul ruleaza INTOTDEAUNA — robotul a fost parcat de Nav2
        exact la poza calculata din pozitia obiectului, deci obiectul e in
        punctul nominal de pick. Daca pozele din Gazebo sunt disponibile,
        IK-ul corecteaza fin restul de eroare de parcare; daca nu, se
        executa secventa nominala (ca pe robotul real)."""
        self.episode_idx += 1
        self._episode_t0 = time.time()

        q_pick, q_pre = None, None
        name, rel = self._find_target_object()
        self._target_obj = name
        if name is not None:
            # tinta coboara sub centrul obiectului, pana aproape de sol
            # (fizica opreste lamelele la contact) — clampeaza obiectul
            # la baza, nu aerul de deasupra lui
            target_pick = np.array([rel[0], rel[1],
                                    max(rel[2] - 0.019, -0.0603)])
            q_pick, err_pick = self.kin.solve_pick(target_pick)
            if q_pick is not None:
                self.get_logger().info(
                    f'Episod {self.episode_idx}: {name} rel='
                    f'({rel[0]:.3f},{rel[1]:.3f}) corectie IK '
                    f'{err_pick * 1000:.0f}mm — HOMING -> RUNNING')
        if q_pick is None:
            # fallback: secventa nominala fixa (scenariul cu un obiect)
            q_pick = POSE_PICK[:4]
            if self._target_obj is None:
                self._target_obj = 'obj_0'
            self.get_logger().info(
                f'Episod {self.episode_idx}: secventa nominala '
                f'(obiect la punctul de pick) — HOMING -> RUNNING')
            target_pick = None

        # punctele intermediare din ACEEASI ramura IK -> coborare/ridicare
        # pe verticala (deviatie laterala < 4 mm), fara matura laterala
        if target_pick is not None:
            q_mid = self.kin.solve_near(q_pick, target_pick + [0, 0, 0.075])
            q_pre = self.kin.solve_near(q_mid, target_pick + [0, 0, 0.145])
        else:
            q_mid = POSE_MID[:4]
            q_pre = POSE_PREPICK[:4]

        pre = list(q_pre) + [GRIP_OPEN]
        mid = list(q_mid) + [GRIP_OPEN]
        pick = list(q_pick) + [GRIP_OPEN]
        grasp = list(q_pick) + [GRIP_CLOSED]
        mid_up = list(q_mid) + [GRIP_CLOSED]      # ridicare dreapta in sus
        lift_adapt = [q_pick[0]] + POSE_LIFT[1:4] + [GRIP_CLOSED]
        # actiunile 'attach'/'detach' (DetachableJoint) se executa la
        # FINALUL segmentului respectiv: dupa strangere obiectul e sudat
        # de gripper (nu mai aluneca), la eliberare e desfacut in cutie
        plan = [
            (2.0, POSE_HOME),
            (3.0, pre),
            (1.5, mid),
            (1.5, pick),                          # coborare verticala
            (1.5, grasp),
            (1.0, grasp, 'attach'),               # strans -> sudat
            (1.5, mid_up),                        # ridicare verticala
            (2.0, lift_adapt),
            (1.5, [0.0] + POSE_LIFT[1:4] + [GRIP_CLOSED]),  # pan inapoi la 0
            (2.5, POSE_BOX_OVER),     # rotire pan spre cutia de langa brat
            (1.5, POSE_BOX_DROP),
            (1.5, POSE_BOX_DROP[:4] + [GRIP_OPEN], 'detach'),  # eliberat
            (1.0, POSE_BOX_DROP[:4] + [GRIP_OPEN]),
            (2.0, POSE_BOX_OVER[:4] + [GRIP_OPEN]),
            (3.0, POSE_HOME),
        ]
        self._start_motion(MState.RUNNING, plan, homing_first=True)

    def _finish_episode(self, success, reason, placed=None,
                        placed_reason=''):
        dur = round(time.time() - self._episode_t0, 2)
        self._session_results.append({
            'episode': self.episode_idx, 'success': success,
            'reason': reason, 'object': self._target_obj,
            'placed_in_box': placed, 'placed_reason': placed_reason,
            'duration_s': dur})
        self.get_logger().info(
            f'Episod {self.episode_idx}: '
            f'{"COMPLET" if success else "ESEC"} ({reason}'
            + (f', obiect in cutie: {"DA" if placed else "NU"}'
               if placed is not None else '')
            + f') in {dur}s')
        self._pending_episodes -= 1
        if self._pending_episodes > 0 and not self.abort_requested:
            self._begin_episode()
            return
        self._finish_session()
        self.busy = False
        self.state = MState.IDLE

    def _verify_placed_in_box(self):
        """Obiectul tintit e fizic in cutia de pe sasiu?"""
        if self._target_obj is None:
            return False, 'no_target'
        wp = self.world_poses.get(self._target_obj)
        if wp is None:
            return False, 'no_pose'
        rel = self._to_base(*wp[:3])
        in_xy = (abs(rel[0] - BOX_CENTER[0]) < BOX_HALF
                 and abs(rel[1] - BOX_CENTER[1]) < BOX_HALF)
        above_deck = rel[2] > 0.05
        if in_xy and above_deck:
            return True, 'placed_in_box'
        # diagnostic: ridicat dar cazut pe langa / ramas pe sol
        if rel[2] < 0.03 and np.linalg.norm(rel[:2] - NOMINAL_PICK) < 0.3:
            return False, 'grasp_failed'
        return False, 'placed_outside_box'

    # ════════════════════════════════════════════════════════════════
    # Executie miscari (interpolare liniara pe segmente, 20 Hz)
    # ════════════════════════════════════════════════════════════════
    def _start_motion(self, state, schedule, homing_first=False):
        self.busy = True
        self.state = MState.HOMING if homing_first else state
        self._target_state = state
        self._plan = []
        prev = list(self.current_pose)
        for item in schedule:
            dur, target = item[0], item[1]
            action = item[2] if len(item) > 2 else None
            ticks = max(1, int(dur * self.ctrl_hz))
            self._plan.append([ticks, list(prev), list(target), action])
            prev = list(target)
        self._seg_tick = 0
        self._homing_first = homing_first

    def _control_tick(self):
        if not self._plan:
            return
        if self.abort_requested and self.state == MState.RUNNING:
            self.abort_requested = False
            self._plan = []
            self._detach_all()    # siguranta: nu ramane nimic sudat
            self.state = MState.ABORTED
            self._session_results.append({
                'episode': self.episode_idx, 'success': False,
                'reason': 'aborted', 'object': self._target_obj})
            self._pending_episodes = 0
            self._start_motion(MState.HOMING, [(3.0, POSE_HOME)])
            return

        ticks, start, target, action = self._plan[0]
        self._seg_tick += 1
        a = min(1.0, self._seg_tick / ticks)
        self.current_pose = [s + (t - s) * a for s, t in zip(start, target)]
        self._send_pose(self.current_pose)

        if self._homing_first and self._seg_tick >= ticks:
            self.state = self._target_state
            self._homing_first = False

        if self._seg_tick >= ticks:
            self._plan.pop(0)
            self._seg_tick = 0
            if action:
                self._grip_action(action)
            if not self._plan:
                self._on_motion_complete()

    def _on_motion_complete(self):
        if self.state == MState.RUNNING:
            # ca pe sistemul real: episodul COMPLET = succes pe /manip_done;
            # verdictul fizic (obiect in cutie) e raportat separat in
            # result JSON (echivalentul scorarii vizuale din teza)
            placed, placed_reason = self._verify_placed_in_box()
            self._finish_episode(True, 'completed',
                                 placed=placed, placed_reason=placed_reason)
            return
        self.busy = False
        self.state = MState.IDLE

    def _finish_session(self):
        n = len(self._session_results)
        ok = sum(1 for r in self._session_results if r['success'])
        summary = {
            'timestamp': datetime.now().isoformat(),
            'mode': 'gazebo_sim_adaptive',
            'episodes': self._session_results,
            'totals': {'n_episodes': n, 'n_success': ok,
                       'n_failed': n - ok,
                       'rate': round(ok / max(n, 1), 3)},
        }
        out = self.log_dir / 'session_summary.json'
        with open(out, 'w') as f:
            json.dump(summary, f, indent=2)
        self.pub_result.publish(String(data=json.dumps(summary)))
        self.pub_done.publish(Bool(data=ok == n and n > 0))
        self.get_logger().info(
            f'Sesiune terminata: {ok}/{n} | /manip_done={ok == n and n > 0}')

    # ════════════════════════════════════════════════════════════════
    def _send_pose(self, pose):
        for i, j in enumerate(ARM_JOINTS):
            self.joint_pubs[j].publish(Float64(data=float(pose[i])))
        for j in GRIPPER_JOINTS:
            self.joint_pubs[j].publish(
                Float64(data=float(GRIP_SIGN[j] * pose[len(ARM_JOINTS)])))

    def _publish_status(self):
        self.pub_status.publish(String(data=json.dumps({
            'state': self.state, 'busy': self.busy,
            'episode': self.episode_idx, 'n_episodes': self.n_episodes,
            'target': self._target_obj,
            'ts': datetime.now().isoformat(timespec='seconds')})))

    def _publish_ready(self):
        self.pub_ready.publish(Bool(data=not self.busy))


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationInferNodeSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
