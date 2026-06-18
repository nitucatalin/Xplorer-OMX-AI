#!/usr/bin/env python3
"""
infer_loop_v5.py — inferenta ACT pe OMX-F prin robotul LeRobot `omx_follower`.

FIX fata de v4: I/O-ul motoarelor + camera se fac prin clasa oficiala `OmxFollower`
(aceeasi folosita de `lerobot-record`, care manipuleaza corect), NU prin acces direct
Dynamixel. Astfel se aplica automat:
  - Operating_Mode EXTENDED_POSITION (articulatii) / CURRENT_POSITION (gripper)
  - PID-ul reglat pe elbow (inchide gap-ul stare<->actiune din antrenare)
  - calibrarea + citirea cu semn a pozitiilor
  - camera livrata deja in RGB
Politica ACT, normalizarea MEAN_STD si logarea JSON raman ca in v4.
"""

import argparse, time, json, csv, os, sys
from pathlib import Path
from datetime import datetime
import numpy as np
import torch

MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


# ══════════════════════════════════════════════════════════════════
# Normalizare / Denormalizare (MEAN_STD din safetensors)
# ══════════════════════════════════════════════════════════════════
class Normalizer:
    def __init__(self, model_path: str):
        from safetensors.torch import load_file
        pre = load_file(f'{model_path}/policy_preprocessor_step_3_normalizer_processor.safetensors')
        post = load_file(f'{model_path}/policy_postprocessor_step_0_unnormalizer_processor.safetensors')
        self.obs_mean = pre['observation.state.mean'].numpy()
        self.obs_std = pre['observation.state.std'].numpy()
        self.img_mean = pre['observation.images.front.mean'].numpy()
        self.img_std = pre['observation.images.front.std'].numpy()
        self.act_mean = post['action.mean'].numpy()
        self.act_std = post['action.std'].numpy()
        print(f'  Normalizer: obs_mean={np.round(self.obs_mean, 2)}, act_mean={np.round(self.act_mean, 2)}')

    def normalize_state(self, state):
        return (np.asarray(state) - self.obs_mean) / (self.obs_std + 1e-8)

    def normalize_image(self, img_01):
        mean = torch.tensor(self.img_mean, device=img_01.device).reshape(1, 3, 1, 1)
        std = torch.tensor(self.img_std, device=img_01.device).reshape(1, 3, 1, 1)
        return (img_01 - mean) / (std + 1e-8)

    def denormalize_action(self, action_norm):
        return np.asarray(action_norm) * self.act_std + self.act_mean


# ══════════════════════════════════════════════════════════════════
# ACT Policy
# ══════════════════════════════════════════════════════════════════
class ACTChunkPolicy:
    def __init__(self, model_path):
        from lerobot.policies.act.modeling_act import ACTPolicy
        self.policy = ACTPolicy.from_pretrained(model_path)
        self.policy.eval().cuda()
        self.chunk_size = self.policy.config.chunk_size
        self.n_joints = self.policy.config.output_features['action'].shape[0]
        self.norm = Normalizer(model_path)
        params = sum(p.numel() for p in self.policy.parameters())
        print(f'  Model: {params/1e6:.1f}M params | chunk={self.chunk_size} | joints={self.n_joints}')

    def get_chunk(self, frame_rgb, joint_lerobot_vals):
        """frame_rgb: imagine RGB (uint8, HxWx3) livrata de robotul LeRobot (NU se mai inverseaza)."""
        state_norm = self.norm.normalize_state(joint_lerobot_vals[:self.n_joints])
        state_t = torch.from_numpy(state_norm.astype(np.float32)).unsqueeze(0).cuda()
        img = np.ascontiguousarray(frame_rgb)
        img_01 = torch.from_numpy(img).permute(2, 0, 1).float().unsqueeze(0).cuda() / 255.0
        img_norm = self.norm.normalize_image(img_01)
        obs = {'observation.state': state_t, 'observation.images.front': img_norm}
        with torch.no_grad():
            actions_norm = self.policy.predict_action_chunk(obs)
            chunk_norm = actions_norm[0].cpu().numpy()
        return self.norm.denormalize_action(chunk_norm)


# ══════════════════════════════════════════════════════════════════
# Temporal Ensemble
# ══════════════════════════════════════════════════════════════════
class TemporalEnsemble:
    def __init__(self, coeff, nj):
        self.coeff = coeff; self._prev = None; self._consumed = 0
    def reset(self):
        self._prev = None; self._consumed = 0
    def apply(self, new_chunk, nas):
        if self.coeff is None or self.coeff == 0.0 or self._prev is None:
            self._prev = new_chunk.copy(); self._consumed = 0
            return new_chunk[:nas].copy()
        old = self._prev[self._consumed:]
        overlap = min(len(old), len(new_chunk))
        bl = new_chunk.copy()
        for i in range(overlap):
            w = self.coeff * (0.9 ** i)
            bl[i] = w * old[i] + (1.0 - w) * new_chunk[i]
        self._prev = bl.copy(); self._consumed = 0
        return bl[:nas].copy()
    def advance(self, steps=1):
        self._consumed += steps


# ══════════════════════════════════════════════════════════════════
# Sistem real: robotul LeRobot omx_follower (I/O corect, identic cu lerobot-record)
# ══════════════════════════════════════════════════════════════════
class OmxSystem:
    def __init__(self, port, camera, w=640, h=480, fps=30):
        from lerobot.robots.omx_follower.omx_follower import OmxFollower
        from lerobot.robots.omx_follower.config_omx_follower import OmxFollowerConfig
        from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
        cams = {"front": OpenCVCameraConfig(index_or_path=camera, width=w, height=h, fps=fps)}
        cfg = OmxFollowerConfig(port=port, id="omx_follower_arm", cameras=cams)
        self.robot = OmxFollower(cfg)
        self.cam_key = "front"

    def connect(self):
        self.robot.connect()  # ruleaza configure(): moduri operare, PID, calibrare
        print("  OmxFollower conectat (moduri/PID setate prin configure()).")

    def get_obs(self):
        obs = self.robot.get_observation()
        state = np.array([obs[f"{m}.pos"] for m in MOTORS], dtype=np.float32)
        frame = obs[self.cam_key]  # RGB
        return state, frame

    def get_state(self):
        d = self.robot.bus.sync_read("Present_Position")
        return np.array([d[m] for m in MOTORS], dtype=np.float32)

    def send(self, vals):
        action = {f"{m}.pos": float(v) for m, v in zip(MOTORS, vals)}
        self.robot.send_action(action)

    def disconnect(self):
        self.robot.disconnect()


# ── Sistem simulat (--dry-run) pentru test loop/model fara hardware ──
class DrySystem:
    def __init__(self, w=640, h=480):
        self.w = w; self.h = h; self._n = 0; self._v = np.zeros(6, dtype=np.float32)
    def connect(self): print("  [DRY-RUN] sistem simulat (fara hardware)")
    def get_obs(self):
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        s = (self._n * 3) % 256; img[:, :, 0] = s; img[:, :, 1] = 128; img[:, :, 2] = 100; self._n += 1
        return self._v.copy(), img
    def get_state(self): return self._v.copy()
    def send(self, vals): self._v = np.asarray(vals, dtype=np.float32).flatten()
    def disconnect(self): pass


# ══════════════════════════════════════════════════════════════════
# Logger (identic cu v4)
# ══════════════════════════════════════════════════════════════════
class EpisodeLogger:
    def __init__(self, out_dir):
        self.out = Path(out_dir); self.out.mkdir(parents=True, exist_ok=True)
        self._steps = []; self._results = []
    def start_episode(self, idx, config):
        self._ep = idx; self._steps = []; self._cfg = config
        self._t0 = time.perf_counter()
    def log_step(self, step, action, obs, infer_ms, loop_ms, reinfer):
        self._steps.append({'step': step, 'act': np.asarray(action).flatten().tolist(),
                            'obs': np.asarray(obs).flatten().tolist(),
                            'infer_ms': round(infer_ms, 3), 'loop_ms': round(loop_ms, 3), 'reinfer': reinfer})
    def end_episode(self, success, reason=''):
        dur = time.perf_counter() - self._t0
        nj = len(self._steps[0]['act']) if self._steps else 6
        csv_path = self.out / f'episode_{self._ep:04d}_steps.csv'
        fields = ['step', 'infer_ms', 'loop_ms', 'reinfer'] + [f'a{i}' for i in range(nj)] + [f'o{i}' for i in range(nj)]
        with open(csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields); w.writeheader()
            for s in self._steps:
                row = {'step': s['step'], 'infer_ms': s['infer_ms'], 'loop_ms': s['loop_ms'], 'reinfer': int(s['reinfer'])}
                for i, v in enumerate(s['act']): row[f'a{i}'] = round(float(v), 4)
                for i, v in enumerate(s['obs']): row[f'o{i}'] = round(float(v), 4)
                w.writerow(row)
        infer_t = [s['infer_ms'] for s in self._steps if s['reinfer']]
        loop_t = [s['loop_ms'] for s in self._steps]
        n_ri = sum(1 for s in self._steps if s['reinfer'])
        r = {'episode': self._ep, 'success': success, 'reason': reason, 'duration_s': round(dur, 2),
             'total_steps': len(self._steps), 'reinfer_count': n_ri,
             'infer_ms_median': round(np.median(infer_t), 2) if infer_t else 0,
             'loop_ms_median': round(np.median(loop_t), 2) if loop_t else 0,
             'loop_hz_median': round(1000 / np.median(loop_t), 1) if loop_t else 0, 'config': self._cfg}
        self._results.append(r)
        print(f'    Ep {self._ep}: {"OK" if success else "FAIL"} | {r["total_steps"]} steps | '
              f'{n_ri} re-inf | {r["loop_hz_median"]} Hz | {dur:.1f}s')
        return r
    def save_summary(self):
        p = self.out / 'session_summary.json'
        n = len(self._results); ok = sum(1 for r in self._results if r['success'])
        with open(p, 'w') as f:
            json.dump({'timestamp': datetime.now().isoformat(), 'episodes': self._results,
                       'totals': {'n_episodes': n, 'n_success': ok, 'n_failed': n - ok,
                                  'rate': round(ok / max(n, 1), 3)}}, f, indent=2)
        print(f'\n  Sumar: {p}')


# ══════════════════════════════════════════════════════════════════
# Episode / Session
# ══════════════════════════════════════════════════════════════════
def run_episode(act, sysm, ens, logger, ep_idx, args):
    cfg = {'n_action_steps': args.n_action_steps, 'ensemble': args.ensemble, 'fps': args.fps,
           'episode_time_s': args.episode_time, 'chunk_size': act.chunk_size}
    logger.start_episode(ep_idx, cfg)
    ens.reset()
    dt = 1.0 / args.fps
    max_steps = int(args.episode_time * args.fps)
    nas = args.n_action_steps
    action_buf = None; buf_idx = 0; step = 0
    print(f'  Ep {ep_idx}: {max_steps} steps @ {args.fps}Hz, NAS={nas}')

    while step < max_steps:
        t0 = time.perf_counter()
        infer_ms = 0.0; is_ri = False
        if action_buf is None or buf_idx >= nas:
            is_ri = True
            state, frame = sysm.get_obs()
            torch.cuda.synchronize(); ti = time.perf_counter()
            chunk = act.get_chunk(frame, state)
            torch.cuda.synchronize(); infer_ms = (time.perf_counter() - ti) * 1000
            action_buf = ens.apply(chunk, nas); buf_idx = 0

        action = action_buf[buf_idx]
        sysm.send(action)
        buf_idx += 1; ens.advance()

        obs = sysm.get_state()
        loop_ms = (time.perf_counter() - t0) * 1000
        logger.log_step(step, action, obs, infer_ms, loop_ms, is_ri)
        step += 1
        sl = dt - (time.perf_counter() - t0)
        if sl > 0: time.sleep(sl)

    logger.end_episode(True, 'completed')
    return True


def run_session(args):
    nas = args.n_action_steps
    ens_str = args.ensemble if args.ensemble else 'None'
    mode = 'DRY-RUN' if (args.dry_run or args.dry_run_cam) else 'PROD'
    print(f'\n{"="*60}\n  infer_loop_v5 | NAS={nas} ENS={ens_str} FPS={args.fps}\n'
          f'  {args.episodes} ep x {args.episode_time}s | {mode}\n{"="*60}\n')

    print('Setup:')
    act = ACTChunkPolicy(args.model)
    if args.dry_run or args.dry_run_cam:
        sysm = DrySystem(args.img_w, args.img_h)
    else:
        sysm = OmxSystem(args.port, args.camera, args.img_w, args.img_h, args.fps)
    sysm.connect()

    ensemble = TemporalEnsemble(args.ensemble, args.n_joints)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    out = f'{args.output}/session_{ts}_nas{nas}_ens{ens_str}'
    logger = EpisodeLogger(out)
    print(f'  Output: {out}\n')

    ok = 0
    try:
        for i in range(args.episodes):
            if run_episode(act, sysm, ensemble, logger, i, args): ok += 1
            if i < args.episodes - 1:
                p = args.reset_time if not (args.dry_run or args.dry_run_cam) else 1.0
                print(f'    Reset {p}s...'); time.sleep(p)
    except KeyboardInterrupt:
        print('\n  Ctrl+C — oprire...')
    finally:
        logger.save_summary()
        print(f'\n  Rezultat: {ok}/{args.episodes}')
        sysm.disconnect()
    return ok == args.episodes


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--dry-run', action='store_true')
    p.add_argument('--dry-run-cam', action='store_true')  # tratat ca dry-run (sim)
    p.add_argument('--model', default='/home/jnfiir/lerobot_models/model_act_licenta')
    p.add_argument('--n-action-steps', type=int, default=100)
    p.add_argument('--ensemble', type=float, default=None)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--episodes', type=int, default=1)
    p.add_argument('--episode-time', type=float, default=30.0)
    p.add_argument('--reset-time', type=float, default=5.0)
    p.add_argument('--port', default='/dev/ttyACM0')
    p.add_argument('--baudrate', type=int, default=1000000)
    p.add_argument('--camera', default='/dev/video0')
    p.add_argument('--n-joints', type=int, default=6)
    p.add_argument('--img-h', type=int, default=480)
    p.add_argument('--img-w', type=int, default=640)
    p.add_argument('--output', default='/home/jnfiir/infer_logs')
    return p.parse_args()


if __name__ == '__main__':
    args = parse_args()
    if not (args.dry_run or args.dry_run_cam) and not os.path.exists(args.port):
        print(f'WARN: {args.port} absent — activez --dry-run')
        args.dry_run = True
    sys.exit(0 if run_session(args) else 1)
