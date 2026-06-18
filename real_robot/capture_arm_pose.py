#!/usr/bin/env python3
"""
capture_arm_pose.py — capteaza pozitiile articulatiilor bratului OMX-AI in
ACELEASI unitati ca modelul (Present_Position prin OmxFollower, scara obs_mean),
cu TORQUE OFF ca sa poti misca bratul cu mana. Implementeaza pasul J1 din jurnal.

Ruleaza pe Jetson (cu env-ul de la setup_manip_bridge / start_jetson):
    python3 ~/ros2_manip/capture_arm_pose.py            # /dev/ttyACM0 implicit
    python3 ~/ros2_manip/capture_arm_pose.py --port /dev/ttyACM0

Procedura:
  1. Ruleaza scriptul -> torque OFF (bratul devine liber).
  2. Asaza bratul CU MANA in poza dorita (intai HOME = impachetat, apoi rulezi
     din nou pentru IDLE = ridicat-neutru, fara coliziuni).
  3. Cand bratul e in poza, citeste valorile afisate live, apoi Ctrl+C.
  4. Scriptul printeaza lista gata de pus in HOME_POSE / IDLE_POSE din start_jetson.

ATENTIE: cu torque OFF bratul cade sub gravitatie — tine-l cu mana.
"""
import argparse
import time

MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM0')
    args = ap.parse_args()

    from lerobot.robots.omx_follower.omx_follower import OmxFollower
    from lerobot.robots.omx_follower.config_omx_follower import OmxFollowerConfig

    cfg = OmxFollowerConfig(port=args.port, id="omx_capture", cameras={})
    robot = OmxFollower(cfg)
    print("Conectare OmxFollower...")
    robot.connect()  # ruleaza configure() (calibrare deja salvata)

    # torque OFF ca sa misti bratul cu mana
    disabled = False
    for fn in ("disable_torque", "disable_torque_all"):
        try:
            getattr(robot.bus, fn)()
            disabled = True
            break
        except Exception:
            pass
    if not disabled:
        try:
            robot.bus.sync_write("Torque_Enable", {m: 0 for m in MOTORS})
            disabled = True
        except Exception as e:
            print(f"[WARN] nu am putut dezactiva torque automat ({e!r}) — "
                  "daca bratul e teapan, opreste si dezactiveaza-l manual.")
    print("[torque OFF]" if disabled else "[torque INCA ON?]")
    print("Misca bratul in poza dorita. Ctrl+C cand e gata.\n")

    last = None
    try:
        while True:
            d = robot.bus.sync_read("Present_Position")
            last = [round(float(d[m]), 2) for m in MOTORS]
            print("   pozitii: [" + ", ".join(f"{v:7.2f}" for v in last) + "]",
                  end="\r", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if last is not None:
            print("\n\nPoza capturata — pune-o in start_jetson.sh "
                  "(HOME_POSE sau IDLE_POSE), cu .0:")
            print("  [" + ", ".join(f"{v}" for v in last) + "]")
        try:
            robot.disconnect()
        except Exception:
            pass


if __name__ == '__main__':
    main()
