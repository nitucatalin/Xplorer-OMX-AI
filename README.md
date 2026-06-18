# Zxplorer-OMX-AI — manipulare mobilă autonomă prin Physical AI

Cod sursă al sistemului din lucrarea de diplomă: un braț robotic OMX-AI montat
pe platforma mobilă autonomă Xplorer-A, care execută autonom un task de apucare
și plasare prin învățare prin imitație (politica ACT), integrat cu navigarea Nav2.

Sistemul este distribuit pe două plăci edge care comunică prin CycloneDDS
(Domain 50, Ethernet dedicat):

- **Raspberry Pi 5 (ROS2 Jazzy)** — navigarea Nav2 și orchestrarea misiunii.
- **Jetson Orin Nano (ROS2 Humble)** — inferența ACT și controlul brațului.

## Structura

```
real_robot/    cod pentru robotul fizic (inferență, orchestrare, pornire)
gazebo_sim/    replica de simulare Gazebo (workspace ROS2)
```

## Scripturi principale (real_robot/)

| Fișier | Ce face |
|---|---|
| `manipulation_infer_node.py` | Nodul de inferență ACT pe Jetson. Mașina de stări IDLE→HOMING→RUNNING→IDLE, topicurile `/manip_*`, comandă brațul direct prin Dynamixel SDK. |
| `infer_loop_v5.py` | Bucla de inferență ACT și I/O braț+cameră prin clasa validată `OmxFollower` (Dynamixel SDK + cameră). |
| `start_all.sh` | Pornire completă pe RPi5: cleanup, mediu CycloneDDS Domain 50, hardware bringup + stack Nav2. |
| `start_jetson.sh` | Pornire pe Jetson: nodul de inferență + peering DDS către RPi5, cu homing și pre-flight cameră. |
| `verify_system.sh` | Verificare stare sistem (se sursează, nu se rulează): noduri, topicuri și comunicația cross-platform pe Domain 50. |
| `go_collect_real.py` | Campanie end-to-end pe robotul real (Nav2 → manipulare → revenire), cu logare JSON. |
| `record_campaign.sh` | Înregistrare rosbag a topicurilor de orchestrare în timpul campaniei. |
| `capture_arm_pose.py` | Captarea pozițiilor articulare ale brațului (home/idle) cu torque oprit. |
| `scan_filter_arm.yaml` | Config laser_filters pentru a scoate brațul din scanul LiDAR. |

## Pornire (rezumat)

```bash
# RPi5
./start_all.sh
# Jetson
./start_jetson.sh
# verificare (terminal nou)
source verify_system.sh
```

Detalii complete în `real_robot/README_REAL.md`.

## Notă

Bag-urile, modelele antrenate, fișierele CAD și media grea nu sunt incluse în
repo (vezi `.gitignore`); modelele ACT sunt publicate pe Hugging Face Hub.
