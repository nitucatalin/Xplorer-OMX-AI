# Campania end-to-end pe ROBOTUL REAL — fișiere, transfer, comenzi

Echivalentul exact al scenariului din simulare, pe sistemul fizic
RPi5 (Jazzy) ↔ Jetson Orin Nano (Humble): POI-A → Nav2 → POI-B
(goal succeeded) → `/manip_trigger` → inferență ACT → `/manip_done` →
revenire POI-A, repetat de N ori, cu JSON + rosbag pentru capitolele
3.4/3.5 — **aceleași câmpuri ca în JSON-urile din simulare**, deci
tabelele din teză se completează identic (coloana „real" lângă „simulare").

## 1. Fișierele și unde merg

| Fișier | Destinație | Rol |
|---|---|---|
| `go_collect_real.py` | RPi5 `~/` | comanda de campanie (parametrizată: --poi --home --runs --pause --episodes --label) |
| `record_campaign.sh` | RPi5 `~/` | rosbag cu toate topicurile de orchestrare |
| `manipulation_infer_node.py` | Jetson `~/ros2_manip/` | nodul de inferență (mașina de stări IDLE→HOMING→RUNNING→IDLE, topicurile `/manip_*`) — construit peste clasele VALIDATE din `infer_loop_v5.py` |

**Despre nodul de pe Jetson:** versiunea de aici este verificată 1:1 contra
contractului orchestratorului (topicuri, QoS RELIABLE/depth 5, formatul JSON
din `/manip_status` și `/manip_result`) și folosește pentru I/O exact
`OmxFollower` din `infer_loop_v5.py` (cel care „manipulează corect" — Jurnal
D2). Față de versiuni anterioare adaugă: `/manip_ready` republicat la 1 Hz
(orchestratorul poate porni oricând), heartbeat 2 Hz neîntrerupt și în
timpul episodului (rulează în thread separat), abort funcțional în mijlocul
episodului, parametru `n_action_steps` (0 = chunk-ul politicii, deci 50 la
`pbn2`) și `home_pose` parametrizabil. **Fă backup la nodul existent înainte
de înlocuire** (comanda mai jos) — dacă cel vechi îți funcționa, îl poți
restaura oricând.

## 2. Transfer de pe MacBook (comenzile scp)

```bash
cd ~/Documents/fac/Licenta/real_robot

# ── RPi5 (pi@10.0.0.1; pe WiFi: pi@192.168.53.177) ──
scp go_collect_real.py record_campaign.sh pi@10.0.0.1:~/
ssh pi@10.0.0.1 'chmod +x ~/record_campaign.sh'

# ── Jetson (jnfiir@10.0.0.2; pe WiFi: jnfiir@192.168.53.57) ──
# 1. BACKUP la nodul existent:
ssh jnfiir@10.0.0.2 'cp ~/ros2_manip/manipulation_infer_node.py \
    ~/ros2_manip/manipulation_infer_node.backup_$(date +%F).py 2>/dev/null; true'
# 2. transferul noului nod + infer_loop_v5 (nodul il importa; il ai si
#    local in Licenta — garantam aceeasi versiune pe Jetson):
scp manipulation_infer_node.py jnfiir@10.0.0.2:~/ros2_manip/
scp ../infer_loop_v5.py jnfiir@10.0.0.2:~/ros2_manip/
```

> Dacă SSH-ul tău merge prin WiFi, înlocuiește IP-urile (RPi5
> `192.168.53.177`, Jetson `192.168.53.57` — ca în verify_lenovo.sh).
> Orchestrarea DDS rămâne oricum pe Ethernetul direct 10.0.0.x.

## 3. Setarea home_pose pe Jetson (IMPORTANT, o singură dată)

Nodul are nevoie de poziția home calibrată (unități LeRobot). Ia cele 6
valori din `omx_init_home.py` / dintr-o rulare `capture_pose.py` și
lansează-l cu parametrul setat — fie editezi `start_jetson.sh` să adauge
parametrul, fie testezi manual:

```bash
# pe Jetson (test manual, fara start_jetson.sh):
cd ~/ros2_manip && source ~/setup_manip_bridge.bash
python3 manipulation_infer_node.py --ros-args -p stub_mode:=false \
    -p home_pose:="[p1, p2, p3, p4, p5, p6]" \
    -p model_path:=/home/jnfiir/lerobot_models/model_act_licenta
```

Fără `home_pose`, nodul folosește poziția curentă a brațului la pornire
(avertizează în log) — funcționează, dar homing-ul nu duce brațul în poza
de repaus calibrată. `stub_mode:=true` testează tot lanțul fără braț.

## 4. Procedura campaniei (ordinea exactă)

1. **RPi5 T1:** `./start_all.sh` — așteaptă bannerul SISTEM PORNIT.
2. **Jetson T2:** `./start_jetson.sh` — așteaptă heartbeat-urile.
3. **RPi5 T3:** inițializează AMCL (RUNBOOK_J5_end2end pașii 3.1–3.4:
   initialpose la HOME + push scurt pentru convergență).
4. **RPi5 T4 (rosbag):**
   `source ~/setup_manip_bridge.bash && source ~/saim_xplorer/install/setup.bash && ./record_campaign.sh licenta`
5. **Pune obiectul la POI** (în calota de lucru, ca la antrenare).
6. **RPi5 T5 — campania:**
   ```bash
   source ~/setup_manip_bridge.bash && source ~/saim_xplorer/install/setup.bash
   python3 ~/go_collect_real.py --poi 2.5 0.5 0.0 --home 1.0 0.0 0.0 \
           --runs 5 --pause 15 --label campania_reala
   ```
   În pauzele dintre rulări scriptul afișează „REPOZIȚIONEAZĂ OBIECTUL" —
   pune obiectul la loc la POI.
7. La final: Ctrl+C în T4 (închide bag-ul), apoi opririle din RUNBOOK_J5
   pasul 12 (Jetson cu mâna sub braț!).

## 5. Recuperarea datelor pe MacBook

```bash
# JSON-ul campaniei (tabelul 3.5, coloana "real"):
scp pi@10.0.0.1:~/mission_logs/campania_*.json \
    ~/Documents/fac/Licenta/real_robot/rezultate/

# rosbag-ul (latente 3.4, mesaje per topic — aceeasi analiza ca in simulare):
scp -r pi@10.0.0.1:~/bags ~/Documents/fac/Licenta/real_robot/rezultate/

# jurnalele nodului de inferenta de pe Jetson:
scp -r jnfiir@10.0.0.2:~/infer_logs ~/Documents/fac/Licenta/real_robot/rezultate/
```

## 6. Maparea pe teză

- **3.4:** din rosbag — latența `/manip_trigger`→HOMING (din `/manip_status`),
  frecvența heartbeat (2 Hz), 0 pierderi (N trigger = N done = N result),
  precizia la POI (`pose_at_poi_amcl` din JSON vs coordonatele goal-ului).
  Comparație directă cu tabelele din simulare (aceeași metodă de extracție).
- **3.5:** din `campania_*.json` — `success_rate` + Wilson 95%,
  `avg_nav_to_poi_s` / `avg_manip_s` / `avg_nav_to_home_s`, per-run în
  `runs[]`; reușita FIZICĂ a manipulării se scorează vizual per rulare
  (notează în caiet: apucat / scăpat / ratat), ca în secțiunea 3.3.
