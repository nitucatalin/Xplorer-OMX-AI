# Simulare Gazebo — scenariul de orchestrare Xplorer-A + OMX-AI

Replica în simulare a sistemului real din licență: platforma Xplorer-A navighează
cu **Nav2** de la HOME la POI-A, la atingerea goal-ului **mașina de stări**
(`mission_orchestrator`) publică `/manip_trigger`, nodul de manipulare
(replica `manipulation_infer_node` de pe Jetson) execută episodul de
pick-and-place pe brațul OMX simulat, publică `/manip_done`, iar platforma
revine la HOME. Aceleași topicuri, aceleași QoS, aceeași mașină de stări ca
în sistemul real RPi5 ↔ Jetson.

**Corespondență sim ↔ real:**

| Real (RPi5 + Jetson) | Simulare (Gazebo) |
|---|---|
| `start_all.sh` (robot + Nav2, Jazzy) | `sim.launch.py` + `nav2.launch.py` |
| `start_jetson.sh` + `manipulation_infer_node` (Humble) | `manip.launch.py` (`manipulation_infer_node_sim`) |
| `mission_orchestrator.py` | `mission_orchestrator_sim` (identic, doar `use_sim_time=True`) |
| Punte CycloneDDS Domain 50 pe eth0 | `ros_gz_bridge` (gz ↔ ROS, același host) |
| Inferență ACT pe GPU (`infer_loop_v5.py`) | Traiectorie pick-place scriptată, aceeași durată/structură de episod |
| Harta cb204 | `lab_map` generată din world-ul `lab_world.sdf` |
| HOME (1.0, 0.0), POI-A (2.5, 0.5) | identice |

> **Limitare asumată (de menționat în teză):** în simulare episodul de
> manipulare NU rulează politica ACT (modelul e legat de domeniul vizual real;
> rularea lui pe imagini sintetice ar eșua din cauza gap-ului sim-real).
> Simularea validează **orchestrarea**: secvențierea Nav2 ↔ trigger ↔ mașini
> de stări ↔ reluarea navigării — exact obiectivele din 3.4/3.5.

---

## 1. Instalare pe macOS (o singură dată)

ROS 2 + Nav2 + Gazebo nu au suport nativ pe macOS, deci totul rulează într-un
container Docker cu desktop Linux accesibil din browser.

1. Instalează **Docker Desktop pentru Mac**: <https://www.docker.com/products/docker-desktop/>
   (Apple Silicon sau Intel — imaginea e multi-arch). Pornește-l și dă-i din
   Settings → Resources minim **4 CPU / 8 GB RAM**.
2. În Terminal:

```bash
cd ~/Documents/fac/Licenta/gazebo_sim/docker
docker compose up --build -d        # prima dată durează ~10-15 min (descarcă imaginea)
```

3. Deschide în browser: **http://localhost:6080** → apare un desktop Linux
   (XFCE). Toate comenzile de mai jos se dau în **terminale din acest desktop**
   (Applications → Terminal Emulator).

4. Build workspace (o singură dată, sau după orice modificare a pachetului):

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

> Workspace-ul e montat din Mac (`gazebo_sim/ros2_ws`), deci poți edita
> fișierele din mac și doar rebuild în container.

---

## 2. Rularea scenariului (4 terminale, ca în RUNBOOK_J5)

| Terminal | Comandă | Echivalent real |
|---|---|---|
| **T1** | `ros2 launch xplorer_omx_sim sim.launch.py` | `start_all.sh` pas 3 (hardware) |
| **T2** | `ros2 launch xplorer_omx_sim nav2.launch.py` | `start_all.sh` pas 4 (Nav2) |
| **T3** | `ros2 launch xplorer_omx_sim manip.launch.py` | `start_jetson.sh` |
| **T4** | orchestratorul (mai jos) | `mission_orchestrator.py` pe RPi5 |

**T1:** se deschide Gazebo cu laboratorul; robotul e spawn-uit la HOME (1.0, 0.0),
cubul roșu și tava sunt la POI-A. Apasă **Play** (▶, stânga-jos) dacă simularea
nu rulează deja (launch-ul pornește cu `-r`, deci ar trebui să ruleze).

**T2:** așteaptă ~10 s; AMCL pornește direct localizat la HOME
(`set_initial_pose=true`) — nu mai e nevoie de 2D Pose Estimate.

**T3:** vezi `manipulation_infer_node (SIM Gazebo) pornit`; brațul face homing
și nodul publică `/manip_ready=True` + heartbeat pe `/manip_status`.

**T4 — O SINGURĂ COMANDĂ (scenariul de bază al tezei):**

În world există **UN SINGUR obiect** (cub la (2.19, 2.06), din
`config/pois.yaml`). Fără niciun argument:

```bash
# POI-A -> POI-B (parcat cu spatele la obiect) -> goal succeeded ->
# inferenta -> obiect in cutie -> inapoi la POI-A:
ros2 run xplorer_omx_sim go_collect
```

Comanda: (1) ia POI-ul precalculat al obiectului din scenă; (2) navighează
cu Nav2; (3) la goal scrie `AM AJUNS LA POI`, apoi rulează **ALINIEREA
FINĂ**: corectează baza din `cmd_vel` (rotire + avans) până când obiectul
e centrat exact între cleștii gripper-ului (eroarea Nav2 de ~10 cm scade
sub 8 mm — controller validat pe 40 de cazuri); (4) publică
`/manip_trigger` — brațul țintește prin IK poziția reală a obiectului și îl
depune în cutia de **lângă** braț; (5) `/manip_done` → revenire la POI-A.
Fiecare rulare scrie `~/mission_logs/go_collect_<ts>.json` cu cronometrarea
segmentelor — **navigare, aliniere, inferență, revenire**, exact câmpurile
tabelului din cap. 3.5 — plus erorile de aliniere și verdictul „obiect în
cutie".

**Campania pentru cap. 3.5 — tot o singură comandă:**

```bash
ros2 run xplorer_omx_sim go_collect --runs 5 --label campania_sim
```

Rulează 5 episoade complete; **obiectul e repoziționat automat la POI între
episoade** (echivalentul pasului manual din RUNBOOK_J5). La final scrie
`~/mission_logs/campania_<label>_<ts>.json` cu: rate de succes (episoade
complete + obiecte în cutie), timpi medii per segment, detalii per rulare.

Alte variante:

```bash
# spawneaza TU un cub la coordonate alese si du-te dupa el:
ros2 run xplorer_omx_sim go_collect --spawn --obiect 3.0 -1.5 --runs 3

# orchestratorul-replica din teza (aceeasi masina de stari ca pe RPi5),
# cu POI-ul din pois.yaml (x, y, yaw):
ros2 run xplorer_omx_sim mission_orchestrator_sim \
    --poi 2.079 1.702 -2.0948 --home 1.0 0.0 0.0 --runs 5 --pause 10
# (intre rulari, in alt terminal: ros2 run xplorer_omx_sim reset_objects)
```

Vei vedea exact secvența din sistemul real:

```
[orch] IDLE -> WAIT_MANIP_READY
[manip] ready=True
[orch] WAIT_MANIP_READY -> NAV_TO_POI (run 1/5)
  Run 1: ajuns la POI în ...s
[orch] AT_POI -> TRIGGER_MANIP
  -> /manip_trigger = True
[orch] TRIGGER_MANIP -> WAIT_MANIP_DONE      (în Gazebo brațul execută pick-place)
[manip] done success=True
[orch] WAIT_MANIP_DONE -> NAV_TO_HOME
  Run 1/5 terminat: nav_poi=...s manip=...s nav_home=...s total=...s success=True
```

> Opțional: `mission_multi_poi` (lanț B→C→D→E cu reîncercare prin POI A) și
> `tools/gen_scene.py --seed N` (alte poziții random) rămân disponibile.

Raportul JSON al campaniei: `~/mission_logs/mission_<ts>_<label>/mission_summary.json`
(în container). Copiere pe Mac:

```bash
# pe Mac:
docker cp xplorer_omx_sim:/home/ubuntu/mission_logs ~/Documents/fac/Licenta/gazebo_sim/rezultate/
docker cp xplorer_omx_sim:/home/ubuntu/manip_sim_logs ~/Documents/fac/Licenta/gazebo_sim/rezultate/
```

### Vizualizare în RViz (opțional, terminal separat)

```bash
ros2 launch xplorer_omx_sim rviz.launch.py
```

Configurația proprie (`config/sim.rviz`) folosește doar pluginuri standard
`rviz_default_plugins` — fără erorile „failed to load plugins" ale config-ului
nav2_bringup (pentru acela am adăugat și `ros-jazzy-nav2-rviz-plugins` în
imagine, dacă vrei să-l folosești).

### Trigger manual pentru debug (ca în start_all.sh)

```bash
ros2 topic pub --once /manip_n_episodes std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /manip_trigger std_msgs/msg/Bool "{data: true}"
ros2 topic echo /manip_status
```

### Înregistrare rosbag pentru teză (echivalent pasul 6 din RUNBOOK_J5_end2end)

```bash
ros2 bag record -o ~/bags/sim_e2e_$(date +%F_%H%M) \
    /tf /tf_static /scan /odom /amcl_pose /cmd_vel /plan \
    /manip_trigger /manip_status /manip_done /manip_result /manip_n_episodes
```

---

## 3. Ce capturezi pentru capitolele 3.4 / 3.5

- **3.4 (integrare ROS2):** `ros2 node list`, `ros2 topic list`, screenshot
  rqt_graph; `ros2 topic hz /scan`, `/manip_status`; tranzițiile de stare din
  log-ul T3/T4 (IDLE→HOMING→RUNNING→IDLE simultan cu NAV_TO_POI→AT_POI→
  TRIGGER_MANIP→WAIT_MANIP_DONE→NAV_TO_HOME).
- **3.5 (end-to-end):** `mission_summary.json` — `success_rate`,
  `avg_nav_to_poi_s`, `avg_manip_s`, `avg_nav_to_home_s`, `avg_total_s`,
  per-run în `runs[]`; capturi video/screenshot din Gazebo cu robotul la POI
  și brațul în mișcare.
- Precizia de poziționare la POI: compară `/amcl_pose` la AT_POI cu goal-ul
  (2.5, 0.5) — orchestratorul loghează pose-ul curent.

**Notă pentru interpretare:** timpii din `mission_summary.json` sunt măsurați
în timp-perete; dacă simularea rulează sub 1.0 RTF (real time factor, afișat
în Gazebo), raportează și RTF-ul sau scalează timpii.

---

## 4. Probleme posibile

| Simptom | Cauză / Fix |
|---|---|
| Robotul merge doar înainte/înapoi, nu virează (sau invers) | era un bug REPARAT: axele roților din exportul CAD aveau semne inconsistente stânga/dreapta și diff-drive-ul inversa virajul cu translația; generatorul normalizează acum toate axele la +Y global — dacă regenerezi URDF-ul, verifică `<axis>` la joint-urile wheel_* |
| Gazebo se deschide negru/gol sau crapă | rendering software lent — folosește `headless:=true` pe T1 și urmărește totul în RViz |
| Robotul nu se mișcă la goal | verifică T2 activ (`Managed nodes are active` în log), simularea pe Play (▶), `ros2 topic echo /odom` publică |
| `manip_ready` nu apare în T4 | T3 nu rulează sau brațul încă face homing (~5 s după pornire) |
| Brațul „flutură" la spawn | normal 1-2 s până nodul de manipulare publică pozițiile home |
| Simulare foarte lentă (RTF < 0.3) | mărește resursele Docker Desktop; închide GUI Gazebo (`headless:=true`) |
| Cubul nu e prins fizic de gripper | grasping-ul fizic în Gazebo e instabil prin natura lui; scopul simulării e orchestrarea, nu fidelitatea apucării — vezi limitarea asumată de mai sus |

---

## 5. Modelul realist din CAD (implicit)

Modelul folosit implicit (`urdf/xplorer_omx_real.urdf`) este **generat direct
din exportul CAD Onshape** al ansamblului real (`amr_2ac_robot_22_01_26` —
Xplorer-A cu OMX-AI-F montat): toate cele 234 de mesh-uri STL ca vizuale,
cele 4 roți SAIM ca link-uri rotative (diff-drive pe 4 roți, rază 55 mm
măsurată din mesh-ul cauciucului), brațul segmentat pe articulațiile
funcționale `dof_joint1..6` din CAD. Fizica rămâne simplificată (coliziuni
primitive), deci simularea e stabilă, dar robotul **arată ca în realitate**.

Detalii tehnice utile:

- Generarea e reproductibilă: `python3 tools/flatten_onshape_urdf.py
  <cale_amr_2ac_robot_22_01_26> <cale_pachet>` — recalculează tot (transformate,
  rază roți, bbox-uri, poziție LiDAR) și rescrie URDF-ul.
- Exportul CAD nu are DOF pentru `wrist_roll` (era oricum 0 în toate pozele);
  lanțul actuat e pan/lift/elbow/wrist + 2 degete revolute.
- **Brațul rămâne în colțul lui original din CAD** (poziția neatinsă față
  de șasiu) și e **rotit la 180° din propria bază** (axa shoulder_pan,
  `ARM_FLIP` în flatten_onshape_urdf.py): gripper-ul împachetat stă **în
  exterior**, peste marginea din spate — nu peste interiorul platformei.
  Robotul **parchează cu spatele la fiecare obiect**: POI-urile din
  pois.yaml sunt calculate din poziția obiectului cu offset-ul exact al
  punctului de pick, (−0.368, −0.083) în frame-ul robotului.
- **Cutia de colectare** e pătrată (16×16 cm) și **stă pe puntea șasiului**
  la (0.10, −0.06) — nivelul punții e măsurat local din mesh-urile CAD
  (z=0.082), nu din bbox-ul global, deci nu „plutește".
- Episodul de manipulare: pick de pe sol din spatele robotului → ridicare →
  **trecere peste creștet** (lift prin verticală, pan rămâne 0) → depunere
  în cutia de pe puntea din față. Traiectoria interpolată a fost verificată
  numeric contra punții, cutiei și stâlpului LiDAR pe toate segmentele.
- Unghiurile tuturor pozelor din `manipulation_infer_node_sim.py` sunt
  calculate prin FK numeric pe lanțul CAD (`tools/fk_data.json`); POI-urile
  din `config/pois.yaml` sunt calculate invers, astfel încât fiecare obiect
  să pice exact în punctul de pick (eroare sub 1 mm).
- Dacă un deget al gripper-ului se mișcă invers în Gazebo, inversează semnul
  în `GRIP_SIGN` din `manipulation_infer_node_sim.py`.
- **Pick adaptiv (echivalentul percepției):** nodul de manipulare citește
  pozele ground-truth ale obiectelor și robotului din Gazebo
  (`/sim/dynamic_poses`) și rezolvă IK numeric pe poziția REALĂ a
  obiectului — compensează erori de parcare Nav2/AMCL de până la ~20 cm
  (validat: 12/12 ținte cu erori ±12 cm, precizie <8 mm). La final
  **verifică fizic** dacă obiectul e în cutia de pe șasiu și publică
  `/manip_done` True/False — eșecurile declanșează reîncercarea prin POI A.
- **LiDAR cu sector mascat:** scanul acoperă ±150° (sectorul din spate de
  ±30° e exclus — acolo lucrează brațul, altfel lidar-ul și-ar vedea
  propriul braț ca obstacol-fantomă fixat de robot, otrăvind AMCL și
  costmap-urile; LD19 real se configurează identic). Mersul în spate e
  limitat (`vx_min=-0.05`) pentru că sectorul e orb.
- **Supervizor de re-localizare (doar în sim):** `mission_multi_poi`
  compară AMCL cu ground-truth înainte de fiecare navigare; dacă deviația
  depășește 0.35 m, re-publică `/initialpose` și curăță costmap-urile —
  echivalentul re-inițializării manuale din RViz pe robotul real. Previne
  cascada de eșecuri ABORTED/coliziuni cu pereții după o pierdere de
  localizare.
- Două cauciucuri din față sunt fixate rigid de șasiu în exportul CAD, deci
  vizual nu se rotesc (cele din spate da) — pur cosmetic.
- `model:=simple` încarcă vechiul model geometric (doar pentru debug de
  navigare; pozele brațului sunt calibrate pentru modelul real).

---

## 6. Structura pachetului

```
gazebo_sim/
├── docker/                  Dockerfile + docker-compose.yml (desktop în browser)
├── ros2_ws/src/xplorer_omx_sim/
│   ├── urdf/xplorer_omx_real.urdf  MODELUL REALIST generat din CAD (implicit;
│   │                               braț rotit 180°, cutie de colectare pe punte)
│   ├── urdf/xplorer_omx.urdf       model geometric simplu (fallback debug)
│   ├── meshes/*.stl                234 mesh-uri Onshape + prism_tri.stl
│   ├── tools/flatten_onshape_urdf.py  generatorul CAD → URDF (+ fk_data.json)
│   ├── tools/gen_scene.py          obiecte random + POI-uri (world + pois.yaml)
│   ├── worlds/lab_world.sdf        laborator 8×6 m + obiectele generate
│   ├── maps/lab_map.{pgm,yaml}     harta generată din world (origin = world)
│   ├── config/nav2_params.yaml     AMCL + MPPI (ca pe robotul real)
│   ├── config/pois.yaml            POI A + lanțul B, C, D... (generat)
│   ├── config/bridge.yaml          ros_gz_bridge (cmd_vel, odom, tf, scan, brațul)
│   ├── launch/{sim,nav2,manip}.launch.py
│   └── xplorer_omx_sim/
│       ├── manipulation_infer_node_sim.py   replica nodului de pe Jetson
│       ├── mission_multi_poi.py             scenariul multi-POI cu colectare
│       └── mission_orchestrator_sim.py      orchestratorul tău + use_sim_time
└── README_SIM.md            acest fișier
```
