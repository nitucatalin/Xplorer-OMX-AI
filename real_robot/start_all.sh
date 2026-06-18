#!/bin/bash
# ============================================================================
# start_all.sh — pornește robot + Nav2 pe Domain 50 cu CycloneDDS (RPi5)
# ----------------------------------------------------------------------------
# Lansează:
#   1. xplorer_bringup robot.launch.py (hardware, senzori, diff_drive, twist_mux)
#   2. amr2ax_nav2 navxplorer.launch.py (stack-ul Nav2 complet)
#
# Folosește setup_manip_bridge.bash (Cyclone Jazzy pe eth0, Domain 50)
# pentru compatibilitate cross-platform cu Jetson.
#
# Pe Jetson (în alt SSH):
#   ./start_jetson.sh
#
# În alt terminal SSH pe RPi5, pentru verificare/teleop/orchestrator:
#   source ~/verify_system.sh    # face setup + listează nodurile
#   # Sau manual:
#   source ~/setup_manip_bridge.bash
#   cd ~/saim_xplorer && source install/setup.bash
#
# Utilizare:
#   ~/start_all.sh                 # default Domain 50
#   ~/start_all.sh 42              # alt Domain (de regulă nu e nevoie)
# ============================================================================

# Notă: NU folosim 'set -u' pentru că ROS setup.bash referențiază
# variabile neinitializate (AMENT_TRACE_SETUP_FILES etc).

# ----- Domain ID (suprascris ca arg: ./start_all.sh 42) -----
DOMAIN_ID=${1:-50}

# ----- Color output (escape codes interpretate cu echo -e) -----
RED=$'\033[0;31m' ; GREEN=$'\033[0;32m' ; YELLOW=$'\033[0;33m' ; BLUE=$'\033[0;34m'
CYAN=$'\033[0;36m' ; BOLD=$'\033[1m' ; NC=$'\033[0m'
log()  { echo -e "${CYAN}[$(date +%H:%M:%S)]${NC} $*"; }
ok()   { echo -e "${GREEN}[$(date +%H:%M:%S)] [OK] $*${NC}"; }
warn() { echo -e "${YELLOW}[$(date +%H:%M:%S)] [WARN] $*${NC}"; }
err()  { echo -e "${RED}[$(date +%H:%M:%S)] [ERR] $*${NC}"; }

# ----- Pregătire foldere de log -----
TS=$(date +%F_%H%M%S)
LOGDIR="$HOME/logs/orchestration_$TS"
mkdir -p "$LOGDIR"
log "Logging în: $LOGDIR"

# ----- PID-uri (pentru cleanup) -----
ROBOT_PID=""
NAV2_PID=""

cleanup() {
    echo ""
    warn "Primit semnal de oprire. Curăț procesele..."
    [ -n "$NAV2_PID"  ] && kill   "$NAV2_PID"  2>/dev/null
    [ -n "$ROBOT_PID" ] && kill   "$ROBOT_PID" 2>/dev/null
    sleep 2
    pkill -9 -f "ros2_control_node|twist_mux|component_container|controller_manager|navxplorer|robot.launch" 2>/dev/null
    ros2 daemon stop 2>/dev/null
    ok "Oprit. Logurile rămân în $LOGDIR"
    exit 0
}
trap cleanup INT TERM

# ============================================================================
# PAS 1 — Cleanup defensiv (procese + daemon + state stale)
# ============================================================================
log "${BOLD}Pas 1/5${NC}: Cleanup procese existente + daemon stale..."
sudo systemctl stop saim_xplorer_robot.service 2>/dev/null || true
pkill -9 -f "ros2_control_node|twist_mux|component_container|controller_manager|navxplorer|robot.launch" 2>/dev/null
pkill -9 -f ros2cli.daemon 2>/dev/null
rm -rf /tmp/ros2_* 2>/dev/null
sleep 2

# Serial buffer cleanup (RoboClaw)
for dev in /dev/ttyACM0 /dev/ttyACM1; do
    if [ -c "$dev" ]; then
        sudo chmod 666 "$dev" 2>/dev/null
        stty -F "$dev" 115200 raw -echo 2>/dev/null
    fi
done

# CPU governor performance (reduce jitter în bucla RT)
sudo cpufreq-set -g performance 2>/dev/null || true
ok "Cleanup terminat"

# ============================================================================
# PAS 2 — Source environment Cyclone Domain 50
# ============================================================================
log "${BOLD}Pas 2/5${NC}: Source environment (CycloneDDS Domain $DOMAIN_ID pe eth0)..."

if [ ! -f "$HOME/setup_manip_bridge.bash" ]; then
    err "Nu găsesc ~/setup_manip_bridge.bash"
    exit 1
fi
source "$HOME/setup_manip_bridge.bash"

if [ ! -f "$HOME/saim_xplorer/install/setup.bash" ]; then
    err "Nu găsesc saim_xplorer/install/setup.bash — workspace neconstruit?"
    exit 1
fi
cd "$HOME/saim_xplorer" && source install/setup.bash

# Asigurare suplimentară Domain + RMW
export ROS_DOMAIN_ID=$DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# ----- Asigură interfața Ethernet directă către Jetson (10.0.0.1) -----
# Adu eth0 UP cu IP static (idempotent) ca DDS să aibă mereu linkul direct.
ETH_IF="${ETH_IF:-eth0}" ; ETH_IP="${ETH_IP:-10.0.0.1}" ; JETSON_ETH_IP="${JETSON_ETH_IP:-10.0.0.2}"
if ip link show "$ETH_IF" >/dev/null 2>&1; then
    sudo ip link set "$ETH_IF" up 2>/dev/null
    if ! ip -4 addr show "$ETH_IF" 2>/dev/null | grep -qw "$ETH_IP"; then
        sudo ip addr add "${ETH_IP}/24" dev "$ETH_IF" 2>/dev/null \
            && ok "Setat ${ETH_IP}/24 pe $ETH_IF" \
            || warn "Nu am putut seta IP pe $ETH_IF (deja există sau lipsă sudo)"
    fi
    ETH_STATE=$(ip -br link show "$ETH_IF" 2>/dev/null | awk '{print $2}')
    [ "$ETH_STATE" != "UP" ] && warn "$ETH_IF nu e UP (stare: ${ETH_STATE:-?}) — verifică cablul direct + Jetson pornit (DDS va folosi și WiFi)."
    log "  Ethernet direct: IF=$ETH_IF IP=$ETH_IP (peer Jetson $JETSON_ETH_IP)"
else
    warn "Interfața $ETH_IF nu există — verifică numele real cu: ip -br addr"
fi

# ----- Write Cyclone XML to /tmp file (mai sigur decât inline) -----
# Asta NU modifică /etc/cyclonedds/cyclonedds_eth.xml de pe disc.
# Doar pentru SESIUNEA asta forțăm RPi5 să inițieze discovery către Jetson (10.0.0.2)
# pe lângă multicast-ul implicit. Bidirecțional => discovery rapid.
CYCLONE_TMP="/tmp/cyclone_rpi5_session_$$.xml"
cat > "$CYCLONE_TMP" << 'CYCEOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" presence_required="false"/>
        <NetworkInterface name="wlan0" priority="2" presence_required="false"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="10.0.0.2"/>
        <Peer address="192.168.53.57"/>
        <Peer address="192.168.53.42"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
    </Discovery>
    <Tracing>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
CYCEOF
export CYCLONEDDS_URI="file://$CYCLONE_TMP"

# Adaugă cleanup al fișierului temporar la trap existent
trap "rm -f '$CYCLONE_TMP' 2>/dev/null; cleanup" INT TERM

ok "Env: DOMAIN=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"
echo "    CYCLONEDDS_URI=$CYCLONEDDS_URI"

# ============================================================================
# PAS 3 — Lansează robot hardware + senzori + controller_manager + twist_mux
# ============================================================================
log "${BOLD}Pas 3/5${NC}: Lansez xplorer_bringup robot.launch.py..."

ros2 launch xplorer_bringup robot.launch.py \
    use_sim_time:=False domain_id:=$DOMAIN_ID \
    > "$LOGDIR/robot.log" 2>&1 &
ROBOT_PID=$!
log "  robot.launch.py PID=$ROBOT_PID, log -> $LOGDIR/robot.log"

log "  Aștept 10s pentru inițializare hardware (RoboClaw + LiDAR + controllers)..."
sleep 10

if ! kill -0 $ROBOT_PID 2>/dev/null; then
    err "robot.launch.py a crăpat. Vezi $LOGDIR/robot.log"
    cleanup
fi
ok "robot.launch.py rulează"

# ============================================================================
# PAS 4 — Lansează Nav2 stack și așteaptă lifecycle ACTIVE
# ============================================================================
log "${BOLD}Pas 4/5${NC}: Lansez Nav2 stack (navxplorer.launch.py)..."

ros2 launch amr2ax_nav2 navxplorer.launch.py \
    localization_type:=2D slam:=False use_sim_time:=False domain_id:=$DOMAIN_ID \
    > "$LOGDIR/nav2.log" 2>&1 &
NAV2_PID=$!
log "  navxplorer.launch.py PID=$NAV2_PID, log -> $LOGDIR/nav2.log"

log "  Aștept ca Nav2 lifecycle să devină ACTIVE (citesc din nav2.log)..."
MAX_WAIT=25
WAITED=0
ACTIVE_SEEN=0
while [ $WAITED -lt $MAX_WAIT ]; do
    sleep 2
    WAITED=$((WAITED + 2))

    # Cea mai sigură detecție: caută în nav2.log mesajul direct emis de lifecycle_manager
    if grep -q "Managed nodes are active" "$LOGDIR/nav2.log" 2>/dev/null; then
        ok "Nav2 ACTIVE (după ${WAITED}s) — lifecycle_manager confirmă în log"
        ACTIVE_SEEN=1
        break
    fi

    log "  ...${WAITED}s — încă se inițializează"
done

if [ $ACTIVE_SEEN -eq 0 ]; then
    warn "Mesajul 'Managed nodes are active' nu apare în nav2.log în ${MAX_WAIT}s."
    warn "Continuă oricum — verifică separat: tail $LOGDIR/nav2.log"
fi

# ============================================================================
# PAS 5 — Verificare finală EXPLICITĂ în acest terminal
# ============================================================================
log "${BOLD}Pas 5/5${NC}: Verificare finală — vede daemonul nodurile?"

# Restart daemon ca să fie sigur că are env actual
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null
sleep 3

NODE_LIST=$(timeout 8 ros2 node list 2>/dev/null | sort)
N_NODES=$(echo "$NODE_LIST" | grep -c "^/")

KEY_NODES=("amcl" "bt_navigator" "controller_server" "planner_server" "behavior_server" \
           "map_server" "controller_manager" "robot_state_publisher" "LD19")
FOUND=0
MISSING=()
for n in "${KEY_NODES[@]}"; do
    if echo "$NODE_LIST" | grep -q "/$n"; then
        FOUND=$((FOUND + 1))
    else
        MISSING+=("$n")
    fi
done

log "  Noduri detectate: $N_NODES"
log "  Noduri-cheie găsite: $FOUND/${#KEY_NODES[@]}"
if [ ${#MISSING[@]} -gt 0 ] && [ $FOUND -gt 0 ]; then
    warn "  Lipsesc (poate doar lag de discovery): ${MISSING[*]}"
fi

if [ $N_NODES -lt 5 ]; then
    warn "Foarte puține noduri. Daemonul poate fi pe alt env. Folosește verify_system.sh în alt terminal."
fi

# ============================================================================
# SUMAR FINAL
# ============================================================================
echo ""
echo -e "${BOLD}${GREEN}============================================================================${NC}"
echo -e "${BOLD}${GREEN} SISTEM PORNIT — Domain $ROS_DOMAIN_ID · CycloneDDS · eth0${NC}"
echo -e "${BOLD}${GREEN}============================================================================${NC}"
echo ""
echo -e "${BOLD}Procese active pe RPi5:${NC}"
echo "  robot.launch.py    PID=$ROBOT_PID   log=$LOGDIR/robot.log"
echo "  navxplorer         PID=$NAV2_PID    log=$LOGDIR/nav2.log"
echo ""
echo -e "${BOLD}Noduri vizibile DIN ACEST TERMINAL ($N_NODES total):${NC}"
echo "$NODE_LIST" | sed 's/^/  /'
echo ""
echo -e "${BOLD}${YELLOW}URMĂTORII PAȘI:${NC}"
echo ""
echo -e "  ${BOLD}1. Jetson (în alt SSH la Jetson):${NC}"
echo "     ./start_jetson.sh"
echo ""
echo -e "  ${BOLD}2. Verificare în alt terminal RPi5:${NC}"
echo "     source ~/verify_system.sh"
echo "     # ↑ asta face source bridge + restart daemon + ros2 node list"
echo ""
echo -e "  ${BOLD}3. Rviz2 de pe Mac (alt terminal pe Mac cu X forwarding):${NC}"
echo "     ssh -Y pi@<IP_RPi5>"
echo "     source ~/setup_manip_bridge.bash"
echo "     cd ~/saim_xplorer && source install/setup.bash"
echo "     ros2 launch nav2_bringup rviz_launch.py"
echo ""
echo -e "  ${BOLD}4. Setare pose inițială în rviz2 (2D Pose Estimate)${NC}"
echo "     Aproximează poziția fizică a robotului, apoi împinge ușor"
echo "     ca AMCL să facă match cu LiDAR."
echo ""
echo -e "  ${BOLD}5. Teleop (alt SSH RPi5):${NC}"
echo "     source ~/setup_manip_bridge.bash"
echo "     cd ~/saim_xplorer && source install/setup.bash"
echo "     ros2 run teleop_twist_keyboard teleop_twist_keyboard \\"
echo "         --ros-args -p stamped:=true -r cmd_vel:=cmd_vel_teleop"
echo ""
echo -e "  ${BOLD}6. Orchestrator end-to-end (alt SSH RPi5):${NC}"
echo "     source ~/setup_manip_bridge.bash"
echo "     python3 ~/mission_orchestrator.py --teach --runs 5 --episodes 1 --pause 5"
echo ""
echo -e "${BOLD}Trigger manual (pentru debug, alt terminal cu env Domain 50):${NC}"
echo "  ros2 topic pub --once /manip_n_episodes std_msgs/Int32 \"{data: 1}\""
echo "  ros2 topic pub --once /manip_trigger std_msgs/Bool \"{data: true}\""
echo ""
echo -e "${CYAN}[Ctrl+C aici oprește robot + Nav2 — NICIODATĂ în alt terminal!]${NC}"
echo ""

# Așteaptă în loop, capturând semnalele
wait
