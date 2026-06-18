#!/bin/bash
# ============================================================================
# start_jetson.sh — pornește manipulation_infer_node pe Jetson Orin Nano
# ----------------------------------------------------------------------------
# Setup:
#   - ROS 2 Humble + CycloneDDS pe enP8p1s0 (10.0.0.2) + Domain 50
#   - Peer explicit RPi5 (10.0.0.1) din /etc/ros2/cyclone_direct.xml
#   - Cross-platform DDS Jazzy(RPi5) <-> Humble(Jetson) validat la J3
#
# Lansează:
#   manipulation_infer_node.py în mod real (stub_mode=false)
#
# Înainte: pe RPi5 rulează ./start_all.sh
#
# Utilizare:
#   ~/start_jetson.sh                  # mod real (stub_mode=false)
#   ~/start_jetson.sh --stub           # mod stub pentru testare fără braț
#   ~/start_jetson.sh 42               # Domain alternativ
# ============================================================================

# Notă: NU folosim 'set -u' pentru variabile ROS neinitializate

# ----- Configurație (modifică dacă paths-urile diferă) -----
JETSON_WS="${JETSON_WS:-$HOME/ros2_ws}"
INFER_NODE_SCRIPT="${INFER_NODE_SCRIPT:-$HOME/ros2_manip/manipulation_infer_node.py}"
MANIP_BRIDGE_BASH="${MANIP_BRIDGE_BASH:-$HOME/setup_manip_bridge.bash}"
ROS2_MANIP_ENV="${ROS2_MANIP_ENV:-$HOME/ros2_manip/setup_ros2_env.bash}"

# ----- Modelul ACT + homing (folosite în mod REAL) -----
# MODEL_PATH = folderul modelului ACT final (pbn2 din teză). Schimbă-l dacă
# folderul are alt nume. N_ACTION_STEPS=0 => chunk_size-ul politicii (50 la pbn2).
# Homing în 2 etape, ca brațul să NU lovească nimic mergând direct la HOME:
#   întâi IDLE_POSE (poziție intermediară sigură), apoi HOME_POSE.
# Ambele în ACELEAȘI unități ca observation.state / Present_Position din
# OmxFollower. HOME_POSE gol ("") => nodul folosește poziția curentă la pornire.
MODEL_PATH="${MODEL_PATH:-/home/jnfiir/lerobot_models/act_licenta_final_pbn2}"
N_ACTION_STEPS="${N_ACTION_STEPS:-0}"
# Poze capturate fizic cu capture_arm_pose.py (în unitățile reale OmxFollower,
# scara obs_mean ~ grade). HOME = poza capturată (atinsă fără coliziune/decuplare).
# IDLE gol => homing direct la HOME; pune o a doua poză capturată dacă vrei idle->home.
HOME_POSE="${HOME_POSE:-[-1.25, -63.17, 54.29, 53.5, -0.32, 59.24]}"
IDLE_POSE="${IDLE_POSE:-}"
# Timeri homing (secunde): rampă spre idle, pauză în idle, rampă spre home.
IDLE_RAMP_S="${IDLE_RAMP_S:-5.0}"
IDLE_SETTLE_S="${IDLE_SETTLE_S:-1.0}"
HOME_RAMP_S="${HOME_RAMP_S:-5.0}"

# Normalizează la float: ROS cere DOUBLE_ARRAY, iar valorile întregi (ex. -90)
# dau 'InvalidParameterTypeException'. Convertim [0, -90, ...] -> [0.0, -90.0, ...].
_to_float_list() {
    python3 -c "import sys,ast; v=ast.literal_eval(sys.argv[1]); print('['+', '.join(repr(float(x)) for x in v)+']')" "$1" 2>/dev/null
}
if [ -n "$HOME_POSE" ]; then _hf=$(_to_float_list "$HOME_POSE"); [ -n "$_hf" ] && HOME_POSE="$_hf"; fi
if [ -n "$IDLE_POSE" ]; then _if=$(_to_float_list "$IDLE_POSE"); [ -n "$_if" ] && IDLE_POSE="$_if"; fi
# scalarii la float (ROS cere DOUBLE; 5 -> 5.0)
_to_float() { python3 -c "print(float('$1'))" 2>/dev/null || echo "$1"; }
IDLE_RAMP_S=$(_to_float "$IDLE_RAMP_S"); IDLE_SETTLE_S=$(_to_float "$IDLE_SETTLE_S"); HOME_RAMP_S=$(_to_float "$HOME_RAMP_S")
# Device-uri hardware. Dacă streamul camerei nu e pe /dev/video0, schimbă CAMERA
# (vezi `v4l2-ctl --list-devices` / `--list-formats-ext`).
CAMERA="${CAMERA:-/dev/video0}"
PORT="${PORT:-/dev/ttyACM0}"

# ----- Parsare argumente -----
STUB_MODE="false"
DOMAIN_ID=50
for arg in "$@"; do
    case "$arg" in
        --stub)  STUB_MODE="true" ;;
        --real)  STUB_MODE="false" ;;
        [0-9]*)  DOMAIN_ID="$arg" ;;
        -h|--help)
            grep "^#" "$0" | head -20
            exit 0
            ;;
    esac
done

# ----- Color output -----
RED=$'\033[0;31m' ; GREEN=$'\033[0;32m' ; YELLOW=$'\033[0;33m' ; CYAN=$'\033[0;36m'
BOLD=$'\033[1m' ; NC=$'\033[0m'
log()  { echo -e "${CYAN}[$(date +%H:%M:%S)]${NC} $*"; }
ok()   { echo -e "${GREEN}[$(date +%H:%M:%S)] [OK] $*${NC}"; }
warn() { echo -e "${YELLOW}[$(date +%H:%M:%S)] [WARN] $*${NC}"; }
err()  { echo -e "${RED}[$(date +%H:%M:%S)] [ERR] $*${NC}"; }

# ----- Pregătire foldere de log -----
TS=$(date +%F_%H%M%S)
LOGDIR="$HOME/logs/jetson_infer_$TS"
mkdir -p "$LOGDIR"
log "Logging în: $LOGDIR"

# ----- PID + cleanup trap -----
INFER_PID=""
cleanup() {
    echo ""
    warn "Primit semnal de oprire. Curăț procesele..."
    [ -n "$INFER_PID" ] && kill "$INFER_PID" 2>/dev/null
    sleep 2
    pkill -9 -f "manipulation_infer_node" 2>/dev/null
    ros2 daemon stop 2>/dev/null
    ok "Oprit. Logurile rămân în $LOGDIR"
    exit 0
}
trap cleanup INT TERM

# ============================================================================
# PAS 1 — Cleanup procese + daemon vechi
# ============================================================================
log "${BOLD}Pas 1/5${NC}: Cleanup procese inferență + daemon stale..."
pkill -9 -f "manipulation_infer_node" 2>/dev/null
pkill -9 -f ros2cli.daemon 2>/dev/null
rm -rf /tmp/ros2_* 2>/dev/null
sleep 2
ok "Cleanup terminat"

# ============================================================================
# PAS 2 — Source environment (Humble + Cyclone Domain 50)
# ============================================================================
log "${BOLD}Pas 2/5${NC}: Source environment (Humble + CycloneDDS Domain $DOMAIN_ID)..."

if [ ! -f "$MANIP_BRIDGE_BASH" ]; then
    err "Nu găsesc $MANIP_BRIDGE_BASH"
    exit 1
fi
source "$MANIP_BRIDGE_BASH"
log "  Sursat $MANIP_BRIDGE_BASH"

if [ -f "$JETSON_WS/install/setup.bash" ]; then
    source "$JETSON_WS/install/setup.bash" 2>/dev/null
    log "  Sursat $JETSON_WS/install/setup.bash"
fi

if [ -f "$ROS2_MANIP_ENV" ]; then
    source "$ROS2_MANIP_ENV"
    log "  Sursat $ROS2_MANIP_ENV"
fi

# Suprascrie cu valorile noastre (în caz că setup_ros2_env.bash setează Domain 42 default)
export ROS_DOMAIN_ID=$DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# ----- Write Cyclone XML to /tmp file -----
# Inline CYCLONEDDS_URI parsing nu merge fiabil pe Humble.
# Scriem XML-ul în /tmp și folosim file:// (acceptat 100%).
# NU modifică /etc/ros2/cyclone_direct.xml.
# Auto-detectează interfața WiFi (caută IP 192.168.53.*)
JETSON_WIFI_IF=$(ip -o addr show | awk '/192\.168\.53/ {print $2; exit}')
if [ -z "$JETSON_WIFI_IF" ]; then
    # Fallback: caută orice interfață cu nume wireless (wl*)
    JETSON_WIFI_IF=$(ip -o link show | awk -F': ' '$2 ~ /^wl/ {print $2; exit}')
fi
log "  Interfață WiFi detectată: ${JETSON_WIFI_IF:-NICIUNA}"

# ----- Asigură interfața Ethernet directă către RPi5 (10.0.0.2) -----
# Cauza erorii "enP8p1s0: does not match an available interface": interfața
# directă era jos / fără IP. O aducem UP cu IP static (idempotent) și, dacă
# numele real diferă, auto-detectăm prima interfață cu fir.
ETH_IF="${ETH_IF:-enP8p1s0}"      # Ethernet direct Jetson<->RPi5
ETH_IP="${ETH_IP:-10.0.0.2}"      # IP-ul Jetson pe linkul direct
RPI_ETH_IP="${RPI_ETH_IP:-10.0.0.1}"
if ! ip link show "$ETH_IF" >/dev/null 2>&1; then
    DET=$(ip -o link show | awk -F': ' '{print $2}' | grep -E '^(en|eth)' | grep -vE '^(wl|lo)' | head -1)
    if [ -n "$DET" ]; then
        warn "Interfața $ETH_IF nu există; folosesc interfața cu fir detectată: $DET"
        ETH_IF="$DET"
    else
        warn "Nu găsesc interfață Ethernet cu fir; DDS va folosi WiFi/fallback."
        ETH_IF=""
    fi
fi
if [ -n "$ETH_IF" ]; then
    sudo ip link set "$ETH_IF" up 2>/dev/null
    if ! ip -4 addr show "$ETH_IF" 2>/dev/null | grep -qw "$ETH_IP"; then
        sudo ip addr add "${ETH_IP}/24" dev "$ETH_IF" 2>/dev/null \
            && ok "Setat ${ETH_IP}/24 pe $ETH_IF" \
            || warn "Nu am putut seta IP pe $ETH_IF (deja există sau lipsă sudo)"
    fi
    ETH_STATE=$(ip -br link show "$ETH_IF" 2>/dev/null | awk '{print $2}')
    if [ "$ETH_STATE" != "UP" ]; then
        warn "$ETH_IF nu e UP (stare: ${ETH_STATE:-?}) — conectează cablul direct + pornește RPi5."
        warn "Până atunci DDS folosește WiFi (peers 192.168.53.*)."
    fi
    log "  Ethernet direct: IF=$ETH_IF IP=$ETH_IP (peer RPi5 $RPI_ETH_IP)"
    if ping -c1 -W2 "$RPI_ETH_IP" >/dev/null 2>&1; then
        ok "Ping RPi5 ($RPI_ETH_IP) OK — link direct activ"
    else
        warn "Ping RPi5 ($RPI_ETH_IP) eșuat deocamdată (discovery încearcă și WiFi)."
    fi
fi

# ----- Scrie config CycloneDDS (Domain 50) -----
# presence_required="false" => o interfață lipsă/jos NU mai oprește crearea
# domeniului (fix-ul erorii). Interfețele se adaugă doar dacă au fost găsite.
CYCLONE_TMP="/tmp/cyclone_jetson_session_$$.xml"
{
  echo '<?xml version="1.0" encoding="UTF-8" ?>'
  echo '<CycloneDDS xmlns="https://cdds.io/config">'
  echo '  <Domain Id="any">'
  echo '    <General>'
  echo '      <Interfaces>'
  [ -n "$ETH_IF" ] && echo "        <NetworkInterface name=\"$ETH_IF\" presence_required=\"false\"/>"
  [ -n "$JETSON_WIFI_IF" ] && echo "        <NetworkInterface name=\"$JETSON_WIFI_IF\" priority=\"2\" presence_required=\"false\"/>"
  echo '      </Interfaces>'
  echo '      <AllowMulticast>true</AllowMulticast>'
  echo '    </General>'
  echo '    <Discovery>'
  echo '      <Peers>'
  echo '        <Peer address="10.0.0.1"/>'
  echo '        <Peer address="192.168.53.177"/>'
  echo '        <Peer address="192.168.53.42"/>'
  echo '      </Peers>'
  echo '      <ParticipantIndex>auto</ParticipantIndex>'
  echo '      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>'
  echo '    </Discovery>'
  echo '    <Tracing><Verbosity>warning</Verbosity></Tracing>'
  echo '  </Domain>'
  echo '</CycloneDDS>'
} > "$CYCLONE_TMP"
export CYCLONEDDS_URI="file://$CYCLONE_TMP"

# Cleanup trap pentru fișier temporar
trap "rm -f '$CYCLONE_TMP' 2>/dev/null; cleanup" INT TERM

ok "Env: DOMAIN=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"
echo "    CYCLONEDDS_URI=$CYCLONEDDS_URI"
echo "    Conținut XML temporar:"
sed 's/^/      /' "$CYCLONE_TMP"

# ============================================================================
# PAS 3 — Lansează manipulation_infer_node ÎNTÂI (ca să publice rapid)
# ============================================================================
log "${BOLD}Pas 3/5${NC}: Lansez manipulation_infer_node (stub_mode=$STUB_MODE)..."
log "  Script: $INFER_NODE_SCRIPT"
log "  Mod:    $([ "$STUB_MODE" = "true" ] && echo 'STUB (fără braț)' || echo 'REAL (braț activ)')"
log "  Log:    $LOGDIR/infer.log"

if [ ! -f "$INFER_NODE_SCRIPT" ]; then
    err "Script-ul $INFER_NODE_SCRIPT nu există"
    exit 1
fi

# ----- Pre-flight cameră (doar în mod real) -----
# Cauza picării homing-ului la restarturi rapide: camera era încă ținută de
# instanța anterioară a nodului (device USB neeliberat). Verificăm cu retry că
# $CAMERA livrează cadre, apoi o eliberăm 1s ca nodul s-o deschidă curat.
if [ "$STUB_MODE" != "true" ]; then
    log "  Verific camera $CAMERA (livrează cadre?)..."
    CAM_OK=0
    for tryc in 1 2 3 4 5; do
        if python3 - "$CAMERA" <<'PYEOF'
import sys, cv2
cap = cv2.VideoCapture(sys.argv[1])
ok = False
if cap.isOpened():
    for _ in range(8):
        r, f = cap.read()
        if r and f is not None:
            ok = True; break
cap.release()
sys.exit(0 if ok else 1)
PYEOF
        then CAM_OK=1; break; fi
        warn "  camera nu livrează încă (încercare $tryc/5) — aștept 2s..."
        sleep 2
    done
    if [ "$CAM_OK" -eq 1 ]; then
        ok "Camera $CAMERA livrează cadre"
        sleep 1   # lasă device-ul USB să se elibereze înainte ca nodul să-l deschidă
    else
        warn "Camera $CAMERA NU livrează cadre — homing/inferența vor eșua."
        warn "Încearcă: alt index (CAMERA=/dev/videoN ./start_jetson.sh), alt port USB, alimentare."
    fi
fi

INFER_PARAMS=( -p stub_mode:=$STUB_MODE -p model_path:="$MODEL_PATH" -p n_action_steps:=$N_ACTION_STEPS )
INFER_PARAMS+=( -p camera:="$CAMERA" -p port:="$PORT" )
INFER_PARAMS+=( -p idle_ramp_s:=$IDLE_RAMP_S -p idle_settle_s:=$IDLE_SETTLE_S -p home_ramp_s:=$HOME_RAMP_S )
[ -n "$IDLE_POSE" ] && INFER_PARAMS+=( -p idle_pose:="$IDLE_POSE" )
if [ -n "$HOME_POSE" ]; then
    INFER_PARAMS+=( -p home_pose:="$HOME_POSE" )
    log "  Model:     $MODEL_PATH  (NAS=$N_ACTION_STEPS)"
    log "  Homing:    IDLE ${IDLE_POSE:-(fara)} -> HOME $HOME_POSE"
else
    log "  Model:     $MODEL_PATH  (NAS=$N_ACTION_STEPS)"
    log "  Homing:    home_pose gol => AUTO din media modelului (obs_mean)"
fi

python3 "$INFER_NODE_SCRIPT" --ros-args "${INFER_PARAMS[@]}" \
    > "$LOGDIR/infer.log" 2>&1 &
INFER_PID=$!

# Așteaptă să se inițializeze (homing + DDS participant)
sleep 5

if ! kill -0 $INFER_PID 2>/dev/null; then
    err "manipulation_infer_node a crăpat. Vezi $LOGDIR/infer.log"
    tail -20 "$LOGDIR/infer.log"
    cleanup
fi
ok "manipulation_infer_node rulează (PID=$INFER_PID)"

# ============================================================================
# PAS 4 — Aștept propagare DDS bidirecțional (local + RPi5)
# ============================================================================
log "${BOLD}Pas 4/5${NC}: Aștept propagare DDS (local + RPi5 cross-platform)..."

# Cross-version Cyclone Jazzy<->Humble + Peer config necesită ~10-15s
# Folosim --no-daemon ca să sărim peste daemonul potențial stale
MAX_WAIT=20
WAITED=0
RPI_SEEN=0
LOCAL_SEEN=0
while [ $WAITED -lt $MAX_WAIT ]; do
    sleep 3
    WAITED=$((WAITED + 3))

    # Dacă nodul a murit în timpul încărcării modelului / conectării brațului,
    # arată motivul (altfel scriptul ar ieși tăcut pe 'wait', ca un Ctrl+C).
    if ! kill -0 "$INFER_PID" 2>/dev/null; then
        err "manipulation_infer_node s-a oprit în timpul inițializării (model/braț/camera)."
        err "Ultimele linii din $LOGDIR/infer.log:"
        echo "------------------------------------------------------------"
        tail -40 "$LOGDIR/infer.log"
        echo "------------------------------------------------------------"
        cleanup
    fi

    # --no-daemon → discovery DDS direct, ignoră cache-ul daemonului
    NL=$(timeout 5 ros2 node list --no-daemon 2>/dev/null)
    RPI_NODES=$(echo "$NL" | grep -c -E "/amcl|/controller_server|/robot_state_publisher")
    LOCAL_NODES=$(echo "$NL" | grep -c "manipulation_infer")

    if [ "$LOCAL_NODES" -ge 1 ]; then LOCAL_SEEN=1; fi
    if [ "$RPI_NODES" -ge 2 ]; then RPI_SEEN=1; fi

    if [ "$LOCAL_SEEN" -eq 1 ] && [ "$RPI_SEEN" -eq 1 ]; then
        ok "DDS gata (${WAITED}s) — local + RPi5 vizibili"
        break
    fi
    log "  ...${WAITED}s — local=$LOCAL_NODES RPi5=$RPI_NODES"
done

if [ "$LOCAL_SEEN" -eq 0 ]; then
    warn "Nodul local nu se vede prin DDS. Probabil daemon stale. Continuă oricum."
fi
if [ "$RPI_SEEN" -eq 0 ]; then
    warn "RPi5 încă nu e vizibil cross-platform după ${MAX_WAIT}s."
    warn "Verifică separat:"
    warn "  - ./start_all.sh rulează pe RPi5"
    warn "  - eth0 / enP8p1s0 ambele UP (test cu: ping 10.0.0.1)"
fi

# ============================================================================
# PAS 5 — Listă finală noduri (fără restart daemon)
# ============================================================================
log "${BOLD}Pas 5/5${NC}: Listare finală (--no-daemon)..."

NODE_LIST=$(timeout 8 ros2 node list --no-daemon 2>/dev/null | sort)
N_TOTAL=$(echo "$NODE_LIST" | grep -c "^/")
N_RPI=$(echo "$NODE_LIST" | grep -cE "/amcl|/bt_navigator|/controller_server|/planner_server|/behavior_server|/robot_state_publisher|/LD19")
N_LOCAL=$(echo "$NODE_LIST" | grep -c "manipulation_infer")

log "  Total noduri: $N_TOTAL"
log "  Noduri RPi5 (Nav2): $N_RPI"
log "  Noduri locale (manip): $N_LOCAL"

# ============================================================================
# SUMAR FINAL
# ============================================================================
echo ""
echo -e "${BOLD}${GREEN}============================================================================${NC}"
echo -e "${BOLD}${GREEN} JETSON INFERENCE NODE PORNIT${NC}"
echo -e "${BOLD}${GREEN}============================================================================${NC}"
echo ""
echo -e "${BOLD}Configurație:${NC}"
echo "  PID:           $INFER_PID"
echo "  Mode:          $STUB_MODE ($([ "$STUB_MODE" = "true" ] && echo 'stub' || echo 'real — braț activ'))"
echo "  Domain:        $ROS_DOMAIN_ID"
echo "  RMW:           $RMW_IMPLEMENTATION"
echo "  Log:           $LOGDIR/infer.log"
echo ""
echo -e "${BOLD}Noduri vizibile DIN ACEST TERMINAL ($N_TOTAL total):${NC}"
echo "$NODE_LIST" | sed 's/^/  /'
echo ""

if [ "$N_RPI" -ge 3 ]; then
    echo -e "  ${GREEN}[OK]${NC} RPi5 (Nav2) vizibil cross-platform"
else
    echo -e "  ${YELLOW}[WARN]${NC} Doar $N_RPI noduri RPi5 vizibile. Verifică:"
    echo "         - ./start_all.sh rulează pe RPi5"
    echo "         - eth0 (RPi5) și enP8p1s0 (Jetson) sunt UP"
    echo "         - Așteaptă încă 15-30s și retry: ros2 node list"
fi
echo ""
echo -e "${BOLD}${YELLOW}Comenzi de pe RPi5 sau aici (alt SSH cu env Domain $ROS_DOMAIN_ID):${NC}"
echo ""
echo "  # Trimite trigger (1 episod):"
echo "  ros2 topic pub --once /manip_n_episodes std_msgs/Int32 \"{data: 1}\""
echo "  ros2 topic pub --once /manip_trigger std_msgs/Bool \"{data: true}\""
echo ""
echo "  # Urmărește răspunsul:"
echo "  ros2 topic echo /manip_status"
echo "  ros2 topic echo /manip_done"
echo "  ros2 topic echo /manip_result"
echo ""
echo "  # Abort imediat:"
echo "  ros2 topic pub --once /manip_abort std_msgs/Bool \"{data: true}\""
echo ""
echo "  # Trimite la home pose:"
echo "  ros2 topic pub --once /manip_go_home std_msgs/Bool \"{data: true}\""
echo ""
echo -e "${BOLD}Live log (urmărește execuția):${NC}"
echo "  tail -f $LOGDIR/infer.log"
echo ""
echo -e "${CYAN}[Ctrl+C aici oprește nodul de inferență — susține brațul cu mâna!]${NC}"
echo ""

# Monitorizează nodul: dacă moare singur (nu prin Ctrl+C), arată motivul
# din log în loc să iasă tăcut.
while kill -0 "$INFER_PID" 2>/dev/null; do
    sleep 2
done
err "manipulation_infer_node s-a oprit. Ultimele linii din log:"
echo "------------------------------------------------------------"
tail -40 "$LOGDIR/infer.log"
echo "------------------------------------------------------------"
cleanup
