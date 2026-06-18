#!/bin/bash
# ============================================================================
# verify_lenovo.sh — sursează env Cyclone Domain 50 pe Lenovo Ubuntu
# ----------------------------------------------------------------------------
# Pentru terminale pe LENOVO (Ubuntu) care vor să vorbească cu RPi5 prin WiFi.
# RPi5 trebuie să aibă start_all.sh actualizat (wlan0 + Peer 192.168.53.42).
#
# Configurație:
#   Lenovo IP: 192.168.53.42 (WiFi)
#   RPi5 IP:   editează RPI5_IP în ENV var sau direct mai jos
#
# Utilizare:
#   RPI5_IP=192.168.53.10 source ~/verify_lenovo.sh
#   sau cu IP-ul default editat în script:
#   source ~/verify_lenovo.sh
#   source ~/verify_lenovo.sh quiet
#
# NU RULA cu ./verify_lenovo.sh — env-ul nu va persista!
# ============================================================================

# ----- Detectează dacă scriptul e RULAT (nu sourced) -----
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo ""
    echo "==========================================================="
    echo " EROARE: verify_lenovo.sh trebuie SOURSAT, nu rulat!"
    echo "==========================================================="
    echo ""
    echo " Ai rulat:  ./verify_lenovo.sh   (greșit — env se pierde)"
    echo ""
    echo " Folosește: source ~/verify_lenovo.sh"
    echo "            sau cu IP custom:"
    echo "            RPI5_IP=192.168.53.10 source ~/verify_lenovo.sh"
    echo "==========================================================="
    exit 1
fi

# ----- IP-uri WiFi (suprascrise cu env var) -----
RPI5_IP_DEFAULT="192.168.53.177"      # IP WiFi al RPi5
JETSON_IP_DEFAULT="192.168.53.57"     # IP WiFi al Jetson
RPI5_IP="${RPI5_IP:-$RPI5_IP_DEFAULT}"
JETSON_IP="${JETSON_IP:-$JETSON_IP_DEFAULT}"

# ----- Auto-detectează interfața WiFi pe Lenovo -----
# Caută interfața care are IP 192.168.53.*
WIFI_IF=$(ip -o addr show | awk '/192\.168\.53/ {print $2; exit}')
if [ -z "$WIFI_IF" ]; then
    # Fallback: prima interfață care nu e lo
    WIFI_IF=$(ip -o link show | awk -F': ' '$2!="lo" {print $2; exit}')
fi

# ----- Verifică ping la RPi5 ÎNAINTE de orice DDS -----
echo "[verify_lenovo] Test ping RPi5 ($RPI5_IP)..."
if ! ping -c 2 -W 2 "$RPI5_IP" >/dev/null 2>&1; then
    echo "[verify_lenovo] EROARE: Nu pot face ping la RPi5 ($RPI5_IP)"
    echo "[verify_lenovo] Verifică:"
    echo "  - RPi5 e pornit și conectat la aceeași WiFi"
    echo "  - IP-ul RPi5 e corect (rulează pe RPi5: ip a show wlan0)"
    echo "  - WiFi-ul nu are client isolation activat (multe rețele publice)"
    echo "  - Firewall-ul Lenovo nu blochează (sudo ufw status)"
    echo "  - Re-rulează cu: RPI5_IP=x.x.x.x source ~/verify_lenovo.sh"
    return 1 2>/dev/null || exit 1
fi
echo "[verify_lenovo] Ping OK"

# ----- Source ROS 2 (Jazzy presupus pe Lenovo Ubuntu 24.04 sau Humble pe 22.04) -----
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[verify_lenovo] EROARE: nu găsesc /opt/ros/jazzy sau humble"
    return 1 2>/dev/null || exit 1
fi

# ----- Asigurare Domain + RMW -----
export ROS_DOMAIN_ID=50
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ----- Write Cyclone XML la /tmp -----
CYCLONE_TMP="/tmp/cyclone_lenovo_$$.xml"
cat > "$CYCLONE_TMP" << CYCEOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="$WIFI_IF"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="$RPI5_IP"/>
        <Peer address="$JETSON_IP"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
CYCEOF
export CYCLONEDDS_URI="file://$CYCLONE_TMP"

# ----- Restart daemon cu env nou -----
ros2 daemon stop >/dev/null 2>&1
sleep 1
ros2 daemon start >/dev/null 2>&1
sleep 3

# ----- Quiet mode -----
if [ "$1" = "quiet" ]; then
    echo "[verify_lenovo] Env: DOMAIN=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION IF=$WIFI_IF PEERS=[$RPI5_IP,$JETSON_IP]"
    return 0 2>/dev/null || exit 0
fi

# ----- Color output -----
GREEN=$'\033[0;32m' ; YELLOW=$'\033[0;33m' ; RED=$'\033[0;31m'
BOLD=$'\033[1m' ; NC=$'\033[0m'

# ----- Header -----
echo ""
echo -e "${BOLD}===========================================================${NC}"
echo -e "${BOLD} VERIFY LENOVO — Domain $ROS_DOMAIN_ID · CycloneDDS · $WIFI_IF${NC}"
echo -e "${BOLD}===========================================================${NC}"
echo ""
echo -e "Env curent:"
echo "  DOMAIN=$ROS_DOMAIN_ID"
echo "  RMW=$RMW_IMPLEMENTATION"
echo "  WIFI_IF=$WIFI_IF"
echo "  PEER RPi5=$RPI5_IP"
echo "  PEER Jetson=$JETSON_IP"
echo "  CYCDDS=$CYCLONEDDS_URI"
echo ""

# ----- Listează nodurile (cross-platform) -----
echo -e "${BOLD}Noduri vizibile (de la RPi5 prin WiFi):${NC}"
NODES=$(timeout 8 ros2 node list 2>/dev/null | sort)
echo "$NODES" | sed 's/^/  /'
N=$(echo "$NODES" | grep -c "^/")
echo ""
echo "  Total: $N noduri"
echo ""

# ----- Nav2 prezență -----
echo -e "${BOLD}Nav2 stack (prezență noduri):${NC}"
for n in amcl bt_navigator controller_server planner_server behavior_server map_server; do
    if echo "$NODES" | grep -q "^/$n$"; then
        echo -e "  ${GREEN}[OK]${NC} /$n"
    else
        echo -e "  ${RED}[NU]${NC} /$n"
    fi
done
echo ""

# ----- Manipulation infer -----
echo -e "${BOLD}Jetson (cross-platform):${NC}"
if echo "$NODES" | grep -q "manipulation_infer"; then
    echo -e "  ${GREEN}[OK]${NC} /manipulation_infer_node vizibil"
else
    echo -e "  ${YELLOW}[?]${NC}  /manipulation_infer_node NU e vizibil"
fi
echo ""

echo -e "${BOLD}===========================================================${NC}"
echo -e "Acum poți rula rviz2 NATIV pe Lenovo:"
echo -e "  ${BOLD}ros2 launch nav2_bringup rviz_launch.py${NC}"
echo -e ""
echo -e "Toate plugin-urile Nav2 vor vedea serverele de pe RPi5 prin WiFi."
echo -e "${BOLD}===========================================================${NC}"
echo ""
