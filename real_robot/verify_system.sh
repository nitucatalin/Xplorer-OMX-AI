#!/bin/bash
# ============================================================================
# verify_system.sh — sursează env Cyclone Domain 50 + verifică starea sistemului
# ----------------------------------------------------------------------------
# Foloseste-l în terminalele DE VERIFICARE / teleop / orchestrator pe RPi5,
# după ce start_all.sh rulează în T1.
#
# Utilizare:
#   source ~/verify_system.sh        # sursează env + verifică (CORECT!)
#   source ~/verify_system.sh quiet  # sursează env, nu listează
#
# NU RULA cu ./verify_system.sh — env-ul nu va persista în terminalul tău!
# ============================================================================

# ----- Detectează dacă scriptul e RULAT (nu sourced) -----
# Dacă e rulat (./verify_system.sh), env-ul se pierde la ieșire.
# Folosim BASH_SOURCE vs $0 — diferite dacă suntem sourced.
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo ""
    echo "==========================================================="
    echo " EROARE: verify_system.sh trebuie SOURSAT, nu rulat!"
    echo "==========================================================="
    echo ""
    echo " Ai rulat:  ./verify_system.sh   (greșit — env se pierde)"
    echo "             sau bash verify_system.sh"
    echo ""
    echo " Folosește: source ~/verify_system.sh"
    echo "            sau:  . ~/verify_system.sh"
    echo ""
    echo " De ce: env-ul (CYCLONEDDS_URI, ROS_DOMAIN_ID etc.) trebuie"
    echo "        să persiste în terminalul tău după ce verificarea termină,"
    echo "        ca să poți rula teleop/orchestrator în același terminal."
    echo "==========================================================="
    exit 1
fi

# ----- Sursare bridge (Cyclone Domain 50) -----
if [ ! -f "$HOME/setup_manip_bridge.bash" ]; then
    echo "[ERR] Nu găsesc ~/setup_manip_bridge.bash"
    return 1 2>/dev/null || exit 1
fi
source "$HOME/setup_manip_bridge.bash"

# ----- Sursare workspace -----
if [ -f "$HOME/saim_xplorer/install/setup.bash" ]; then
    source "$HOME/saim_xplorer/install/setup.bash"
fi

# ----- Asigurare Domain + RMW -----
export ROS_DOMAIN_ID=50
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ----- Write Cyclone XML la /tmp (mai sigur decât inline) -----
# Match cu start_all.sh ca să avem același view DDS.
CYCLONE_TMP="/tmp/cyclone_verify_$$.xml"
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

# ----- Quiet mode (doar sursare, fără listare) -----
if [ "$1" = "quiet" ]; then
    echo "[verify] Env: DOMAIN=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"
    return 0 2>/dev/null || exit 0
fi

# ----- Color output -----
GREEN=$'\033[0;32m' ; YELLOW=$'\033[0;33m' ; RED=$'\033[0;31m'
BOLD=$'\033[1m' ; NC=$'\033[0m'

# ----- Header -----
echo ""
echo -e "${BOLD}===========================================================${NC}"
echo -e "${BOLD} VERIFY SYSTEM — Domain $ROS_DOMAIN_ID · CycloneDDS${NC}"
echo -e "${BOLD}===========================================================${NC}"
echo ""
echo -e "Env curent:"
echo "  DOMAIN=$ROS_DOMAIN_ID"
echo "  RMW=$RMW_IMPLEMENTATION"
echo "  CYCDDS=${CYCLONEDDS_URI:-<unset>}"
echo ""

# ----- Listează nodurile -----
echo -e "${BOLD}Noduri vizibile:${NC}"
NODES=$(timeout 8 ros2 node list 2>/dev/null | sort)
echo "$NODES" | sed 's/^/  /'
N=$(echo "$NODES" | grep -c "^/")
echo ""
echo "  Total: $N noduri"
echo ""

# ----- Verifică Nav2 prin prezența nodurilor (rapid, fără lifecycle queries) -----
echo -e "${BOLD}Nav2 stack (prezență noduri):${NC}"
for n in amcl bt_navigator controller_server planner_server behavior_server map_server; do
    if echo "$NODES" | grep -q "^/$n$"; then
        echo -e "  ${GREEN}[OK]${NC} /$n"
    else
        echo -e "  ${RED}[NU]${NC} /$n"
    fi
done
echo ""

# ----- Verifică Jetson -----
echo -e "${BOLD}Jetson (cross-platform):${NC}"
if echo "$NODES" | grep -q "manipulation_infer"; then
    echo -e "  ${GREEN}[OK]${NC} /manipulation_infer_node vizibil"
else
    echo -e "  ${YELLOW}[?]${NC}  /manipulation_infer_node NU e vizibil"
    echo "       (poate Jetson încă n-a pornit start_jetson.sh, sau discovery lag)"
fi
echo ""

# ----- Topice cheie -----
echo -e "${BOLD}Topice cheie:${NC}"
for t in /scan /odom /amcl_pose /cmd_vel /manip_trigger /manip_status; do
    info=$(timeout 2 ros2 topic info $t 2>/dev/null)
    pub=$(echo "$info" | grep "Publisher count:" | grep -oE "[0-9]+")
    sub=$(echo "$info" | grep "Subscription count:" | grep -oE "[0-9]+")
    pub=${pub:-0}; sub=${sub:-0}
    if [ "$pub" -gt 0 ] && [ "$sub" -gt 0 ]; then
        echo -e "  ${GREEN}[OK]${NC} $t  (pub=$pub, sub=$sub)"
    elif [ "$pub" -gt 0 ] || [ "$sub" -gt 0 ]; then
        echo -e "  ${YELLOW}[?]${NC}  $t  (pub=$pub, sub=$sub)"
    else
        echo -e "  ${RED}[NU]${NC} $t  inexistent sau fără endpoints"
    fi
done
echo ""

echo -e "${BOLD}===========================================================${NC}"
echo -e "Dacă vezi nodurile + lifecycle active + manipulation_infer vizibil,"
echo -e "sistemul e gata pentru orchestrator sau teleop."
echo -e "${BOLD}===========================================================${NC}"
echo ""
