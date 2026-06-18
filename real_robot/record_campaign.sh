#!/bin/bash
# ============================================================================
# record_campaign.sh — inregistrare rosbag pentru campania end-to-end (RPi5)
# Porneste-l INAINTE de go_collect_real.py, intr-un terminal separat.
# La final Ctrl+C. Aceleasi topicuri ca in simulare -> aceleasi analize 3.4.
#
# Utilizare:
#   source ~/setup_manip_bridge.bash && source ~/saim_xplorer/install/setup.bash
#   ./record_campaign.sh [eticheta]
# ============================================================================
LABEL="${1:-campania}_$(date +%F_%H%M)"
mkdir -p ~/bags
echo "Inregistrez in ~/bags/$LABEL — Ctrl+C la finalul campaniei"
exec ros2 bag record -o ~/bags/"$LABEL" \
    /tf /tf_static /scan /odom /amcl_pose /goal_pose /plan /cmd_vel \
    /navigate_to_pose/_action/feedback /navigate_to_pose/_action/status \
    /manip_trigger /manip_status /manip_done /manip_result /manip_n_episodes
