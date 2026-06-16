#!/usr/bin/env bash
set -euo pipefail

WS="/home/digital-twin-admin/Dhruv/sparky/ros2_ws"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

CFG="$WS/install/go2_semantic_nav_agent/share/go2_semantic_nav_agent/config/semantic_nav.rviz"

if [ -f "$CFG" ]; then
  echo "[run_isaac_rviz] Launching RViz config: $CFG"
  exec rviz2 -d "$CFG"
else
  echo "[run_isaac_rviz] No semantic_nav.rviz found. Launching plain RViz."
  exec rviz2
fi
