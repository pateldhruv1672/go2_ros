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
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

URDF_FILE="${GO2_URDF_FILE:-$WS/src/go2_robot_sdk/urdf/go2_on_steroids.urdf}"

if [ ! -f "$URDF_FILE" ]; then
  echo "[run_isaac_robot_description] URDF not found: $URDF_FILE"
  echo "Available URDF/Xacro candidates:"
  find "$WS/src" "$WS/install" -type f \( -name "*.urdf" -o -name "*.xacro" \) 2>/dev/null | sort | head -80
  exit 1
fi

echo "[run_isaac_robot_description] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[run_isaac_robot_description] Using URDF: $URDF_FILE"

if [[ "$URDF_FILE" == *.xacro ]]; then
  TMP_URDF="/tmp/go2_robot_description_$$.urdf"
  ros2 run xacro xacro "$URDF_FILE" > "$TMP_URDF"
  URDF_FILE="$TMP_URDF"
  echo "[run_isaac_robot_description] Expanded xacro to: $URDF_FILE"
fi

exec ros2 run robot_state_publisher robot_state_publisher "$URDF_FILE" \
  --ros-args \
  -p use_sim_time:=true
