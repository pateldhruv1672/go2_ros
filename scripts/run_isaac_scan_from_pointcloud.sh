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

if ! ros2 pkg prefix pointcloud_to_laserscan >/dev/null 2>&1; then
  echo "[run_isaac_scan_from_pointcloud] Missing pointcloud_to_laserscan."
  echo "Install it with:"
  echo "  sudo apt update && sudo apt install -y ros-jazzy-pointcloud-to-laserscan"
  exit 1
fi

exec ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/point_cloud2 \
  -r scan:=/scan \
  -p use_sim_time:=true \
  -p target_frame:=base_link \
  -p transform_tolerance:=0.25 \
  -p min_height:=-0.20 \
  -p max_height:=0.80 \
  -p angle_min:=-1.5708 \
  -p angle_max:=1.5708 \
  -p angle_increment:=0.0087 \
  -p scan_time:=0.10 \
  -p range_min:=0.20 \
  -p range_max:=8.00 \
  -p use_inf:=true
