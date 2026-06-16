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

echo "[sensor_static_tfs] Publishing base_link -> base_scan and base_link -> camera_link"

ros2 run tf2_ros static_transform_publisher \
  0.25 0.0 0.24 0 0 0 \
  base_link base_scan &

ros2 run tf2_ros static_transform_publisher \
  0.32 0.0 0.26 0 -1.5708 0 \
  base_link camera_link &

wait
