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

echo "=== ROS_DOMAIN_ID=$ROS_DOMAIN_ID ==="
echo
echo "=== Topic list with types ==="
ros2 topic list -t | sort

echo
echo "=== Required real-Go2-like topic contract ==="
for t in \
  /clock \
  /tf \
  /tf_static \
  /odom \
  /imu \
  /joint_states \
  /cmd_vel_out \
  /camera/image_raw \
  /camera/camera_info \
  /scan \
  /point_cloud2 \
  /robot_description \
  /map
do
  echo
  echo "--- $t ---"
  timeout 5 ros2 topic info "$t" || true
done

echo
echo "=== Quick rates ==="
for t in /odom /imu /joint_states /scan /camera/image_raw /point_cloud2; do
  echo
  echo "--- hz $t ---"
  timeout 5 ros2 topic hz "$t" || true
done

echo
echo "=== TF check: odom -> base_link ==="
timeout 5 ros2 run tf2_ros tf2_echo odom base_link || true
