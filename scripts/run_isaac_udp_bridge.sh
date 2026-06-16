#!/usr/bin/env bash
set -e

cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
source scripts/env_isaac.sh

echo "GO2_TARGET=$GO2_TARGET"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

ros2 launch go2_isaac_bridge go2_isaac_udp_bridge.launch.py \
  state_host:=127.0.0.1 \
  state_port:=15001 \
  cmd_host:=127.0.0.1 \
  cmd_port:=15000 \
  odom_frame:=odom \
  base_frame:=base_link \
  use_sim_time:=false \
  publish_tf:=true
