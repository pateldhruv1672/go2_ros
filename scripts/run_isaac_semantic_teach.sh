#!/usr/bin/env bash
set -euo pipefail

cd ~/Dhruv/sparky/ros2_ws

set +u
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
set -u

export GO2_TARGET=isaac
export ROS_DOMAIN_ID=17
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

MAP_LABEL="${MAP_LABEL:-isaac_lab_teach}"

ros2 launch go2_semantic_nav_agent semantic_nav_teach.launch.py \
  map_label:="$MAP_LABEL" \
  semantic_rviz:=true \
  auto_save_places:=true \
  auto_save_interval_sec:=5.0 \
  auto_save_use_vlm:=true \
  clear_places_on_start:=false \
  save_map_on_shutdown:=true
