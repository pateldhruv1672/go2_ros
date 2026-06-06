#!/usr/bin/env bash
set -euo pipefail

export ROS_HOME=/tmp/ros_home_sparky
export ROS_LOG_DIR=/tmp/ros_logs_sparky
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

exec ros2 run go2_semantic_nav_agent semantic_nav_node --ros-args \
  -p mode:=teach \
  -p session_root:=/tmp/semantic_nav_smoke \
  -p map_label:=smoke \
  -p route_name:=smoke_route \
  -p tour_mode:=false \
  -p auto_save_places:=false \
  -p clear_places_on_start:=false \
  -p restore_spawn_on_start:=false \
  -p fallback_enable:=false
