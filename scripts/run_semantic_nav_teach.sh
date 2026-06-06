#!/usr/bin/env bash
set -euo pipefail

export ROS_HOME=/tmp/ros_home_sparky
export ROS_LOG_DIR=/tmp/ros_logs_sparky
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

AUTO_SAVE_TARGET_SAMPLES="${AUTO_SAVE_TARGET_SAMPLES:-3}"
AUTO_SAVE_INTERVAL_SEC="${AUTO_SAVE_INTERVAL_SEC:-5.0}"

exec ros2 launch go2_semantic_nav_agent semantic_nav_teach.launch.py \
  auto_save_target_samples:="$AUTO_SAVE_TARGET_SAMPLES" \
  auto_save_interval_sec:="$AUTO_SAVE_INTERVAL_SEC" \
  auto_save_allow_repeat_samples:=true \
  auto_save_min_distance_m:=0.0 \
  save_map_on_shutdown:=false
