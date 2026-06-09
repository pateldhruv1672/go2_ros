#!/usr/bin/env bash
set -euo pipefail

export ROS_HOME=/tmp/ros_home_sparky
export ROS_LOG_DIR=/tmp/ros_logs_sparky
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -f "$ROOT_DIR/.env.local" ]; then
  # Load local secrets without printing them.
  set -a
  . "$ROOT_DIR/.env.local"
  set +a
fi

source "$SCRIPT_DIR/sparky_ros_env.sh"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

if [ -z "${DISPLAY:-}" ]; then
  export QT_QPA_PLATFORM=offscreen
fi

ros2 daemon stop || true
ros2 daemon start || true

MAP_LABEL="${MAP_LABEL:-lab_live_teach}"
AUTO_SAVE_TARGET_SAMPLES="${AUTO_SAVE_TARGET_SAMPLES:-10}"
AUTO_SAVE_INTERVAL_SEC="${AUTO_SAVE_INTERVAL_SEC:-5.0}"
AUTO_SAVE_USE_VLM="${AUTO_SAVE_USE_VLM:-true}"
AUTO_SAVE_ALLOW_REPEAT_SAMPLES="${AUTO_SAVE_ALLOW_REPEAT_SAMPLES:-true}"
AUTO_SAVE_MIN_DISTANCE_M="${AUTO_SAVE_MIN_DISTANCE_M:-0.0}"
SAVE_MAP_ON_SHUTDOWN="${SAVE_MAP_ON_SHUTDOWN:-true}"

exec ros2 launch go2_semantic_nav_agent semantic_nav_teach.launch.py \
  map_label:="$MAP_LABEL" \
  semantic_rviz:=true \
  auto_save_target_samples:="$AUTO_SAVE_TARGET_SAMPLES" \
  auto_save_interval_sec:="$AUTO_SAVE_INTERVAL_SEC" \
  auto_save_use_vlm:="$AUTO_SAVE_USE_VLM" \
  auto_save_allow_repeat_samples:="$AUTO_SAVE_ALLOW_REPEAT_SAMPLES" \
  auto_save_min_distance_m:="$AUTO_SAVE_MIN_DISTANCE_M" \
  save_map_on_shutdown:="$SAVE_MAP_ON_SHUTDOWN"
