#!/usr/bin/env bash
set -euo pipefail

export ROS_HOME=/tmp/ros_home_sparky
export ROS_LOG_DIR=/tmp/ros_logs_sparky
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -f "$ROOT_DIR/.env.local" ]; then
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

SESSION_ROOT="${SESSION_ROOT:-~/.ros/go2_semantic_nav_sessions}"
SESSION_NAME="${SESSION_NAME:-}"
NETWORK_INTERFACE="${NETWORK_INTERFACE:-}"
INPUT_BACKEND="${INPUT_BACKEND:-auto}"
SEMANTIC_RVIZ="${SEMANTIC_RVIZ:-true}"
SPEAKER_TTS_ENABLED="${SPEAKER_TTS_ENABLED:-true}"

exec ros2 launch go2_agentic_system sparky_full_system.launch.py \
  session_root:="$SESSION_ROOT" \
  session_name:="$SESSION_NAME" \
  network_interface:="$NETWORK_INTERFACE" \
  input_backend:="$INPUT_BACKEND" \
  semantic_rviz:="$SEMANTIC_RVIZ" \
  speaker_tts_enabled:="$SPEAKER_TTS_ENABLED"
