#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

export RVIZ2=false
export NAV2_START_DELAY_SEC="${NAV2_START_DELAY_SEC:-6.0}"
export SCAN_INPUT_TOPIC="${SCAN_INPUT_TOPIC:-/scan}"
export SCAN_NAV_TOPIC="${SCAN_NAV_TOPIC:-/scan}"

bash "$SCRIPT_DIR/run_semantic_nav_resume.sh" &
RESUME_PID=$!

cleanup() {
  kill "$RESUME_PID" 2>/dev/null || true
}
trap cleanup INT TERM

if ! "$SCRIPT_DIR/nav2_action_server_recover.sh"; then
  echo "[stable] Nav2 action server did not become ready."
  echo "[stable] Leaving resume launch running for log inspection. PID=$RESUME_PID"
  wait "$RESUME_PID"
  exit 1
fi

source "$SCRIPT_DIR/sparky_ros_env.sh" || true
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

RVIZ_CFG="$(ros2 pkg prefix go2_semantic_nav_agent)/share/go2_semantic_nav_agent/config/semantic_nav.rviz"

echo "[stable] /navigate_to_pose is ready."
echo "[stable] Launching RViz now. Set the initial pose, then send a Nav2 goal."

ros2 run rviz2 rviz2 -d "$RVIZ_CFG" &
RVIZ_PID=$!

wait "$RESUME_PID"
