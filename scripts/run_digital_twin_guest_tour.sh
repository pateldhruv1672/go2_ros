#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f "$ROOT_DIR/src/.venv/bin/activate" ]] && source "$ROOT_DIR/src/.venv/bin/activate"
source "$ROOT_DIR/install/setup.bash"
set -u

export ROBOT_IP="${ROBOT_IP:-192.168.12.1}"
export CONN_TYPE="${CONN_TYPE:-webrtc}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export SESSION_ROOT="${SESSION_ROOT:-$HOME/.ros/go2_semantic_nav_sessions}"
export SESSION_NAME="${SESSION_NAME:-latest}"
export ENABLE_OBJECT_PERCEPTION="${ENABLE_OBJECT_PERCEPTION:-1}"
export ENABLE_OBJECT_OVERLAY="${ENABLE_OBJECT_OVERLAY:-1}"
export ENABLE_SAM2="${ENABLE_SAM2:-1}"
export SPARKY_GREEN_PATH_HZ="${SPARKY_GREEN_PATH_HZ:-15}"

CLEAR_TOUR_MEMORY="${CLEAR_TOUR_MEMORY:-1}"
CONFIG_ARGS=(--session-root "$SESSION_ROOT" --session-name "$SESSION_NAME")
[[ "$CLEAR_TOUR_MEMORY" == 1 ]] && CONFIG_ARGS+=(--clear-tour-memory)
python3 "$SCRIPT_DIR/configure_digital_twin_guest_tour.py" "${CONFIG_ARGS[@]}"

# Resolve the actual session name selected by the configurator/runner.
if [[ -z "$SESSION_NAME" || "$SESSION_NAME" == latest || "$SESSION_NAME" == auto ]]; then
  SESSION_NAME="$(find "$SESSION_ROOT" -mindepth 1 -maxdepth 1 -type d -exec test -f '{}/map.yaml' ';' -exec test -f '{}/route.yaml' ';' -printf '%T@ %f\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
  export SESSION_NAME
fi

echo "[guest-tour] session=$SESSION_NAME"
echo "[guest-tour] overlay=ON SAM2=$ENABLE_SAM2 green_path=${SPARKY_GREEN_PATH_HZ}Hz"

bash "$SCRIPT_DIR/run_sparky_resume_agentic_tour.sh" &
RUNNER_PID=$!
cleanup(){
  rc=$?
  kill "$RUNNER_PID" 2>/dev/null || true
  wait "$RUNNER_PID" 2>/dev/null || true
  exit $rc
}
trap cleanup INT TERM EXIT

ready=0
for _ in $(seq 1 120); do
  if ros2 node list 2>/dev/null | grep -qx /semantic_nav_node && \
     timeout 2 ros2 lifecycle get /controller_server 2>/dev/null | grep -q 'active'; then
    ready=1
    break
  fi
  kill -0 "$RUNNER_PID" 2>/dev/null || { echo '[guest-tour] base runner exited' >&2; exit 3; }
  sleep 1
done
[[ "$ready" == 1 ]] || { echo '[guest-tour] semantic Resume/Nav2 did not become ready' >&2; exit 4; }

# Load the freshly curated route into an already-running semantic_nav_node too.
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String \
  "{data: '{\"type\":\"reload_tour\"}'}" >/dev/null
sleep 0.5

AUTO_START_TOUR="${AUTO_START_TOUR:-1}"
if [[ "$AUTO_START_TOUR" == 1 ]]; then
  echo '[guest-tour] Welcoming guests and starting the 3-stop tour.'
  ros2 topic pub --once /semantic_nav/command std_msgs/msg/String \
    "{data: '{\"type\":\"start_tour\",\"reset_index\":true}'}" >/dev/null
fi

wait "$RUNNER_PID"
