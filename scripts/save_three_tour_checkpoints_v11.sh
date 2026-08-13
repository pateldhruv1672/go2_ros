#!/usr/bin/env bash
set -eo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash

cmd="${1:-show}"
SESSION="${SESSION:-digital_twin_lab_20260808_024634}"
SESSION_DIR="$HOME/.ros/go2_semantic_nav_sessions/$SESSION"

need_semantic() {
  local n
  n="$(ros2 topic info /semantic_nav/command 2>/dev/null | awk '/Subscription count:/ {print $3}' | tail -1)"
  if [[ -z "$n" || "$n" -lt 1 ]]; then
    echo "ERROR: /semantic_nav/command has no subscriber. Tour/semantic_nav is not ready." >&2
    exit 3
  fi
  if ! timeout 4 ros2 run tf2_ros tf2_echo map base_link >/dev/null 2>&1; then
    echo "ERROR: map->base_link TF is not ready; refusing to save a bad checkpoint." >&2
    exit 4
  fi
}

send() {
  local json="$1"
  timeout 6 ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: '$json'}"
}

case "$cmd" in
  welcome)
    need_semantic
    send '{"type":"save_tour_stop","name":"welcome_checkpoint","script":"Welcome to the Digital Twin Lab. I am Sparky, your robotic guide. I will show you our humanoid and robotic arm research, and you can ask questions along the way.","fact":"Welcome to the Digital Twin Lab.","room":"entrance","source":"tour_admin"}'
    ;;
  g1)
    need_semantic
    send '{"type":"save_tour_stop","name":"g1_checkpoint","script":"This is the Unitree G1 humanoid robot. We use it to explore embodied AI, perception, control, and human robot interaction.","fact":"Unitree G1 humanoid research.","room":"g1_area","source":"tour_admin"}'
    ;;
  xarm7|xarm)
    need_semantic
    send '{"type":"save_tour_stop","name":"xarm7_checkpoint","script":"These are xArm 7 robotic arms. We use them for manipulation, teleoperation, perception, and digital twin experiments.","fact":"xArm 7 robotic arm research.","room":"xarm_area","source":"tour_admin"}'
    ;;
  order)
    need_semantic
    send '{"type":"reorder_tour_stops","order":["welcome_checkpoint","g1_checkpoint","xarm7_checkpoint"],"source":"tour_admin"}'
    ;;
  show)
    echo "=== $SESSION_DIR/places.yaml ==="
    cat "$SESSION_DIR/places.yaml" 2>/dev/null || true
    echo
    echo "=== $SESSION_DIR/route.yaml ==="
    cat "$SESSION_DIR/route.yaml" 2>/dev/null || true
    ;;
  *)
    echo "usage: $0 {welcome|g1|xarm7|order|show}" >&2
    exit 2
    ;;
esac
