#!/usr/bin/env bash
set -eo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash

case "${1:-}" in
  welcome) place=welcome_checkpoint ;;
  g1|humanoid) place=g1_checkpoint ;;
  xarm7|xarm|arms) place=xarm7_checkpoint ;;
  *) echo "usage: $0 {welcome|g1|xarm7}" >&2; exit 2 ;;
esac
subs="$(ros2 topic info /semantic_nav/command 2>/dev/null | awk '/Subscription count:/ {print $3}' | tail -1)"
if [[ -z "$subs" || "$subs" -lt 1 ]]; then
  echo "ERROR: semantic_nav is not ready." >&2
  exit 3
fi
timeout 6 ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: '{\"type\":\"go\",\"place\":\"$place\",\"source\":\"tour_admin_test\"}'}"
