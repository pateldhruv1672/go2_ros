#!/usr/bin/env bash
set -euo pipefail
DEST="${1:-}"
case "${DEST,,}" in
  welcome|entrance) PLACE=welcome_checkpoint ;;
  g1|humanoid|unitree-g1|unitree_g1) PLACE=g1_checkpoint ;;
  xarm7|xarm-7|arms|robot-arms|roboarms) PLACE=xarm7_checkpoint ;;
  *) echo "Usage: $0 welcome|g1|xarm7" >&2; exit 2 ;;
esac
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String \
  "{data: '{\"type\":\"go\",\"place\":\"$PLACE\",\"source\":\"tour_checkpoint_helper\"}'}"
