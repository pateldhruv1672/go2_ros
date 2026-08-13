#!/usr/bin/env bash
set -euo pipefail
MODE="${1:-help}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

require_semantic_nav() {
  "$ROOT/scripts/check_semantic_nav_ready.sh" >/dev/null
}

publish_wire() {
  local wire="$1"
  require_semantic_nav
  local msg
  msg="$(python - "$wire" <<'PY'
import json, sys
print('{data: ' + json.dumps(sys.argv[1]) + '}')
PY
)"
  if ! timeout 8 ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "$msg"; then
    echo 'ERROR: semantic-nav did not accept the command within 8 seconds.' >&2
    exit 5
  fi
}

publish_stop() {
  local name="$1" room="$2" narration="$3"
  local wire
  wire="$(python - "$name" "$room" "$narration" <<'PY'
import json, sys
print(json.dumps({
  'type':'save_tour_stop',
  'name':sys.argv[1],
  'room':sys.argv[2],
  'script':sys.argv[3],
  'fact':sys.argv[3],
  'pause_seconds':0.0,
  'tags':['tour_stop','digital_twin_lab'],
  'source':'tour_checkpoint_helper',
}, separators=(',',':')))
PY
)"
  publish_wire "$wire"
  echo "Requested save of $name at CURRENT robot map pose."
  echo "Check: ros2 topic echo /semantic_nav/status --once"
}

case "$MODE" in
  welcome)
    publish_stop welcome_checkpoint digital_twin_lab "Welcome to the Digital Twin Lab. I am Sparky, your robotic guide. I will show you our humanoid and robotic-arm research, and you can ask me questions along the way."
    ;;
  g1|humanoid)
    publish_stop g1_checkpoint digital_twin_lab "This is the Unitree G1 humanoid robot. We use it to explore embodied AI, perception, control, and human-robot interaction."
    ;;
  xarm7|arms|roboarms)
    publish_stop xarm7_checkpoint digital_twin_lab "These are xArm 7 robotic arms. We use them for manipulation, teleoperation, perception, and digital-twin experiments."
    ;;
  order)
    publish_wire '{"type":"reorder_tour_stops","order":["welcome_checkpoint","g1_checkpoint","xarm7_checkpoint"],"source":"tour_checkpoint_helper"}'
    ;;
  show)
    SESSION="${SESSION:-$(basename "$(ls -td "$HOME/.ros/go2_semantic_nav_sessions"/* | head -1)")}"
    echo "SESSION=$SESSION"
    echo '--- places.yaml ---'
    cat "$HOME/.ros/go2_semantic_nav_sessions/$SESSION/places.yaml" 2>/dev/null || true
    echo '--- route.yaml ---'
    cat "$HOME/.ros/go2_semantic_nav_sessions/$SESSION/route.yaml" 2>/dev/null || true
    ;;
  *)
    cat <<'HELP'
Usage:
  bash scripts/save_three_tour_checkpoints.sh welcome
  bash scripts/save_three_tour_checkpoints.sh g1
  bash scripts/save_three_tour_checkpoints.sh xarm7
  bash scripts/save_three_tour_checkpoints.sh order
  bash scripts/save_three_tour_checkpoints.sh show

IMPORTANT: semantic_nav_tour_world.launch.py must already be running and localized.
HELP
    ;;
esac
