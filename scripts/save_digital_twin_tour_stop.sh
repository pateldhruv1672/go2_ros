#!/usr/bin/env bash
set -euo pipefail

KEY="${1:-}"
if [[ -z "$KEY" ]]; then
  echo "Usage: $0 welcome|humanoid|roboarms [custom narration]" >&2
  exit 2
fi

case "$KEY" in
  welcome|welcome_checkpoint)
    NAME="welcome_checkpoint"
    DEFAULT_SCRIPT="Welcome to the Digital Twin Lab. I am Sparky, your robotic guide. During this tour I can navigate a previously taught map, remember objects and places, use live vision to verify the environment, answer your questions, and demonstrate selected robot motions. Please feel free to interrupt me with questions at any checkpoint."
    FACT="Digital Twin Lab welcome and Sparky introduction."
    ;;
  humanoid|humanoid_checkpoint)
    NAME="humanoid_checkpoint"
    DEFAULT_SCRIPT="We have reached the humanoid robotics checkpoint. This area highlights work in embodied intelligence, perception, control, simulation, and human robot interaction. I can pause here while you ask questions, and when you are ready you can say, Sparky, continue the tour."
    FACT="Humanoid robotics demonstration checkpoint."
    ;;
  roboarms|roboarms_checkpoint|robotarms)
    NAME="roboarms_checkpoint"
    DEFAULT_SCRIPT="We have reached the robot arms checkpoint. This area focuses on robotic manipulation, control, perception, teleoperation, and digital twin workflows that connect simulated experiments with physical robotic systems. I can answer follow up questions before we continue."
    FACT="Robot arms and manipulation checkpoint."
    ;;
  *)
    echo "Unknown stop '$KEY'. Use welcome, humanoid, or roboarms." >&2
    exit 3
    ;;
esac

SCRIPT="${2:-$DEFAULT_SCRIPT}"
ROOM="${TOUR_ROOM:-digital_twin_lab}"
PAUSE="${TOUR_PAUSE_SEC:-4.0}"

PAYLOAD="$(python3 - "$NAME" "$ROOM" "$SCRIPT" "$FACT" "$PAUSE" <<'PY'
import json,sys
name,room,script,fact,pause=sys.argv[1:]
print(json.dumps({
  "type":"save_tour_stop",
  "name":name,
  "room":room,
  "category":"tour_stop",
  "tags":["tour_stop","digital_twin_lab"],
  "script":script,
  "fact":fact,
  "pause_seconds":float(pause),
}, separators=(",",":")))
PY
)"

echo "Saving current map pose as: $NAME"
echo "Narration: $SCRIPT"
ESCAPED="${PAYLOAD//\'/\'\'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: '$ESCAPED'}"
echo
echo "Watch confirmation with: ros2 topic echo /semantic_nav/status"
