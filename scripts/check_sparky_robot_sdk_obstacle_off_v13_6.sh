#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== SOURCE DEFAULTS ==="
grep -n "obstacle_avoidance" src/go2_robot_sdk/go2_robot_sdk/presentation/go2_driver_node.py | head -n 6 || true
grep -n "obstacle_avoidance" src/go2_robot_sdk/launch/robot.launch.py | head -n 4 || true

echo
echo "=== GENERATED MOVE COMMAND CONTRACT ==="
python - <<'PY'
import json
from go2_robot_sdk.application.utils.command_generator import gen_mov_command

normal = json.loads(gen_mov_command(0.45, 0.0, 0.0, False))
avoid = json.loads(gen_mov_command(0.45, 0.0, 0.0, True))

nid = normal['data']['header']['identity']['api_id']
ntopic = normal['topic']
aid = avoid['data']['header']['identity']['api_id']
atopic = avoid['topic']
print(f"OFF  -> api_id={nid} topic={ntopic}")
print(f"ON   -> api_id={aid} topic={atopic}")
assert nid == 1008, normal
assert ntopic == 'rt/api/sport/request', normal
assert aid == 1003, avoid
assert atopic == 'rt/api/obstacles_avoid/request', avoid
print("PASS: obstacle_avoidance=False uses standard Sport Move API 1008")
PY

echo
echo "=== LIVE DRIVER ==="
if ros2 node list 2>/dev/null | grep -qx '/go2_driver_node'; then
  ros2 param get /go2_driver_node obstacle_avoidance || true
  echo "If this says True, stop/restart the base robot stack after installing v13.6."
else
  echo "/go2_driver_node is not currently running"
fi
