#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== V13.12 NAV/STAND LOCK ==="
python - <<'PY'
from pathlib import Path
import ast
p = Path("src/go2_robot_sdk/go2_robot_sdk/application/services/robot_control_service.py")
s = p.read_text()
tree = ast.parse(s)
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RobotControlService")
fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "handle_cmd_vel")
attrs = [
    n.func.attr for n in ast.walk(fn)
    if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
]
print("handle_cmd_vel calls:", attrs)
print("hidden StandUp:", "send_stand_up_command" in attrs)
print("hidden StandDown:", "send_stand_down_command" in attrs)
print("movement forwarding:", "send_movement_command" in attrs)
print("marker:", "SPARKY_NAV_NO_HIDDEN_STAND_V13_12" in s)
PY

echo
echo "=== LIVE DRIVER ==="
ros2 node list 2>/dev/null | grep -x /go2_driver_node || echo "MISSING /go2_driver_node"
printf "obstacle_avoidance: "
ros2 param get /go2_driver_node obstacle_avoidance 2>/dev/null || true

echo
echo "=== COMMAND PATH ==="
for t in /cmd_vel_nav2 /cmd_vel_nav /cmd_vel_out /cmd_vel_sdk; do
  echo "-- $t"
  ros2 topic info "$t" 2>/dev/null | grep -E 'Publisher count|Subscription count' || true
done

echo
echo "Expected:"
echo "  hidden StandUp: False"
echo "  hidden StandDown: False"
echo "  movement forwarding: True"
echo "  obstacle_avoidance: False"
