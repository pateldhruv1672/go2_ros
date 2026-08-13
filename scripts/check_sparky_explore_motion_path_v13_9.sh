#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== EXPLORE_MODE DRIVER CONTRACT ==="
for p in obstacle_avoidance cmd_vel_linear_gain cmd_vel_angular_gain cmd_vel_min_linear_x cmd_vel_min_angular_z cmd_vel_max_linear_x cmd_vel_max_angular_z; do
  printf '%-30s ' "$p"
  ros2 param get /go2_driver_node "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== GENERATED SPORT MOVE ==="
python - <<'PY'
import json
from go2_robot_sdk.application.utils.command_generator import gen_mov_command
m = json.loads(gen_mov_command(0.45, 0.0, 0.0, False))
print('type=', m.get('type'))
print('topic=', m.get('topic'))
print('api_id=', m['data']['header']['identity']['api_id'])
print('parameter=', m['data']['parameter'])
PY

echo
echo "=== ADAPTER FLOOR ==="
python - <<'PY'
from go2_robot_sdk.infrastructure.webrtc.webrtc_adapter import WebRTCAdapter
for x in (0.0, 0.01, 0.05, 0.08, 0.20, 0.45, 0.90):
    print(f'{x:>5.2f} -> {WebRTCAdapter._apply_axis_gain(x,1.0,0.08,0.75):>5.2f}')
PY
