#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== WEBRTC RPC WIRE FORMAT ==="
PYTHONPATH="$WS/src/go2_robot_sdk${PYTHONPATH:+:$PYTHONPATH}" python3 - <<'PY'
import json
from go2_robot_sdk.application.utils.command_generator import gen_mov_command, gen_command
for name, raw in [
    ('Move', gen_mov_command(0.45, 0.0, 0.0, False)),
    ('NormalMode', gen_command(1002, {'name':'normal'}, 'rt/api/motion_switcher/request')),
    ('StandUp', gen_command(1004)),
    ('BalanceStand', gen_command(1002)),
]:
    obj = json.loads(raw)
    ident = obj.get('data',{}).get('header',{}).get('identity',{})
    print(f"{name:12s} type={obj.get('type')} topic={obj.get('topic')} api_id={ident.get('api_id')} parameter={obj.get('data',{}).get('parameter')}")
PY

echo
echo "=== LIVE DRIVER ==="
ros2 node list 2>/dev/null | grep -E '^/go2_driver_node$' || echo 'MISSING /go2_driver_node'
printf 'obstacle_avoidance: '; ros2 param get /go2_driver_node obstacle_avoidance 2>/dev/null || true

echo
echo "=== RECENT RPC ACCEPTANCE LOGS ==="
LATEST="$HOME/.ros/log/latest"
if [[ -e "$LATEST" ]]; then
  grep -R -h -E 'WEBRTC_RPC_RESPONSE|WEBRTC_RPC_ERROR|transport_send_failed|data channel not open|locomotion_prepare_failed' "$LATEST" 2>/dev/null | tail -n 40 || true
else
  echo "No ~/.ros/log/latest yet"
fi

echo
echo "Expected wire format: type=req for Move/NormalMode/StandUp/BalanceStand"
echo "Expected robot acceptance: WEBRTC_RPC_RESPONSE ... code=0"
