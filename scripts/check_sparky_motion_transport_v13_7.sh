#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== DRIVER CONNECTION / PARAMS ==="
ros2 node list 2>/dev/null | grep -E '^/go2_driver_node$' || echo 'MISSING /go2_driver_node'
for p in obstacle_avoidance cmd_vel_linear_gain cmd_vel_min_linear_x cmd_vel_max_linear_x cmd_vel_max_angular_z; do
  printf '%-30s ' "$p"
  ros2 param get /go2_driver_node "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== REAL ROBOT TELEMETRY ==="
for t in /odom /go2_states /camera/image_raw /cmd_vel_out /cmd_vel_sdk; do
  echo "--- $t"
  ros2 topic info "$t" 2>/dev/null | grep -E 'Publisher count|Subscription count' || true
done

echo
echo "=== COMMAND OWNERS ==="
ros2 topic info /cmd_vel_out --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:' || true

echo
echo "Important driver log markers after restart:"
echo "  Robot 0 validated; preparing normal Sport locomotion mode"
echo "  motion_switcher_select_normal sent robot=0"
echo "  locomotion_ready robot=0 mode=normal posture=BalanceStand"
echo "Any transport_send_failed / data channel not open error is a driver/WebRTC root cause."
