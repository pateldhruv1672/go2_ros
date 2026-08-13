#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

count_node_exact(){
  local name="$1"
  ros2 node list 2>/dev/null | awk -v n="$name" '$0==n{c++} END{print c+0}'
}

echo "=== AUDIO OWNERSHIP ==="
printf '/go2_tts_node count:       '; count_node_exact /go2_tts_node
printf '/go2_speech_arbiter count: '; count_node_exact /go2_speech_arbiter
echo "-- /go2_tts/say"
ros2 topic info /go2_tts/say --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:' || true
echo "-- local speaker processes"
pgrep -af 'espeak-ng|(^|/)espeak |spd-say' || true

echo
echo "=== NAV / DRIVER CHAIN ==="
for t in /cmd_vel_nav2 /cmd_vel_nav /cmd_vel_out /cmd_vel_sdk; do
  echo "-- $t"
  ros2 topic info "$t" 2>/dev/null | grep -E 'Publisher count|Subscription count' || true
done

echo
echo "=== NAV2 LIFECYCLE ==="
for n in controller_server planner_server behavior_server bt_navigator collision_monitor; do
  printf '%-22s ' "$n"
  timeout 3 ros2 lifecycle get "/$n" 2>/dev/null || echo NOT_AVAILABLE
done

echo
echo "=== DRIVER ==="
ros2 node list 2>/dev/null | grep -E '^/go2_driver_node$' || echo 'MISSING /go2_driver_node'
echo "GO2_LOCOMOTION_REARM_IDLE_SEC=${GO2_LOCOMOTION_REARM_IDLE_SEC:-2.0}"

if [[ "${1:-}" == "--motion-test" ]]; then
  echo
  echo "WARNING: this WILL move the robot forward. Clear the area first."
  echo "Publishing 0.30 m/s for 0.8 s, then zero for 1.5 s."
  python - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist
rclpy.init()
n=rclpy.create_node('sparky_v13_3_motion_test')
p=n.create_publisher(Twist,'/cmd_vel_out',10)
for _ in range(15): rclpy.spin_once(n,timeout_sec=0.05)
m=Twist(); m.linear.x=0.30
end=time.monotonic()+0.8
while time.monotonic()<end:
    p.publish(m); rclpy.spin_once(n,timeout_sec=0.01); time.sleep(0.04)
z=Twist()
end=time.monotonic()+1.5
while time.monotonic()<end:
    p.publish(z); rclpy.spin_once(n,timeout_sec=0.01); time.sleep(0.04)
n.destroy_node(); rclpy.shutdown()
print('motion test complete')
PY
fi
