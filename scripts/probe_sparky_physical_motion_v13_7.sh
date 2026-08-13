#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== SPARKY PHYSICAL MOTION PROBE ==="
echo "This test deliberately bypasses Nav2 planning and collision_monitor output."
echo "Clear ~1.5 m in front of the robot. The Move test lasts 0.8 s."
echo

ros2 node list 2>/dev/null | grep -qx /go2_driver_node || { echo 'FAIL: /go2_driver_node missing'; exit 2; }

# Remove collision_monitor as a competing /cmd_vel_out publisher for this one
# diagnostic. The direct test is intentionally driver-only.
CM_WAS_ACTIVE=0
if timeout 3 ros2 lifecycle get /collision_monitor 2>/dev/null | grep -qi active; then
  echo "[probe] temporarily deactivating collision_monitor for driver-only test"
  ros2 lifecycle set /collision_monitor deactivate >/dev/null 2>&1 || true
  CM_WAS_ACTIVE=1
  sleep 0.5
fi
restore_cm(){
  if [[ "$CM_WAS_ACTIVE" == "1" ]]; then
    ros2 lifecycle set /collision_monitor activate >/dev/null 2>&1 || true
  fi
}
trap restore_cm EXIT

# Obstacle avoidance should not own movement in this stack.
ros2 param set /go2_driver_node obstacle_avoidance false >/dev/null 2>&1 || true

# Put Unitree motion switcher into standard normal Sport mode. The current
# upstream WebRTC sport example performs this before Move(1008).
echo "[probe] selecting motion-switcher mode=normal"
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{id: 0, topic: 'rt/api/motion_switcher/request', api_id: 1002, parameter: '{\"name\":\"normal\"}', priority: 1}" >/dev/null
sleep 4

echo "[probe] StandUp + BalanceStand"
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{id: 0, topic: 'rt/api/sport/request', api_id: 1004, parameter: '1004', priority: 1}" >/dev/null
sleep 0.5
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{id: 0, topic: 'rt/api/sport/request', api_id: 1002, parameter: '1002', priority: 1}" >/dev/null
sleep 1.0

echo "[probe] direct driver Move test via /cmd_vel_out: vx=0.45 for 0.8 s"
python - <<'PY'
import math, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rclpy.init()
n = rclpy.create_node('sparky_v13_7_physical_probe')
pub = n.create_publisher(Twist, '/cmd_vel_out', 10)
last_odom = None

def cb(msg):
    global last_odom
    last_odom = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

sub = n.create_subscription(Odometry, '/odom', cb, 10)
end = time.monotonic() + 2.0
while time.monotonic() < end and last_odom is None:
    rclpy.spin_once(n, timeout_sec=0.1)
start = last_odom
print('odom_start=', start)

m = Twist(); m.linear.x = 0.45
end = time.monotonic() + 2.8
while time.monotonic() < end:
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.01)
    time.sleep(0.04)

z = Twist()
end = time.monotonic() + 1.5
while time.monotonic() < end:
    pub.publish(z)
    rclpy.spin_once(n, timeout_sec=0.02)
    time.sleep(0.04)

# allow odom to settle
end = time.monotonic() + 1.0
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
finish = last_odom
print('odom_finish=', finish)
if start is not None and finish is not None:
    d = math.hypot(finish[0]-start[0], finish[1]-start[1])
    print(f'odom_displacement_m={d:.3f}')
    if d >= 0.05:
        print('RESULT=PHYSICAL_MOVE_WORKS')
        print('INTERPRETATION=driver/WebRTC/Sport Move works; remaining fault is above the driver (Nav2/localization/command ownership).')
    else:
        print('RESULT=NO_PHYSICAL_MOVE')
        print('INTERPRETATION=Nav2 is exonerated; investigate WebRTC send acceptance / motion lease / robot mode.')
else:
    print('RESULT=NO_ODOM_EVIDENCE')
    print('INTERPRETATION=real-robot telemetry is incomplete; fix driver/WebRTC connection before Nav2.')

n.destroy_node(); rclpy.shutdown()
PY

restore_cm
trap - EXIT

echo
echo "If RESULT=NO_PHYSICAL_MOVE, inspect the base-stack terminal for:"
echo "  transport_send_failed"
echo "  data channel not open"
echo "  locomotion_prepare_failed"
echo "If none appear and gestures also fail, the robot is rejecting outgoing Sport RPCs despite telemetry being connected."
