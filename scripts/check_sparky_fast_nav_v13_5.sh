#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== V13.5 FAST NAV RUNTIME ==="
for p in \
  controller_frequency \
  FollowPath.min_speed_xy \
  FollowPath.min_speed_theta \
  FollowPath.max_vel_x \
  FollowPath.max_speed_xy \
  FollowPath.max_vel_theta \
  FollowPath.acc_lim_x \
  FollowPath.acc_lim_theta \
  FollowPath.decel_lim_x \
  FollowPath.decel_lim_theta \
  FollowPath.sim_time \
  FollowPath.vx_samples \
  FollowPath.vtheta_samples \
  general_goal_checker.xy_goal_tolerance \
  general_goal_checker.yaw_goal_tolerance \
  progress_checker.required_movement_radius \
  progress_checker.required_movement_angle \
  progress_checker.movement_time_allowance
do
  printf '%-52s ' "$p"
  ros2 param get /controller_server "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== LIFECYCLE ==="
for n in controller_server planner_server behavior_server bt_navigator collision_monitor; do
  printf '%-22s ' "$n"
  timeout 3 ros2 lifecycle get "/$n" 2>/dev/null || echo NOT_AVAILABLE
done

echo
echo "=== COMMAND PATH ==="
for t in /cmd_vel_nav2 /cmd_vel_nav /cmd_vel_out /cmd_vel_sdk; do
  echo "--- $t"
  ros2 topic info "$t" 2>/dev/null | grep -E 'Publisher count|Subscription count' || true
done

if [[ "${1:-}" == "--motion-test" ]]; then
  echo
  echo "WARNING: physical test. Clear the robot's path."
  echo "Direct /cmd_vel_out: +0.45 m/s for 0.8 s, then explicit zero for 1.5 s."
  python - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("sparky_v13_5_direct_motion_test")
pub = node.create_publisher(Twist, "/cmd_vel_out", 10)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.05)

cmd = Twist()
cmd.linear.x = 0.45
end = time.monotonic() + 0.8
while time.monotonic() < end:
    pub.publish(cmd)
    rclpy.spin_once(node, timeout_sec=0.01)
    time.sleep(0.04)

stop = Twist()
end = time.monotonic() + 1.5
while time.monotonic() < end:
    pub.publish(stop)
    rclpy.spin_once(node, timeout_sec=0.01)
    time.sleep(0.04)

node.destroy_node()
rclpy.shutdown()
print("direct motion test complete")
PY
fi
