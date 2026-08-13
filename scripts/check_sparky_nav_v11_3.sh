#!/usr/bin/env bash
set -eo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
[[ -f scripts/sparky_runtime_env.sh ]] && source scripts/sparky_runtime_env.sh || true

echo "=== V11.3 CONTROL CONTRACT ==="
for p in cmd_vel_min_linear_x cmd_vel_min_angular_z; do
  printf '%-30s ' "$p"
  ros2 param get /go2_driver_node "$p" || true
done
for p in fallback_enable fallback_cmd_topic; do
  printf '%-30s ' "$p"
  ros2 param get /semantic_nav_node "$p" || true
done
for p in FollowPath.min_speed_xy FollowPath.min_speed_theta FollowPath.max_vel_x FollowPath.max_vel_theta FollowPath.acc_lim_x FollowPath.acc_lim_theta; do
  printf '%-30s ' "$p"
  ros2 param get /controller_server "$p" || true
done

echo
echo "=== COMMAND OWNERSHIP ==="
for t in /cmd_vel_nav2 /cmd_vel_nav /cmd_vel_out; do
  echo "--- $t ---"
  ros2 topic info "$t" --verbose | grep -E 'Publisher count:|Subscription count:|Node name:' || true
done

echo
echo "=== ODOM TWIST SAMPLE ==="
ros2 topic echo /odom --once --field twist.twist || true

echo
echo "NOTE: while physically stationary, zero odom twist is correct."
echo "Before autonomous navigation, move the robot briefly using your normal manual control"
echo "and repeat: ros2 topic echo /odom --once --field twist.twist"
echo "linear.x and/or angular.z must become non-zero while it is actually moving."
