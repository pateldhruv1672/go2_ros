#!/usr/bin/env bash
set -euo pipefail
ROOT="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$ROOT"
set +u
source /opt/ros/jazzy/setup.bash
[ -f src/.venv/bin/activate ] && source src/.venv/bin/activate
[ -f install/setup.bash ] && source install/setup.bash
set -u

echo '=== V12.8 effective Nav2 profile ==='
for p in \
  FollowPath.min_speed_xy FollowPath.min_speed_theta FollowPath.max_vel_x FollowPath.max_speed_xy \
  FollowPath.max_vel_theta FollowPath.acc_lim_x FollowPath.acc_lim_theta \
  FollowPath.decel_lim_x FollowPath.decel_lim_theta FollowPath.sim_time \
  general_goal_checker.xy_goal_tolerance general_goal_checker.yaw_goal_tolerance \
  progress_checker.required_movement_radius progress_checker.required_movement_angle progress_checker.movement_time_allowance
do
  printf '%-52s ' "$p"
  ros2 param get /controller_server "$p" 2>/dev/null || echo MISSING
done

echo; echo '=== actuator contract ==='
for p in nav2_min_effective_x nav2_zero_subfloor_x source_timeout_sec; do
  printf '%-32s ' "$p"; ros2 param get /go2_motion_arbiter "$p" 2>/dev/null || echo MISSING
done

echo; echo '=== command ownership ==='
for t in /cmd_vel_nav2 /cmd_vel_escape /cmd_vel_nav /cmd_vel_out; do
  echo "--- $t"; ros2 topic info "$t" --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:' || true
done

echo; echo '=== agent/VLM path ==='
for t in /go2_voice/transcript /go2_agent/user_command /go2_agent/query /go2_vlm/query /go2_vlm/query_result; do
  echo "--- $t"; ros2 topic info "$t" 2>/dev/null | grep -E 'Publisher count|Subscription count' || true
done

if [ "${1:-}" = "--vlm-freshness-test" ]; then
  REQUEST_ID="freshness_$(date +%s%N)"
  NOW="$(python3 - <<'PY'
import time
print(f'{time.time():.9f}')
PY
)"
  echo; echo "Sending safe live-VLM freshness test request_id=$REQUEST_ID command_received_unix=$NOW"
  ros2 topic pub --once /go2_vlm/query std_msgs/msg/String "{data: '{\"request_id\":\"$REQUEST_ID\",\"question\":\"What do you see in front of you?\",\"source\":\"v12_8_check\",\"command_received_unix\":$NOW}'}" >/dev/null
  echo 'Waiting for result; verify freshness_policy=post_command_frame_required and image_received_unix >= command_received_unix.'
  timeout 35 ros2 topic echo /go2_vlm/query_result --once || true
fi
