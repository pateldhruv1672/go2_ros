#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
set +u
source /opt/ros/jazzy/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u

count_ep(){
  local topic="$1" section="$2" node="$3"
  ros2 topic info "$topic" --verbose 2>/dev/null | awk -v want="$section" -v node="$node" '
    /^Publishers:/{sec="pub";next} /^Subscribers:/{sec="sub";next}
    /^Node name:/{n=$0;sub(/^Node name:[[:space:]]*/,"",n); if(sec==want && n==node)c++}
    END{print c+0}'
}

echo '=== Nav2 lifecycle ==='
for n in controller_server planner_server behavior_server bt_navigator collision_monitor; do
  printf '%-24s ' "$n"
  timeout 3 ros2 lifecycle get "/$n" 2>/dev/null || echo 'NOT AVAILABLE'
done

echo
echo '=== NavigateToPose action ==='
if timeout 4 ros2 action info /navigate_to_pose 2>/dev/null; then :; else echo 'FAIL: /navigate_to_pose unavailable'; fi

echo
echo '=== controller runtime ==='
for p in FollowPath.min_speed_xy FollowPath.max_vel_x FollowPath.max_speed_xy FollowPath.max_vel_theta FollowPath.acc_lim_x FollowPath.decel_lim_x general_goal_checker.xy_goal_tolerance; do
  printf '%-45s ' "$p"
  timeout 3 ros2 param get /controller_server "$p" 2>/dev/null || echo 'MISSING'
done

echo
echo '=== arbiter fidelity ==='
printf '%-32s ' 'nav2_zero_subfloor_x'
timeout 3 ros2 param get /go2_motion_arbiter nav2_zero_subfloor_x 2>/dev/null || echo 'MISSING'
nav2_sub="$(count_ep /cmd_vel_nav2 sub go2_motion_arbiter)"
esc_sub="$(count_ep /cmd_vel_escape sub go2_motion_arbiter)"
nav_pub="$(count_ep /cmd_vel_nav pub go2_motion_arbiter)"
echo "arbiter endpoints nav2_sub=$nav2_sub escape_sub=$esc_sub nav_pub=$nav_pub"
if [ "$nav2_sub" = 1 ] && [ "$esc_sub" = 1 ] && [ "$nav_pub" = 1 ]; then
  echo 'PASS: single motion arbiter owner'
else
  echo 'FAIL: duplicated/missing motion arbiter owner'
  pgrep -af 'motion_arbiter|semantic_nav_resume.launch.py' || true
fi

echo
echo '=== AMCL / scan ==='
printf '%-32s ' 'AMCL scan_topic'
timeout 3 ros2 param get /amcl scan_topic 2>/dev/null || echo 'MISSING'
printf '%-32s ' 'AMCL pose sample'
timeout 3 ros2 topic echo /amcl_pose --once 2>/dev/null | head -12 || true

echo
echo '=== command ownership ==='
for t in /cmd_vel_nav2 /cmd_vel_escape /cmd_vel_nav /cmd_vel_out; do
  echo "--- $t"
  timeout 3 ros2 topic info "$t" --verbose 2>/dev/null | egrep 'Publisher count|Subscription count|Node name' || true
done

echo
echo '=== semantic Tour status ==='
timeout 3 ros2 topic echo /semantic_nav/status --once 2>/dev/null || true
