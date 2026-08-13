#!/usr/bin/env bash
set -eo pipefail

echo "============================================================"
echo " SPARKY V11.6.2 NAV2 RECOVERY + DWB DIAGNOSTIC CHECK"
echo "============================================================"

echo
echo "=== COMMAND OWNERSHIP ==="
ros2 topic info /cmd_vel_nav2 --verbose 2>/dev/null | \
  grep -E 'Publisher count:|Node name: (controller_server|behavior_server)|Subscription count:|Node name: go2_motion_arbiter' || true

echo
echo "EXPECTED:"
echo "  /cmd_vel_nav2 publisher count = 2"
echo "    controller_server"
echo "    behavior_server"
echo "  /cmd_vel_nav2 subscriber = go2_motion_arbiter"
echo
echo "Two publishers here are intentional: both are the single logical Nav2 authority."
echo "The BT should activate controller motion or a recovery behavior, not both."

echo
echo "=== DWB DEBUG CONTRACT ==="
for p in \
  current_goal_checker \
  current_progress_checker \
  FollowPath.debug_trajectory_details \
  FollowPath.publish_evaluation \
  FollowPath.publish_trajectories \
  FollowPath.publish_cost_grid_pc \
  FollowPath.min_speed_xy \
  FollowPath.min_speed_theta \
  FollowPath.max_vel_x \
  FollowPath.max_vel_theta
do
  printf "%-38s " "$p"
  ros2 param get /controller_server "$p" 2>/dev/null || echo "UNAVAILABLE"
done

echo
echo "=== RECOVERY SAFETY CONTRACT ==="
for p in \
  cycle_frequency \
  simulate_ahead_time \
  max_rotational_vel \
  min_rotational_vel \
  rotational_acc_lim \
  transform_tolerance
do
  printf "%-28s " "$p"
  ros2 param get /behavior_server "$p" 2>/dev/null || echo "UNAVAILABLE"
done

echo
echo "EXPECTED RECOVERY ROTATION:"
echo "  max_rotational_vel = 0.20 rad/s"
echo "  min_rotational_vel = 0.05 rad/s"
echo "  rotational_acc_lim = 0.35 rad/s^2"

echo
echo "=== LOCAL COSTMAP / FOOTPRINT ==="
for p in \
  footprint \
  plugins \
  obstacle_layer.observation_sources \
  obstacle_layer.scan.topic \
  inflation_layer.inflation_radius \
  inflation_layer.cost_scaling_factor
do
  printf "%-40s " "$p"
  ros2 param get /local_costmap/local_costmap "$p" 2>/dev/null || echo "UNAVAILABLE"
done

echo
echo "=== DWB DEBUG TOPICS ==="
ros2 topic list 2>/dev/null | grep -E 'evaluation|trajectory|cost_cloud|local_plan' | sort || true

echo
echo "=== TEST RULE ==="
echo "Use ONE 0.5-1.0 m RViz goal in open space."
echo "Do not keep preempting it with new goals."
echo "If DWB prints 'No valid trajectories', copy the FIRST detailed"
echo "critic rejection lines immediately above/below that message."
