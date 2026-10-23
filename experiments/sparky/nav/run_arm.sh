#!/usr/bin/env bash
# usage: run_arm.sh <sim_ws> <repo_exp_dir> <delay_ms> <out_dir> [repeat]
set -o pipefail
SIM_WS=$1; EXP=$2; DELAY=$3; OUT=$4; REP=${5:-1}
mkdir -p "$OUT"
source /opt/ros/jazzy/setup.bash; source "$SIM_WS/install/setup.bash"
export ROS_DOMAIN_ID=$((50 + REP)) ROS_LOCALHOST_ONLY=1 LIBGL_ALWAYS_SOFTWARE=1 GZ_IP=127.0.0.1 GZ_PARTITION=arm$$
WORLD=$OUT/world; python3 "$EXP/nav/make_world.py" "$EXP/nav/scenario.json" "$SIM_WS/src/unitree_go2_ros2/unitree_go2_description/worlds/default.sdf" "$WORLD" > "$OUT/world.log" 2>&1
python3 "$EXP/nav/prepare_nav_params.py" "$NAV_CFG" "$WORLD/arena.yaml" "$EXP/nav/scenario.json" "$OUT/nav_params.yaml" "${PLANNER:-navfn}" "${CONTROLLER:-dwb}" > "$OUT/params.log" 2>&1
# Disable the EKF's own odom->base_footprint TF (the relay re-publishes it, with the arm's delay). Record this change.
EKF=$(ros2 pkg prefix champ_base)/share/champ_base/config/ekf/footprint_to_odom.yaml
sed -i 's/publish_tf: true/publish_tf: false/' "$EKF"; cp "$EKF" "$OUT/footprint_to_odom.effective.yaml"
ros2 launch unitree_go2_sim unitree_go2_launch.py rviz:=false gui:=false disable_camera:=true disable_d455:=true disable_lidar_l1:=true \
  world:="$WORLD/sparky_arena.sdf" > "$OUT/sim.log" 2>&1 &
SIM=$!
python3 "$EXP/nav/tf_delay_relay.py" "$DELAY" "$OUT/tf_delay_log.csv" > "$OUT/relay.log" 2>&1 &
RELAY=$!
ros2 launch "$EXP/nav/sparky_nav.launch.py" params:="$OUT/nav_params.yaml" > "$OUT/nav.log" 2>&1 &
NAV=$!
cleanup() { kill -INT $BAG 2>/dev/null; sleep 3; kill -INT $NAV $RELAY $SIM 2>/dev/null; sleep 5; kill -9 $NAV $RELAY $SIM 2>/dev/null; pkill -9 -f "gz sim" 2>/dev/null; true; }
trap cleanup EXIT
STORAGE=mcap; ros2 bag record -s mcap --help >/dev/null 2>&1 || STORAGE=sqlite3
ros2 bag record -s $STORAGE --compression-mode file --compression-format zstd -o "$OUT/bag" /clock /odom /odom/ground_truth /tf /tf_static /scan /plan /cmd_vel /amcl_pose /joint_states > "$OUT/bag.log" 2>&1 &
BAG=$!
python3 "$EXP/nav/run_goal.py" "$EXP/nav/scenario.json" "$OUT/run" 90 > "$OUT/run_goal.log" 2>&1
RC=$?
# plugin proof from the live system, before teardown
{ for i in 1 2 3 4 5; do ros2 param get /controller_server FollowPath.plugin 2>&1 | grep -v WARN && break; sleep 2; done
  for i in 1 2 3 4 5; do ros2 param get /planner_server GridBased.plugin 2>&1 | grep -v WARN && break; sleep 2; done
  echo "PLANNER=${PLANNER:-navfn} CONTROLLER=${CONTROLLER:-dwb}"; ros2 param get /controller_server current_goal_checker 2>&1 | grep -v WARN; ros2 node list; } > "$OUT/loaded_plugins.txt" 2>&1
echo $RC > "$OUT/run_goal.exit"; exit 0
