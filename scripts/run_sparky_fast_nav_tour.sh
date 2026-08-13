#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

# Fast but bounded Go2 profile.
# The base driver allows up to 0.75 m/s linear and 0.90 rad/s angular.
export GO2_NAV_MIN_SPEED_XY="${GO2_NAV_MIN_SPEED_XY:-0.35}"
export GO2_NAV_MIN_SPEED_THETA="${GO2_NAV_MIN_SPEED_THETA:-0.28}"
export GO2_NAV_MAX_X="${GO2_NAV_MAX_X:-0.55}"
export GO2_NAV_MAX_THETA="${GO2_NAV_MAX_THETA:-0.75}"

export GO2_NAV_ACC_X="${GO2_NAV_ACC_X:-0.90}"
export GO2_NAV_DECEL_X="${GO2_NAV_DECEL_X:-1.00}"
export GO2_NAV_ACC_THETA="${GO2_NAV_ACC_THETA:-1.60}"
export GO2_NAV_DECEL_THETA="${GO2_NAV_DECEL_THETA:-1.80}"

export GO2_NAV_CONTROLLER_HZ="${GO2_NAV_CONTROLLER_HZ:-15.0}"
export GO2_NAV_SIM_TIME="${GO2_NAV_SIM_TIME:-1.0}"
export GO2_NAV_VX_SAMPLES="${GO2_NAV_VX_SAMPLES:-12}"
export GO2_NAV_VTHETA_SAMPLES="${GO2_NAV_VTHETA_SAMPLES:-20}"

export GO2_NAV_XY_TOLERANCE="${GO2_NAV_XY_TOLERANCE:-0.20}"
export GO2_NAV_YAW_TOLERANCE="${GO2_NAV_YAW_TOLERANCE:-0.30}"
export GO2_NAV_PROGRESS_RADIUS_M="${GO2_NAV_PROGRESS_RADIUS_M:-0.08}"
export GO2_NAV_PROGRESS_ANGLE_RAD="${GO2_NAV_PROGRESS_ANGLE_RAD:-0.12}"
export GO2_NAV_PROGRESS_TIMEOUT_SEC="${GO2_NAV_PROGRESS_TIMEOUT_SEC:-8.0}"

# Green paths are already appearing. Skip Smac smoothing to reduce planning delay.
export GO2_SMAC_SMOOTH_PATH="${GO2_SMAC_SMOOTH_PATH:-0}"

# Preserve collision monitoring; do not bypass the safety layer.
export GO2_COLLISION_SOURCE_TIMEOUT_SEC="${GO2_COLLISION_SOURCE_TIMEOUT_SEC:-1.6}"

echo "Sparky FAST Nav profile"
echo "  vx moving range:   ${GO2_NAV_MIN_SPEED_XY} .. ${GO2_NAV_MAX_X} m/s"
echo "  wz range:          ${GO2_NAV_MIN_SPEED_THETA} .. ${GO2_NAV_MAX_THETA} rad/s"
echo "  accel/decel x:     ${GO2_NAV_ACC_X} / ${GO2_NAV_DECEL_X} m/s^2"
echo "  controller:        ${GO2_NAV_CONTROLLER_HZ} Hz"
echo "  DWB horizon:       ${GO2_NAV_SIM_TIME} s"
echo "  planner smoothing: ${GO2_SMAC_SMOOTH_PATH}"
echo

set +u
source /opt/ros/jazzy/setup.bash
[[ -f "$ROOT_DIR/src/.venv/bin/activate" ]] && source "$ROOT_DIR/src/.venv/bin/activate"
source "$ROOT_DIR/install/setup.bash"
set -u

# These parameters are generated at Resume startup. Force a clean Resume-owned
# Nav2 restart if a controller is already active so the fast profile is not
# silently ignored by an old process.
if ros2 node list 2>/dev/null | grep -qx '/controller_server'; then
  echo "[fast-nav] Existing Resume/Nav2 detected; stopping only Resume-owned navigation so the fast profile takes effect."
  pkill -f "ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py" 2>/dev/null || true
  pkill -f "semantic_nav_node|scan_retimestamp_node|resume_map_server|resume_map_lifecycle_manager|controller_server|planner_server|bt_navigator|waypoint_follower|collision_monitor|lifecycle_manager_navigation|behavior_server|opennav_docking|go2_motion_arbiter" 2>/dev/null || true
  sleep 2
fi

exec "$SCRIPT_DIR/run_sparky_resume_agentic_tour.sh"
