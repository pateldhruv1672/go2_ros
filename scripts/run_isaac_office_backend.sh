#!/usr/bin/env bash
set -eo pipefail

ISAACLAB_ROOT="/home/digital-twin-admin/Dhruv/IsaacLab"
BACKEND="$ISAACLAB_ROOT/scripts/go2_ros_backend.py"
WS="/home/digital-twin-admin/Dhruv/sparky/ros2_ws"

cd "$ISAACLAB_ROOT"

unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset ROS_VERSION
unset ROS_PYTHON_VERSION
unset VIRTUAL_ENV
unset PYTHONPATH

export ZSH_VERSION="${ZSH_VERSION:-}"

if [ -f "/home/digital-twin-admin/miniforge3/etc/profile.d/conda.sh" ]; then
  set +u
  source "/home/digital-twin-admin/miniforge3/etc/profile.d/conda.sh"
  conda activate env_isaaclab_go2
  set -u
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

ISAAC_ASSET_ROOT="${ISAAC_ASSET_ROOT:-https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1}"
WORLD_USD="${WORLD_USD:-$ISAAC_ASSET_ROOT/Isaac/Environments/Office/office.usd}"
WORLD_PRIM_PATH="${WORLD_PRIM_PATH:-/World/Office}"

TASK="${TASK:-Isaac-Velocity-Rough-Unitree-Go2-v0}"
CHECKPOINT="${CHECKPOINT:-$WS/models/go2/model_7850.pt}"

SPAWN_X="${SPAWN_X:-0.0}"
SPAWN_Y="${SPAWN_Y:-0.0}"
SPAWN_Z="${SPAWN_Z:-0.55}"
SPAWN_YAW="${SPAWN_YAW:-0.0}"

EXTRA_ARGS=()

if [ "${MAKE_WORLD_COLLIDERS:-1}" = "1" ]; then
  EXTRA_ARGS+=("--make-world-colliders")
fi

if [ "${DISABLE_GROUND_COLLISION:-1}" = "1" ]; then
  EXTRA_ARGS+=("--disable-ground-collision")
fi

if [ "${HIDE_ROUGH_TERRAIN:-1}" = "1" ]; then
  EXTRA_ARGS+=("--hide-rough-terrain")
fi

if [ "${FLAT_GENERATED_TERRAIN:-1}" = "1" ]; then
  EXTRA_ARGS+=("--flat-generated-terrain")
fi

exec ./isaaclab.sh -p "$BACKEND" \
  --task "$TASK" \
  --num-envs 1 \
  --env-device cuda:0 \
  --checkpoint "$CHECKPOINT" \
  --world-usd "$WORLD_USD" \
  --world-prim-path "$WORLD_PRIM_PATH" \
  --spawn-x "$SPAWN_X" \
  --spawn-y "$SPAWN_Y" \
  --spawn-z "$SPAWN_Z" \
  --spawn-yaw "$SPAWN_YAW" \
  "${EXTRA_ARGS[@]}"
