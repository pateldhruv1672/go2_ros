#!/usr/bin/env bash
set -eo pipefail
sim_workspace=${1:?Usage: build_simulator.sh /absolute/sim_workspace}
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'BLOCKED: /opt/ros/jazzy/setup.bash is absent.' >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
cd "$sim_workspace"
colcon build --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
