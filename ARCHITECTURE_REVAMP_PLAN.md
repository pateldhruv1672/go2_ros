# Go2 Agentic Navigation Revamp Patch

This patch is an additive implementation scaffold for the Unitree Go2 agentic navigation roadmap. It keeps the existing base, teach, and resume flows intact while adding feature-flagged packages for memory, semantic voxels, perception tools, Nav2 tool wrappers, LangGraph-style supervision, debate, explore, and tour modes.

## Non-breaking contract

The existing launch commands remain the source of truth:

```bash
BASE_MODE=base bash scripts/run_robot_live.sh
BASE_MODE=teach bash scripts/run_robot_live.sh
BASE_MODE=resume bash scripts/run_robot_live.sh
```

The new packages are opt-in. They do not start motion by default, and motion tools default to dry-run unless explicitly enabled.

## Added packages

- `go2_memory_core_interfaces`: service interfaces for unified memory read/write.
- `go2_memory_core`: JSON/Kuzu-compatible memory API, memory server, spawn/checkpoint/place writes, graph edges, legacy exports, resume context.
- `go2_semantic_voxel_memory`: persistent semantic voxel store and RViz marker publisher.
- `go2_perception_tools`: LiDAR and point-cloud summarizers that emit JSON context.
- `go2_nav_tools`: safe Nav2 wrappers, graph-route planner, frontier helper, safe anchors, recovery manager.
- `go2_langgraph_agent`: LangGraph-style supervisor, persistent state, debate council, memory/nav/perception/tour tools.
- `go2_agentic_system/launch`: additive launch files for memory stack, explore mode, and tour mode.

## Feature flags

Every new launch file exposes flags matching the roadmap:

```text
enable_memory_core
enable_graph_memory
enable_voxel_memory
enable_perception_tools
enable_langgraph_agent
enable_debate_layer
enable_explore_mode
enable_tour_mode
enable_unified_voice
```

## Implementation status

This patch completes the architecture-level package wiring and provides working MVP implementations for local file-backed memory, graph edges, semantic voxel observations, perception summaries, dry-run navigation tools, debate records, and tour/explore command orchestration. It does not claim to replace the remaining real-robot validation work: TF stability, Nav2 tuning, AMCL confidence thresholds, VLM integration, real point-cloud clustering, Omi audio integration, and paper-grade experiments still need to be run on the Go2 and Isaac backends.

## Apply and build

```bash
cd ~/Dhruv/sparky/ros2_ws
git apply /path/to/go2_agentic_navigation_revamp.patch
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
python -m colcon build --symlink-install --packages-select \
  go2_memory_core_interfaces \
  go2_memory_core \
  go2_semantic_voxel_memory \
  go2_perception_tools \
  go2_nav_tools \
  go2_langgraph_agent \
  go2_agentic_system \
  --cmake-args -DPython3_EXECUTABLE=$VIRTUAL_ENV/bin/python -Wno-dev
source install/setup.bash
```

## New launch examples

Memory stack only:

```bash
ros2 launch go2_agentic_system agentic_memory_stack.launch.py \
  enable_memory_core:=true \
  enable_graph_memory:=true \
  enable_voxel_memory:=true \
  enable_perception_tools:=true \
  enable_langgraph_agent:=false
```

Explore dry run:

```bash
ros2 launch go2_agentic_system explore_mode.launch.py \
  enable_explore_mode:=true \
  enable_motion:=false
```

Tour dry run:

```bash
ros2 launch go2_agentic_system tour_mode.launch.py \
  session_name:=latest \
  enable_tour_mode:=true \
  enable_motion:=false
```

## Acceptance checkpoints

- Existing base/teach/resume commands are untouched.
- New memory writes go through `go2_memory_core`.
- Session folder includes `spawn.yaml`, `memory/`, `graph_memory/`, `voxel_memory/`, `vector_memory/`, `artifacts/`, `langgraph/`, and `exports/`.
- Checkpoints store map pose, odom pose, velocity, TF snapshot, confidence, VLM/LiDAR/point-cloud summaries, layer, and artifact refs.
- Graph memory stores `CONNECTED_TO`, `OBSERVED_FROM`, `NEAR`, `PART_OF`, and `HAS_TOUR_STOP`-style edges as JSONL/Kuzu-compatible records.
- Resume context can be queried without starting SLAM.
- Nav2 wrappers default to dry-run for safety.
- Debate records are persisted before risky actions when enabled.
