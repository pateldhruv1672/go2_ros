# Go2 ROS 2 Workspace

ROS 2 Jazzy workspace for the wireless Unitree Go2 stack:
- base robot bringup and Nav2
- SLAM and localization
- semantic teach/resume navigation
- voice, motion, and agentic overlay packages

The expected workspace root is:

```bash
~/Dhruv/sparky/ros2_ws
```

## Known-good setup

Use ROS 2 Jazzy and the workspace virtualenv. Do not mix Conda Python with ROS Python.

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'
```

If `python3` points to the wrong interpreter, use the venv Python explicitly for builds:

```bash
python -m colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=$VIRTUAL_ENV/bin/python -Wno-dev
```

## Build

Preferred build flow:

```bash
cd ~/Dhruv/sparky/ros2_ws
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate

python -m colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=$VIRTUAL_ENV/bin/python -Wno-dev

source install/setup.bash
```

For rebuilding only a package:

```bash
python -m colcon build --symlink-install --packages-select go2_semantic_nav_agent \
  --cmake-args -DPython3_EXECUTABLE=$VIRTUAL_ENV/bin/python -Wno-dev
source install/setup.bash
```

## Clean restart workaround

Before relaunching the stack, clear stale ROS processes and restart the ROS daemon:

```bash
pkill -9 -f "rviz2|go2_rviz2|robot.launch.py|go2_driver_node|bt_navigator|planner_server|controller_server|behavior_server|lifecycle_manager|nav2|collision_monitor|docking_server|slam_toolbox|foxglove_bridge|pointcloud|lidar|semantic_nav|local_omi_stt_node|dialogue_orchestrator_node|camera_agent_node|speaker_tts_node" || true
ros2 daemon stop || true
ros2 daemon start
```

If `ros2` CLI calls act stale or fail to discover nodes, rerun them with `--disable-daemon`.

## Launch order

The stack is intentionally split into a base robot bringup and overlay packages.

### 1. Base robot bringup

For normal/base mode:

```bash
BASE_MODE=base bash scripts/run_robot_live.sh
```

For teach mode:

```bash
BASE_MODE=teach bash scripts/run_robot_live.sh
```

For resume mode:

```bash
BASE_MODE=resume bash scripts/run_robot_live.sh
```

Wait 10 to 15 seconds after the robot bringup before starting the overlay launch.
Base mode starts only the robot driver and sensor pipeline. Teach mode starts SLAM only. Resume mode starts Nav2 only. Do not run both at once.
For semantic resume overlay, start `BASE_MODE=base` first, then launch `semantic_nav_resume.launch.py` as the overlay. Do not pair the overlay with a second Nav2 bringup.

### 2. Semantic teach mode

The helper script `scripts/run_semantic_nav_teach.sh` now defaults to VLM labeling, launches semantic RViz, and will load `OPENROUTER_API_KEY` from `.env.local` if present.

```bash
export OPENROUTER_API_KEY=YOUR_KEY
ros2 launch go2_semantic_nav_agent semantic_nav_teach.launch.py \
  map_label:=digital_twin_lab \
  auto_save_places:=true \
  auto_save_interval_sec:=5.0 \
  auto_save_use_vlm:=true \
  semantic_rviz:=true \
  clear_places_on_start:=false \
  save_map_on_shutdown:=true
```

If you want to persist the map before shutting the node down, send:

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'save_map'}"
```

### 3. Semantic resume mode

Pick the latest saved session:

```bash
SESSION=$(basename "$(ls -td ~/.ros/go2_semantic_nav_sessions/* | head -1)")
```

Then launch resume:

```bash
ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py \
  session_name:=$SESSION \
  rviz2:=false
```

The resume launcher falls back to the live `/map` topic when a session is missing `map.yaml`.
If the robot is already running SLAM, stop `slam_toolbox` before starting the resume overlay so `amcl` is the only map-to-odom source.

### 4. Full agentic stack

Use the wrapper only after the base robot stack is already healthy:

```bash
export OPENROUTER_API_KEY=YOUR_KEY
scripts/run_sparky_full_system.sh
```

Do not launch a second RViz instance if one is already running.

## Session validation

A resume-ready session must contain:
- `map.yaml`
- `map.pgm`
- `places.yaml`
- `session.yaml`

Check with:

```bash
SESSION=$(basename "$(ls -td ~/.ros/go2_semantic_nav_sessions/* | head -1)")
find ~/.ros/go2_semantic_nav_sessions/$SESSION -maxdepth 1 -type f | sort
```

If `places.yaml` is empty after a restart, the semantic nav node can rehydrate places from shared memory on startup.

## Common workarounds

- Use `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=7`, and `ROS_LOCALHOST_ONLY=0` in every terminal.
- Keep the same `ROS_DOMAIN_ID` in all terminals or nodes will not see each other.
- If Omi is unavailable, the voice stack falls back to the device microphone.
- The agentic voice stack uses on-device TTS only through `speaker_tts_node`; cloud TTS is not part of the validation path.
- Store `OPENROUTER_API_KEY` in the shell or in a local ignored env file if you want VLM labeling during teach/resume.
- Always send Nav2 goals in `frame_id=map`; never send an empty frame id.
- Avoid duplicate RViz windows; if the base launch already has one, set the overlay launch to `rviz2:=false` where supported.
- If you need Nav2 without docking bringup, use the no-docking navigation launch instead of a copied launch that still manages dock plugins.
- If the stack reports stale TF or stale localization, restart the base robot bringup and semantic overlay in that order.
- If live logs clutter the workspace, set `ROS_HOME` and `ROS_LOG_DIR` to a temporary directory before launch.

## Recommended debug flow

```bash
ros2 node list --disable-daemon
ros2 topic list --disable-daemon
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 action info /navigate_to_pose
```

## Project handoff

Project-specific working notes and backlog live in:
- `src/CODEX_CONTINUE_WORK.md`
- `src/AGENTS.md`
- package-specific handoff files under `src/`
