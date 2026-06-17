# Sparky Go2 ROS 2 Workspace

This workspace runs the Sparky Unitree Go2 ROS 2 Jazzy stack. It combines:

- Unitree Go2 WebRTC robot bringup
- point cloud to `/scan` conversion
- SLAM teach mode
- saved-map semantic resume mode
- Nav2 execution and collision monitoring
- LangGraph-style agentic supervision
- semantic memory, perception, and VLM checkpointing
- Omi-style transcript voice control and tour commands

Expected workspace path:

```bash
/home/digital-twin-admin/Dhruv/sparky/ros2_ws
```

For the detailed system diagrams, read [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Quick Start

Use the project environment in every terminal:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh
source install/setup.bash
```

The expected runtime defaults are:

```bash
ROBOT_IP=192.168.12.1
CONN_TYPE=webrtc
ROS_DOMAIN_ID=7
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///home/digital-twin-admin/Dhruv/sparky/ros2_ws/cyclonedds_go2.xml
```

For OpenRouter, Gemini, or VLM features, also source secrets:

```bash
source ./go2_secrets.sh
```

Do not print or commit secrets.

## Build

Preferred full workspace build:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh

python -m colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=$VIRTUAL_ENV/bin/python -Wno-dev

source install/setup.bash
```

For focused rebuilds:

```bash
python -m colcon build --symlink-install --packages-select go2_omi_voice_bridge \
  --cmake-args -DPython3_EXECUTABLE=$VIRTUAL_ENV/bin/python -Wno-dev
source install/setup.bash
```

The `tests_require` warnings from setuptools are currently harmless.

## Launch Modes

The stack has separate layers. Do not start every mode at once.

| Mode | Main command | Owns |
| --- | --- | --- |
| base | `BASE_MODE=base bash scripts/run_robot_live.sh` | driver, TF, odom, camera, LiDAR scan pipeline, RViz by default |
| teach | `BASE_MODE=teach bash scripts/run_robot_live.sh` | base robot plus SLAM map building |
| resume overlay | `bash scripts/run_semantic_nav_resume.sh` | saved map, AMCL, Nav2, semantic resume node |
| agentic observe/explore | `ros2 launch go2_agentic_system explore_mode.launch.py ...` | memory, perception, LangGraph, Nav2 tool wrapper |
| Omi voice | `ros2 launch go2_omi_voice_bridge omi_voice_stack.launch.py ...` | transcript bridge, intent gate, TTS, tour voice router |

### Base Bringup

Start the robot/sensor layer:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh
source install/setup.bash

BASE_MODE=base bash scripts/run_robot_live.sh
```

Check core topics:

```bash
ros2 topic list | sort | egrep "/camera/image_raw|/scan|/odom|/tf|/cmd_vel_out"
ros2 topic info -v /scan
```

Expected:

- `/scan` exists
- `/odom`, `/tf`, `/tf_static` exist
- `/camera/image_raw` exists when the WebRTC video path is healthy
- `/cmd_vel_out` exists

### Teach Mode

Teach mode is for building a map and saving semantic places.

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh
source ./go2_secrets.sh
source install/setup.bash

ros2 launch go2_semantic_nav_agent semantic_nav_teach.launch.py \
  map_label:=digital_twin_lab \
  auto_save_places:=true \
  auto_save_interval_sec:=5.0 \
  auto_save_use_vlm:=true \
  semantic_rviz:=true \
  clear_places_on_start:=false \
  save_map_on_shutdown:=true
```

Save the map before shutting down:

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'save_map'}"
```

Saved sessions live under:

```text
~/.ros/go2_semantic_nav_sessions/
```

A resume-ready session should include:

```text
map.yaml
map.pgm
places.yaml
session.yaml
```

### Resume Mode

Resume mode is for navigating on a saved map. It should use AMCL/localization, not live SLAM, as the `map -> odom` owner.

Recommended helper:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh
source install/setup.bash

bash scripts/run_semantic_nav_resume.sh
```

Manual launch:

```bash
SESSION=$(basename "$(ls -td ~/.ros/go2_semantic_nav_sessions/* | head -1)")

ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py \
  session_name:=$SESSION \
  rviz2:=true \
  restore_spawn_on_start:=true
```

Current resume launch behavior:

- starts `resume_map_server`
- starts `amcl`
- starts the Nav2 no-docking stack
- starts `semantic_nav_node` in `mode=resume`
- currently retimestamps raw `/scan` into `/scan_nav` before feeding AMCL, costmaps, collision monitor, and semantic nav
- delays Nav2 startup so saved-map localization can come up first

If you are debugging global costmap stability, keep global planning map-based and let the local costmap/collision monitor handle live obstacles. The runtime flag is:

```bash
export GO2_GLOBAL_LIVE_OBSTACLES=0
```

### Agentic Observe/Explore

The agentic stack should start after the base robot stack is healthy. Keep motion disabled first.

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh
source ./go2_secrets.sh
source install/setup.bash

ros2 launch go2_agentic_system explore_mode.launch.py \
  enable_motion:=false \
  enable_open_vocab_detector:=true \
  open_vocab_backend:=openrouter \
  open_vocab_model:=google/gemini-2.5-flash \
  enable_vlm_checkpointing:=true \
  vlm_provider:=openrouter \
  vlm_model:=google/gemini-2.5-flash \
  camera_topic:=/camera/image_raw \
  enable_dynamic_obstacle_tracking:=false \
  voice_input_mode:=text_topic
```

Send an observe-only command:

```bash
ros2 topic pub --once /go2_agent/user_command std_msgs/msg/String \
"{data: 'Observe the current scene, summarize visible landmarks, check map/odom/scan context, and recommend the safest next exploration direction. Do not move.'}"
```

Watch:

```bash
ros2 topic echo /go2_agent/stream
ros2 topic echo /go2_agent/status
ros2 topic echo /go2_agent/speech
```

### Omi Voice Stack

The current Omi integration is transcript-first. It supports simulated or mobile-provided transcripts through `/omi/transcript_raw`.

Launch:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh
source install/setup.bash

ros2 launch go2_omi_voice_bridge omi_voice_stack.launch.py \
  adapter_mode:=transcript_only \
  tts_enabled:=true \
  require_confirmation_for_motion:=true
```

Simulate voice:

```bash
ros2 topic pub --once /omi/transcript_raw std_msgs/msg/String \
"{data: 'Sparky, where are we?'}"
```

Start a tour command:

```bash
ros2 topic pub --once /omi/transcript_raw std_msgs/msg/String \
"{data: 'Sparky, start the Applied Data Science tour'}"
```

Confirm:

```bash
ros2 topic pub --once /omi/transcript_raw std_msgs/msg/String \
"{data: 'yes, proceed'}"
```

Useful voice topics:

```bash
ros2 topic echo /go2_voice/transcript
ros2 topic echo /go2_voice/verification_request
ros2 topic echo /go2_voice/verification_state
ros2 topic echo /go2_agent/user_command
ros2 topic echo /go2_tts/status
ros2 topic echo /go2_tour/status
```

Motion/tour commands require confirmation. Stop commands do not.

Immediate stop examples:

```text
stop
halt
freeze
emergency stop
cancel navigation
```

The voice gate publishes zero velocity to `/cmd_vel_out` and sends a `stop_robot` command to `/go2_nav/command`.

## Tour Route File

The voice tour router looks for:

```text
~/.ros/go2_semantic_nav_sessions/default/tours/sjsu_ads_department_tour.json
```

Minimum structure:

```json
{
  "tour_id": "sjsu_ads_department_tour",
  "title": "Applied Data Science Department Tour",
  "requires_resume_mode": true,
  "checkpoints": [
    {
      "checkpoint_id": "digital_twin_lab",
      "place_id": "digital_twin_lab",
      "name": "Digital Twin Lab",
      "narration": "This area supports robotics, simulation, and digital twin experimentation.",
      "fun_fact": "Digital twins let researchers test systems virtually before deploying them in the real world."
    }
  ]
}
```

If the file is missing, the tour router refuses safely and publishes an error on `/go2_tour/status`.

## Safety Rules

- Do not bypass Nav2 for autonomous movement.
- Voice motion commands must pass the confirmation gate.
- Use short goals in teach/SLAM mode.
- Use saved map/resume mode for long navigation and tours.
- Never run SLAM and AMCL as competing `map -> odom` owners.
- If localization is uncertain, do not start autonomous navigation.
- If collision monitor is STOP or blocked, refuse new navigation.
- Keep `enable_motion:=false` until base Nav2/resume is stable.

## Debug Checklist

Check the robot graph:

```bash
ros2 node list --disable-daemon
ros2 topic list --disable-daemon
ros2 action info /navigate_to_pose
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
```

Check sensing:

```bash
ros2 topic info -v /scan
ros2 topic hz /scan
ros2 topic echo --once /odom
```

Check Nav2:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
```

Check voice:

```bash
ros2 topic echo /go2_voice/verification_state
ros2 topic echo /go2_agent/user_command
ros2 topic echo /go2_nav/command
```

## Clean Restart

Before a clean relaunch:

```bash
pkill -f "go2_driver_node|robot_state_publisher|slam_toolbox|amcl|map_server|rviz2|bt_navigator|planner_server|controller_server|lifecycle_manager|foxglove_bridge|pointcloud_aggregator|pointcloud_to_laserscan_node|go2_pointcloud_to_laserscan|lidar_to_pointcloud|semantic_nav_node|go2_omi_bridge|go2_voice_stt_node|go2_voice_intent_gate|go2_tts_node|go2_tour_voice_command_router" || true
ros2 daemon stop || true
ros2 daemon start || true
```

## Important Files

| File | Purpose |
| --- | --- |
| `scripts/run_robot_live.sh` | base/teach/resume robot bringup wrapper |
| `scripts/run_semantic_nav_resume.sh` | semantic resume overlay wrapper |
| `src/go2_robot_sdk/launch/robot.launch.py` | base robot launch |
| `src/go2_robot_sdk/launch/navigation_no_docking.launch.py` | Nav2 no-docking launch used by semantic resume |
| `src/go2_robot_sdk/config/nav2_params.yaml` | Nav2, costmap, collision monitor parameters |
| `src/go2_semantic_nav_agent/launch/semantic_nav_teach.launch.py` | semantic teach launch |
| `src/go2_semantic_nav_agent/launch/semantic_nav_resume.launch.py` | semantic resume launch |
| `src/go2_agentic_system/launch/explore_mode.launch.py` | agentic observe/explore launch |
| `src/go2_omi_voice_bridge/launch/omi_voice_stack.launch.py` | Omi-style voice stack launch |
| `docs/ARCHITECTURE.md` | architecture diagrams and topic flows |

