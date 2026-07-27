# Go2 Omi Navigation Stack README

This README explains the correct way to launch the Unitree Go2 Omi navigation demo.

This stack is for:

- Real Unitree Go2 base connection
- Saved-map semantic resume navigation
- Nav2 navigation
- RViz visualization
- Omi BLE speech input
- Omi voice commands
- Sport/action skills such as wave, sit, stand, stretch
- Voice navigation commands such as "Sparky, go to spawn"

---

## 1. Correct launch topology

Use four terminals:

```text
Terminal 1: base robot only
Terminal 2: semantic resume navigation stack
Terminal 3: RViz only
Terminal 4: Omi voice + sport/action stack
```

The most important rule:

```text
Only one stack should own Nav2.
Only one stack should own RViz.
Only one stack should own Omi.
```

For Omi navigation with saved maps, the base stack must run with:

```text
slam:=false
nav2:=false
rviz2:=false
```

The semantic resume stack owns:

```text
saved map
AMCL/localization
Nav2
collision monitor
semantic navigation commands
```

---

## 2. Do not run these together

Do not run base Nav2 and semantic Nav2 at the same time:

```bash
ros2 launch go2_robot_sdk robot.launch.py slam:=true nav2:=true
ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py
```

Do not run old Omi stack and sport Omi stack together:

```bash
ros2 launch go2_omi_voice_bridge omi_voice_stack.launch.py
ros2 launch go2_omi_voice_bridge omi_voice_sport_stack.launch.py
```

Do not launch RViz from multiple places:

```bash
robot.launch.py rviz2:=true
semantic_nav_resume.launch.py rviz:=true
ros2 run rviz2 rviz2
```

Correct pattern:

```bash
robot.launch.py rviz2:=false
semantic_nav_resume.launch.py rviz:=false rviz2:=false
ros2 run rviz2 rviz2
```

---

## 3. Terminal 0: clean old processes

Run this once before starting the demo.

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source ./go2_env.sh
source install/setup.bash

pkill -f "robot.launch.py|semantic_nav_resume.launch.py|semantic_nav_node|scan_retimestamp_node|resume_map_server|resume_map_lifecycle_manager|controller_server|planner_server|bt_navigator|waypoint_follower|collision_monitor|lifecycle_manager_navigation|behavior_server|opennav_docking|semantic_nav_rviz2|rviz2|omi_voice_sport_stack.launch.py|omi_voice_stack.launch.py|go2_omi_bridge|go2_voice_stt_node|go2_voice_intent_gate|omi_skill_intent_router|webrtc_motion_skill_agent_node|motion_skill_agent_node|go2_tts_node|frontier_object_explore.launch.py|frontier_object_explorer_node|padded_map_node" || true

ros2 daemon stop || true
ros2 daemon start
```

---

## 4. Terminal 1: base robot only

This terminal owns the real robot connection, WebRTC, scan, camera, odom, TF, and low-level driver.

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source ./go2_env.sh
source install/setup.bash

export ROBOT_IP=192.168.12.1
export CONN_TYPE=webrtc

ros2 launch go2_robot_sdk robot.launch.py \
  foxglove:=false \
  slam:=false \
  nav2:=false \
  rviz2:=false
```

Leave this terminal running.

Check base topics:

```bash
ros2 topic list | sort | egrep "^/camera/image_raw$|^/scan$|^/odom$|^/tf$|^/tf_static$|^/point_cloud2$|^/cmd_vel_out$|^/webrtc_req$"
```

Expected:

```text
/camera/image_raw
/cmd_vel_out
/odom
/point_cloud2
/scan
/tf
/tf_static
/webrtc_req
```

---

## 5. Terminal 2: semantic resume navigation

This terminal owns map loading, AMCL, Nav2, collision monitor, and semantic navigation.

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source ./go2_env.sh
source install/setup.bash

export ROBOT_IP=192.168.12.1
export CONN_TYPE=webrtc

export GO2_RESUME_INTERNAL_RVIZ=0
export GO2_GLOBAL_LIVE_OBSTACLES=0
export GO2_SMAC_SMOOTH_PATH=0
export GO2_SAFETY_SCAN_TOPIC=/scan

ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py \
  session_name:=latest_usable \
  rviz:=false \
  rviz2:=false \
  restore_spawn_on_start:=true \
  nav2_start_delay_sec:=8.0 \
  scan_input_topic:=/scan \
  scan_nav_topic:=/scan_nav \
  pointcloud_topic:=/point_cloud2 \
  stvl_enabled:=auto \
  scan_frame_id:=base_link \
  scan_stamp_offset_sec:=0.25
```

Leave this terminal running.

Check navigation:

```bash
ros2 action list | grep navigate_to_pose
ros2 topic list | sort | egrep "/map$|/amcl_pose|/goal_pose|/plan|/cmd_vel_nav2|/cmd_vel_out|/semantic_nav"
```

Expected:

```text
/navigate_to_pose
/map
/amcl_pose
/plan
/cmd_vel_nav2
/cmd_vel_out
/semantic_nav/command
```

---

## 6. Terminal 3: RViz only

Launch RViz separately.

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source ./go2_env.sh
source install/setup.bash

RVIZ_CFG="$(ros2 pkg prefix go2_semantic_nav_agent)/share/go2_semantic_nav_agent/config/semantic_nav.rviz"

ros2 run rviz2 rviz2 -d "$RVIZ_CFG"
```

In RViz:

```text
Fixed Frame: map
```

Useful displays:

```text
TF
Map: /map
LaserScan: /scan
PointCloud2: /point_cloud2
Path: /plan
MarkerArray: /semantic_nav/markers
MarkerArray: /visualization_marker_array
Image: /camera/image_raw
```

For `/scan` and `/camera/image_raw`, set:

```text
Reliability Policy: Best Effort
```

---

## 7. Terminal 4: Omi voice + sport/action stack

This terminal owns Omi BLE, speech-to-text, text-to-speech, skill routing, sport actions, and voice navigation commands.

Use:

```text
omi_voice_sport_stack.launch.py
```

Do not use the older `omi_voice_stack.launch.py` at the same time.

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source ./go2_env.sh
source install/setup.bash

export ROBOT_IP=192.168.12.1
export CONN_TYPE=webrtc

pkill -f "omi_voice_sport_stack.launch.py|omi_voice_stack.launch.py|omi_skill_intent_router|go2_voice_stt_node|go2_omi_bridge|go2_tts_node|motion_skill_agent_node|webrtc_motion_skill_agent_node|go2_voice_intent_gate" || true

ros2 launch go2_omi_voice_bridge omi_voice_sport_stack.launch.py \
  adapter_mode:=ble_audio \
  ble_device_name:="Omi DevK" \
  ble_device_address:=EF:1C:34:C6:25:92 \
  omi_connect_preflight:=true \
  omi_connect_required:=true \
  tts_enabled:=true \
  local_speaker_enabled:=false \
  local_speaker_backend:=auto \
  require_wake_word:=true \
  require_confirmation_for_skills:=true \
  allow_medium_risk:=true \
  allow_high_risk:=false \
  allow_parameter_commands:=false
```

Use `local_speaker_enabled:=false` during testing so the laptop speaker does not feed back into the Omi microphone.

---

## 8. Voice commands to test

Sport/action commands:

```text
Sparky, wave.
Yes.

Sparky, sit down.
Yes.

Sparky, stand up.
Yes.

Sparky, stretch.
Yes.

Sparky, dance.
Yes.
```

Navigation commands:

```text
Sparky, go to spawn.

Sparky, go to lobby.

Sparky, go to the classroom.

Sparky, stop.
```

Navigation commands should publish to:

```text
/semantic_nav/command
```

Sport/action commands should publish to:

```text
/motion_skills/command
/webrtc_req
```

---

## 9. Debug Omi and voice

Omi/STT:

```bash
ros2 topic echo /go2_voice/transcript
ros2 topic echo /go2_voice/processed_stt
```

Motion skills:

```bash
ros2 topic echo /motion_skills/command
ros2 topic echo /motion_skills/status
ros2 topic echo /webrtc_req
```

Navigation commands:

```bash
ros2 topic echo /semantic_nav/command
ros2 topic echo /goal_pose
ros2 topic echo /plan
```

Velocity chain:

```bash
ros2 topic echo /cmd_vel_nav2
ros2 topic echo /cmd_vel_out
ros2 topic echo /cmd_vel_sdk
```

Expected velocity chain:

```text
Nav2 controller_server
  -> /cmd_vel_nav2
  -> collision_monitor
  -> /cmd_vel_out
  -> go2_driver_node
  -> /cmd_vel_sdk
  -> WebRTC robot motion
```

---

## 10. Check for duplicate nodes

Run:

```bash
ros2 node list | sort | egrep "slam|map|amcl|nav|controller|planner|bt|collision|rviz|omi|voice|tts|skill|semantic"
```

For this mode, you should not see duplicate:

```text
controller_server
planner_server
bt_navigator
collision_monitor
rviz2
go2_voice_stt_node
go2_omi_bridge
go2_tts_node
omi_skill_intent_router
```

If duplicates appear, stop everything and relaunch from Terminal 0.

---

## 11. Direct movement test

Only run this in a safe open area.

Make the robot stand:

```bash
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{api_id: 1004, topic: 'rt/api/sport/request', parameter: '', priority: 1}"
```

Test through the Nav2 command path:

```bash
timeout 3s ros2 topic pub --rate 10 /cmd_vel_nav2 geometry_msgs/msg/Twist \
  "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub --once /cmd_vel_nav2 geometry_msgs/msg/Twist "{}"
```

Watch output:

```bash
ros2 topic echo /cmd_vel_out
```

And:

```bash
ros2 topic echo /cmd_vel_sdk
```

Interpretation:

```text
/cmd_vel_nav2 publishes, /cmd_vel_out silent
=> collision monitor wiring issue.

/cmd_vel_out publishes, /cmd_vel_sdk silent
=> driver subscription or driver path issue.

/cmd_vel_sdk publishes, robot still does not move
=> robot mode, WebRTC, or physical robot state issue.
```

---

## 12. Full demo order

1. Terminal 0: clean old processes.
2. Terminal 1: launch base robot only.
3. Confirm `/scan`, `/camera/image_raw`, `/odom`, `/tf`, `/webrtc_req`.
4. Terminal 2: launch semantic resume navigation.
5. Confirm `/map`, `/amcl_pose`, `/navigate_to_pose`, `/semantic_nav/command`.
6. Terminal 3: launch RViz.
7. Terminal 4: launch Omi sport stack.
8. Test: `Sparky, wave. Yes.`
9. Test: `Sparky, go to spawn.`
10. Test: `Sparky, stop.`

