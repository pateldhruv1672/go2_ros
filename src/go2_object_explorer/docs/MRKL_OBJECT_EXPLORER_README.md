# Go2 MRKL Object Explorer README

This README documents the correct way to launch the agentic object-exploration stack for the Unitree Go2.

The goal of this mode is:

```text
User mission:
  "explore and find chair"

MRKL agent:
  observes map, frontiers, YOLO detections, SAM2 masks, robot pose, Nav2 status
  asks Ollama to choose a valid tool call
  validates the tool call
  sends only safe/validated goals to Nav2

Robot:
  explores frontiers
  scans locally
  approaches detected objects
```

---

# 1. Architecture

The system follows MRKL principles:

```text
LLM / Ollama:
  router and reasoner only

Tools:
  scan_here
  navigate_to_frontier
  navigate_to_object
  finish

Validators:
  check candidate IDs
  check map coordinates
  check valid/free map cells
  reject hallucinated coordinates
  prevent repeated near-identical goals

Executors:
  Nav2 NavigateToPose
  cmd_vel scan behavior
  YOLO/SAM2 perception
  frontier extraction

Trace:
  every observation, decision, tool call, and result is published to:
  /mrkl_explorer/trace
```

Important rule:

```text
Do not send user missions directly to /object_explorer/goal.
Send missions to /mrkl_explorer/goal.
```

The old object explorer runs in passive mode and publishes observations. The MRKL agent owns navigation decisions.

---

# 2. Runtime topology

Use this launch order:

```text
Terminal 0: clean all old processes
Terminal 1: base robot only
Terminal 2: live SLAM + Nav2 + scan_nav + motion arbiter
Terminal 3: passive object explorer
Terminal 4: YOLO + SAM2 overlay
Terminal 5: web dashboard
Terminal 6: RViz
Terminal 7: MRKL agent + send mission
```

Motion chain:

```text
Nav2 controller_server
  -> /cmd_vel_nav2
  -> go2_motion_arbiter
  -> /cmd_vel_nav
  -> collision_monitor
  -> /cmd_vel_out
  -> go2_driver_node
  -> Unitree Go2
```

MRKL local scan chain:

```text
MRKL scan_here tool
  -> /cmd_vel_omi
  -> go2_motion_arbiter
  -> /cmd_vel_nav
  -> collision_monitor
  -> /cmd_vel_out
  -> go2_driver_node
```

Sensor timing chain:

```text
/scan
  -> object_explorer_scan_retimestamp_node
  -> /scan_nav
  -> local_costmap + collision_monitor + object explorer
```

---

# 3. Do not run these together

Do not run base Nav2 and explorer Nav2 together:

```bash
ros2 launch go2_robot_sdk robot.launch.py slam:=true nav2:=true
ros2 launch go2_object_explorer explorer_live_nav.launch.py
```

Correct pattern:

```bash
ros2 launch go2_robot_sdk robot.launch.py slam:=false nav2:=false rviz2:=false
ros2 launch go2_object_explorer explorer_live_nav.launch.py
```

Do not run semantic resume and explorer live nav together:

```bash
ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py
ros2 launch go2_object_explorer explorer_live_nav.launch.py
```

Do not send missions directly to the passive explorer:

```bash
ros2 topic pub --once /object_explorer/goal std_msgs/msg/String "{data: 'explore and find chair'}"
```

Correct:

```bash
ros2 topic pub --once /mrkl_explorer/goal std_msgs/msg/String "{data: 'explore and find chair'}"
```

---

# 4. Terminal 0: clean everything

Run once before starting.

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source ./go2_env.sh
source install/setup.bash

pkill -f "robot.launch.py|navigation_no_docking.launch.py|explorer_live_nav.launch.py|frontier_object_explore.launch.py|frontier_object_explorer_node|mrkl_explorer_agent_node|mrkl_explorer_agent.launch.py|fast_sam2_tracker_overlay_node|fast_sam2_tracker_overlay.launch.py|topic_web_dashboard_node|topic_web_dashboard.launch.py|padded_map_node|object_explorer_scan_retimestamp_node|go2_motion_arbiter|slam_toolbox|planner_server|controller_server|bt_navigator|behavior_server|collision_monitor|rviz2" || true

ros2 daemon stop || true
ros2 daemon start
```

Optional: clear stale object memory.

```bash
rm -f ~/.ros/go2_object_explorer/object_memory.yaml
```

---

# 5. Terminal 1: base robot only

This terminal owns the real robot connection, camera, scan, odom, TF, WebRTC, and driver.

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

# 6. Terminal 2: live SLAM + Nav2 + scan_nav + arbiter

This terminal owns:

```text
SLAM Toolbox
/map
/map_padded
/scan_nav
go2_motion_arbiter
Nav2
collision_monitor
```

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source ./go2_env.sh
source install/setup.bash

ros2 launch go2_object_explorer explorer_live_nav.launch.py \
  padding_m:=3.0 \
  nav2_start_delay_sec:=30.0
```

Wait until you see:

```text
published padded map
Managed nodes are active
```

Verify:

```bash
ros2 topic hz /scan_nav
ros2 action list | grep navigate_to_pose
ros2 topic info /cmd_vel_nav -v
ros2 topic info /cmd_vel_out -v
ros2 param get /collision_monitor cmd_vel_in_topic
ros2 param get /collision_monitor cmd_vel_out_topic
```

Expected:

```text
/scan_nav is publishing
/navigate_to_pose exists
/cmd_vel_nav has subscriber collision_monitor
/cmd_vel_out has subscriber go2_driver_node
cmd_vel_in_topic: cmd_vel_nav
cmd_vel_out_topic: cmd_vel_out
```

---

# 7. Terminal 3: passive object explorer

This node should publish observations and frontiers. It should not own final navigation decisions.

Important:

```text
auto_navigate:=false
use_ollama_frontier_selector:=false
```

MRKL owns decisions.

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source ./go2_env.sh
source install/setup.bash

rm -f ~/.ros/go2_object_explorer/object_memory.yaml

ros2 launch go2_object_explorer frontier_object_explore.launch.py \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  scan_topic:=/scan_nav \
  map_topic:=/map \
  cmd_vel_topic:=/cmd_vel_omi \
  yolo_model:=yolov8n.pt \
  yolo_conf:=0.28 \
  enable_sam2:=true \
  auto_navigate:=false \
  frontier_camera_map_mode:=fallback_and \
  use_ollama_frontier_selector:=false \
  local_scan_duration_sec:=6.0 \
  search_timeout_sec:=300.0 \
  frontier_goal_timeout_sec:=120.0
```

Useful topics:

```bash
ros2 topic echo /object_explorer/state
ros2 topic echo /object_explorer/detections
ros2 topic echo /object_explorer/markers
```

---

# 8. Terminal 4: YOLO + SAM2 overlay

This node publishes:

```text
/object_explorer/annotated_image
/object_explorer/sam2_detections
```

It uses YOLO boxes to prompt SAM2, then cleans mask edges using shrink/open/close/erode parameters.

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source ./go2_env.sh
source install/setup.bash

ros2 launch go2_object_explorer fast_sam2_tracker_overlay.launch.py \
  image_topic:=/camera/image_raw \
  annotated_image_topic:=/object_explorer/annotated_image \
  detections_topic:=/object_explorer/sam2_detections \
  device:=cuda:0 \
  yolo_model:=yolov8n.pt \
  yolo_imgsz:=640 \
  yolo_conf:=0.20 \
  max_detections:=12 \
  target_classes:="person,chair,dining table,tv,bottle,laptop,backpack,cup,book" \
  class_aliases:="refrigerator=cabinet" \
  blocked_classes:="" \
  enable_sam2:=true \
  sam2_model:=sam2_t.pt \
  sam2_imgsz:=384 \
  sam2_every_n:=2 \
  sam2_box_shrink_ratio:=0.035 \
  mask_open_px:=3 \
  mask_close_px:=5 \
  mask_erode_px:=1 \
  min_mask_area_px:=180 \
  inference_period_sec:=0.12 \
  track_ttl_sec:=2.0
```

Check image and detection output:

```bash
ros2 topic hz /object_explorer/annotated_image
ros2 topic echo /object_explorer/sam2_detections --once
```

Check GPU:

```bash
watch -n 0.5 nvidia-smi
```

For a faster but less detailed mode:

```bash
ros2 launch go2_object_explorer fast_sam2_tracker_overlay.launch.py \
  image_topic:=/camera/image_raw \
  annotated_image_topic:=/object_explorer/annotated_image \
  detections_topic:=/object_explorer/sam2_detections \
  device:=cuda:0 \
  yolo_model:=yolov8n.pt \
  yolo_imgsz:=640 \
  yolo_conf:=0.20 \
  max_detections:=8 \
  target_classes:="person,chair,dining table,tv,bottle,laptop,backpack,cup,book" \
  enable_sam2:=true \
  sam2_model:=sam2_t.pt \
  sam2_imgsz:=320 \
  sam2_every_n:=3 \
  inference_period_sec:=0.10 \
  track_ttl_sec:=2.0
```

---

# 9. Terminal 5: web dashboard

This replaces opening many `ros2 topic echo` terminals.

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source ./go2_env.sh
source install/setup.bash

ros2 launch go2_object_explorer topic_web_dashboard.launch.py \
  port:=8766 \
  topic_allow_regex:="(/rosout|/mrkl_explorer/.*|/object_explorer/.*|/cmd_vel.*|/scan.*|/map.*|/plan|/goal_pose|/tf|/collision_monitor.*)" \
  topic_deny_regex:="^/parameter_events$" \
  max_messages_per_topic:=150 \
  max_global_events:=500
```

Open:

```text
http://localhost:8766
```

Important dashboard topics:

```text
/mrkl_explorer/trace
/mrkl_explorer/status
/object_explorer/state
/object_explorer/detections
/object_explorer/sam2_detections
/object_explorer/markers
/cmd_vel_nav2
/cmd_vel_omi
/cmd_vel_nav
/cmd_vel_out
/scan_nav
/plan
/rosout
```

The dashboard logs to:

```text
~/.ros/go2_object_explorer/topic_web_logs/
```

---

# 10. Terminal 6: RViz

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source ./go2_env.sh
source install/setup.bash

ros2 run rviz2 rviz2 -d \
  "$(ros2 pkg prefix go2_object_explorer)/share/go2_object_explorer/config/object_explorer_demo.rviz"
```

RViz should show:

```text
/map
/map_padded
/global_costmap/costmap
/local_costmap/costmap
/scan
/plan
/object_explorer/markers
/object_explorer/detection_markers
/object_explorer/annotated_image
```

Add this manually if not already present:

```text
MarkerArray: /mrkl_explorer/current_goal_marker
```

This red marker shows where MRKL told Nav2 to go.

---

# 11. Terminal 7: MRKL agent

Warm up Ollama:

```bash
curl -s http://127.0.0.1:11434/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "model": "gemma4:e4b",
    "messages": [
      {
        "role": "user",
        "content": "Return JSON only: {\"tool\":\"scan_here\",\"reason\":\"warmup\"}"
      }
    ],
    "stream": false,
    "format": "json",
    "keep_alive": "30m",
    "options": {
      "temperature": 0.1,
      "num_ctx": 8192,
      "num_predict": 256
    }
  }' | python -m json.tool
```

Launch MRKL:

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source ./go2_env.sh
source install/setup.bash

ros2 launch go2_object_explorer mrkl_explorer_agent.launch.py \
  ollama_url:=http://127.0.0.1:11434 \
  ollama_model:=gemma4:e4b \
  ollama_timeout_sec:=45.0 \
  ollama_keep_alive:=30m \
  ollama_num_ctx:=8192 \
  ollama_num_predict:=512 \
  ollama_temperature:=0.1 \
  decision_period_sec:=3.0 \
  scan_duration_sec:=5.0 \
  object_approach_distance_m:=0.85
```

---

# 12. Send a mission

Send missions to MRKL:

```bash
ros2 topic pub --once /mrkl_explorer/goal std_msgs/msg/String \
  "{data: 'explore and find chair'}"
```

Other examples:

```bash
ros2 topic pub --once /mrkl_explorer/goal std_msgs/msg/String \
  "{data: 'explore and find person'}"
```

```bash
ros2 topic pub --once /mrkl_explorer/goal std_msgs/msg/String \
  "{data: 'explore and find bottle'}"
```

```bash
ros2 topic pub --once /mrkl_explorer/goal std_msgs/msg/String \
  "{data: 'explore and find laptop'}"
```

---

# 13. Expected behavior

The correct sequence is:

```text
1. User publishes mission to /mrkl_explorer/goal
2. MRKL triggers passive object explorer
3. Object explorer publishes state, detections, frontiers
4. SAM2 overlay publishes annotated image and masks
5. MRKL builds observation
6. Ollama chooses one tool call:
     scan_here
     navigate_to_frontier
     navigate_to_object
     finish
7. MRKL validator checks the tool call
8. MRKL executes validated tool using Nav2 or cmd_vel scan
9. /mrkl_explorer/trace records every decision/result
```

Example trace:

```json
{
  "event": "mrkl_decision",
  "source": "ollama",
  "decision": {
    "tool": "navigate_to_frontier",
    "args": {
      "frontier_id": 2
    },
    "reason": "best valid frontier near unexplored area"
  }
}
```

---

# 14. Debug checklist

## Check MRKL trace

```bash
ros2 topic echo /mrkl_explorer/trace
```

## Check passive explorer state

```bash
ros2 topic echo /object_explorer/state
```

## Check SAM2 detections

```bash
ros2 topic echo /object_explorer/sam2_detections --once
```

## Check map and scan

```bash
ros2 topic hz /map
ros2 topic hz /scan_nav
```

## Check Nav2 path

```bash
ros2 topic echo /plan --once
```

## Check command chain

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_out
```

## Check collision monitor wiring

```bash
ros2 param get /collision_monitor cmd_vel_in_topic
ros2 param get /collision_monitor cmd_vel_out_topic
ros2 param get /collision_monitor scan.topic
ros2 param get /collision_monitor source_timeout
```

Expected:

```text
cmd_vel_in_topic: cmd_vel_nav
cmd_vel_out_topic: cmd_vel_out
scan.topic: /scan_nav
source_timeout: 2.0
```

## Check planner

```bash
ros2 param get /planner_server planner_plugins
ros2 param get /planner_server GridBased.plugin
ros2 param get /planner_server GridBased.tolerance
ros2 param get /planner_server GridBased.allow_unknown
```

Expected:

```text
GridBased.plugin: nav2_navfn_planner::NavfnPlanner
GridBased.tolerance: 1.25
GridBased.allow_unknown: true
```

---

# 15. Common failure modes

## Robot does not move, but /plan exists

Check:

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_out
```

Interpretation:

```text
/cmd_vel_nav has messages, /cmd_vel_out silent or zero
  -> collision_monitor is blocking

/cmd_vel_out has messages, robot does not move
  -> Go2 driver/WebRTC/robot state issue
```

## Nav2 receives goals but does not plan

Check:

```bash
ros2 topic echo /plan --once
ros2 topic echo /mrkl_explorer/current_goal_marker
```

Likely causes:

```text
goal is inside obstacle
goal is outside explored map
goal is in unknown space
frontier marker is not a valid free-cell goal
```

MRKL should reject bad map cells, but if this happens, inspect `/mrkl_explorer/trace`.

## SAM2 labels are wrong

`yolov8n.pt` is trained on COCO classes. It does not know every object class.

For example, cabinets may appear as refrigerators. Use aliases or filters:

```bash
class_aliases:="refrigerator=cabinet"
target_classes:="person,chair,dining table,tv,bottle,laptop,backpack,cup,book"
```

For a real fix, train a custom YOLO model or add an open-vocabulary detector.

## RViz image says No Image

Check:

```bash
ros2 topic hz /object_explorer/annotated_image
```

If it publishes, RViz QoS is wrong. Set Image reliability to:

```text
Best Effort
```

## System is laggy

Use faster SAM2 settings:

```bash
sam2_imgsz:=320
sam2_every_n:=3
max_detections:=8
inference_period_sec:=0.10
```

## Collision monitor repeatedly stops robot

Check `/rosout` in the web dashboard for:

```text
Latest source and current collision monitor node timestamps differ
Robot to stop due to invalid source
```

Make sure Nav2 uses:

```text
/scan_nav
```

not raw:

```text
/scan
```

---

# 16. Recommended demo order

1. Start Terminal 1 base robot only.
2. Start Terminal 2 explorer live navigation.
3. Confirm `/scan_nav`, `/map`, `/map_padded`, `/navigate_to_pose`.
4. Start Terminal 3 passive explorer.
5. Start Terminal 4 SAM2 overlay.
6. Start Terminal 5 web dashboard.
7. Start Terminal 6 RViz.
8. Start Terminal 7 MRKL agent.
9. Send mission to `/mrkl_explorer/goal`.
10. Watch `/mrkl_explorer/trace`, `/plan`, `/cmd_vel_nav`, `/cmd_vel_out`, and RViz goal marker.

