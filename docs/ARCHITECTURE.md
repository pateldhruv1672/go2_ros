# Sparky Go2 Architecture

This document explains how the major packages, launch layers, topics, and safety gates work together.

## Layered System View

```mermaid
flowchart TB
  subgraph Robot["Robot and Sensor Layer"]
    Go2["Unitree Go2\nROBOT_IP=192.168.12.1\nCONN_TYPE=webrtc"]
    Driver["go2_driver_node\nsrc/go2_robot_sdk"]
    RSP["robot_state_publisher"]
    PCL["/point_cloud2"]
    PCL2Scan["pointcloud_to_laserscan\n/go2_pointcloud_to_laserscan"]
    Scan["/scan\nsensor_msgs/LaserScan"]
    Odom["/odom"]
    Camera["/camera/image_raw"]
    TF["/tf and /tf_static"]
  end

  subgraph Mapping["Teach and Mapping Layer"]
    Slam["slam_toolbox\nmap -> odom owner in teach/base SLAM"]
    Teach["semantic_nav_node\nmode=teach"]
    Session["~/.ros/go2_semantic_nav_sessions/<session>\nmap.yaml, map.pgm, places.yaml, session.yaml"]
  end

  subgraph Resume["Saved Map Resume Layer"]
    MapServer["resume_map_server\nsaved map"]
    AMCL["amcl\nmap -> odom owner in resume"]
    ResumeLaunch["semantic_nav_resume.launch.py"]
    ResumeNode["semantic_nav_node\nmode=resume"]
  end

  subgraph Nav2["Nav2 Execution Layer"]
    Planner["planner_server\n/global_costmap"]
    Controller["controller_server\n/local_costmap"]
    Behavior["behavior_server"]
    BTN["bt_navigator"]
    Collision["collision_monitor"]
    CmdNav["/cmd_vel_nav"]
    CmdOut["/cmd_vel_out"]
  end

  subgraph Agent["Agentic Layer"]
    Memory["memory and semantic context"]
    Perception["perception summaries\nscan, pointcloud, VLM, open vocab"]
    LangGraph["go2_langgraph_main_supervisor"]
    NavTool["go2_nav2_tool_server\ndry-run first when enable_motion=false"]
  end

  subgraph Voice["Omi Voice and Tour Layer"]
    Omi["Omi/mobile/simulated transcript"]
    STT["go2_voice_stt_node"]
    Gate["go2_voice_intent_gate\nconfirmation and safety gate"]
    Tour["go2_tour_voice_command_router"]
    TTS["go2_tts_node"]
  end

  Go2 --> Driver
  Driver --> PCL
  Driver --> Odom
  Driver --> Camera
  Driver --> TF
  PCL --> PCL2Scan --> Scan

  Scan --> Slam
  Odom --> Slam
  Slam --> Session
  Teach --> Session

  Session --> MapServer
  MapServer --> AMCL
  Scan --> ResumeLaunch
  ResumeLaunch --> AMCL
  ResumeLaunch --> ResumeNode
  ResumeLaunch --> Planner
  ResumeLaunch --> Controller
  ResumeLaunch --> Behavior
  ResumeLaunch --> BTN
  ResumeLaunch --> Collision

  MapServer --> Planner
  AMCL --> Planner
  AMCL --> Controller
  Scan --> Controller
  Scan --> Collision
  BTN --> Planner
  BTN --> Controller
  Controller --> CmdNav --> Collision --> CmdOut --> Driver

  Scan --> Perception
  PCL --> Perception
  Camera --> Perception
  Odom --> LangGraph
  Perception --> LangGraph
  Memory --> LangGraph
  LangGraph --> NavTool --> BTN

  Omi --> STT --> Gate
  Gate -->|verified /go2_agent/user_command| LangGraph
  Gate -->|stop_robot /go2_nav/command| NavTool
  Gate -->|zero Twist| CmdOut
  Gate -->|tour commands| Tour
  Tour --> TTS
  LangGraph --> TTS
```

## Runtime Ownership Rules

```mermaid
flowchart LR
  Base["Base mode\nDriver + sensors + TF"] --> BaseTopics["/odom\n/scan\n/camera/image_raw\n/tf\n/cmd_vel_out"]
  Teach["Teach mode\nSLAM active"] --> SlamOwner["slam_toolbox owns map -> odom"]
  Resume["Resume mode\nSaved map active"] --> AmclOwner["amcl owns map -> odom"]
  Agentic["Agentic layer"] --> AgentRule["Propose or request Nav2 actions\nnever bypass Nav2"]
  Voice["Voice layer"] --> VoiceRule["Confirm motion commands\nstop immediately on stop phrases"]

  Teach -. do not run with .- AmclOwner
  Resume -. do not run with .- SlamOwner
```

The most important rule is that only one localization system should own `map -> odom` at a time:

- Teach/SLAM: `slam_toolbox`
- Resume/saved map: `amcl`

## Base Bringup Flow

```mermaid
sequenceDiagram
  participant User
  participant Script as scripts/run_robot_live.sh
  participant Robot as robot.launch.py
  participant Driver as go2_driver_node
  participant Scan as pointcloud_to_laserscan
  participant Topics as ROS topics

  User->>Script: BASE_MODE=base bash scripts/run_robot_live.sh
  Script->>Robot: foxglove, slam, nav2, rviz flags
  Robot->>Driver: start WebRTC driver
  Robot->>Scan: start pointcloud_to_laserscan
  Driver->>Topics: /odom, /camera/image_raw, /point_cloud2, /tf
  Scan->>Topics: /scan
```

Mode defaults in `scripts/run_robot_live.sh`:

| `BASE_MODE` | SLAM | Nav2 | RViz |
| --- | --- | --- | --- |
| `base` | false | false | true |
| `teach` | true | false | false |
| `resume` | false | true | true |

For semantic resume overlay, prefer base bringup plus `scripts/run_semantic_nav_resume.sh` so the overlay owns the saved map, AMCL, and Nav2 instance.

## Semantic Teach Flow

```mermaid
flowchart TB
  Scan["/scan"] --> Slam["slam_toolbox"]
  Odom["/odom"] --> Slam
  Camera["/camera/image_raw"] --> VLM["optional VLM labels"]
  Slam --> Map["live /map"]
  Map --> Teach["semantic_nav_node mode=teach"]
  VLM --> Teach
  Teach --> Places["places.yaml"]
  Teach --> Session["session.yaml"]
  Teach --> SavedMap["map.yaml + map.pgm"]
```

Teach mode creates the assets that resume mode and tours depend on:

- occupancy map
- saved session metadata
- named semantic places
- optional VLM summaries and labels

## Semantic Resume and Nav2 Flow

```mermaid
flowchart TB
  Session["saved session"] --> MapYaml["map.yaml"]
  MapYaml --> MapServer["resume_map_server"]
  RawScan["/scan"] --> Retimer["scan_retimestamp_node\ncurrent launch path"]
  Retimer --> ScanNav["/scan_nav"]
  ScanNav --> AMCL["amcl"]
  ScanNav --> LocalCostmap["local_costmap obstacle_layer"]
  ScanNav --> Collision["collision_monitor"]
  MapServer --> GlobalCostmap["global_costmap static_layer"]
  AMCL --> Nav2["Nav2 lifecycle nodes"]
  GlobalCostmap --> Planner["planner_server"]
  LocalCostmap --> Controller["controller_server"]
  Planner --> BTN["bt_navigator"]
  BTN --> Controller
  Controller --> CmdVelNav["/cmd_vel_nav"]
  CmdVelNav --> Collision
  Collision --> CmdVelOut["/cmd_vel_out"]
```

Current launch file:

```text
src/go2_semantic_nav_agent/launch/semantic_nav_resume.launch.py
```

Current scan path in that launch:

```text
/scan -> scan_retimestamp_node -> /scan_nav -> AMCL, costmaps, collision_monitor, semantic_nav_node
```

Costmap stability guidance:

- global costmap should primarily use the saved map
- local costmap and collision monitor should handle live obstacles
- avoid making the global planner chase scan noise unless intentionally testing dynamic global rerouting

Runtime flag:

```bash
export GO2_GLOBAL_LIVE_OBSTACLES=0
```

## Agentic Observe/Explore Flow

```mermaid
flowchart LR
  User["operator command\n/go2_agent/user_command"] --> Supervisor["go2_langgraph_main_supervisor"]
  ScanSummary["/go2_perception/scan_summary"] --> Supervisor
  PointSummary["/go2_perception/pointcloud_summary"] --> Supervisor
  VLM["/go2_vlm_checkpoint/status"] --> Supervisor
  NavStatus["/go2_nav/status"] --> Supervisor
  Memory["semantic/session memory"] --> Supervisor
  Supervisor --> Stream["/go2_agent/stream"]
  Supervisor --> Status["/go2_agent/status"]
  Supervisor --> Speech["/go2_agent/speech"]
  Supervisor --> NavCommand["/go2_nav/command"]
  NavCommand --> NavTool["go2_nav2_tool_server"]
  NavTool -->|enable_motion=false| DryRun["dry-run status only"]
  NavTool -->|enable_motion=true| Nav2Action["navigate_to_pose / navigate_through_poses"]
```

The agentic layer should be brought up in observe-only mode first:

```bash
ros2 launch go2_agentic_system explore_mode.launch.py enable_motion:=false
```

Enable motion only after base Nav2/resume mode is stable.

## Omi Voice and Tour Flow

```mermaid
sequenceDiagram
  participant Omi as Omi/mobile/simulated transcript
  participant STT as go2_voice_stt_node
  participant Gate as go2_voice_intent_gate
  participant Agent as go2_langgraph_main_supervisor
  participant Tour as go2_tour_voice_command_router
  participant Nav as go2_nav2_tool_server
  participant TTS as go2_tts_node

  Omi->>STT: /omi/transcript_raw
  STT->>Gate: /go2_voice/transcript JSON
  Gate->>Gate: parse intent and confidence
  alt stop command
    Gate->>Nav: /go2_nav/command stop_robot
    Gate->>Nav: zero /cmd_vel_out
    Gate->>TTS: "Stopping now."
  else observe command
    Gate->>Agent: verified /go2_agent/user_command
    Agent->>TTS: /go2_agent/speech
  else motion or tour command
    Gate->>TTS: confirmation prompt
    Gate->>Omi: waits for yes/no transcript
    Omi->>STT: "yes, proceed"
    STT->>Gate: confirmation transcript
    Gate->>Agent: verified /go2_agent/user_command
    Gate->>Tour: /go2_tour/start or continue
    Tour->>TTS: narration/status
  end
```

Key voice topics:

| Topic | Direction | Purpose |
| --- | --- | --- |
| `/omi/transcript_raw` | input | simulated or Omi/mobile transcript text |
| `/go2_voice/transcript` | STT output | normalized transcript JSON |
| `/go2_voice/verification_request` | gate output | confirmation prompt |
| `/go2_voice/verification_state` | gate output | state machine events |
| `/go2_agent/user_command` | gate output | verified commands for LangGraph |
| `/go2_nav/command` | gate/tool output | stop/navigation tool commands |
| `/go2_tts/say` | gate output | immediate speech request |
| `/go2_tour/status` | tour output | tour state |
| `/go2_tour/narration` | tour output | checkpoint narration |

Voice command policy:

| Command type | Confirmation | Motion |
| --- | --- | --- |
| stop/halt/freeze | no | immediate stop only |
| where are we | no | none |
| what do you see | no | none |
| tell me a fun fact | no | none |
| navigate to place | yes | via agent/Nav2 only |
| start/continue tour | yes | via agent/Nav2 only |

## Tour Route and Memory Files

Tour router path:

```text
~/.ros/go2_semantic_nav_sessions/<session_name>/tours/sjsu_ads_department_tour.json
```

Example session layout:

```text
~/.ros/go2_semantic_nav_sessions/default/
  map.yaml
  map.pgm
  places.yaml
  session.yaml
  tours/
    sjsu_ads_department_tour.json
```

Example route:

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

The current tour voice router prepares status and narration. It does not directly drive the robot. Navigation should still flow through the agentic supervisor and `go2_nav2_tool_server`.

## Safety-Critical Paths

```mermaid
flowchart TB
  VoiceStop["voice: stop/halt/freeze"] --> Gate["go2_voice_intent_gate"]
  Gate --> Zero["publish zero Twist\n/cmd_vel_out"]
  Gate --> StopCmd["publish stop_robot\n/go2_nav/command"]
  StopCmd --> NavTool["go2_nav2_tool_server"]
  NavTool --> Zero2["publish zero Twist\n/cmd_vel_out"]
  Gate --> CancelTour["/go2_tour/cancel"]

  Nav2Cmd["Nav2 controller\n/cmd_vel_nav"] --> Collision["collision_monitor"]
  Collision --> CmdOut["/cmd_vel_out"]
  CmdOut --> Driver["go2_driver_node"]
```

Autonomous motion must pass through:

```text
agent or Nav2 tool -> Nav2 action -> controller_server -> /cmd_vel_nav -> collision_monitor -> /cmd_vel_out
```

Direct autonomous movement around Nav2 is not part of the intended architecture.

## Common Failure Boundaries

| Symptom | Likely layer |
| --- | --- |
| no `/scan` | driver, point cloud, or pointcloud_to_laserscan |
| `/scan` exists but Nav2 will not activate | Nav2 lifecycle, costmap config, TF, or map/localization |
| map warnings in base mode | map-consuming path started without map source |
| `navigate_to_pose` unavailable | Nav2 lifecycle did not reach active state |
| voice asks confirmation but nothing moves | expected if `enable_motion:=false`, route missing, or agent/Nav2 not running |
| tour says route missing | create the JSON route under the selected session `tours/` directory |

