# AGENTS.md

## Project intent
This repository is a layered robotics stack for a Go2 robot.

### Foundation
- The **foundation** is `go2_ros2_sdk` over **wireless/WebRTC**.
- Keep `go2_ros2_sdk` as the runtime base unless the user explicitly asks to replace it.
- Treat `unitree_sdk2_python` as an optional reference or sidecar for specific motions, not the main system foundation.

### Overlay packages
- `go2_semantic_nav_agent` = teach mode, resume mode, semantic places, spawn restore
- `go2_agentic_multiagent_voice_nav` = speech input, local STT, dialogue orchestrator, camera agent, TTS
- `go2_agentic_motion_skills` = optional motion-skills layer
- `go2_agentic_system` = wrapper launch package

## Core architecture rules
1. Do not tightly couple the voice layer to nav internals.
   - Voice should interact with nav through command/status interfaces.
2. Prefer **small, reversible edits** over broad rewrites.
3. Do not introduce duplicate RViz instances.
4. Never send a Nav2 goal with an empty `header.frame_id`.
5. Default missing saved place/session `frame_id` to `map`.
6. Do not silently fall back to `(0, 0)` when a place lookup fails.
7. If Omi is unavailable, use the device microphone fallback.

## Current focus areas
- Stable resume mode
- Stable teach mode saves
- Omi -> device microphone fallback
- Nav2 goal construction with correct frame ids
- Clean session validation (`map.yaml`, `map.pgm`, `places.yaml`, `session.yaml`)
- Avoid duplicate RViz and duplicate publishers

## Runtime expectations

### Resume mode
Terminal 1:
```bash
ros2 launch go2_robot_sdk robot.launch.py foxglove:=false slam:=false nav2:=true
```

Wait 10-15 seconds.

Terminal 2:
```bash
SESSION=$(basename "$(ls -td ~/.ros/go2_semantic_nav_sessions/* | head -1)")
ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py session_name:=$SESSION rviz2:=false
```

Terminal 3:
```bash
export OPENROUTER_API_KEY=YOUR_KEY
ros2 launch go2_agentic_system sparky_full_system.launch.py
```

### Teach mode
Terminal 1:
```bash
ros2 launch go2_robot_sdk robot.launch.py foxglove:=false slam:=true nav2:=false
```

Terminal 2:
```bash
export OPENROUTER_API_KEY=YOUR_KEY
ros2 launch go2_semantic_nav_agent semantic_nav_teach.launch.py map_label:=digital_twin_lab auto_save_places:=true auto_save_interval_sec:=5.0 auto_save_use_vlm:=true clear_places_on_start:=false
```

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

## Nav2 goal safety rules
Before sending a goal:
- ensure `PoseStamped.header.frame_id` is non-empty
- default it to `map` when missing
- log the frame and coordinates
- reject invalid or unknown places instead of using `(0,0)`

## Omi fallback rules
- `input_backend=auto` should try Omi first
- if Omi is unavailable or BLE connect fails, fall back to device microphone
- all speech inputs must publish to the same transcript topic

## Strong anti-patterns
- launching multiple RViz windows by accident
- keeping old processes alive between tests
- changing package names or entry points without need
- mixing teach-mode and resume-mode simultaneously
- adding fake motion placeholders

## Definition of done for a nav fix
A change is only done if:
- resume mode launches cleanly
- a valid place resolves to a goal in `frame=map`
- Nav2 accepts the goal
- no empty-frame goal is sent
- only one RViz is running
