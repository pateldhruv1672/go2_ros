# CODEX_CONTINUE_WORK.md

## What this project is
A wireless Go2 robotics stack built on top of `go2_ros2_sdk` with semantic navigation and an agentic voice layer.

## Foundation
- `go2_ros2_sdk` is the base runtime and should stay the foundation.
- Use wireless/WebRTC mode for the main robot runtime.
- `unitree_sdk2_python` is reference-only or optional for targeted motion-sidecar work.

## Main packages
- `go2_semantic_nav_agent`
- `go2_agentic_multiagent_voice_nav`
- `go2_agentic_motion_skills`
- `go2_agentic_system`

## Most important known issues
1. Resume sessions can fail if `map.yaml` or `map.pgm` are missing.
2. Duplicate RViz instances can make TF debugging noisy.
3. Nav2 failures like `Failed to transform from  to map` come from goals with empty frame ids.
4. Omi BLE may fail; system should fall back to the device microphone.
5. Voice validation should use on-device TTS only through `speaker_tts_node`.
6. The user prefers fast, practical fixes over large refactors.

## Current desired behavior
- Teach mode saves working sessions.
- Resume mode loads latest valid session.
- Base bringup uses `BASE_MODE=teach` for SLAM or `BASE_MODE=resume` for Nav2, never both at once.
- Voice mode uses Omi when available, device mic when Omi is unavailable.
- Voice output uses local speaker TTS only, not cloud TTS.
- Camera questions route to camera/VLM path.
- Semantic place navigation uses `frame=map`.
- Motion features should not destabilize the wireless base runtime.
- Semantic RViz should show places and route previews during validation.
- The full validation stack should be launchable from one wrapper command.

## Priority backlog
1. Keep `go2_ros2_sdk` foundation stable.
2. Enforce map-frame goals in semantic nav.
3. Keep voice/orchestrator loosely coupled to nav.
4. Make session validation easy.
5. Reduce operator friction with repeatable launch/debug workflows.
6. Keep the teach/resume/RViz/voice validation path reproducible from one command.

## Quick debug commands
```bash
# clean stop
pkill -f "go2_driver_node|robot_state_publisher|slam_toolbox|amcl|map_server|rviz2|semantic_nav|bt_navigator|planner_server|controller_server|lifecycle_manager|foxglove_bridge|local_omi_stt_node|dialogue_orchestrator_node|camera_agent_node|speaker_tts_node" || true
sleep 3

# latest session
SESSION=$(basename "$(ls -td ~/.ros/go2_semantic_nav_sessions/* | head -1)")
echo "$SESSION"
find ~/.ros/go2_semantic_nav_sessions/$SESSION -maxdepth 1 -type f | sort

# ROS visibility
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 daemon stop
ros2 daemon start
ros2 node list
ros2 topic list
```

## Success criteria for any Codex change
- small diff
- build succeeds
- launch path is documented
- no new placeholder logic
- robot operator can run the stack with minimal manual cleanup
