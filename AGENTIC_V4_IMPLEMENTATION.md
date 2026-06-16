# Go2 Agentic Navigation v4 Implementation Notes

This patch replaces remaining toy exploration/voice/perception behavior with concrete ROS components that are still opt-in and dry-run-first.

## What is now implemented

- Multimodal agent goal selection:
  - `/go2_nav/frontier_candidates`
  - `/go2_nav/coverage_plan`
  - `/go2_perception/scan_summary`
  - `/go2_perception/pointcloud_summary`
  - `/go2_perception/traversability_summary`
  - `/go2_perception/open_vocab_detections`
  - `/go2_perception/dynamic_obstacles`
  - `/odom`
  - VLM checkpoint status
- Frontier exploration:
  - occupancy-grid frontier clustering
  - information-gain/distance scoring
  - map-frame candidate poses
- Coverage exploration:
  - map-derived lawnmower waypoint generation over safe free cells
- Nav2 execution layer:
  - `NavigateToPose`
  - `NavigateThroughPoses`
  - selected frontier/coverage goal execution
  - recovery request publication
  - dry-run by default, motion only with `enable_motion:=true`
- Recovery manager:
  - stop robot
  - recovery plan publication
  - optional `/initialpose` publishing from saved spawn/pose
- Open-vocabulary perception:
  - optional GroundingDINO/OWLv2 via `transformers` + `torch`
  - conservative heuristic fallback marked as `backend=heuristic`
- Dynamic obstacle tracking:
  - LiDAR cluster tracking
  - speed estimates
  - front-blocked flag for safety gating
- Omi and laptop voice:
  - Omi BLE audio characteristic, no webhook path
  - local STT on laptop via `faster-whisper`, `whisper`, or `vosk`
  - local TTS output via Piper, pyttsx3, or espeak

## Runtime example

```bash
ros2 launch go2_agentic_system explore_mode.launch.py \
  enable_motion:=false \
  enable_open_vocab_detector:=true \
  enable_dynamic_obstacle_tracking:=true \
  voice_input_mode:=text_topic
```

For Omi BLE + local laptop STT:

```bash
ros2 launch go2_agentic_system agentic_memory_stack.launch.py \
  enable_langgraph_agent:=true \
  enable_unified_voice:=true \
  voice_input_mode:=omi_ble \
  stt_backend:=faster_whisper \
  stt_model:=tiny.en \
  omi_device_name:=Omi
```

For local TTS:

```bash
ros2 launch go2_agentic_system agentic_memory_stack.launch.py \
  enable_langgraph_agent:=true \
  enable_tts:=true \
  tts_backend:=piper \
  piper_model_path:=/path/to/piper_voice.onnx
```

## Optional dependencies

The ROS packages still build without these optional packages, but the full runtime requires installing the relevant backend:

```bash
pip install faster-whisper speechrecognition pyaudio bleak
pip install transformers torch pillow
# Optional alternatives:
pip install vosk pyttsx3
sudo apt-get install espeak alsa-utils libopus0 libopus-dev
```

If you use Omi Opus firmware/audio, install the Omi SDK or an Opus decoder so the BLE audio stream can be decoded locally before STT.

## Validation

Pure-Python validation in the patch workspace:

```bash
python3 -m compileall -q src
PYTHONPATH=src/go2_memory_core:src/go2_semantic_voxel_memory:src/go2_langgraph_agent:src/go2_nav_tools:src/go2_perception_tools \
pytest -q src/go2_memory_core/test src/go2_semantic_voxel_memory/test src/go2_langgraph_agent/test src/go2_nav_tools/test src/go2_perception_tools/test
```

Expected in a non-ROS container:

```text
15 passed, 2 skipped
```

The two skipped tests require ROS message packages available from a sourced ROS 2 environment.
