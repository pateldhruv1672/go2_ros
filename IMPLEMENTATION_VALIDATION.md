# Go2 Agentic Navigation Implementation Validation

This add-on turns the previous architecture scaffold into a runnable MVP for the four remaining implementation areas:

1. Real Kuzu-backed graph persistence validation.
2. SQLite-backed semantic voxel memory with aggregation, query, route context, and point-cloud comparison helpers.
3. VLM checkpoint pipeline for camera + odom checkpoints through the unified memory API.
4. Unified text/laptop-mic/Omi voice input bridge.

## Optional Python dependencies

The code is additive and still builds without these optional packages. Install only the features you want to run:

```bash
# Kuzu graph persistence
python3 -m pip install kuzu

# VLM image encoding from ROS Image messages
python3 -m pip install opencv-python numpy

# Laptop microphone STT
python3 -m pip install SpeechRecognition openai-whisper pyaudio

# Omi websocket transcript stream
python3 -m pip install websockets
```

## Build

```bash
cd ~/Dhruv/sparky/ros2_ws
colcon build --symlink-install --packages-select \
  go2_memory_core_interfaces \
  go2_memory_core \
  go2_semantic_voxel_memory \
  go2_langgraph_agent \
  go2_agentic_system
source install/setup.bash
```

## Kuzu graph persistence validation

```bash
ros2 launch go2_agentic_system agentic_memory_stack.launch.py \
  session_name:=kuzu_validation \
  enable_memory_core:=true \
  enable_graph_memory:=true

ros2 service call /go2_memory/query_graph go2_memory_core_interfaces/srv/QueryGraph \
  "{session_name: kuzu_validation, query_json: '{\"validate_persistence\": true}'}"
```

Expected result: `success: true` when the `kuzu` wheel is installed. If Kuzu is absent, the response is structured and says Kuzu is unavailable; JSONL graph persistence still works.

## Semantic voxel memory validation

```bash
ros2 launch go2_agentic_system agentic_memory_stack.launch.py \
  session_name:=voxel_validation \
  enable_voxel_memory:=true

ros2 topic pub --once /go2_voxel/write_observation std_msgs/msg/String \
  "{data: '{\"center_xyz\": {\"x\": 1.0, \"y\": 2.0, \"z\": 0.1}, \"semantic_labels\": [\"doorway\", \"sign\"], \"properties\": {\"occupancy_probability\": 0.2, \"traversability_score\": 0.8}}'}"

ros2 topic pub --once /go2_voxel/query std_msgs/msg/String \
  "{data: '{\"mode\": \"landmarks\", \"x\": 1.0, \"y\": 2.0, \"radius_m\": 2.0}'}"

ros2 topic echo /go2_voxel/summary
```

The persistent database is written to:

```text
~/.ros/go2_semantic_nav_sessions/<session_name>/voxel_memory/semantic_voxels.db
```

## VLM checkpoint validation

Offline deterministic mode, no API key required:

```bash
ros2 launch go2_agentic_system agentic_memory_stack.launch.py \
  session_name:=vlm_validation \
  enable_memory_core:=true \
  enable_graph_memory:=true \
  enable_voxel_memory:=true \
  enable_vector_memory:=true \
  enable_vlm_checkpointing:=true \
  vlm_provider:=offline
```

OpenRouter/Gemini mode:

```bash
export OPENROUTER_API_KEY=...
ros2 launch go2_agentic_system agentic_memory_stack.launch.py \
  session_name:=vlm_validation \
  enable_memory_core:=true \
  enable_graph_memory:=true \
  enable_voxel_memory:=true \
  enable_vector_memory:=true \
  enable_vlm_checkpointing:=true \
  vlm_provider:=openrouter \
  vlm_model:=google/gemini-2.5-flash \
  camera_topic:=/camera/image_raw
```

The node writes image artifacts, raw VLM JSON, `vlm_summary`, odom velocity, graph checkpoint nodes, vector text entries, and semantic voxel observations.

## Unified voice input validation

Text-topic command path:

```bash
ros2 launch go2_agentic_system agentic_memory_stack.launch.py \
  enable_unified_voice:=true \
  voice_input_mode:=text_topic

ros2 topic pub --once /go2_voice/text_command std_msgs/msg/String "{data: 'return to spawn'}"
ros2 topic echo /go2_agent/user_command
```

Laptop microphone:

```bash
ros2 run go2_langgraph_agent voice_input_node --ros-args \
  -p input_mode:=laptop_mic \
  -p stt_backend:=whisper
```

Omi websocket transcript stream:

```bash
export OMI_API_KEY=...
ros2 run go2_langgraph_agent voice_input_node --ros-args \
  -p input_mode:=omi_ws \
  -p omi_websocket_url:=wss://api.omi.me/v4/listen
```

All input modes publish normalized JSON commands to `/go2_agent/user_command`.

## Local test command

```bash
PYTHONPATH=src/go2_memory_core:src/go2_semantic_voxel_memory:src/go2_langgraph_agent \
pytest -q src/go2_memory_core/test src/go2_semantic_voxel_memory/test src/go2_langgraph_agent/test
```
