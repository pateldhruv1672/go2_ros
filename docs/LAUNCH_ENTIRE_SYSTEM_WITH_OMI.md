# Launch Entire Sparky System With Omi Voice

This is the current operational launch path for the Go2 semantic resume stack plus Omi voice, local Ollama agent debate, and local Ollama VLM camera summaries.

Do not use the repo README as the source of truth for this flow. Use the scripts and launch files referenced here.

## Hardware/Network Assumptions

- Go2 is reachable at `192.168.12.1`.
- Omi DevKit BLE address is `EF:1C:34:C6:25:92`.
- Ollama is installed and serving at `http://127.0.0.1:11434`.
- Recommended local models are pulled: `qwen3:14b` for text reasoning and `qwen3-vl:8b` for vision. Both stay under a practical 20B-parameter ceiling.
- Camera is publishing `/camera/image_raw`.
- Robot base publishes `/odom` and `/scan`.

## Terminal 0: Prepare Ollama Models

If Ollama is not installed:

```bash
curl -fsSL https://ollama.com/install.sh | sh
```

Check the install:

```bash
ollama --version
```

`qwen3-vl` requires a recent Ollama release. If the pull fails, update Ollama from `https://ollama.com/download`.

Run once before launching the robot stack:

```bash
ollama serve
```

In another terminal, pull the recommended local models:

```bash
ollama pull qwen3:14b
ollama pull qwen3-vl:8b
```

Optional fallback if `qwen3-vl:8b` is not available on your installed Ollama version:

```bash
ollama pull qwen2.5vl:7b
```

Check models:

```bash
ollama list
```

Check the local API:

```bash
python - <<'PY'
import json, urllib.request
print(json.loads(urllib.request.urlopen("http://127.0.0.1:11434/api/tags", timeout=3).read().decode())["models"][0]["name"])
PY
```

## Terminal 1: Start Robot + Nav2 Resume

From the workspace root:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
./scripts/run_semantic_nav_resume.sh
```

This script:

- loads `.env.local`
- starts base bringup if `/go2_driver_node`, `/odom`, and `/scan` are not already present
- starts semantic resume navigation
- starts RViz
- owns the resume-mode Nav2 stack

Wait until Nav2/RViz is up before issuing motion commands.

## Terminal 2: Start Omi + Agent + VLM

From the workspace root:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh
source install/setup.bash

ros2 launch go2_omi_voice_bridge omi_voice_stack.launch.py \
  adapter_mode:=ble_audio \
  ble_device_address:=EF:1C:34:C6:25:92 \
  require_confirmation_for_motion:=true \
  agent_enabled:=true \
  enable_llm_debate:=true \
  debate_llm_provider:=ollama \
  debate_llm_model:=qwen3:14b \
  enable_vlm_checkpointing:=true \
  vlm_provider:=ollama \
  vlm_model:=qwen3-vl:8b \
  vlm_auto_write_checkpoints:=false \
  tts_enabled:=true \
  local_speaker_enabled:=true
```

This starts:

- `go2_voice_stt_node`: Omi BLE audio to local faster-whisper transcript
- `go2_voice_intent_gate`: wake words, intent parsing, confirmation gate
- `go2_langgraph_main_supervisor`: LangGraph agent and local Ollama debate
- `go2_vlm_checkpoint_node`: local Ollama camera summaries
- `go2_tts_node`: response relay and local speaker output
- `go2_tour_voice_command_router`: saved-tour voice adapter

## Verify Connections

Omi BLE connected:

```bash
ros2 topic echo --full-length /go2_voice/stt_status
```

Expected shape:

```json
{"source":"omi_ble_status","raw":{"connected":true,"address":"EF:1C:34:C6:25:92","codec":20}}
```

What Omi hears:

```bash
ros2 topic echo --full-length /go2_voice/transcript
```

Agent/Ollama responses:

```bash
ros2 topic echo --full-length /go2_agent/speech
ros2 topic echo --full-length /go2_agent/status
```

VLM camera summaries:

```bash
ros2 topic echo --full-length /go2_vlm_checkpoint/status
```

TTS relay:

```bash
ros2 topic echo --full-length /go2_tts/status
ros2 topic echo --full-length /go2_tts/say
```

## Voice Commands To Try

Non-motion commands:

```text
Hey Sparky, what do you see?
Hey Sparky, where are we?
Hey Sparky, give me a fun fact.
```

Motion commands require confirmation:

```text
Hey Sparky, go to lobby.
Hey Sparky, navigate to entrance.
Hey Sparky, start the tour.
Hey Sparky, continue the tour.
```

After Sparky asks for confirmation, say:

```text
Yes, proceed.
```

To reject:

```text
No, cancel.
```

Immediate stop commands:

```text
Stop.
Emergency stop.
Freeze.
Cancel navigation.
```

## Manual VLM Test

If you want to test Ollama VLM without speaking:

```bash
ros2 topic pub --once /go2_vlm_checkpoint/write_now std_msgs/msg/String "{data: 'manual_test'}"
ros2 topic echo --once --full-length /go2_vlm_checkpoint/status
```

A healthy result has:

```json
{"vlm_success":true,"summary":"...camera description..."}
```

## Manual Agent Test

If you want to test Ollama agent debate without Omi:

```bash
ros2 topic pub --once /go2_agent/user_command std_msgs/msg/String \
  "{data: '{\"text\":\"where are we\",\"intent\":\"where_am_i\",\"verified\":true,\"source\":\"manual_test\"}'}"

ros2 topic echo --once --full-length /go2_agent/speech
```

Healthy output should be a fresh agent response with `nav_action: null` for non-motion chat.

## Shutdown

Stop the Omi/agent/VLM stack with `Ctrl+C` in Terminal 2.

Stop the robot/Nav2 resume stack with `Ctrl+C` in Terminal 1.

If stale voice processes remain:

```bash
ps -eo pid,ppid,cmd | rg 'go2_omi_voice_bridge|go2_langgraph_agent|go2_memory_core|omi_voice_stack|main_supervisor|vlm_checkpoint_node'
```

Then kill only the matching stale process IDs.

## Common Failure Checks

Ollama not reachable:

```bash
python - <<'PY'
import json, urllib.request
req = urllib.request.Request(
    "http://127.0.0.1:11434/api/tags",
    headers={"Content-Type": "application/json"},
)
print(json.loads(urllib.request.urlopen(req, timeout=3).read().decode())["models"][0]["name"])
PY
```

If that fails, start Ollama and pull models:

```bash
ollama serve
ollama pull qwen3:14b
ollama pull qwen3-vl:8b
```

No VLM response:

```bash
ros2 topic info -v /go2_vlm_checkpoint/status
ros2 topic echo --once --full-length /camera/image_raw
```

No agent response:

```bash
ros2 topic info -v /go2_agent/user_command
ros2 topic info -v /go2_agent/speech
```

`/go2_agent/user_command` must have `go2_langgraph_main_supervisor` as a subscriber.

Omi hears text but robot does not answer:

```bash
ros2 topic echo --full-length /go2_voice/transcript
ros2 topic echo --full-length /go2_tts/say
ros2 topic echo --full-length /go2_agent/speech
ros2 topic echo --full-length /go2_vlm_checkpoint/status
```

If transcripts appear but `/go2_agent/speech` and `/go2_vlm_checkpoint/status` do not, restart Terminal 2 with the full Omi launch command above.
