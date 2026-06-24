# Launch Entire Sparky System With Omi Voice

This is the current operational launch path for the Go2 semantic resume stack plus Omi voice, local Ollama agent debate, and local Ollama VLM camera summaries.

Do not use the repo README as the source of truth for this flow. Use the scripts and launch files referenced here.

## Hardware/Network Assumptions

- Go2 is reachable at `192.168.12.1`.
- Omi DevKit BLE address is `EF:1C:34:C6:25:92`.
- Ollama is installed and serving at `http://127.0.0.1:11434`.
- Recommended local model is pulled: `gemma4:12b` for both text reasoning and vision. It stays under a practical 20B-parameter ceiling.
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

`gemma4` requires a recent Ollama release. If the pull fails, update Ollama from `https://ollama.com/download`.

Run once before launching the robot stack:

```bash
ollama serve
```

In another terminal, pull the recommended local models:

```bash
ollama pull gemma4:12b
```

Optional fallback if `gemma4:12b` is not available on your installed Ollama version:

```bash
ollama pull qwen3:14b
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

## Terminal 1: Start Robot + Semantic Resume + Omi

From the workspace root:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
./scripts/run_sparky_voice_resume.sh
```

This script:

- loads `.env.local`
- starts base bringup if `/go2_driver_node`, `/odom`, and `/scan` are not already present
- starts semantic resume navigation, saved-map localization, Nav2, and RViz
- starts Omi BLE audio, local STT, voice safety gate, LangGraph, VLM checkpointing, TTS, tour router, and lightweight perception context
- makes `semantic_nav_node` the owner of all resume-mode movement through `/semantic_nav/command`

`session_name:=auto` is intentional. It skips an empty `default` session and uses the latest usable semantic resume session containing `map.yaml`, `places.yaml`, or `route.yaml`.

Wait until Nav2/RViz is up before issuing motion commands.

To override defaults:

```bash
SESSION_NAME=lab_live_teach_20260608_151525 \
OMI_BLE_DEVICE_ADDRESS=EF:1C:34:C6:25:92 \
DEBATE_LLM_MODEL=gemma4:12b \
VLM_MODEL=gemma4:12b \
./scripts/run_sparky_voice_resume.sh
```

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

Resume/tour commands sent to semantic navigation:

```bash
ros2 topic echo --full-length /semantic_nav/command
```

TTS relay:

```bash
ros2 topic echo --full-length /go2_tts/status
ros2 topic echo --full-length /go2_tts/say
```

## Voice Commands To Try

Start normal commands with `Sparky`, `Go2`, or `robot`. Background speech without a wake word is ignored. Emergency stop commands still work without a wake word.

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

Those tour commands should produce JSON on `/semantic_nav/command`, such as `start_tour`, `resume_tour`, or `advance_tour`. The semantic resume node owns the real checkpoint navigation from `route.yaml` and `places.yaml`.

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

Stop the unified voice/resume stack with `Ctrl+C` in Terminal 1.

If stale overlay processes remain:

```bash
ps -eo pid,ppid,cmd | rg 'go2_omi_voice_bridge|go2_langgraph_agent|go2_memory_core|semantic_nav|sparky_voice_resume|main_supervisor|vlm_checkpoint_node'
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
ollama pull gemma4:12b
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

If transcripts appear but `/go2_agent/speech` and `/go2_vlm_checkpoint/status` do not, restart the unified script in Terminal 1.
