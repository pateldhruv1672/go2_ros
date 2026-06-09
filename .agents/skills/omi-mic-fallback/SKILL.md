# omi-mic-fallback

## When to use
Use for speech input work when Omi may be unavailable.
Use this whenever voice input needs to keep working on Sparky even if BLE or device discovery fails.
# Skill: Connect to Omi DevKit via Python SDK

## Device
- Name: Omi DevK
- MAC: EF:1C:34:C6:25:92
- Audio UUID: 19B10001-E8F2-537E-4F6C-D104768A1214

## Dependencies
```bash
sudo apt-get install libopus0 libopus-dev
pip install omi-sdk
```

## Connect & Stream Audio
```python
import asyncio
import os
from omi import listen_to_omi, OmiOpusDecoder, transcribe
from asyncio import Queue

OMI_MAC = "EF:1C:34:C6:25:92"
OMI_CHAR_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
DEEPGRAM_API_KEY = os.getenv("DEEPGRAM_API_KEY")

async def main():
    audio_queue = Queue()
    decoder = OmiOpusDecoder()

    def handle_audio(sender, data):
        pcm_data = decoder.decode_packet(data)
        if pcm_data:
            audio_queue.put_nowait(pcm_data)

    def handle_transcript(transcript):
        print(f"Transcription: {transcript}")

    await asyncio.gather(
        listen_to_omi(OMI_MAC, OMI_CHAR_UUID, handle_audio),
        transcribe(audio_queue, DEEPGRAM_API_KEY, on_transcript=handle_transcript)
    )

if __name__ == "__main__":
    asyncio.run(main())
```

## Notes
- Requires Python 3.10+
- Run `omi-scan` to rediscover device if MAC changes
- For permanent connection, run as a systemd service with `Restart=always`

## Desired behavior
- `input_backend=auto`
- try Omi first
- if Omi is unavailable, fall back to device microphone
- publish all transcripts to the same topic
- keep the voice stack alive even if the Omi connection drops mid-session
- expose input status so the orchestrator can tell which source is active

## Install note
```bash
pip3 install sounddevice
```
If the sandbox or robot image already has `sounddevice`, do not reinstall it.

## Runtime checks
```bash
ros2 topic echo /agent/speech_input_status
ros2 topic echo /agent/transcript
```
```bash
ros2 topic echo /agent/transcript --once
```

If the local node is used directly:
```bash
ros2 launch go2_agentic_multiagent_voice_nav multiagent_resume.launch.py input_backend:=auto
```

## Implementation rules
- Omi connection failure must not crash the entire voice stack
- device-mic fallback should be automatic when enabled
- orchestrator should not care which input source produced the transcript
- all backends should publish to the same transcript topic and the same command parser
- `auto` should prefer Omi, then mic, without requiring operator intervention
- if the mic backend is active, publish a clear status message rather than silently switching
- speech input should stay compatible with tour commands like `resume tour`, `pause tour`, and `what is this stop`
