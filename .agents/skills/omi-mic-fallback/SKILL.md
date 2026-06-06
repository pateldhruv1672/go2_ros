# omi-mic-fallback

## When to use
Use for speech input work when Omi may be unavailable.

## Desired behavior
- `input_backend=auto`
- try Omi first
- if Omi is unavailable, fall back to device microphone
- publish all transcripts to the same topic

## Install note
```bash
pip3 install sounddevice
```

## Runtime checks
```bash
ros2 topic echo /agent/speech_input_status
ros2 topic echo /agent/transcript
```

## Implementation rules
- Omi connection failure must not crash the entire voice stack
- device-mic fallback should be automatic when enabled
- orchestrator should not care which input source produced the transcript
