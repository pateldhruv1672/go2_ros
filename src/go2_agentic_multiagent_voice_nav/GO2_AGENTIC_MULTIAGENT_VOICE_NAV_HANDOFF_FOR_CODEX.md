# go2_agentic_multiagent_voice_nav Handoff for Codex

Date: 2026-06-05

This package is the interaction overlay for Sparky. It handles voice input, dialogue routing, camera questions, and TTS, but it is still mostly a command bridge rather than a full behavioral controller.

## What Is Implemented

### 1. Voice-to-command orchestration
The main entry point is [dialogue_orchestrator_node.py](go2_agentic_multiagent_voice_nav/dialogue_orchestrator_node.py).

It already:
- Listens to `/agent/transcript`.
- Strips wake words such as sparky and robot.
- Routes navigation requests to `/semantic_nav/command`.
- Routes motion skills to `/motion_skills/command`.
- Sends camera queries to `/agent/camera/request`.
- Publishes spoken replies on `/agent/reply`.

### 2. Basic guest interaction exists
Current text handling already supports:
- greetings
- stop / cancel / halt
- list places
- asking what the robot sees
- saving a place
- bare navigation phrases like “go to kitchen”
- direct motion phrases like dance, sit, wave, stand, and related skills

### 3. Omi + local STT + TTS launch path exists
The package includes launch files for both teach and resume modes:
- [multiagent_teach.launch.py](launch/multiagent_teach.launch.py)
- [multiagent_resume.launch.py](launch/multiagent_resume.launch.py)

The setup file exports nodes for:
- local Omi STT
- dialogue orchestrator
- camera agent
- speaker TTS

### 4. Place resolution and chat fallback exist
The package uses [place_resolver.py](go2_agentic_multiagent_voice_nav/place_resolver.py) to map spoken place names into saved locations.

If a command does not match a direct behavior, it can fall back to a chat completion using OpenRouter.

## What Is Not Done Yet

### 1. No true agentic behavior layer
The package parses user intent, but it does not yet implement the behavior system you want for Sparky.

Missing pieces:
- visitor-facing route scripts
- patrol-stop explanation logic
- blocked-route recovery dialogue
- teach/resume-aware conversation state
- behavior chaining across multiple stops

### 2. No route-state ownership
The node knows about places and commands, but it does not own a route state machine.

Missing pieces:
- current stop index
- paused / resuming / blocked route states
- teach-time route capture
- resume-time route replay

### 3. No debate orchestration
This package does not yet expose separate perception, safety, twin, maintenance, and verification agents.

At the moment it is still just:
- STT
- dialogue parsing
- camera request / response
- TTS reply

### 4. Voice path is not yet route-aware
There is no guaranteed end-to-end connection from guest speech to:
- patrol state updates
- teach session capture
- navigation recovery decisions

## Current Interpretation

This package is the human interface layer, not the policy layer.

It is already useful for:
- voice commands
- scene description
- motion skill requests
- saved-place navigation

But it is not yet the layer that makes Sparky feel like a consistent tour robot.

## Best Next Steps

1. Add a route-aware dialogue state tied to patrol stops.
2. Add explicit speech intents for greet, pause, explain, resume, and recover.
3. Connect voice commands to teach/resume route storage instead of only generic place navigation.
4. Keep the camera and TTS plumbing, but move behavior decisions into a route/patrol controller.
5. Add structured outputs so the next package can consume intent reliably.

## Files To Start From

- [go2_agentic_multiagent_voice_nav/dialogue_orchestrator_node.py](go2_agentic_multiagent_voice_nav/dialogue_orchestrator_node.py)
- [go2_agentic_multiagent_voice_nav/local_omi_stt_node.py](go2_agentic_multiagent_voice_nav/local_omi_stt_node.py)
- [go2_agentic_multiagent_voice_nav/camera_agent_node.py](go2_agentic_multiagent_voice_nav/camera_agent_node.py)
- [go2_agentic_multiagent_voice_nav/speaker_tts_node.py](go2_agentic_multiagent_voice_nav/speaker_tts_node.py)
- [launch/multiagent_teach.launch.py](launch/multiagent_teach.launch.py)
- [launch/multiagent_resume.launch.py](launch/multiagent_resume.launch.py)
