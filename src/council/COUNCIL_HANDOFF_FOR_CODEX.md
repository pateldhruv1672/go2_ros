# Council / Sparky Handoff for Codex

Date: 2026-06-05

This note is a status handoff for the current Go2 / Sparky agentic stack. It is meant to help the next coding pass separate what is already working from what is only partially wired or still missing.

## Goal Set

Current target behavior:

1. Build and refine a patrol route with waypoints, tour stops, safety boundaries, and visitor-facing behavior scripts.
2. Implement Sparky behaviors: greeting, route-following, stop explanations, pauses, blocked-route recovery.
3. Implement or mock debate agents: perception, safety, twin, maintenance, verification, with a structured decision output.
4. Support guest interaction using Omi + speaker + laptop Bluetooth, with agentic layers for motion skills and a Nav2 agent.
5. Add teach mode and resume mode.
6. Fix the current teach-mode data flow so VLM info is stored and actually used, instead of only being logged.
7. Fix the broken agentic behavior so the system reliably produces coordinated actions instead of ad hoc reactions.

## What Is Already In Place

### 1. Council orchestration exists and is reasonably mature
The core council stack is implemented in [council/orchestrator.py](council/orchestrator.py) and is already doing more than a simple sensor-to-motion pass.

- It initializes the main agent set: IMU, Camera, LiDAR, SLAM, plus teleop support.
- It has a weighted-vote decision path with safety-aware overrides.
- It tracks decision history and reasoning traces.
- It has peer reflection / disagreement handling.
- It injects structured task context into agent prompts.
- It can save compact decision JSON for later analysis.

Relevant references:
- [agents/__init__.py](council/agents/__init__.py)
- [orchestrator.py](council/orchestrator.py)
- [ARCHITECTURE_SUMMARY.md](ARCHITECTURE_SUMMARY.md)

### 2. Task decomposition and checkpointing exist
The system already has a task planner in [council/task_planner.py](council/task_planner.py).

What it currently does:
- Breaks a task into checkpoints.
- Tracks checkpoint status and progress.
- Supports explicit step-based tasks like “move forward N steps”.
- Produces summaries for the AI loop.
- Lets the main node decide when a task is complete.

This means the project already has a usable notion of stepwise task execution, but it is still generic navigation planning, not a true patrol-route / route-script engine.

### 3. Navigation and map handling already exist
There is an actual high-level planner in [council/navigation_planner.py](council/navigation_planner.py).

What it does today:
- Stores waypoints as `Waypoint` objects.
- Tracks a current goal and current waypoint index.
- Produces simple forward / turn / stop commands.
- Uses nav2 map data when available.
- Has basic obstacle and recovery hooks.

There is also a map pipeline:
- [council/map_manager.py](council/map_manager.py) collects LiDAR, image, and pose data into maps and trajectory summaries.
- [council/ros_interfaces/sensor_hub.py](council/ros_interfaces/sensor_hub.py) publishes nav2 map data and enhanced sensor payloads.
- [council/survey_recorder.py](council/survey_recorder.py) saves survey sessions, maps, keyframes, and pointcloud frames.
- [council/memory_store.py](council/memory_store.py) loads survey-backed episodic memory.

This is important: route-like data exists, but there is no clear patrol-route product layer yet that defines named tour stops, pause scripts, or visitor-facing explanations.

### 4. Voice and mode switching are partially implemented
The main node already subscribes to voice command topics in [council/main.py](council/main.py).

Current capabilities:
- Voice task updates via `/council/voice_task`.
- Manual / AI mode switching via `/council/voice_mode`.
- E-stop handling via `/council/voice_estop`.
- Voice-triggered vision queries via `/council/vision_query`.
- Survey recording toggle with key `s` in manual mode.
- Manual drive and emergency stop support.

There is also a standalone voice controller in [council/voice_controller.py](council/voice_controller.py) that already knows how to publish:
- task
- stop / resume
- manual mode
- AI mode
- vision queries

That said, this is still command plumbing, not a complete guest-interaction or conversational behavior layer.

### 5. Teach-like survey recording exists, but it is not yet a true learning loop
The current “teach” behavior in [council/main.py](council/main.py) is really a survey-recording mode:

- It toggles `survey_mode` in manual control.
- It feeds frames into `MapManager`.
- It captures survey data with `SurveyRecorder`.
- It attaches semantic labels asynchronously when possible.
- It reloads the latest survey dataset into `MemoryStore`.
- It injects memory summaries back into orchestrator task context.

This is useful, but it is not the same as:
- learning a reusable route policy,
- storing a route as a runnable patrol plan,
- replaying a taught route with recovery logic,
- or using stored VLM labels to drive future decisions in a deterministic way.

So the teach path is present, but mostly as data capture + episodic memory, not as behavior synthesis.

### 6. The code already supports some safety-first motion constraints
There are already several safety layers:

- LiDAR / SLAM / IMU agents can veto dangerous motion.
- The orchestrator has planner overrides when a nav2 map blocks a direction.
- Reverse motion is constrained.
- Visual tasks can boost camera priority.
- Decision records include attribution such as `decided_by` and planner-blocked flags.

This is a real foundation for blocked-route recovery and safety boundaries, but it still needs route-aware policy logic to become visitor-friendly and predictable.

### 7. There is partial support for the broader Sparky stack outside `council`
The workspace also contains packages that look related to the larger Sparky system:

- `go2_agentic_system`
- `go2_agentic_multiagent_voice_nav`
- `go2_agentic_motion_skills`
- `voice_assistant`
- `go2_remote_teleop`

Notably, the voice-nav package already contains teach/resume launch files and Omi-related setup, which suggests the broader stack has been scaffolded.

However, from the current inspection, those packages do not yet appear to provide the missing high-level patrol / behavior / debate orchestration end-to-end. The stable, inspectable behavior is still centered in `council`.

## What Is Not Done Yet

### 1. No real patrol-route product layer
Missing or incomplete pieces:

- Named patrol routes with tour-stop metadata.
- Route boundaries and geofenced visitor safety areas.
- Route scripts for each stop.
- A formal route state machine.
- Persistent route storage and route editing tools.
- Clear pause / resume semantics per stop.

Right now the system has general waypoint and survey tools, but not a patrol product that a non-technical operator can rely on.

### 2. Sparky visitor behaviors are not fully implemented
Missing or incomplete pieces:

- Greeting behavior that is actually tied to a visitor interaction state.
- Stop explanations that trigger when the robot pauses at a tour stop.
- Intentional conversational prompts for a guest-facing route.
- Blocked-route recovery behavior that explains why the robot stopped and what it will do next.
- Behavior scripts that are consistent across manual, teach, and resume modes.

### 3. Debate agents are not implemented as dedicated agents
The current council has sensor agents, not the requested debate roles.

What exists:
- IMU / Camera / LiDAR / SLAM / Teleop

What is missing:
- perception debate agent
- safety debate agent
- twin agent
- maintenance agent
- verification agent
- a structured debate transcript that produces one explicit decision object

The orchestrator can already aggregate opinions, but it is not yet role-modelled around those separate debate personas.

### 4. Teach mode does not yet use learned data correctly
Current issue:
- Teach mode stores sensor / semantic / survey data.
- The stored data is loaded into memory.
- But that memory is not yet being turned into an actual route policy or route replay policy.

Missing pieces:
- route retrieval from stored examples
- policy synthesis from taught runs
- semantic stop selection from stored labels
- using learned scene labels to bias future motion decisions
- confidence-aware fallback to the taught route when live perception is weak

### 5. Resume mode is only partially defined
There is command-level support for “resume”, but not a complete resume policy.

Missing pieces:
- resume from current patrol stop
- resume from interrupted taught route
- resume from blocked route after recovery
- resume after voice interruption without losing route state
- resume after manual override or safety stop with explicit re-entry logic

### 6. Omi / speaker guest interaction is not yet integrated as a first-class behavior layer
The workspace has voice and audio pieces, including Omi and speaker-related nodes in the broader agentic voice-nav package and robot audio support in `voice_assistant`.

What is still missing at the system level:
- one clear runtime path that connects guest speech input to council state
- route-aware replies from Sparky
- agentic motion actions driven by spoken visitor requests
- a consistent handoff between dialogue and navigation behavior

So the hardware path may be partially present, but the behavior layer above it is not yet complete.

## Current Best Interpretation of the System

If I had to summarize the current state in one sentence:

> The repository already has a solid council, planner, survey, and voice-control foundation, but it does not yet have a finished Sparky patrol system with teach/resume semantics, visitor scripts, and dedicated debate roles.

## Recommended Next Implementation Order

1. Define a route data model.
   - Routes, stops, stop types, boundaries, pause reasons, and fallback behavior.
   - Store routes separately from raw survey sessions.

2. Add a patrol state machine.
   - States like `idle`, `greeting`, `moving_to_stop`, `stopped_for_script`, `blocked`, `recovering`, `paused`, `resuming`, `complete`.

3. Map teach mode to route capture, not just survey capture.
   - Preserve the current survey recorder, but add route annotations and stop intent.
   - Use VLM labels to infer stop context.

4. Build a route replay / resume engine.
   - Load a taught route.
   - Advance through stops.
   - Resume after interruption.
   - Recover when the route is blocked.

5. Add dedicated debate roles in the orchestrator.
   - Keep the existing sensor agents.
   - Add role wrappers or structured prompts for perception / safety / twin / maintenance / verification.
   - Make the final output a typed decision object with explicit reasoning fields.

6. Wire guest interaction into the patrol state machine.
   - Greeting.
   - Stop explanations.
   - Pause / wait behavior.
   - Visitor-following or tour-guide behavior.

7. Only after the above, polish the Omi / speaker integration.
   - Ensure the voice path is truly route-aware.
   - Ensure spoken interactions can trigger route actions cleanly.

## Practical Notes for the Next Agent

- Start in [council/main.py](council/main.py) and [council/orchestrator.py](council/orchestrator.py).
- Keep the existing survey recorder and memory store, but do not confuse them with a finished teach/resume route system.
- Treat [council/navigation_planner.py](council/navigation_planner.py) as a low-level planner, not the final patrol product.
- Treat the current voice controller as transport, not as the behavior system.
- If you add new route behavior, make it explicit in the state machine and in the saved route format.
- If you add debate roles, keep the existing IMU/Camera/LiDAR/SLAM agents intact and layer the debate roles on top.

## Files Most Relevant to Continue From

- [council/main.py](council/main.py)
- [council/orchestrator.py](council/orchestrator.py)
- [council/task_planner.py](council/task_planner.py)
- [council/navigation_planner.py](council/navigation_planner.py)
- [council/map_manager.py](council/map_manager.py)
- [council/survey_recorder.py](council/survey_recorder.py)
- [council/memory_store.py](council/memory_store.py)
- [council/voice_controller.py](council/voice_controller.py)
- [go2_agentic_multiagent_voice_nav](../go2_agentic_multiagent_voice_nav)
- [voice_assistant](../voice_assistant)

## Bottom Line

There is real implementation progress here, but the main missing piece is the productization of that progress into a stable Sparky behavior system. The current code can perceive, plan, survey, and switch modes; it cannot yet reliably act like a visitor-facing tour robot with teach/resume semantics and explicit debate roles.
