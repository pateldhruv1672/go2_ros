# Go2 / Sparky Global Handoff for Codex

Date: 2026-06-05

This is the collective status note for the current Sparky / Go2 workspace. It summarizes what is actually implemented across the repo, what is still missing, and what needs real improvement next.

## Executive Summary

The workspace already has a usable foundation for:
- low-level motion skills
- voice / Omi / TTS interaction
- semantic navigation with teach and resume modes
- council-based perception and motion voting
- survey capture, memory, and map persistence

What is still missing is the product layer that turns those pieces into a stable visitor-facing robot:
- patrol routes
- stop scripts
- greeting and explanation behavior
- blocked-route recovery as a scripted behavior
- a real teach-to-route learning loop
- a reliable resume-from-interruption loop
- dedicated debate roles with structured decision output

## What Is Done Collectively

### 1. Motion execution exists
The repo already has a concrete motion backend in [go2_agentic_motion_skills](go2_agentic_motion_skills).

What it does:
- runs Unitree SDK2 motion primitives
- exposes motion commands over ROS
- blocks obvious conflicts with active navigation unless allowed
- publishes command status and replies

This is a real execution layer, not a stub.

### 2. Voice interaction exists
The repo already has a voice / dialogue overlay in [go2_agentic_multiagent_voice_nav](go2_agentic_multiagent_voice_nav).

What it does:
- receives Omi/local STT transcripts
- handles wake words
- routes navigation commands
- routes motion skill commands
- routes camera requests
- speaks replies through TTS
- can fall back to chat for freeform responses

This is a working interaction layer, but still not a full behavior controller.

### 3. Teach/resume navigation exists
The strongest route-related package is [go2_semantic_nav_agent](go2_semantic_nav_agent).

What it already supports:
- teach mode
- resume mode
- session storage
- place storage
- spawn restore
- map restore
- fallback recovery
- semantic resolution of saved places
- VLM-assisted place labeling

This is the best foundation for route memory and replay in the repo.

### 4. Council / perception / safety voting exists
The [council](council) package already provides:
- multi-agent perception
- weighted voting
- safety overrides
- task planning / checkpointing
- structured task context
- survey capture and memory loading
- nav2 map awareness
- compact reasoning history

This is the main decision-making core today.

### 5. System-level launch composition exists
The [go2_agentic_system](go2_agentic_system) package already ties together the runtime packages with launch files.

This gives the repo a real deployment structure, even if the behavior layer is still incomplete.

## What Is Not Done Collectively

### 1. No true patrol product
The workspace does not yet have a real patrol system with:
- named routes
- waypoints with visitor meaning
- tour stops
- route boundaries
- per-stop behavior scripts
- pause / resume per stop
- blocked-route scripts

Right now the system has route-adjacent pieces, not a polished patrol product.

### 2. No finished visitor-facing behavior
The robot does not yet behave like a consistent tour guide.

Missing behavior pieces:
- greeting visitors at the right time
- explaining why it stopped
- pausing intentionally at tour stops
- recovering from blocked paths with spoken explanation
- keeping behavior consistent across manual, teach, and resume modes

### 3. No dedicated debate roles
The council can aggregate sensor opinions, but it does not yet have the requested role model:
- perception
- safety
- twin
- maintenance
- verification

Those roles are not yet implemented as first-class agents with a structured decision contract.

### 4. Teach mode is still mostly data capture
Teach mode stores useful information, but it does not yet learn a reusable patrol behavior.

Current limitation:
- it records places, maps, and labels
- it does not yet convert that data into a real route policy
- it does not yet use learned VLM information as a durable behavior model

### 5. Resume mode is only partially behavioral
Resume mode can restore sessions and spawn state, but it is not yet a full behavior resume.

Missing pieces:
- resuming a tour script mid-run
- resuming dialogue context
- resuming a partially executed patrol
- resuming after blocked-route recovery with explicit state

### 6. The system is not yet unified around one behavior state machine
Different packages handle parts of the problem, but there is no single explicit state machine that owns:
- patrol progress
- guest dialogue state
- teach state
- resume state
- recovery state
- verification state

## What Needs Actual Improvement

These are the highest-value improvements, in the order that would most improve the system.

### 1. Add a route and patrol model
Create a real data model for:
- route
- stop
- stop type
- stop script
- boundary / no-go zone
- fallback action
- resume point

This should be the first real product layer above the current navigation and semantic session storage.

### 2. Add a patrol state machine
The system needs explicit states such as:
- idle
- greeting
- moving_to_stop
- stopped_for_script
- paused
- blocked
- recovering
- resuming
- verifying
- complete

Without this, the robot will continue to feel ad hoc instead of purposeful.

### 3. Turn teach mode into route learning
Teach mode should not just save observations.
It should store route behavior:
- which stop was visited
- why the robot paused there
- what it said there
- what the VLM thought mattered
- what recovery was used when blocked

That is the difference between map logging and real teach mode.

### 4. Make resume mode behavior-aware
Resume should restore more than map and pose.
It should restore:
- current route
- current stop index
- current dialogue context
- current recovery state
- current visitor interaction state

### 5. Add structured debate outputs
The council should output a structured decision object with explicit role inputs, for example:
- perception summary
- safety summary
- twin / expectation summary
- maintenance concerns
- verification status
- final action
- confidence
- risk
- reason

This should be formalized before adding more behavior complexity.

### 6. Make voice interaction route-aware
The voice overlay should map speech to route intent, not just generic commands.
It should understand:
- start tour
- go to stop 2
- pause here
- explain this stop
- resume tour
- stop because blocked
- continue after visitors move

### 7. Connect motion skills to behavior scripts
Motion skills should be triggered by behavior intent, not by one-off raw commands.
Examples:
- greet
- point / pose / wave
- pause and hold position
- recover and turn
- follow the patrol route

### 8. Use semantic memory as a policy input
The existing semantic memory and session store should influence behavior:
- recognize repeated places
- map labels to stop roles
- bias route replay from learned sessions
- use prior recoveries to choose safer actions

## Package Role Map

### [go2_agentic_motion_skills](go2_agentic_motion_skills)
Low-level actuation and SDK2 motion primitives.

### [go2_agentic_multiagent_voice_nav](go2_agentic_multiagent_voice_nav)
Voice, speech, camera queries, TTS, and command routing.

### [go2_agentic_system](go2_agentic_system)
Top-level launch and runtime composition.

### [go2_semantic_nav_agent](go2_semantic_nav_agent)
Teach/resume navigation, session persistence, saved places, and fallback recovery.

### [council](council)
Multi-agent perception, memory, safety, checkpointing, and final action selection.

## Best Current Architecture Interpretation

The repo is best understood as five layers:

1. Interaction layer: voice / STT / TTS
2. Behavior memory layer: teach / resume / sessions / semantic places
3. Decision layer: council / perception / safety / voting
4. Execution layer: motion skills and navigation commands
5. Deployment layer: top-level launch composition

The missing piece is a real behavior layer that sits between memory and execution.

## Recommended Next Build Order

1. Define a route / stop / patrol state model.
2. Add a patrol controller that owns state.
3. Extend teach mode to save behavior, not just data.
4. Extend resume mode to restore behavior, not just session geometry.
5. Add structured debate roles and final decision schema.
6. Wire voice commands into patrol states.
7. Connect motion skills to scripted behavior actions.
8. Use semantic memory to bias route replay and recovery.

## Files Most Relevant Now

- [go2_semantic_nav_agent/semantic_nav_node.py](go2_semantic_nav_agent/semantic_nav_node.py)
- [go2_semantic_nav_agent/session_store.py](go2_semantic_nav_agent/session_store.py)
- [go2_semantic_nav_agent/place_store.py](go2_semantic_nav_agent/place_store.py)
- [council/orchestrator.py](council/orchestrator.py)
- [council/task_planner.py](council/task_planner.py)
- [go2_agentic_multiagent_voice_nav/dialogue_orchestrator_node.py](go2_agentic_multiagent_voice_nav/dialogue_orchestrator_node.py)
- [go2_agentic_motion_skills/motion_skill_agent_node.py](go2_agentic_motion_skills/motion_skill_agent_node.py)
- [go2_agentic_system/launch/sparky_full_system.launch.py](go2_agentic_system/launch/sparky_full_system.launch.py)

## Bottom Line

The workspace is not starting from zero. It already has the foundation for navigation, voice, memory, and motion. What it lacks is a single route/patrol behavior layer that turns those capabilities into a reliable Sparky tour robot.
