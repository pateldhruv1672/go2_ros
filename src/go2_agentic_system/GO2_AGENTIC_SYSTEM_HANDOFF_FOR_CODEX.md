# go2_agentic_system Handoff for Codex

Date: 2026-06-05

This package is the top-level Sparky launch wrapper. It does not contain the real behavior logic; it composes the other packages into a single runtime.

## What Is Implemented

### 1. System-level launch composition
The package exposes:
- [sparky_full_system.launch.py](launch/sparky_full_system.launch.py)
- [agent_system.launch.py](launch/agent_system.launch.py)
- [survey_memory.launch.py](launch/survey_memory.launch.py)

The full-system launcher currently includes:
- `go2_agentic_multiagent_voice_nav`
- `go2_agentic_motion_skills`

### 2. A few support nodes exist
The package includes standalone nodes such as:
- command console
- navigation agent
- supervisor
- memory manager
- survey mode
- speech output
- safety supervisor
- task planner
- voice command node

That means the package is more than a blank wrapper, but these nodes still look like system glue rather than the final behavior stack.

### 3. The package is the likely integration point for the larger system
If a future Codex pass needs to wire together voice, navigation, motion skills, memory, and safety, this package is the natural place to compose those launch graphs.

## What Is Not Done Yet

### 1. No finished end-to-end Sparky runtime
The package does not yet define a polished visitor-facing behavior flow.

Missing pieces:
- patrol route runtime
- tour stop sequence control
- teach / resume policy composition
- route-aware conversation behavior
- explicit blocked-route recovery loop

### 2. No clearly enforced role separation
The package still looks like a launcher for components rather than a system with clear ownership boundaries.

Missing pieces:
- which node owns the route state
- which node owns visitor dialogue
- which node owns recovery decisions
- which node owns teach/resume persistence

### 3. No debate output contract
The system package does not define a structured decision schema for debate roles.

That should be added in a lower-level orchestration package first, then exposed here through launch and configuration.

## Current Interpretation

Treat this package as the deployment harness.

It should be responsible for:
- choosing which sub-packages launch together
- setting the right parameters
- wiring ROS topics between packages

It should not become the place where behavior logic is reimplemented.

## Best Next Steps

1. Decide the canonical runtime graph for Sparky.
2. Make the launch files explicit about teach vs resume vs live-tour modes.
3. Add a clean way to launch the route/patrol controller once it exists.
4. Keep this package thin so future behavior changes stay in the right owner package.

## Files To Start From

- [launch/sparky_full_system.launch.py](launch/sparky_full_system.launch.py)
- [launch/agent_system.launch.py](launch/agent_system.launch.py)
- [launch/survey_memory.launch.py](launch/survey_memory.launch.py)
- [setup.py](setup.py)
