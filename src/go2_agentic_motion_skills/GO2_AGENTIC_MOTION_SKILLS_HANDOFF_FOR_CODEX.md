# go2_agentic_motion_skills Handoff for Codex

Date: 2026-06-05

This package is the low-level motion execution layer for Sparky. It is not the decision-maker; it takes a motion command and tries to run the corresponding Unitree SDK2 skill safely.

## What Is Implemented

### 1. Concrete Unitree motion skill execution
The main node is [motion_skill_agent_node.py](go2_agentic_motion_skills/motion_skill_agent_node.py).

It already:
- Initializes Unitree SDK2 clients.
- Accepts commands on `/motion_skills/command`.
- Normalizes many human-friendly skill names into concrete SDK actions.
- Executes motion primitives like stand up, stand down, hello, dance, jumps, walking modes, recovery stand, and stop move.
- Publishes replies on `/agent/reply` and status on `/motion_skills/status`.

### 2. Basic navigation safety gating
The node watches `/semantic_nav/status` and sets `navigation_active` when navigation is in progress.

Current behavior:
- If navigation is active, it refuses motion skills unless `allow_during_navigation` is enabled.
- This prevents obvious conflicts between skill execution and navigation.

### 3. ROS launch entry point exists
The package ships a ROS launch file in [motion_skills.launch.py](launch/motion_skills.launch.py) and exports the console entry point in [setup.py](setup.py).

## What Is Not Done Yet

### 1. No route-aware motion policy
The package does not know about:
- patrol routes
- tour stops
- route state
- guest-facing behavior scripts
- teach/resume route replay

It only executes motion primitives on command.

### 2. No motion skill planner
There is no higher-level planner that decides when to:
- greet a visitor
- pause at a stop
- back up from a blocked route
- recover after a failed movement
- chain motion skills into a scripted behavior

### 3. No direct semantic-nav integration beyond status gating
The current link to semantic navigation is only the status subscription.

Missing pieces:
- consuming taught places or route metadata
- selecting a skill based on route context
- exposing route-aware recovery or scripted motion sequences

### 4. No debate layer
This package has no perception / safety / twin / maintenance / verification role structure.
Those decisions belong elsewhere and are not represented here.

## Current Interpretation

This package should be treated as a thin execution backend:
- good for concrete SDK2 skills
- good for low-level actuation
- not yet a behavioral layer

## Best Next Steps

1. Add a command schema that can carry richer intent than a flat skill string.
2. Add a route-aware wrapper that can request skills from a patrol state machine.
3. Add explicit motion sequences for greeting, pause, blocked-route recovery, and resume.
4. Keep the current navigation-active safety gate.
5. Let higher-level packages decide what to do and use this package to execute it.

## Files To Start From

- [go2_agentic_motion_skills/motion_skill_agent_node.py](go2_agentic_motion_skills/motion_skill_agent_node.py)
- [launch/motion_skills.launch.py](launch/motion_skills.launch.py)
- [setup.py](setup.py)
