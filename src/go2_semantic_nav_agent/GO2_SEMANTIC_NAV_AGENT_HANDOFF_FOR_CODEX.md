# go2_semantic_nav_agent Handoff for Codex

Date: 2026-06-05

This package is the closest thing in the workspace to a teach/resume navigation product. It already has saved sessions, places, semantic memory, a teach mode, a resume mode, and fallback recovery. It is still not a full visitor-tour system, but it is the most route-relevant package in the repository.

## What Is Implemented

### 1. Teach and resume runtime modes
The main node is [semantic_nav_node.py](go2_semantic_nav_agent/semantic_nav_node.py).

It already supports:
- teach mode
- resume mode
- session selection
- restoring a saved spawn pose
- saving and reloading map / places / session metadata

### 2. Persistent session and place storage
The package already has:
- [session_store.py](go2_semantic_nav_agent/session_store.py)
- [place_store.py](go2_semantic_nav_agent/place_store.py)
- [semantic_memory.py](go2_semantic_nav_agent/semantic_memory.py)

These provide a real saved-session model with:
- session directories
- `session.yaml`
- `places.yaml`
- map files
- semantic matching from spoken labels into places

### 3. Route-adjacent teach behavior exists
In teach mode the node can:
- auto-save places over time
- use VLM-assisted labeling
- publish markers and route previews
- store session artifacts for later resumption

This is much closer to a route memory system than the other packages.

### 4. Resume path exists
The resume launcher [semantic_nav_resume.launch.py](launch/semantic_nav_resume.launch.py) already restores:
- the saved map server
- AMCL
- the semantic nav node in resume mode
- the saved spawn pose when TF is ready

That is a concrete resume implementation, not just a command alias.

### 5. Fallback recovery exists
The node includes a fallback behavior path for navigation failures, including retry logic and motion recovery.

## What Is Not Done Yet

### 1. Teach mode still captures places more than behaviors
This package stores useful semantic data, but it still does not fully learn a patrol behavior.

Missing pieces:
- explicit tour stops with scripts
- route sequencing across multiple stops
- visitor-facing explanations tied to place transitions
- blocked-route recovery as a first-class behavior

### 2. Resume mode is session-resume, not full behavior-resume
The current resume logic restores map and pose state and resumes navigation from saved sessions.

What is still missing:
- resuming a behavioral script mid-tour
- resuming dialogue state
- resuming a partially completed patrol with explicit stop-state memory

### 3. Semantic memory is not yet a complete policy engine
The package can resolve places and save labels, but it does not yet turn those labels into a full patrol controller.

Missing pieces:
- route templates learned from prior runs
- stop-level policies
- confidence-aware behavior selection from VLM labels

### 4. No debate-agent layer
Even though this package is route-adjacent, it still does not contain the requested debate roles:
- perception
- safety
- twin
- maintenance
- verification

## Current Interpretation

This package is the best foundation for teach/resume navigation, but it is still only a foundation.

It already knows how to:
- save sessions
- remember places
- reload maps
- restore spawn
- run teach vs resume

What it does not yet know is how to behave like a full Sparky tour controller with scripts, explanations, and blocked-route recovery.

## Best Next Steps

1. Add a route model on top of `PlaceStore` and `SessionStore`.
2. Add explicit patrol stops and stop scripts.
3. Preserve the current teach/resume flows, but extend them to store route behavior, not only geometry and labels.
4. Connect semantic memory to route choice and blocked-route recovery.
5. Add a clean output contract so higher-level dialogue or council layers can ask this package for the next behavior step.

## Files To Start From

- [go2_semantic_nav_agent/semantic_nav_node.py](go2_semantic_nav_agent/semantic_nav_node.py)
- [go2_semantic_nav_agent/session_store.py](go2_semantic_nav_agent/session_store.py)
- [go2_semantic_nav_agent/place_store.py](go2_semantic_nav_agent/place_store.py)
- [go2_semantic_nav_agent/semantic_memory.py](go2_semantic_nav_agent/semantic_memory.py)
- [launch/semantic_nav_teach.launch.py](launch/semantic_nav_teach.launch.py)
- [launch/semantic_nav_resume.launch.py](launch/semantic_nav_resume.launch.py)
- [launch/semantic_nav_rviz.launch.py](launch/semantic_nav_rviz.launch.py)
