# wireless-foundation-guardrails

## When to use
Use whenever changing architecture, runtime, or motion integrations.

## Guardrails
- `go2_ros2_sdk` is the foundation
- keep wireless/WebRTC runtime as the main robot interface
- do not replace the base runtime with direct SDK2 unless the user explicitly asks
- treat `unitree_sdk2_python` as optional or sidecar behavior, not the main transport

## Overlay model
- base runtime: `go2_ros2_sdk`
- semantic nav: `go2_semantic_nav_agent`
- voice/orchestration: `go2_agentic_multiagent_voice_nav`
- motion skills: `go2_agentic_motion_skills`

## Change strategy
- prefer overlays over deep changes to the foundation
- keep voice and nav loosely coupled
- keep changes reversible
