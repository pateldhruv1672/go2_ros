# Robust Patrol, Recovery, and Real Motion Skills for Sparky

## Summary
We’ll turn the current stack into a route-aware robot that can patrol, explain stops, recover from blocked paths, and continue autonomously even when the first Nav2 goal fails. The main behavior owner will remain `go2_semantic_nav_agent`, with `go2_agentic_multiagent_voice_nav` as the human interface, `go2_agentic_motion_skills` as the actuation layer, and `council` as the debate/verification brain.

## Key Changes
- Replace the current “place-only” resume flow with a true patrol engine:
  - persist routes, stops, stop scripts, safe anchors, and resume points separately from raw `places.yaml`
  - use VLM data during resume to choose the next goal, not just to label the scene
  - support known-area replay and partial unknown-area exploration instead of assuming the user pre-recorded every destination
- Add a nav failure diagnosis and recovery loop:
  - classify failures into localization loss, map mismatch, dynamic blockage, semantic ambiguity, or goal-inaccessible
  - inspect AMCL health, TF availability, scan/costmap state, route context, and VLM/VPR confidence before choosing a recovery action
  - choose one of: re-localize in place, retreat to a safe anchor, rotate-and-reobserve, switch to a local visual waypoint mode, request verification, or hand off to a safe stop
- Make the semantic nav fallback behavior route-aware:
  - if Nav2 fails, do root-cause analysis instead of just retrying the same goal
  - route execution should be able to move the robot to a safe staging location and then relaunch Nav2 from there
  - if the environment is only partially known, use a local VLM/VPR waypoint proposal plus Nav2 execution, rather than hard failing
- Rework `go2_agentic_motion_skills` into a real behavior library:
  - replace placeholder-style motion handling with explicit, validated action sequences
  - model real pose/action scripts like greet, stand, sit, recovery stand, pause, wait, attention-getting motions, and blocked-route recovery
  - keep safety gating while letting higher-level behaviors request composed motion sequences instead of single flat skill strings
- Add a pure VLM navigation mode:
  - use VLM proposals for next waypoint / stop / recovery suggestion
  - keep keyboard teleop only as a manual override, emergency fallback, or data-collection tool
  - build it in the style of a research system, not an operator toy: propose, verify, execute, re-evaluate
- Implement or mock the five debate roles:
  - perception, safety, twin, maintenance, verification
  - each role emits structured JSON/YAML including `event_worthy`, `label`, `replay_variations[]`, and `update_strength`
  - route these roles through a single structured decision object rather than ad hoc log text
- Add the verification and twin pipeline:
  - capture close-up RGB, depth, manipulation cues, thermal, and IMU-derived evidence for human/humanoid verification
  - convert verified incidents into twin variants covering lighting, occlusion, pose, clutter, and geometry
  - label outputs as `pos`, `near-miss`, or `hard-neg`
- Add the full lifecycle:
  - patrol detection → verification dispatch → twin replay → lightweight update → redeployment evaluation
  - require at least one incident class to improve before accepting a detector update
  - measure recall gain, calibration, and false-positive control against baseline
- Keep `go2_agentic_system` thin:
  - it should only compose launches and parameters
  - no duplicated behavior logic should move into the wrapper
- Update the shared operator workflow:
  - maintain a concise weekly Friday entry with decisions, blockers, commits, evidence links, and next steps
  - keep the existing handoff docs as the live project memory

## Test Plan
- Route state tests:
  - verify patrol creation, stop ordering, safe anchor persistence, pause/resume, and blocked-route recovery
  - verify resume mode restores route state, not just map/spawn pose
- Recovery tests:
  - force Nav2 failure and confirm the agent diagnoses the root cause before retrying
  - confirm the agent can retreat to a safe anchor and re-attempt navigation from a better state
  - confirm “known area” and “partially known area” flows do not collapse into the same retry path
- VLM/VPR tests:
  - confirm VLM proposals carry 3D/map coordinates and confidence
  - confirm low-confidence or inconsistent proposals are rejected or routed to verification
  - confirm the system can use visual place recognition to relocalize or verify a location estimate
- Motion skills tests:
  - verify each named behavior maps to a real SDK-backed pose/action sequence
  - verify no placeholder-only skill remains in the motion package
  - verify navigation safety gating still prevents conflicts
- Debate and verification tests:
  - verify all five debate roles emit structured outputs
  - verify the human/humanoid verification flow consumes RGB + depth + manipulation + thermal/IMU evidence
  - verify incident replay variants are generated and labeled correctly
- System tests:
  - verify autonomous patrol, stop explanation, pause, blocked-route recovery, and tour continuation in one demo run
  - verify detector update acceptance requires measurable improvement over baseline on at least one incident class
  - verify no duplicate RViz or brittle hard-coded workspace launch paths are introduced

## Research Anchors
- Visual-language waypointing and map grounding: [Online Visual Language Mapping for real-world VLN](https://arxiv.org/abs/2310.10822)
- Teach-and-repeat style navigation with visual place recognition: [Multi-Platform Teach-and-Repeat Navigation](https://arxiv.org/abs/2503.13090)
- VPR integrity monitoring for safer localization decisions: [Improving Visual Place Recognition Based Robot Navigation By Verifying Localization Estimates](https://arxiv.org/abs/2407.08162)
- Safe motion planning under dynamic obstacles: [CIAO* MPC-based Safe Motion Planning](https://arxiv.org/pdf/2001.05449)
- Fast, robust place recognition for relocalization: [Self-Supervised Visual Place Recognition Learning in Mobile Robots](https://arxiv.org/abs/1905.04453)

## Assumptions
- `go2_semantic_nav_agent` will own route/patrol state and recovery logic.
- `council` will remain the multi-agent debate and verification engine rather than being duplicated in the nav packages.
- `go2_agentic_motion_skills` will stay as the actuation backend, but its skills will become real, validated sequences instead of boilerplate wrappers.
- `go2_agentic_system` stays a launch/integration layer only.
- Nav2 tuning will be updated in `/home/digital-twin-admin/Dhruv/sparky/ros2_ws/src/go2_robot_sdk/config/nav2_params.yaml` to support recovery, relocalization, and blocked-route handling rather than only nominal success.
- Keyboard teleop remains available, but only as fallback and operator override, not the primary autonomy path.
