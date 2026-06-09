# README.md

This pack is for Codex in VS Code.

## How to use
1. Copy `AGENTS.md` to the repository root.
2. Copy `.agents/skills/` into the repository root.
3. Keep `CODEX_CONTINUE_WORK.md` in the repo root for project-specific handoff context.

## Recommended repo layout
```text
repo-root/
  AGENTS.md
  CODEX_CONTINUE_WORK.md
  .agents/
    skills/
      ros2-clean-restart/
      nav2-goal-frame-debug/
      session-audit/
      omi-mic-fallback/
      rviz-single-instance/
      wireless-foundation-guardrails/
```

## Notes
- AGENTS.md gives Codex project-level instructions.
- Skills are reusable task guides.
- This pack is tailored to Go2 robotics development with a semantic-nav + agentic overlay.
