#!/usr/bin/env bash
cat <<'EOF'
VOICE EXAMPLES after Tour Mode is running and localized:

At the welcome pose say:
  "Sparky, save this as welcome checkpoint. Say: Welcome to the Digital Twin Lab. I am Sparky, your robotic guide."

At the humanoid pose say:
  "Sparky, save this as humanoid checkpoint. Say: This checkpoint highlights our humanoid robotics work in embodied AI, perception, control, and human robot interaction."

At the robot-arms pose say:
  "Sparky, save this as roboarms checkpoint. Say: This checkpoint presents our robot arm work in manipulation, teleoperation, digital twins, perception, and control."

Then say:
  "Sparky, order the tour as welcome checkpoint, humanoid checkpoint, then roboarms checkpoint."

Navigation examples:
  "Sparky, go to humanoid checkpoint."
  "Sparky, start the tour."
  "Sparky, continue the tour."

Narration update without changing pose:
  "Sparky, update the humanoid checkpoint description to: <new narration>."
EOF
