UPDATED ZIP CONTENTS

This zip fixes the broken go2_agentic_system package build by including:
- package.xml
- setup.py
- setup.cfg
- resource/go2_agentic_system
- launch/sparky_full_system.launch.py

What it launches
- go2_agentic_multiagent_voice_nav/multiagent_resume.launch.py
- go2_agentic_motion_skills/motion_skills.launch.py

It does NOT launch:
- go2_robot_sdk base stack
- go2_semantic_nav_agent resume stack
