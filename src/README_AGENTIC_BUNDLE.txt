SPARKY AGENTIC FULL SYSTEM BUNDLE

Included:
- go2_semantic_nav_agent working baseline
- go2_agentic_multiagent_voice_nav (Omi + local faster-whisper + orchestrator + camera agent + TTS)
- go2_agentic_motion_skills (grounded Unitree SDK2 motion skills, no placeholder commands)
- go2_agentic_system (top-level launch)

Motion skills implementation:
- uses unitree_sdk2_python directly
- uses SportClient methods from the official SDK2 Python repo
- uses MotionSwitcherClient.SelectMode() from the official motionSwitcher example

Before building motion skills:
1) install unitree_sdk2_python from the official repo
2) if needed, set UNITREE_NET_IF or pass network_interface:=<iface>

Build:
  cd ~/ros2_ws/src
  unzip /path/to/this_bundle.zip
  cd ~/ros2_ws
  source /opt/ros/humble/setup.bash
  colcon build --packages-select go2_semantic_nav_agent go2_agentic_multiagent_voice_nav go2_agentic_motion_skills go2_agentic_system --symlink-install
  source install/setup.bash

Run motion + voice overlay:
  ros2 launch go2_agentic_system sparky_full_system.launch.py network_interface:=enp2s0
