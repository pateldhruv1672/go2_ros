# Sparky Handoff

## Current State

This workspace is a ROS 2 Jazzy stack for Sparky, a Unitree Go2, with:
- base robot bringup
- SLAM/teach flow
- semantic resume flow using saved sessions
- Nav2 no-docking bringup
- RViz overlays for semantic places and route checkpoints

The latest active semantic session used during live testing was:
- `lab_live_teach_20260608_151525`

Saved session artifacts live under:
- `~/.ros/go2_semantic_nav_sessions/`

## What Is Working

- Teach mode can save:
  - `session.yaml`
  - `route.yaml`
  - `places.yaml`
  - saved `spawn`
- Resume mode now restores spawn through `/initialpose`
- Semantic resume owns:
  - saved map loading
  - AMCL
  - semantic RViz
  - trimmed Nav2 include
- Base resume bringup is now driver-first and does not also launch its own Nav2/RViz stack
- The resume route state is reset from stale active states to `paused` on startup
- `behavior_server` was restored so `bt_navigator` can load BT actions like `spin`

## Known Good Commands

### Build only the changed packages

Full workspace `colcon build` is currently blocked by a dependency cycle. Use isolated package builds:

```bash
cd ~/Dhruv/sparky/ros2_ws
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
colcon build --symlink-install --base-paths src/go2_robot_sdk
colcon build --symlink-install --base-paths src/go2_semantic_nav_agent
source install/setup.bash
```

### Teach mode

```bash
cd ~/Dhruv/sparky/ros2_ws
BASE_MODE=teach bash scripts/run_robot_live.sh
```

Then:

```bash
bash scripts/run_semantic_nav_teach.sh
```

Useful teach commands:

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'save_spawn'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'save_map'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'places'}"
```

### Resume mode

Bring up the base stack first:

```bash
cd ~/Dhruv/sparky/ros2_ws
BASE_MODE=resume bash scripts/run_robot_live.sh
```

Then bring up semantic resume:

```bash
cd ~/Dhruv/sparky/ros2_ws
bash scripts/run_semantic_nav_resume.sh
```

Send navigation commands:

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'places'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'go spawn'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'go digital_twin_lab_center'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'pause_tour'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'status'}"
```

## Important Ownership Split

### Base bringup in resume mode

`scripts/run_robot_live.sh` with `BASE_MODE=resume` now defaults to:
- `slam:=false`
- `nav2:=false`
- `rviz2:=false`

This is intentional.

The base stack should provide:
- robot driver
- robot state publisher
- sensor streams
- TF base pipeline

### Semantic resume overlay

`scripts/run_semantic_nav_resume.sh` / `semantic_nav_resume.launch.py` now own:
- saved map server
- AMCL
- semantic Nav2 include
- semantic RViz
- semantic nav node

This avoids having two localization or Nav2 owners fighting each other.

## Important Files Changed

- `scripts/run_robot_live.sh`
- `scripts/run_semantic_nav_resume.sh`
- `src/go2_semantic_nav_agent/launch/semantic_nav_resume.launch.py`
- `src/go2_robot_sdk/launch/navigation_no_docking.launch.py`
- `src/go2_robot_sdk/config/nav2_params.yaml`
- `src/go2_semantic_nav_agent/go2_semantic_nav_agent/semantic_nav_node.py`
- `src/go2_semantic_nav_agent/config/semantic_nav.rviz`
- `src/go2_robot_sdk/go2_robot_sdk/infrastructure/ros2/ros2_publisher.py`

## Known Issues Still Open

### 1. Full workspace build cycle

There is still a pre-existing dependency cycle involving:
- `go2_agentic_multiagent_voice_nav`
- `go2_agentic_system`
- `go2_semantic_nav_agent`

So full `colcon build --symlink-install` is not currently the reliable path.

### 2. Camera image QoS mismatch

There is still an intermittent QoS mismatch on `/camera/image_raw`.

Symptoms:
- RViz may show `No Image`
- logs may report incompatible reliability QoS

Current semantic RViz camera settings are intended to be:
- topic: `/camera/image_raw`
- reliability: `Best Effort`
- durability: `Volatile`

There may still be another subscriber in the live graph requesting `Reliable`.

### 3. TTS noise

`tts_node` still complains about missing ElevenLabs API key in some bringup flows.

The intended direction is on-device TTS only, but the noisy cloud-TTS path has not been fully removed from all launch combinations yet.

### 4. Final live nav verification after latest Nav2 fix

After restoring `behavior_server`, Nav2 lifecycle looked healthier, but the final live `go <place>` confirmation should still be re-run and watched from fresh logs.

## Recent Root Causes Already Fixed

- Semantic resume script used to kill the base stack because it matched generic `rviz2` in `pkill`
- Resume used to republish `/initialpose` repeatedly, causing flicker
- Resume used to overlap with base SLAM/Nav2 ownership
- `bt_navigator` failed because `behavior_server` was removed, making `spin` unavailable
- Resume route state could restart in `moving_to_stop` instead of a safe paused state
- Raw image publishing could fail if camera calibration publishing failed

## Recommended Next Steps

1. Relaunch fresh in resume mode:
   - `BASE_MODE=resume bash scripts/run_robot_live.sh`
   - `bash scripts/run_semantic_nav_resume.sh`
2. Confirm Nav2 lifecycle reaches active state
3. Run:
   - `go spawn`
   - `go digital_twin_lab_center`
4. Watch for:
   - `navigate_to_pose` availability
   - planner/controller feedback
   - AMCL staying on-map after spawn restore
5. If camera is still blank:
   - inspect live QoS on `/camera/image_raw`
   - identify which subscriber is still requesting incompatible reliability
6. Clean up TTS launch path so on-device speech is the only active output path

## Notes For The Next Person

- Do not assume the full workspace builds cleanly. Build only the changed packages unless the dependency cycle is resolved.
- For resume mode, do not launch SLAM.
- For resume mode, do not let base bringup own Nav2 or RViz.
- If the robot appears off-map, first check:
  - whether the saved `spawn` is sensible in `session.yaml`
  - whether only one `/initialpose` restore happened
  - whether the correct saved map is loaded
  - whether AMCL and the semantic map server are the only localization/map owners
