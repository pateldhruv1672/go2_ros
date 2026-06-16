# Current Launch Contract

The new agentic packages are additive and must not replace the established launch sequence.

## Existing flows that must keep working

```bash
BASE_MODE=base bash scripts/run_robot_live.sh
BASE_MODE=teach bash scripts/run_robot_live.sh
BASE_MODE=resume bash scripts/run_robot_live.sh
```

Semantic teach remains:

```bash
ros2 launch go2_semantic_nav_agent semantic_nav_teach.launch.py \
  map_label:=digital_twin_lab \
  auto_save_places:=true \
  auto_save_interval_sec:=5.0 \
  auto_save_use_vlm:=true \
  semantic_rviz:=true \
  clear_places_on_start:=false \
  save_map_on_shutdown:=true
```

Semantic resume remains:

```bash
SESSION=$(basename "$(ls -td ~/.ros/go2_semantic_nav_sessions/* | head -1)")
ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py \
  session_name:=$SESSION \
  rviz2:=false
```

## New additive overlays

- `agentic_memory_stack.launch.py` starts memory/perception/agent nodes.
- `explore_mode.launch.py` starts the memory stack plus frontier/explore helpers.
- `tour_mode.launch.py` starts the memory stack plus tour supervisor.

## Safety defaults

- `enable_motion:=false` unless explicitly set.
- Nav commands publish status/dry-run records before sending goals.
- The memory stack never starts SLAM, AMCL, map server, or Nav2 lifecycle nodes.
- Resume mode must keep AMCL/localization as the only `map -> odom` authority.
