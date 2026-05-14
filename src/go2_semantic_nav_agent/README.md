# go2_semantic_nav_agent

Full ROS 2 package for Go2 semantic teach/resume navigation with:
- teach sessions (`map.yaml`, `map.pgm`, `places.yaml`, `session.yaml`)
- spawn persistence and AMCL restore on resume
- semantic matching with synonym memory and graph relationships
- VLM-supervised fallback recovery using bounded teleop-like motion primitives

## Resume architecture
1. Start `go2_robot_sdk robot.launch.py foxglove:=false slam:=false nav2:=true`
2. Start `semantic_nav_resume.launch.py`
3. Resume launch starts its own `map_server`, `amcl`, lifecycle manager, `scan_retimestamp_node`, and `semantic_nav_node`
4. `semantic_nav_node` waits for odom TF, then publishes spawn to `/initialpose` with stamp zero
5. Nav2 handles normal navigation; on goal failure, semantic fallback can run short bounded recoveries and retry

## Important limitations
- Not hardware-verified in this archive. Python and launch files were syntax-checked only.
- VLM is used for supervisory recovery, not for direct motor control.
- Safety still depends on LiDAR, local costmaps, and bounded motion primitives.
