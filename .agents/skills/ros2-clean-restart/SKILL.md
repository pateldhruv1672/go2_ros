# ros2-clean-restart

## When to use
Use when ROS state is stale, nodes are duplicated, or launches are behaving inconsistently.

## Steps
```bash
pkill -f "go2_driver_node|robot_state_publisher|slam_toolbox|amcl|map_server|rviz2|semantic_nav|bt_navigator|planner_server|controller_server|lifecycle_manager|foxglove_bridge|local_omi_stt_node|dialogue_orchestrator_node|camera_agent_node|speaker_tts_node" || true
sleep 3

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 daemon stop
ros2 daemon start
ros2 node list
ros2 topic list
```

## Success check
- old nodes are gone
- `ros2 node list` and `ros2 topic list` reflect only the current run
