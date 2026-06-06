# Go2 ROS 2 Workspace

ROS 2 Jazzy workspace for Unitree Go2 navigation, SLAM, Nav2, collision monitor, and semantic navigation experiments.

## Build

```bash
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
colcon build --symlink-install
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

ros2 launch go2_robot_sdk robot.launch.py
```
