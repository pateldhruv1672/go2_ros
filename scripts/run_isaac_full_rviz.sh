#!/usr/bin/env bash
set -euo pipefail

WS="/home/digital-twin-admin/Dhruv/sparky/ros2_ws"
CFG="$WS/config/isaac_full_real_go2.rviz"

cd "$WS"
mkdir -p config

set +u
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

RVIZ_FIXED_FRAME="${RVIZ_FIXED_FRAME:-odom}"

cat > "$CFG" <<EOF
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
      Plane: XY
      Reference Frame: <Fixed Frame>
      Cell Size: 1
      Plane Cell Count: 30

    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
      Show Axes: true
      Show Names: false
      Update Interval: 0

    - Class: rviz_default_plugins/RobotModel
      Enabled: true
      Name: RobotModel
      Description Topic:
        Value: /robot_description
      Visual Enabled: true
      Collision Enabled: false
      Update Interval: 0

    - Class: rviz_default_plugins/Odometry
      Enabled: true
      Name: Odometry
      Topic:
        Value: /odom
      Shape:
        Value: Arrow
      Keep: 200

    - Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: LaserScan /scan
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /scan
      Size (m): 0.04
      Style: Points
      Decay Time: 0

    - Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: LaserScan /scan_nav
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /scan_nav
      Size (m): 0.04
      Style: Points
      Decay Time: 0

    - Class: rviz_default_plugins/PointCloud2
      Enabled: true
      Name: PointCloud2 /point_cloud2
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /point_cloud2
      Size (m): 0.03
      Style: Points
      Decay Time: 0

    - Class: rviz_default_plugins/Image
      Enabled: true
      Name: Camera /camera/image_raw
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /camera/image_raw

    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Map /map
      Topic:
        Value: /map
      Update Topic:
        Value: /map_updates
      Alpha: 0.7
      Color Scheme: map

    - Class: rviz_default_plugins/Path
      Enabled: true
      Name: Nav2 /plan
      Topic:
        Value: /plan
      Line Style: Lines
      Line Width: 0.05

    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Semantic Places
      Marker Topic:
        Value: /semantic_nav/places_markers

    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Semantic Route Preview
      Marker Topic:
        Value: /semantic_nav/route_preview

  Enabled: true
  Global Options:
    Fixed Frame: $RVIZ_FIXED_FRAME
    Frame Rate: 30
    Background Color: 48; 48; 48
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Value: /goal_pose
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 8
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Pitch: 0.785
      Yaw: 0.785
EOF

echo "[run_isaac_full_rviz] Fixed frame: $RVIZ_FIXED_FRAME"
echo "[run_isaac_full_rviz] Config: $CFG"

exec rviz2 -d "$CFG" --ros-args -p use_sim_time:=true
