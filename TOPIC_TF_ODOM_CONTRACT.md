# Topic, TF, and Odom Contract

## Required robot/sim topic contract

```text
/tf
/tf_static
/odom
/imu
/joint_states
/camera/image_raw
/camera/camera_info
/scan
/point_cloud2
/cmd_vel_out
```

## Frame policy

```text
odom = live local motion truth
map = persistent memory truth
```

Use odom for local control, velocity, drift, trajectory logging, slip evidence, and short-term dead reckoning.

Use map-frame pose for spawn, checkpoints, places, tour stops, graph nodes, semantic voxel alignment, resume initial pose, and durable memory.

## Checkpoint pose schema

Every checkpoint can include both map and odom snapshots:

```yaml
pose:
  canonical_frame: map
  map_pose:
    frame_id: map
    x: 0.0
    y: 0.0
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 0.0
    qw: 1.0
  odom_pose:
    frame_id: odom
    x: 0.0
    y: 0.0
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 0.0
    qw: 1.0
  velocity_odom:
    linear_x: 0.0
    linear_y: 0.0
    linear_z: 0.0
    angular_x: 0.0
    angular_y: 0.0
    angular_z: 0.0
  tf_snapshot:
    map_to_odom: null
    odom_to_base: null
    timestamp: null
  confidence:
    map_pose_confidence: 0.0
    odom_confidence: 0.0
    localization_confidence: 0.0
```

## Duplicate publisher rule

Do not run duplicate SLAM, AMCL, Nav2, RViz, or `map -> odom` publishers. Base mode should be sensor/driver only, teach should be SLAM only, resume should be localization/Nav2 only.
