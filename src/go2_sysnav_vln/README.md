# go2_sysnav_vln 0.2.0

An additive SysNav-aligned object-navigation overlay for the Unitree Go2
`explore_mode` branch. It preserves the existing SLAM/localization, Nav2,
motion-arbiter, and collision-monitor ownership boundaries.

This package ports the **publicly observable SysNav cooperation pattern** into
the existing Go2/Nav2 software contract. It is not a byte-for-byte port of the
upstream TARE planner, custom sensor rig, or distributed ROS graph.

## What changed after auditing the public SysNav code

The first patch treated the VLM like a generic tool-calling frontier agent. The
public SysNav code instead uses separate, schema-constrained semantic queries
inside a deterministic hierarchy. Version 0.2.0 therefore implements:

1. Structured instruction decomposition.
2. Room classification and room-ID selection.
3. Classical frontier exploration inside the selected room.
4. Persistent object candidates built from map-frame geometry.
5. Structured target verification and early stopping.
6. Nav2-only physical execution with path preflight.

The model never emits velocity, coordinates, frontier geometry, or generic tool
calls.

## Architecture

```text
language instruction + current RGB
        |
        v
structured Ollama queries
  - target_object
  - room_condition
  - spatial_condition
  - attribute_condition
  - anchor_object
  - anchor attributes
        |
        v
room/object semantic state machine
  - classify current room
  - select one supplied room_id
  - verify one supplied object_id
        |
        +---------------------------+
        |                           |
        v                           v
room graph                    persistent object map
  - free-space room cores       preferred geometry:
  - propagated room labels      image-aligned organized cloud
  - room objects/frontiers      -> map-frame 3D points/extents
  - room visit state            -> adaptive same-class fusion
        |                        -> novel-view confirmations
        v                           |
classical in-room frontier planner |
  - one-cell frontiers retained    |
  - connected components           |
  - known-free standoff            |
  - clearance/reachability         |
  - tracked frontier viewpoints    |
        +-------------+-------------+
                      v
safe deterministic pose selection
                      |
                      v
/go2_nav/command
  -> ComputePathToPose preflight
  -> NavigateToPose
  -> motion arbiter
  -> collision monitor
  -> robot
```

## Object mapping and duplicate suppression

The preferred mapping input contains one of:

- `points_map`: registered object points in the `map` frame.
- `centroid_map` plus `bbox3d`.
- An organized, image-aligned `PointCloud2` projected by
  `registered_cloud_object_projector`.

The mapper then:

- Fuses only same-class instances.
- Uses object extent to adapt the direct merge distance.
- Uses 2D map-footprint IoU/overlap as a second merge test.
- Stores every observation in SQLite.
- Counts a new confirmation only for a novel object-relative view.
- Uses the upstream-style default novelty thresholds of 5 degrees and 0.3 m.
- Adds a robot-pose gate so simultaneous detector streams at one pose do not
  manufacture independent evidence.

A 2D detection plus `/scan_nav` remains available as a degraded fallback. It is
not equivalent to registered 3D object geometry and should not be treated as
high-confidence mapping for tabletop or non-scan-plane objects.

## Registered point-cloud projector

Enable this only when the point cloud is organized and pixel-aligned with the
RGB detector image:

```bash
ros2 launch go2_sysnav_vln sysnav_vln.launch.py \
  enable_registered_cloud_projector:=true \
  organized_cloud_topic:=/camera/depth/color/points \
  enable_motion:=false
```

The node refuses:

- Unorganized clouds.
- Cloud/image dimension mismatch.
- Stale clouds.
- Missing cloud-to-map transforms.

A raw Go2 LiDAR cloud is not pixel-aligned with a monocular image. A calibrated
camera-LiDAR fusion node must generate the registered cloud first.

## Frontier and room logic

The frontier planner does not send arithmetic frontier centroids. It:

- Builds frontier cells from known-free cells adjacent to unknown cells.
- Preserves narrow, one-cell-wide frontiers.
- Clusters connected frontiers.
- Searches inward for a known-free standoff goal.
- Checks robot clearance and map-connected reachability.
- Scores information gain, geodesic distance, clearance, and heading.
- Tracks frontier identity between map updates.
- Increases frontier stability only from a novel robot perspective.
- Assigns each frontier and object to a free-space room component.
- Publishes a room graph for high-level semantic selection.

## Ollama contract

The supervisor calls `/api/chat` with a JSON Schema `format`. It uses structured
outputs rather than free-form tool calls. Default model:

```text
gemma4:e4b
```

Install and start Ollama:

```bash
ollama pull gemma4:e4b
ollama serve
```

The model must support the requested image and structured-output features. Set
`send_image_to_vlm:=false` when using a text-only model.

## Build

```bash
cd ~/Dhruv/sparky/ros2_ws
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate

python -m colcon build --symlink-install \
  --packages-select go2_robot_sdk go2_nav_tools go2_object_explorer go2_sysnav_vln \
  --cmake-args -DPython3_EXECUTABLE=$VIRTUAL_ENV/bin/python -Wno-dev

source install/setup.bash
```

## Dry-run first

```bash
export ROBOT_IP=192.168.12.1
export CONN_TYPE=webrtc

ros2 launch go2_sysnav_vln sysnav_vln.launch.py \
  enable_motion:=false \
  auto_start:=false \
  enable_registered_cloud_projector:=false
```

Submit an instruction:

```bash
ros2 topic pub --once /go2_vln/goal std_msgs/msg/String \
  "{data: '{\"instruction\":\"find the blue trash can in the classroom\"}'}"
```

Start the paused mission explicitly:

```bash
ros2 topic pub --once /go2_vln/command std_msgs/msg/String \
  "{data: '{\"action\":\"start\"}'}"
```

Inspect:

```bash
ros2 topic echo /go2_vln/target_spec
ros2 topic echo /go2_vln/state
ros2 topic echo /go2_vln/decision
ros2 topic echo /go2_vln/room_graph
ros2 topic echo /go2_vln/object_map
ros2 topic echo /go2_vln/frontier_candidates
```

RViz markers:

```text
/go2_vln/object_markers
/go2_vln/frontier_markers
/go2_vln/room_markers
```

With `enable_motion=false`, the supervisor records decisions but cannot dispatch
physical navigation.

## Physical motion gate

Enable motion only after the base stack passes lifecycle, manual-goal,
orientation, recovery, TF-time, and collision-monitor acceptance tests:

```bash
ros2 launch go2_sysnav_vln sysnav_vln.launch.py \
  enable_motion:=true \
  auto_start:=false
```

Emergency mission stop/cancel:

```bash
ros2 topic pub --once /go2_vln/command std_msgs/msg/String \
  "{data: '{\"action\":\"stop\"}'}"
```

## Do not run concurrently

Do not run another frontier executor, MRKL motion loop, legacy object explorer,
or `safe_agentic_explorer.launch.py` at the same time. There must be exactly one
mission supervisor and one Nav2 tool server. Set
`launch_nav2_tool_server:=false` when a verified safe server is already running.

## Remaining gap from upstream SysNav

This package does not include upstream TARE, its custom room/navigation message
interfaces, the Ricoh/Livox calibration stack, or the exact distributed
base-station deployment. The portable correspondence is:

```text
upstream semantic reasoning  -> structured Ollama room/object stages
upstream room hierarchy      -> occupancy-derived room graph
upstream in-room exploration -> safe Nav2 frontier standoff planner
upstream semantic map        -> SQLite map-frame object fusion
upstream motion control      -> existing Go2 Nav2/arbiter/collision chain
```

## RViz integration

`sysnav_vln.launch.py` owns one RViz instance and loads `config/sysnav_vln.rviz`.
The included base launch is forced to `launch_rviz:=false` to avoid duplicate RViz
processes. The SysNav display enables:

- `/go2_vln/object_markers` — confirmed map-frame object points, extents, and labels.
- `/go2_vln/frontier_markers` — actual frontier boundary samples and safe standoff goals.
- `/go2_vln/room_markers` — room labels and exploration counts.

All three marker publishers use reliable transient-local QoS so RViz receives the
latest marker set even when it starts after the mapper/planner. Legacy
`/object_explorer/*` markers are disabled in this RViz profile.
