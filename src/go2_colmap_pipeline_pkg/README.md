# go2_colmap_pipeline

A starter ROS 2 package that turns a Go2 rosbag into a COLMAP dataset, runs a reconstruction, and prepares the reconstructed environment for Isaac Sim.

## What it does

- Exports images from a rosbag2 recording into `dataset/images`
- Saves camera intrinsics into `dataset/camera_info.yaml`
- Saves image timestamps and nearest odometry poses into `dataset/image_metadata.csv`
- Writes `dataset/ref_images.txt` for COLMAP `model_aligner`
- Runs a sparse and optional dense COLMAP reconstruction
- Converts the resulting mesh into OBJ if needed
- Includes an Isaac Sim script that converts the mesh into USD and saves a stage

## Workspace layout

Put this package beside `go2_ros2_sdk` in your ROS 2 workspace:

```text
ros2_ws/
  src/
    go2_ros2_sdk/
    go2_colmap_pipeline/
```

Then build it:

```bash
cd ~/ros2_ws
colcon build --packages-select go2_colmap_pipeline
source install/setup.bash
```

## ROS 2 dependencies

You need the normal Python ROS 2 runtime plus:

```bash
sudo apt install python3-opencv ros-$ROS_DISTRO-cv-bridge
```

For mesh conversion outside Isaac Sim:

```bash
pip install open3d
```

## Record a bag from the robot

Launch the robot stack and record the topics COLMAP needs:

```bash
ros2 launch go2_robot_sdk robot.launch.py

ros2 bag record \
  /camera/image_raw \
  /camera/camera_info \
  /odom \
  /tf \
  /tf_static
```

## Export a COLMAP dataset

```bash
ros2 run go2_colmap_pipeline bag_to_dataset \
  --bag /data/go2_bag \
  --output-dir /data/go2_dataset \
  --image-topic /camera/image_raw \
  --camera-info-topic /camera/camera_info \
  --odom-topic /odom \
  --camera-info-yaml ~/ros2_ws/src/go2_ros2_sdk/go2_robot_sdk/calibration/front_camera.yaml \
  --camera-extrinsics ~/ros2_ws/src/go2_colmap_pipeline/config/example_camera_extrinsics.yaml \
  --min-dt-sec 0.25
```

Notes:

- If `/camera/camera_info` is missing in the bag, the package can fall back to the Go2 calibration YAML.
- Odometry is matched by nearest timestamp and used only to create position priors for metric alignment.
- `example_camera_extrinsics.yaml` is a placeholder. Replace it with your measured base-to-camera transform.

## Run COLMAP

```bash
ros2 run go2_colmap_pipeline run_colmap \
  --dataset-dir /data/go2_dataset \
  --workspace-dir /data/go2_colmap \
  --align-with-priors
```

This produces:

```text
/data/go2_colmap/
  database.db
  sparse/
  sparse_aligned/
  dense/
    fused.ply
    scene_poisson.ply
```

### Optional: convert the mesh to OBJ

```bash
ros2 run go2_colmap_pipeline mesh_to_obj \
  --input-mesh /data/go2_colmap/dense/scene_poisson.ply \
  --output-obj /data/go2_colmap/dense/scene_poisson.obj
```

## End-to-end command

```bash
ros2 run go2_colmap_pipeline full_pipeline \
  --bag /data/go2_bag \
  --dataset-dir /data/go2_dataset \
  --workspace-dir /data/go2_colmap \
  --align-with-priors
```

## Isaac Sim import

Run the included script from Isaac Sim's Python environment, for example with `./python.sh` from the Isaac Sim root:

```bash
./python.sh /path/to/go2_colmap_pipeline/scripts/isaacsim_import_colmap_env.py \
  --input-asset /data/go2_colmap/dense/scene_poisson.obj \
  --converted-usd /data/isaac/scene_poisson_asset.usd \
  --stage-usd /data/isaac/go2_environment.usd
```

That will:

- convert the mesh into a USD asset
- create a stage at `/World`
- reference the converted asset at `/World/Environment`
- save the stage for later use in Isaac Sim

## Suggested next improvements

- Replace nearest-pose matching with interpolation over `/odom` or `/tf`
- Add a dedicated TF lookup path for camera-frame priors
- Generate a simplified collision mesh for Isaac Sim physics
- Add texture export and material cleanup for better rendering
- Add Docker or Conda environment files for COLMAP + Open3D

## Assumptions in this starter package

- Images come from one forward-facing camera topic.
- Position priors come from `/odom`.
- Scale recovery depends on reasonable odometry and camera extrinsics.
- The Isaac Sim script focuses on asset conversion and stage creation, not full physics setup.
