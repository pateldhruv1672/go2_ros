import argparse
import os
import time

os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "17")
os.environ["ROS_DISTRO"] = os.environ.get("ROS_DISTRO", "jazzy")
os.environ["RMW_IMPLEMENTATION"] = os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Isaac RTX LiDAR ROS2 smoke test")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni
import omni.kit.commands
import omni.replicator.core as rep
from pxr import Gf, UsdGeom, UsdPhysics

try:
    from isaacsim.core.utils.extensions import enable_extension
except Exception:
    from omni.isaac.core.utils.extensions import enable_extension

print("[RTX_SMOKE] ROS_DOMAIN_ID =", os.environ.get("ROS_DOMAIN_ID"), flush=True)
print("[RTX_SMOKE] Enabling ROS2 bridge + RTX sensor extensions", flush=True)

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
simulation_app.update()
simulation_app.update()

stage = omni.usd.get_context().get_stage()

# Add a ground plane.
UsdGeom.Xform.Define(stage, "/World")
ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
ground.AddScaleOp().Set(Gf.Vec3d(20.0, 20.0, 0.02))
ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.02))
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

# Add obstacle so lidar has returns.
cube = UsdGeom.Cube.Define(stage, "/World/obstacle_cube")
cube.CreateSizeAttr(1.0)
cube.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 0.5))
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

print("[RTX_SMOKE] Creating 2D RTX lidar", flush=True)
ok_2d, sensor_2d = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/World/base_scan_2d",
    parent=None,
    config="Example_Rotary_2D",
    translation=(0.0, 0.0, 0.35),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
)
print("[RTX_SMOKE] 2D lidar ok:", ok_2d, "prim:", sensor_2d, flush=True)

print("[RTX_SMOKE] Creating 3D RTX lidar", flush=True)
ok_3d, sensor_3d = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/World/base_scan_3d",
    parent=None,
    config="Example_Rotary",
    translation=(0.0, 0.0, 0.35),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
)
print("[RTX_SMOKE] 3D lidar ok:", ok_3d, "prim:", sensor_3d, flush=True)

simulation_app.update()
simulation_app.update()

if not ok_2d or not ok_3d:
    print("[RTX_SMOKE] ERROR: failed to create one or more RTX lidar sensors", flush=True)
    while simulation_app.is_running():
        simulation_app.update()
        time.sleep(0.01)
    simulation_app.close()
    raise SystemExit(1)

print("[RTX_SMOKE] Creating /scan writer", flush=True)
rp_2d = rep.create.render_product(sensor_2d.GetPath(), [1, 1], name="Go2SmokeLidar2D")
scan_writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
scan_writer.initialize(
    topicName="scan",
    frameId="base_scan",
)
scan_writer.attach([rp_2d])

print("[RTX_SMOKE] Creating /point_cloud2 writer", flush=True)
rp_3d = rep.create.render_product(sensor_3d.GetPath(), [1, 1], name="Go2SmokeLidar3D")
pc_writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
pc_writer.initialize(
    topicName="point_cloud2",
    frameId="base_scan",
)
pc_writer.attach([rp_3d])

print("[RTX_SMOKE] READY", flush=True)
print("[RTX_SMOKE] Expected ROS topics:", flush=True)
print("[RTX_SMOKE]   /scan", flush=True)
print("[RTX_SMOKE]   /point_cloud2", flush=True)
print("[RTX_SMOKE] Keep this Isaac window running.", flush=True)

while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.001)

simulation_app.close()
