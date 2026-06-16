#!/usr/bin/env python3

import argparse
import json
import math
import os
import socket

import torch
import torch.nn as nn

from isaaclab.app import AppLauncher


def build_arg_parser():
    parser = argparse.ArgumentParser(description="IsaacLab Go2 ROS UDP backend with Office world support.")
    parser.add_argument("--task", type=str, default="Isaac-Velocity-Rough-Unitree-Go2-v0")
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--env-device", type=str, default="cuda:0")
    parser.add_argument("--checkpoint", type=str, default="/home/digital-twin-admin/Dhruv/sparky/ros2_ws/models/go2/model_7850.pt")

    parser.add_argument("--cmd-host", type=str, default="127.0.0.1")
    parser.add_argument("--cmd-port", type=int, default=15000)
    parser.add_argument("--state-host", type=str, default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=15001)

    parser.add_argument("--max-vx", type=float, default=0.6)
    parser.add_argument("--max-vy", type=float, default=0.35)
    parser.add_argument("--max-wz", type=float, default=1.2)

    parser.add_argument("--world-usd", type=str, default="")
    parser.add_argument("--world-prim-path", type=str, default="/World/Office")
    parser.add_argument("--make-world-colliders", action="store_true")
    parser.add_argument("--disable-ground-collision", action="store_true")
    parser.add_argument("--hide-rough-terrain", action="store_true")
    parser.add_argument("--flat-generated-terrain", action="store_true")

    parser.add_argument("--spawn-x", type=float, default=0.0)
    parser.add_argument("--spawn-y", type=float, default=0.0)
    parser.add_argument("--spawn-z", type=float, default=0.55)
    parser.add_argument("--spawn-yaw", type=float, default=0.0)

    parser.add_argument("--print-every", type=int, default=100)
    parser.add_argument("--enable-rtx-lidar", action="store_true")
    parser.add_argument("--enable-ros-sensors", action="store_true")
    parser.add_argument("--camera-frame", type=str, default="camera_link")
    parser.add_argument("--camera-topic", type=str, default="camera/image_raw")
    parser.add_argument("--camera-info-topic", type=str, default="camera/camera_info")
    parser.add_argument("--pointcloud-topic", type=str, default="point_cloud2")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=10)

    AppLauncher.add_app_launcher_args(parser)
    return parser


class LegacyRslRlActor(nn.Module):
    def __init__(self, obs_dim=235, action_dim=12):
        super().__init__()
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, action_dim),
        )

    def forward(self, obs):
        return self.actor(obs)


def load_legacy_actor(checkpoint_path, device):
    checkpoint_path = os.path.abspath(os.path.expanduser(checkpoint_path))
    if not os.path.exists(checkpoint_path):
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    ckpt = torch.load(checkpoint_path, map_location=device)
    model_state = ckpt["model_state_dict"]

    actor_state = {}
    for key, value in model_state.items():
        if key.startswith("actor."):
            actor_state[key.replace("actor.", "", 1)] = value

    policy = LegacyRslRlActor(obs_dim=235, action_dim=12).to(device)
    policy.actor.load_state_dict(actor_state, strict=True)
    policy.eval()
    print(f"[go2_ros_backend] Loaded legacy Go2 actor from: {checkpoint_path}", flush=True)
    return policy


def add_usd_world(world_usd, prim_path):
    if not world_usd:
        return

    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    prim = UsdGeom.Xform.Define(stage, prim_path).GetPrim()
    prim.GetReferences().AddReference(world_usd)
    print(f"[go2_ros_backend] Added world USD: {world_usd}", flush=True)
    print(f"[go2_ros_backend] World prim path: {prim_path}", flush=True)


def make_world_static_colliders(root_path):
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics

    try:
        from pxr import PhysxSchema
    except Exception:
        PhysxSchema = None

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        print(f"[go2_ros_backend] WARNING: collider root not found: {root_path}", flush=True)
        return

    collider_count = 0
    disabled_rigid_count = 0

    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            disabled_rigid_count += 1

        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)

            try:
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
            except Exception:
                pass

            if PhysxSchema is not None:
                try:
                    physx_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                    physx_api.CreateContactOffsetAttr(0.02)
                    physx_api.CreateRestOffsetAttr(0.0)
                except Exception:
                    pass

            collider_count += 1

    print(
        f"[go2_ros_backend] Static world colliders enabled under {root_path}: "
        f"{collider_count} mesh colliders, {disabled_rigid_count} rigid bodies disabled",
        flush=True,
    )


def hide_rough_terrain():
    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    for path in ["/World/ground", "/World/ground/terrain"]:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            UsdGeom.Imageable(prim).MakeInvisible()
            print(f"[go2_ros_backend] Hid rough terrain visual: {path}", flush=True)


def set_ground_collision_enabled(enabled):
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    changed = 0

    for root_path in ["/World/ground", "/World/ground/terrain"]:
        root = stage.GetPrimAtPath(root_path)
        if not root or not root.IsValid():
            continue

        for prim in Usd.PrimRange(root):
            if prim.IsA(UsdGeom.Mesh) or prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(bool(enabled))
                changed += 1

    print(f"[go2_ros_backend] Rough ground collision enabled={enabled}; changed {changed} prims", flush=True)


def force_flat_generated_terrain(env_cfg):
    terrain = getattr(env_cfg.scene, "terrain", None)
    if terrain is None:
        print("[go2_ros_backend] WARNING: env_cfg.scene.terrain not found", flush=True)
        return

    if hasattr(terrain, "terrain_type"):
        terrain.terrain_type = "plane"

    if hasattr(terrain, "terrain_generator"):
        terrain.terrain_generator = None

    if hasattr(terrain, "prim_path"):
        terrain.prim_path = "/World/ground"

    if hasattr(env_cfg, "curriculum") and hasattr(env_cfg.curriculum, "terrain_levels"):
        env_cfg.curriculum.terrain_levels = None

    print("[go2_ros_backend] Forced generated terrain to flat plane", flush=True)


def yaw_to_quat_wxyz(yaw, device):
    half = 0.5 * yaw
    return torch.tensor([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=torch.float32, device=device)


def set_robot_spawn(robot, args, device):
    root_state = robot.data.default_root_state.clone()
    root_state[:, 0] = args.spawn_x
    root_state[:, 1] = args.spawn_y
    root_state[:, 2] = args.spawn_z

    q = yaw_to_quat_wxyz(args.spawn_yaw, device)
    root_state[:, 3] = q[0]
    root_state[:, 4] = q[1]
    root_state[:, 5] = q[2]
    root_state[:, 6] = q[3]
    root_state[:, 7:13] = 0.0

    robot.write_root_state_to_sim(root_state)
    print(
        f"[go2_ros_backend] Robot spawn set to x={args.spawn_x:.2f}, "
        f"y={args.spawn_y:.2f}, z={args.spawn_z:.2f}, yaw={args.spawn_yaw:.2f}",
        flush=True,
    )


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def quat_wxyz_to_yaw(q):
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def get_policy_obs(obs):
    if isinstance(obs, dict):
        if "policy" in obs:
            return obs["policy"]
        if "obs" in obs:
            return obs["obs"]
        raise RuntimeError(f"Could not find policy observation. Obs keys: {list(obs.keys())}")
    return obs


def set_base_velocity_command(env, vx, vy, wz):
    command_manager = env.unwrapped.command_manager
    term = command_manager._terms.get("base_velocity", None)

    if term is None:
        raise RuntimeError(f"No base_velocity command term. Available: {list(command_manager._terms.keys())}")

    for attr in ("command", "_command"):
        if hasattr(term, attr):
            cmd = getattr(term, attr)
            cmd[:, 0] = vx
            cmd[:, 1] = vy
            cmd[:, 2] = wz
            return

    cmd = command_manager.get_command("base_velocity")
    cmd[:, 0] = vx
    cmd[:, 1] = vy
    cmd[:, 2] = wz


def make_udp(cmd_host, cmd_port, state_host, state_port):
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    cmd_sock.bind((cmd_host, cmd_port))
    cmd_sock.setblocking(False)

    state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state_addr = (state_host, state_port)

    print(f"[go2_ros_backend] UDP command input : {cmd_host}:{cmd_port}", flush=True)
    print(f"[go2_ros_backend] UDP state output   : {state_host}:{state_port}", flush=True)
    return cmd_sock, state_sock, state_addr


def read_latest_cmd(cmd_sock, current_cmd, args):
    latest = None

    while True:
        try:
            data, _ = cmd_sock.recvfrom(4096)
            latest = json.loads(data.decode("utf-8"))
        except BlockingIOError:
            break
        except Exception as exc:
            print(f"[go2_ros_backend] bad UDP cmd: {exc!r}", flush=True)
            break

    if latest is None:
        return current_cmd

    vx = clamp(float(latest.get("vx", 0.0)), -args.max_vx, args.max_vx)
    vy = clamp(float(latest.get("vy", 0.0)), -args.max_vy, args.max_vy)
    wz = clamp(float(latest.get("wz", 0.0)), -args.max_wz, args.max_wz)
    return vx, vy, wz


def send_state(state_sock, state_addr, sim_t, robot):
    root_pos = robot.data.root_pos_w[0].detach().cpu()
    root_quat = robot.data.root_quat_w[0].detach().cpu()
    root_lin_vel = robot.data.root_lin_vel_w[0].detach().cpu()
    root_ang_vel = robot.data.root_ang_vel_w[0].detach().cpu()

    payload = {
        "t": float(sim_t),
        "x": float(root_pos[0]),
        "y": float(root_pos[1]),
        "z": float(root_pos[2]),
        "yaw": float(quat_wxyz_to_yaw(root_quat)),
        "vx": float(root_lin_vel[0]),
        "vy": float(root_lin_vel[1]),
        "wz": float(root_ang_vel[2]),
        "joint_pos": robot.data.joint_pos[0].detach().cpu().tolist(),
        "joint_vel": robot.data.joint_vel[0].detach().cpu().tolist(),
    }

    state_sock.sendto(json.dumps(payload).encode("utf-8"), state_addr)
    return payload






def setup_ros_camera_publishers(simulation_app, args):
    """Stable Go2-mounted USD camera publisher.

    Publishes:
      /camera/image_raw
      /camera/camera_info
      /point_cloud2
    """
    if not args.enable_ros_sensors:
        return

    import omni
    import omni.replicator.core as rep
    import omni.syntheticdata._syntheticdata as sd
    from pxr import UsdGeom, Gf
    from isaacsim.core.utils import extensions
    from isaacsim.ros2.bridge import read_camera_info

    extensions.enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()
    simulation_app.update()

    stage = omni.usd.get_context().get_stage()

    # Put camera outside the robot body, higher than before.
    camera_path = "/World/envs/env_0/Robot/base/camera_link"
    width = int(getattr(args, "camera_width", 640))
    height = int(getattr(args, "camera_height", 480))

    cam = UsdGeom.Camera.Define(stage, camera_path)
    cam.CreateFocalLengthAttr(14.0)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(15.2908)

    xform = UsdGeom.Xformable(cam.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.62, 0.0, 0.42))

    # USD camera looks along local -Z. This should point roughly along Go2 forward +X.
    # If view is still wrong, only change this one line.
    xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, -90.0, 0.0))

    simulation_app.update()
    simulation_app.update()

    render_product = rep.create.render_product(camera_path, (width, height), name="Go2FrontCamera")
    simulation_app.update()
    simulation_app.update()

    try:
        rep.orchestrator.set_capture_on_play(True)
    except Exception:
        pass

    frame_id = "camera_link"
    queue_size = 5

    print(f"[go2_ros_backend][CAMERA] Camera prim: {camera_path}", flush=True)
    print(f"[go2_ros_backend][CAMERA] Render product: {render_product}", flush=True)

    rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    rgb_writer = rep.writers.get(rv_rgb + "ROS2PublishImage")
    rgb_writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=queue_size,
        topicName="camera/image_raw_raw",
    )
    rgb_writer.attach([render_product])

    camera_info, _ = read_camera_info(render_product_path=render_product)
    info_writer = rep.writers.get("ROS2PublishCameraInfo")
    info_writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=queue_size,
        topicName="camera/camera_info",
        width=camera_info.width,
        height=camera_info.height,
        projectionType=camera_info.distortion_model,
        k=camera_info.k.reshape([1, 9]),
        r=camera_info.r.reshape([1, 9]),
        p=camera_info.p.reshape([1, 12]),
        physicalDistortionModel=camera_info.distortion_model,
        physicalDistortionCoefficients=camera_info.d,
    )
    info_writer.attach([render_product])

    rv_depth = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )
    pc_writer = rep.writers.get(rv_depth + "ROS2PublishPointCloud")
    pc_writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=queue_size,
        topicName="point_cloud2",
    )
    pc_writer.attach([render_product])

    print("[go2_ros_backend][CAMERA] Publishing /camera/image_raw_raw", flush=True)
    print("[go2_ros_backend][CAMERA] Publishing /camera/camera_info", flush=True)
    
    try:
        rep.orchestrator.set_capture_on_play(True)
        rep.orchestrator.run()
        print("[go2_ros_backend][CAMERA] Replicator orchestrator running continuously", flush=True)
    except Exception as e:
        print("[go2_ros_backend][CAMERA] Replicator orchestrator run failed:", repr(e), flush=True)

print("[go2_ros_backend][CAMERA] Publishing /point_cloud2", flush=True)


def setup_rtx_lidar_ros_publishers(simulation_app, args):
    """Create Isaac RTX lidar sensors and publish ROS 2 /scan and /point_cloud2.

    This uses Isaac Sim ROS 2 bridge Replicator writers:
      RtxLidarROS2PublishLaserScan
      RtxLidarROS2PublishPointCloud
    """
    if not getattr(args, "enable_rtx_lidar", False):
        return

    import omni
    import omni.kit.commands
    import omni.replicator.core as rep
    from isaacsim.core.utils import extensions
    from pxr import Gf

    extensions.enable_extension("isaacsim.ros2.bridge")
    extensions.enable_extension("isaacsim.sensors.rtx")
    simulation_app.update()

    stage = omni.usd.get_context().get_stage()

    parent_candidates = [
        "/World/envs/env_0/Robot/base",
        "/World/envs/env_0/Robot/base_link",
        "/World/envs/env_0/Robot/trunk",
        "/World/envs/env_0/Robot",
    ]

    parent_path = None
    for cand in parent_candidates:
        prim = stage.GetPrimAtPath(cand)
        if prim and prim.IsValid():
            parent_path = cand
            break

    if parent_path is None:
        print("[go2_ros_backend][RTX_LIDAR] WARNING: robot parent prim not found; using world-mounted lidar", flush=True)
    else:
        print(f"[go2_ros_backend][RTX_LIDAR] Parent prim: {parent_path}", flush=True)

    # 2D RTX lidar for Nav2/SLAM LaserScan.
    _, lidar_2d = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path="/base_scan_2d",
        parent=parent_path,
        config="Example_Rotary_2D",
        translation=(0.25, 0.0, 0.24),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )

    # 3D RTX lidar for PointCloud2.
    _, lidar_3d = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path="/base_scan_3d",
        parent=parent_path,
        config="Example_Rotary",
        translation=(0.25, 0.0, 0.24),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )

    simulation_app.update()

    rp_2d = rep.create.render_product(lidar_2d.GetPath(), [1, 1], name="Go2RtxLidar2D")
    scan_writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
    scan_writer.initialize(
        topicName="scan",
        frameId="base_scan",
    )
    scan_writer.attach([rp_2d])

    rp_3d = rep.create.render_product(lidar_3d.GetPath(), [1, 1], name="Go2RtxLidar3D")
    pc_writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
    pc_writer.initialize(
        topicName="point_cloud2",
        frameId="base_scan",
    )
    pc_writer.attach([rp_3d])

    print("[go2_ros_backend][RTX_LIDAR] Publishing /scan", flush=True)
    print("[go2_ros_backend][RTX_LIDAR] Publishing /point_cloud2", flush=True)
    print("[go2_ros_backend][RTX_LIDAR] frame_id=base_scan", flush=True)


def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import gymnasium as gym
    import isaaclab_tasks  # noqa: F401
    from isaaclab_tasks.utils import parse_env_cfg

    device = torch.device(args.env_device)

    env_cfg = parse_env_cfg(args.task, device=args.env_device, num_envs=args.num_envs)

    if args.flat_generated_terrain:
        force_flat_generated_terrain(env_cfg)

    env = gym.make(args.task, cfg=env_cfg)
    robot = env.unwrapped.scene["robot"]

    add_usd_world(args.world_usd, args.world_prim_path)

    if args.make_world_colliders and args.world_usd:
        make_world_static_colliders(args.world_prim_path)

    if args.disable_ground_collision:
        set_ground_collision_enabled(False)

    policy = load_legacy_actor(args.checkpoint, device)

    cmd_sock, state_sock, state_addr = make_udp(args.cmd_host, args.cmd_port, args.state_host, args.state_port)

    obs, _ = env.reset()

    if args.hide_rough_terrain:
        hide_rough_terrain()

    set_robot_spawn(robot, args, device)
    # ---- FORCED ISAAC ROS SENSOR SETUP BEGIN ----
    args.enable_rtx_lidar = True
    args.enable_ros_sensors = True
    print(f"[go2_ros_backend] SENSOR FLAGS FORCED: enable_rtx_lidar={args.enable_rtx_lidar}, enable_ros_sensors={args.enable_ros_sensors}", flush=True)

    try:
        setup_rtx_lidar_ros_publishers(simulation_app, args)
    except Exception as e:
        import traceback
        print("[go2_ros_backend][RTX_LIDAR] FAILED:", repr(e), flush=True)
        traceback.print_exc()

    try:
        setup_ros_camera_publishers(simulation_app, args)
    except Exception as e:
        import traceback
        print("[go2_ros_backend][CAMERA] FAILED:", repr(e), flush=True)
        traceback.print_exc()
    # ---- FORCED ISAAC ROS SENSOR SETUP END ----
    policy_obs = get_policy_obs(obs).to(device)

    print("[go2_ros_backend] READY", flush=True)
    print(f"[go2_ros_backend] task={args.task}", flush=True)
    print(f"[go2_ros_backend] policy_obs_shape={tuple(policy_obs.shape)}", flush=True)

    if policy_obs.shape[-1] != 235:
        raise RuntimeError(f"Policy expects obs dim 235, got {tuple(policy_obs.shape)}")

    cmd = (0.0, 0.0, 0.0)
    sim_t = 0.0
    step_dt = float(getattr(env.unwrapped, "step_dt", 1.0 / 60.0))
    step_count = 0

    try:
        while simulation_app.is_running():
            cmd = read_latest_cmd(cmd_sock, cmd, args)
            vx, vy, wz = cmd

            set_base_velocity_command(env, vx, vy, wz)

            with torch.inference_mode():
                actions = policy(policy_obs)

            step_out = env.step(actions)

            if len(step_out) == 5:
                obs, reward, terminated, truncated, extras = step_out
                done = bool((terminated | truncated).any().item())
            else:
                obs, reward, done, extras = step_out
                done = bool(done.any().item()) if hasattr(done, "any") else bool(done)

            if done:
                obs, _ = env.reset()
                set_robot_spawn(robot, args, device)

            policy_obs = get_policy_obs(obs).to(device)
            sim_t += step_dt
            state = send_state(state_sock, state_addr, sim_t, robot)

            if step_count % args.print_every == 0:
                print(
                    f"[go2_ros_backend] cmd=({vx:.2f},{vy:.2f},{wz:.2f}) "
                    f"pos=({state['x']:.2f},{state['y']:.2f},{state['z']:.2f}) "
                    f"yaw={state['yaw']:.2f}",
                    flush=True,
                )

            step_count += 1

    except KeyboardInterrupt:
        pass
    finally:
        env.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
