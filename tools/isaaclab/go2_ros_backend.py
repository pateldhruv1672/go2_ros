#!/usr/bin/env python3

import argparse
import json
import math
import os
import socket
import time

import torch
import torch.nn as nn

from isaaclab.app import AppLauncher


def build_arg_parser():
    parser = argparse.ArgumentParser(description="IsaacLab Go2 ROS UDP backend with pretrained policy inference.")
    parser.add_argument("--task", type=str, default="Isaac-Velocity-Rough-Unitree-Go2-v0")
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--env-device", type=str, default="cuda:0")
    parser.add_argument(
        "--checkpoint",
        type=str,
        default="/home/digital-twin-admin/Dhruv/sparky/ros2_ws/models/go2/model_7850.pt",
    )
    parser.add_argument("--cmd-host", type=str, default="127.0.0.1")
    parser.add_argument("--cmd-port", type=int, default=15000)
    parser.add_argument("--state-host", type=str, default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=15001)
    parser.add_argument("--max-vx", type=float, default=0.6)
    parser.add_argument("--max-vy", type=float, default=0.35)
    parser.add_argument("--max-wz", type=float, default=1.2)
    parser.add_argument("--print-every", type=int, default=100)
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


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def quat_wxyz_to_yaw(q):
    w = float(q[0])
    x = float(q[1])
    y = float(q[2])
    z = float(q[3])
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
    }

    state_sock.sendto(json.dumps(payload).encode("utf-8"), state_addr)
    return payload


def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import gymnasium as gym
    import isaaclab_tasks  # noqa: F401
    from isaaclab_tasks.utils import parse_env_cfg

    device = torch.device(args.env_device)

    env_cfg = parse_env_cfg(
        args.task,
        device=args.env_device,
        num_envs=args.num_envs,
    )

    env = gym.make(args.task, cfg=env_cfg)
    robot = env.unwrapped.scene["robot"]

    policy = load_legacy_actor(args.checkpoint, device)

    cmd_sock, state_sock, state_addr = make_udp(
        args.cmd_host,
        args.cmd_port,
        args.state_host,
        args.state_port,
    )

    obs, _ = env.reset()
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

            policy_obs = get_policy_obs(obs).to(device)
            sim_t += step_dt
            state = send_state(state_sock, state_addr, sim_t, robot)

            if step_count % args.print_every == 0:
                print(
                    "[go2_ros_backend] "
                    f"cmd=({vx:.2f},{vy:.2f},{wz:.2f}) "
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
