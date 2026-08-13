#!/usr/bin/env python3
"""
SPARKY Nav2 benchmark suite.

Runs bounded relative Nav2 goals around a fixed "home" pose and writes:
  - report.md
  - results.csv
  - results.json
  - controller_params.txt
  - environment.txt
  - samples/<case>.csv

Safety:
  * Requires --i-understand-robot-will-move.
  * Every NavigateToPose target is bounded by --max-radius from the suite home.
  * Default max radius is 3.0 m.
  * Default stops after first failed navigation unless --continue-on-failure.
  * Ctrl+C cancels the active Nav2 goal.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose, BackUp, Spin
from tf2_ros import Buffer, TransformListener, TransformException


def q_to_yaw(x, y, z, w):
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def yaw_to_q(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def planar_dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def path_length(msg):
    if msg is None or len(msg.poses) < 2:
        return float("nan")
    total = 0.0
    prev = msg.poses[0].pose.position
    for ps in msg.poses[1:]:
        p = ps.pose.position
        total += math.hypot(p.x - prev.x, p.y - prev.y)
        prev = p
    return total


def duration_sec(msg):
    if msg is None:
        return None
    return float(msg.sec) + float(msg.nanosec) * 1e-9


def fmt(v, digits=3):
    if v is None:
        return ""
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return ""
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def mean_abs(values):
    values = [abs(v) for v in values if math.isfinite(v)]
    return statistics.mean(values) if values else float("nan")


class SparkyBench(Node):
    def __init__(self, args, outdir):
        super().__init__("sparky_nav2_benchmark_v11_13")
        self.args = args
        self.outdir = outdir
        self.samples_dir = outdir / "samples"
        self.samples_dir.mkdir(parents=True, exist_ok=True)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.backup_client = ActionClient(self, BackUp, "/backup")
        self.spin_client = ActionClient(self, Spin, "/spin")

        self.odom = None
        self.amcl = None
        self.plan = None
        self.cmd = {
            "nav2": (0.0, 0.0, None),
            "out": (0.0, 0.0, None),
            "sdk": (0.0, 0.0, None),
        }

        self.active_case = None
        self.active_samples = []
        self.active_start_wall = None
        self.active_start_odom = None
        self.first_nonzero_cmd_wall = None
        self.first_motion_wall = None
        self.feedback = {}
        self.latest_result_future = None
        self.latest_goal_handle = None

        self.create_subscription(Odometry, "/odom", self._on_odom, 50)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 20)
        self.create_subscription(NavPath, "/plan", self._on_plan, 10)
        self.create_subscription(Twist, "/cmd_vel_nav2", lambda m: self._on_cmd("nav2", m), 50)
        self.create_subscription(Twist, "/cmd_vel_out", lambda m: self._on_cmd("out", m), 50)
        self.create_subscription(Twist, "/cmd_vel_sdk", lambda m: self._on_cmd("sdk", m), 50)

    def _on_amcl(self, msg):
        self.amcl = msg

    def _on_plan(self, msg):
        self.plan = msg

    def _on_cmd(self, key, msg):
        now = time.monotonic()
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)
        self.cmd[key] = (vx, wz, now)
        if self.active_case and key == "nav2":
            if self.first_nonzero_cmd_wall is None and (abs(vx) > 0.01 or abs(wz) > 0.02):
                self.first_nonzero_cmd_wall = now

    def _on_odom(self, msg):
        now = time.monotonic()
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        yaw = q_to_yaw(q.x, q.y, q.z, q.w)
        self.odom = {
            "t": now,
            "x": float(p.x),
            "y": float(p.y),
            "yaw": yaw,
            "vx": float(msg.twist.twist.linear.x),
            "wz": float(msg.twist.twist.angular.z),
        }

        if not self.active_case:
            return

        if self.active_start_odom and self.first_motion_wall is None:
            d = math.hypot(
                self.odom["x"] - self.active_start_odom["x"],
                self.odom["y"] - self.active_start_odom["y"],
            )
            dyaw = abs(wrap(self.odom["yaw"] - self.active_start_odom["yaw"]))
            if d >= self.args.motion_threshold or dyaw >= self.args.yaw_motion_threshold:
                self.first_motion_wall = now

        row = {
            "t": now - self.active_start_wall if self.active_start_wall else 0.0,
            "odom_x": self.odom["x"],
            "odom_y": self.odom["y"],
            "odom_yaw": self.odom["yaw"],
            "odom_vx": self.odom["vx"],
            "odom_wz": self.odom["wz"],
            "nav2_vx": self.cmd["nav2"][0],
            "nav2_wz": self.cmd["nav2"][1],
            "out_vx": self.cmd["out"][0],
            "out_wz": self.cmd["out"][1],
            "sdk_vx": self.cmd["sdk"][0],
            "sdk_wz": self.cmd["sdk"][1],
        }
        self.active_samples.append(row)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_future(self, fut, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.done()

    def get_map_pose(self, timeout=3.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.args.map_frame,
                    self.args.base_frame,
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
                q = tr.transform.rotation
                return (
                    float(tr.transform.translation.x),
                    float(tr.transform.translation.y),
                    q_to_yaw(q.x, q.y, q.z, q.w),
                )
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(
            f"Cannot lookup {self.args.map_frame}->{self.args.base_frame}"
        )

    def make_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = self.args.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_q(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def home_relative_pose(self, home, local_x, local_y, yaw_delta):
        c = math.cos(home[2])
        s = math.sin(home[2])
        x = home[0] + c * local_x - s * local_y
        y = home[1] + s * local_x + c * local_y
        yaw = wrap(home[2] + yaw_delta)
        return x, y, yaw

    def _feedback_cb(self, msg):
        fb = msg.feedback
        if hasattr(fb, "distance_remaining"):
            self.feedback["distance_remaining"] = float(fb.distance_remaining)
        if hasattr(fb, "number_of_recoveries"):
            self.feedback["number_of_recoveries"] = int(fb.number_of_recoveries)
        if hasattr(fb, "navigation_time"):
            self.feedback["navigation_time_sec"] = duration_sec(fb.navigation_time)
        if hasattr(fb, "estimated_time_remaining"):
            self.feedback["estimated_time_remaining_sec"] = duration_sec(
                fb.estimated_time_remaining
            )

    def _begin_case(self, name):
        self.active_case = name
        self.active_samples = []
        self.active_start_wall = time.monotonic()
        self.active_start_odom = dict(self.odom) if self.odom else None
        self.first_nonzero_cmd_wall = None
        self.first_motion_wall = None
        self.feedback = {}
        self.plan = None

    def _end_case(self):
        self.active_case = None

    def _write_samples(self, name):
        path = self.samples_dir / f"{name}.csv"
        fields = [
            "t",
            "odom_x", "odom_y", "odom_yaw", "odom_vx", "odom_wz",
            "nav2_vx", "nav2_wz",
            "out_vx", "out_wz",
            "sdk_vx", "sdk_wz",
        ]
        with path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(self.active_samples)
        return str(path)

    def cancel_active(self):
        if self.latest_goal_handle is not None:
            try:
                fut = self.latest_goal_handle.cancel_goal_async()
                self.wait_future(fut, 2.0)
            except Exception:
                pass

    def _sample_metrics(self):
        samples = self.active_samples
        if not samples:
            return {
                "odom_path_m": float("nan"),
                "peak_nav2_vx": float("nan"),
                "peak_nav2_wz": float("nan"),
                "peak_out_vx": float("nan"),
                "peak_out_wz": float("nan"),
                "peak_sdk_vx": float("nan"),
                "peak_sdk_wz": float("nan"),
                "mean_abs_nav2_vx": float("nan"),
                "mean_abs_nav2_wz": float("nan"),
            }

        odom_path = 0.0
        for a, b in zip(samples[:-1], samples[1:]):
            odom_path += math.hypot(
                b["odom_x"] - a["odom_x"],
                b["odom_y"] - a["odom_y"],
            )

        def peak(field):
            return max(abs(r[field]) for r in samples)

        return {
            "odom_path_m": odom_path,
            "peak_nav2_vx": peak("nav2_vx"),
            "peak_nav2_wz": peak("nav2_wz"),
            "peak_out_vx": peak("out_vx"),
            "peak_out_wz": peak("out_wz"),
            "peak_sdk_vx": peak("sdk_vx"),
            "peak_sdk_wz": peak("sdk_wz"),
            "mean_abs_nav2_vx": mean_abs([r["nav2_vx"] for r in samples]),
            "mean_abs_nav2_wz": mean_abs([r["nav2_wz"] for r in samples]),
        }

    def run_nav_case(self, case, home):
        name = case["name"]
        local_x = case["x"]
        local_y = case["y"]
        yaw_delta = case["yaw"]

        radius = math.hypot(local_x, local_y)
        if radius > self.args.max_radius + 1e-9:
            raise RuntimeError(
                f"{name}: target radius {radius:.2f} > max {self.args.max_radius:.2f}"
            )

        target = self.home_relative_pose(home, local_x, local_y, yaw_delta)
        start_pose = self.get_map_pose()
        start_odom = dict(self.odom) if self.odom else None

        self._begin_case(name)
        start_wall = self.active_start_wall
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(*target)

        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb,
        )
        if not self.wait_future(send_future, 5.0):
            self._end_case()
            raise RuntimeError(f"{name}: NavigateToPose goal send timeout")

        goal_handle = send_future.result()
        self.latest_goal_handle = goal_handle

        if not goal_handle.accepted:
            self._end_case()
            return {
                "case": name,
                "type": "NavigateToPose",
                "status": "REJECTED",
                "elapsed_sec": 0.0,
                "target_local_x": local_x,
                "target_local_y": local_y,
                "target_yaw_delta_deg": math.degrees(yaw_delta),
            }

        result_future = goal_handle.get_result_async()
        self.latest_result_future = result_future
        timed_out = False

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - start_wall > self.args.timeout:
                timed_out = True
                cancel = goal_handle.cancel_goal_async()
                self.wait_future(cancel, 2.0)
                self.wait_future(result_future, 3.0)
                break

        elapsed = time.monotonic() - start_wall

        status_code = None
        result_msg = None
        if result_future.done():
            wrapped = result_future.result()
            status_code = int(wrapped.status)
            result_msg = wrapped.result

        status_map = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        status = "TIMEOUT" if timed_out else status_map.get(status_code, str(status_code))

        endpoint = self.get_map_pose()
        post_start_pose = endpoint
        self.spin_for(self.args.post_goal_observe)
        post_end_pose = self.get_map_pose()

        sample_metrics = self._sample_metrics()
        sample_file = self._write_samples(name)

        first_cmd = (
            self.first_nonzero_cmd_wall - start_wall
            if self.first_nonzero_cmd_wall is not None
            else float("nan")
        )
        first_motion = (
            self.first_motion_wall - start_wall
            if self.first_motion_wall is not None
            else float("nan")
        )

        final_error = math.hypot(endpoint[0] - target[0], endpoint[1] - target[1])
        yaw_error = abs(wrap(endpoint[2] - target[2]))
        post_drift = planar_dist(post_start_pose, post_end_pose)
        post_yaw_drift = abs(wrap(post_end_pose[2] - post_start_pose[2]))

        result = {
            "case": name,
            "category": case.get("category", "relative_goal"),
            "type": "NavigateToPose",
            "status": status,
            "elapsed_sec": elapsed,
            "target_local_x": local_x,
            "target_local_y": local_y,
            "target_radius_m": radius,
            "target_yaw_delta_deg": math.degrees(yaw_delta),
            "start_map_x": start_pose[0],
            "start_map_y": start_pose[1],
            "start_map_yaw": start_pose[2],
            "goal_map_x": target[0],
            "goal_map_y": target[1],
            "goal_map_yaw": target[2],
            "final_map_x": endpoint[0],
            "final_map_y": endpoint[1],
            "final_map_yaw": endpoint[2],
            "final_position_error_m": final_error,
            "final_yaw_error_deg": math.degrees(yaw_error),
            "planned_path_m": path_length(self.plan),
            "first_nonzero_nav2_cmd_sec": first_cmd,
            "first_motion_sec": first_motion,
            "cmd_to_motion_delay_sec": (
                first_motion - first_cmd
                if math.isfinite(first_motion) and math.isfinite(first_cmd)
                else float("nan")
            ),
            "post_goal_drift_m": post_drift,
            "post_goal_yaw_drift_deg": math.degrees(post_yaw_drift),
            "number_of_recoveries": int(self.feedback.get("number_of_recoveries", 0)),
            "feedback_navigation_time_sec": self.feedback.get("navigation_time_sec"),
            "final_distance_remaining_m": self.feedback.get("distance_remaining"),
            "result_error_code": int(getattr(result_msg, "error_code", 0)) if result_msg else None,
            "result_error_msg": str(getattr(result_msg, "error_msg", "")) if result_msg else "",
            "samples_file": sample_file,
        }
        result.update(sample_metrics)

        self._end_case()
        self.latest_goal_handle = None
        return result

    def run_backup_case(self, name, distance, speed):
        if not self.backup_client.server_is_ready():
            return {
                "case": name,
                "category": "true_backup_behavior",
                "type": "BackUp",
                "status": "SKIPPED_SERVER_UNAVAILABLE",
                "target_radius_m": distance,
            }

        start_pose = self.get_map_pose()
        self._begin_case(name)
        start_wall = self.active_start_wall

        goal = BackUp.Goal()
        goal.target.x = -abs(float(distance))
        goal.speed = abs(float(speed))
        goal.time_allowance.sec = int(self.args.timeout)
        goal.time_allowance.nanosec = 0

        send_future = self.backup_client.send_goal_async(goal)
        if not self.wait_future(send_future, 5.0):
            self._end_case()
            return {
                "case": name,
                "category": "true_backup_behavior",
                "type": "BackUp",
                "status": "SEND_TIMEOUT",
            }

        gh = send_future.result()
        self.latest_goal_handle = gh
        if not gh.accepted:
            self._end_case()
            return {
                "case": name,
                "category": "true_backup_behavior",
                "type": "BackUp",
                "status": "REJECTED",
            }

        rf = gh.get_result_async()
        timed_out = False
        while rclpy.ok() and not rf.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - start_wall > self.args.timeout:
                timed_out = True
                cf = gh.cancel_goal_async()
                self.wait_future(cf, 2.0)
                break

        elapsed = time.monotonic() - start_wall
        status_code = int(rf.result().status) if rf.done() else None
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        status = "TIMEOUT" if timed_out else status_map.get(status_code, str(status_code))

        endpoint = self.get_map_pose()
        self.spin_for(self.args.post_goal_observe)
        post_end = self.get_map_pose()

        metrics = self._sample_metrics()
        sample_file = self._write_samples(name)
        result = {
            "case": name,
            "category": "true_backup_behavior",
            "type": "BackUp",
            "status": status,
            "elapsed_sec": elapsed,
            "requested_backup_m": distance,
            "requested_backup_speed_mps": speed,
            "map_displacement_m": planar_dist(start_pose, endpoint),
            "map_yaw_change_deg": math.degrees(abs(wrap(endpoint[2] - start_pose[2]))),
            "post_goal_drift_m": planar_dist(endpoint, post_end),
            "post_goal_yaw_drift_deg": math.degrees(abs(wrap(post_end[2] - endpoint[2]))),
            "samples_file": sample_file,
        }
        result.update(metrics)

        self._end_case()
        self.latest_goal_handle = None
        return result

    def run_spin_case(self, name, angle_rad):
        if not self.spin_client.server_is_ready():
            return {
                "case": name,
                "category": "spin_behavior",
                "type": "Spin",
                "status": "SKIPPED_SERVER_UNAVAILABLE",
            }

        start_pose = self.get_map_pose()
        self._begin_case(name)
        start_wall = self.active_start_wall

        goal = Spin.Goal()
        goal.target_yaw = float(angle_rad)
        goal.time_allowance.sec = int(self.args.timeout)
        goal.time_allowance.nanosec = 0

        sf = self.spin_client.send_goal_async(goal)
        if not self.wait_future(sf, 5.0):
            self._end_case()
            return {"case": name, "type": "Spin", "status": "SEND_TIMEOUT"}

        gh = sf.result()
        self.latest_goal_handle = gh
        if not gh.accepted:
            self._end_case()
            return {"case": name, "type": "Spin", "status": "REJECTED"}

        rf = gh.get_result_async()
        timed_out = False
        while rclpy.ok() and not rf.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - start_wall > self.args.timeout:
                timed_out = True
                cf = gh.cancel_goal_async()
                self.wait_future(cf, 2.0)
                break

        elapsed = time.monotonic() - start_wall
        status_code = int(rf.result().status) if rf.done() else None
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        status = "TIMEOUT" if timed_out else status_map.get(status_code, str(status_code))

        endpoint = self.get_map_pose()
        self.spin_for(self.args.post_goal_observe)
        post_end = self.get_map_pose()

        metrics = self._sample_metrics()
        sample_file = self._write_samples(name)
        result = {
            "case": name,
            "category": "spin_behavior",
            "type": "Spin",
            "status": status,
            "elapsed_sec": elapsed,
            "requested_spin_deg": math.degrees(angle_rad),
            "actual_map_yaw_change_deg": math.degrees(wrap(endpoint[2] - start_pose[2])),
            "position_drift_during_spin_m": planar_dist(start_pose, endpoint),
            "post_goal_drift_m": planar_dist(endpoint, post_end),
            "post_goal_yaw_drift_deg": math.degrees(abs(wrap(post_end[2] - endpoint[2]))),
            "samples_file": sample_file,
        }
        result.update(metrics)

        self._end_case()
        self.latest_goal_handle = None
        return result


def suite_cases(name):
    smoke = [
        {"name": "forward_0p5", "x": 0.5, "y": 0.0, "yaw": 0.0, "category": "forward"},
        {"name": "left_target_0p75", "x": 0.0, "y": 0.75, "yaw": 0.0, "category": "left_target"},
        {"name": "right_target_0p75", "x": 0.0, "y": -0.75, "yaw": 0.0, "category": "right_target"},
        {"name": "behind_target_0p75", "x": -0.75, "y": 0.0, "yaw": 0.0, "category": "behind_target"},
    ]

    core = [
        {"name": "forward_0p5", "x": 0.5, "y": 0.0, "yaw": 0.0, "category": "forward"},
        {"name": "forward_1p5", "x": 1.5, "y": 0.0, "yaw": 0.0, "category": "forward"},
        {"name": "forward_3p0", "x": 3.0, "y": 0.0, "yaw": 0.0, "category": "forward"},
        {"name": "behind_target_1p0", "x": -1.0, "y": 0.0, "yaw": 0.0, "category": "behind_target"},
        {"name": "behind_target_2p0", "x": -2.0, "y": 0.0, "yaw": 0.0, "category": "behind_target"},
        {"name": "behind_target_3p0", "x": -3.0, "y": 0.0, "yaw": 0.0, "category": "behind_target"},
        {"name": "left_target_1p0", "x": 0.0, "y": 1.0, "yaw": 0.0, "category": "left_target"},
        {"name": "left_target_2p0", "x": 0.0, "y": 2.0, "yaw": 0.0, "category": "left_target"},
        {"name": "right_target_1p0", "x": 0.0, "y": -1.0, "yaw": 0.0, "category": "right_target"},
        {"name": "right_target_2p0", "x": 0.0, "y": -2.0, "yaw": 0.0, "category": "right_target"},
        {"name": "diag_front_left_1p5", "x": 1.06066, "y": 1.06066, "yaw": 0.0, "category": "diagonal"},
        {"name": "diag_front_right_1p5", "x": 1.06066, "y": -1.06066, "yaw": 0.0, "category": "diagonal"},
        {"name": "diag_back_left_1p5", "x": -1.06066, "y": 1.06066, "yaw": 0.0, "category": "diagonal"},
        {"name": "diag_back_right_1p5", "x": -1.06066, "y": -1.06066, "yaw": 0.0, "category": "diagonal"},
        {"name": "nav_rotate_plus_90", "x": 0.0, "y": 0.0, "yaw": math.pi / 2, "category": "rotate_goal"},
        {"name": "nav_rotate_minus_90", "x": 0.0, "y": 0.0, "yaw": -math.pi / 2, "category": "rotate_goal"},
        {"name": "nav_rotate_180", "x": 0.0, "y": 0.0, "yaw": math.pi, "category": "rotate_goal"},
    ]

    if name == "smoke":
        return smoke
    return core


def run_command_capture(cmd, timeout=8):
    try:
        p = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
        return p.stdout
    except Exception as exc:
        return f"ERROR running {' '.join(cmd)}: {exc}\n"


def write_reports(outdir, args, home, results):
    json_path = outdir / "results.json"
    json_path.write_text(json.dumps(results, indent=2, allow_nan=True))

    keys = []
    for r in results:
        for k in r:
            if k not in keys:
                keys.append(k)

    csv_path = outdir / "results.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in results:
            w.writerow(r)

    nav_results = [r for r in results if r.get("type") == "NavigateToPose"]
    success = [r for r in nav_results if r.get("status") == "SUCCEEDED"]
    failed = [r for r in nav_results if r.get("status") != "SUCCEEDED"]

    elapsed = [r["elapsed_sec"] for r in success if isinstance(r.get("elapsed_sec"), (int, float))]
    drift = [
        r["post_goal_drift_m"] for r in results
        if isinstance(r.get("post_goal_drift_m"), (int, float))
        and math.isfinite(r["post_goal_drift_m"])
    ]
    phase = [
        r["cmd_to_motion_delay_sec"] for r in nav_results
        if isinstance(r.get("cmd_to_motion_delay_sec"), (int, float))
        and math.isfinite(r["cmd_to_motion_delay_sec"])
    ]
    peak_wz = [
        r["peak_nav2_wz"] for r in results
        if isinstance(r.get("peak_nav2_wz"), (int, float))
        and math.isfinite(r["peak_nav2_wz"])
    ]

    recommendations = []
    if failed:
        recommendations.append(
            f"{len(failed)} NavigateToPose cases failed. Tune only after grouping failures by direction."
        )
    if phase and statistics.median(phase) > 0.5:
        recommendations.append(
            f"Median command-to-motion delay is {statistics.median(phase):.2f}s (>0.5s): "
            "keep investigating command/plant latency before critic tuning."
        )
    if drift and max(drift) > 0.10:
        recommendations.append(
            f"Maximum post-goal drift is {max(drift):.2f}m (>0.10m): stopping/zero-command dynamics still need work."
        )
    if peak_wz and max(peak_wz) >= 0.95 * 1.0:
        recommendations.append(
            "DWB reached the 1.0 rad/s angular ceiling in at least one case. "
            "If turn-heavy cases remain slow without oscillation, evaluate 1.2 rad/s next."
        )
    if success and not failed and drift and max(drift) < 0.05:
        recommendations.append(
            "All navigation cases passed with low post-goal drift. Next tuning can focus on time-to-goal and path efficiency."
        )
    if not recommendations:
        recommendations.append(
            "Use per-case timings, peak commands, final errors, recoveries, and post-goal drift to select the next parameter change."
        )

    lines = []
    lines.append("# Sparky Nav2 Benchmark Report")
    lines.append("")
    lines.append(f"- Created: {datetime.now().isoformat(timespec='seconds')}")
    lines.append(f"- Suite: `{args.suite}`")
    lines.append(f"- Home map pose: x={home[0]:.3f}, y={home[1]:.3f}, yaw={math.degrees(home[2]):.1f} deg")
    lines.append(f"- Maximum target radius: {args.max_radius:.2f} m")
    lines.append(f"- Timeout per case: {args.timeout:.1f} s")
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append(f"- NavigateToPose success: {len(success)}/{len(nav_results)}")
    if elapsed:
        lines.append(f"- Median successful completion time: {statistics.median(elapsed):.2f} s")
    if phase:
        lines.append(f"- Median command-to-motion delay: {statistics.median(phase):.3f} s")
    if drift:
        lines.append(f"- Maximum post-goal drift: {max(drift):.3f} m")
    if peak_wz:
        lines.append(f"- Maximum observed Nav2 |wz|: {max(peak_wz):.3f} rad/s")
    lines.append("")
    lines.append("## Cases")
    lines.append("")
    lines.append(
        "| Case | Type | Status | Time s | Goal err m | Yaw err deg | "
        "Odom path m | Plan m | Peak vx | Peak wz | Recoveries | Cmd→motion s | Post drift m |"
    )
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for r in results:
        lines.append(
            "| {case} | {type} | {status} | {time} | {ge} | {ye} | {odom} | {plan} | "
            "{pvx} | {pwz} | {rec} | {phase} | {drift} |".format(
                case=r.get("case", ""),
                type=r.get("type", ""),
                status=r.get("status", ""),
                time=fmt(r.get("elapsed_sec")),
                ge=fmt(r.get("final_position_error_m")),
                ye=fmt(r.get("final_yaw_error_deg"), 1),
                odom=fmt(r.get("odom_path_m")),
                plan=fmt(r.get("planned_path_m")),
                pvx=fmt(r.get("peak_nav2_vx")),
                pwz=fmt(r.get("peak_nav2_wz")),
                rec=r.get("number_of_recoveries", ""),
                phase=fmt(r.get("cmd_to_motion_delay_sec")),
                drift=fmt(r.get("post_goal_drift_m")),
            )
        )

    lines.append("")
    lines.append("## Interpretation / next tuning")
    lines.append("")
    for rec in recommendations:
        lines.append(f"- {rec}")
    lines.append("")
    lines.append("## Direction semantics")
    lines.append("")
    lines.append(
        "- `behind_target_*` is a NavigateToPose goal behind the starting pose. "
        "With DWB `min_vel_x=0`, this normally tests turn-and-drive behavior, not continuous reverse walking."
    )
    lines.append(
        "- `backup_*` uses Nav2's BackUp behavior and is the true reverse-motion test."
    )
    lines.append(
        "- Left/right targets are points to the robot's left/right of the fixed home frame; "
        "they test turning + path following, not lateral strafing when `max_vel_y=0`."
    )
    lines.append("")
    lines.append("Raw 10+ Hz samples for every case are in `samples/`.")

    (outdir / "report.md").write_text("\n".join(lines) + "\n")



def verify_runtime_contract():
    """Fail fast if Resume/Nav2 is still running an older parameter contract."""
    required = {
        "FollowPath.plugin": "nav2_rotation_shim_controller::RotationShimController",
        "FollowPath.rotate_to_heading_angular_vel": 0.80,
        "FollowPath.max_angular_accel": 5.0,
        "FollowPath.primary_controller.max_vel_x": 0.35,
        "FollowPath.primary_controller.max_vel_theta": 1.00,
        "FollowPath.primary_controller.acc_lim_theta": 5.0,
        "FollowPath.primary_controller.decel_lim_theta": -5.0,
        "general_goal_checker.xy_goal_tolerance": 0.10,
        "general_goal_checker.yaw_goal_tolerance": 0.20,
    }

    problems = []
    observed = {}

    for param, expected in required.items():
        try:
            p = subprocess.run(
                ["ros2", "param", "get", "/controller_server", param],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=4,
                check=False,
            )
            out = p.stdout.strip()
            observed[param] = out

            if p.returncode != 0:
                problems.append(f"{param}: unavailable ({out})")
                continue

            if isinstance(expected, str):
                if expected not in out:
                    problems.append(
                        f"{param}: expected {expected!r}, got {out!r}"
                    )
            else:
                m = re.search(
                    r"(?:Double|Integer) value is:\s*([-+0-9.eE]+)",
                    out,
                )
                if not m:
                    problems.append(
                        f"{param}: could not parse numeric value from {out!r}"
                    )
                    continue
                got = float(m.group(1))
                if abs(got - float(expected)) > 1e-6:
                    problems.append(
                        f"{param}: expected {expected}, got {got}"
                    )
        except Exception as exc:
            problems.append(f"{param}: query failed: {exc}")

    return problems, observed

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", choices=["smoke", "core", "full"], default="core")
    ap.add_argument("--max-radius", type=float, default=3.0)
    ap.add_argument("--timeout", type=float, default=75.0)
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--post-goal-observe", type=float, default=1.0)
    ap.add_argument("--motion-threshold", type=float, default=0.005)
    ap.add_argument("--yaw-motion-threshold", type=float, default=0.005)
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--home-between", action="store_true", default=True)
    ap.add_argument("--no-home-between", dest="home_between", action="store_false")
    ap.add_argument("--continue-on-failure", action="store_true")
    ap.add_argument("--skip-backup-behavior", action="store_true")
    ap.add_argument("--skip-spin-behavior", action="store_true")
    ap.add_argument(
        "--i-understand-robot-will-move",
        action="store_true",
        help="Required physical-safety acknowledgement.",
    )
    ap.add_argument(
        "--output-root",
        default=os.path.expanduser("~/sparky_nav_benchmarks"),
    )
    args = ap.parse_args()

    if not args.i_understand_robot_will_move:
        print(
            "REFUSING TO MOVE ROBOT.\n"
            "Re-run with --i-understand-robot-will-move only after verifying a clear area.\n"
            "The core/full suites use targets up to 3 m from the starting home pose."
        )
        return 2

    if args.max_radius <= 0.0 or args.max_radius > 3.0:
        print("--max-radius must be > 0 and <= 3.0 m")
        return 2

    problems, observed = verify_runtime_contract()
    if problems:
        print("REFUSING BENCHMARK: runtime Nav2 contract is stale or incomplete.")
        print()
        for item in problems:
            print("  - " + item)
        print()
        print("Restart Resume/Nav2 after applying V11.13, then rerun the benchmark.")
        return 3

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = Path(args.output_root) / f"nav2_benchmark_{stamp}"
    outdir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = SparkyBench(args, outdir)
    results = []

    try:
        node.spin_for(1.0)

        if not node.nav_client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("/navigate_to_pose action server is unavailable")

        node.backup_client.wait_for_server(timeout_sec=1.0)
        node.spin_client.wait_for_server(timeout_sec=1.0)

        if node.odom is None:
            raise RuntimeError("No /odom received")

        home = node.get_map_pose()

        (outdir / "environment.txt").write_text(
            "HOME\n"
            f"x={home[0]:.6f}\n"
            f"y={home[1]:.6f}\n"
            f"yaw={home[2]:.6f}\n\n"
            + "ROS NODES\n"
            + run_command_capture(["ros2", "node", "list"])
            + "\nROS TOPICS\n"
            + run_command_capture(["ros2", "topic", "list"])
        )

        (outdir / "controller_params.txt").write_text(
            run_command_capture(["ros2", "param", "dump", "/controller_server"])
        )

        cases = suite_cases(args.suite)
        if args.suite == "full":
            # Core positional tests plus Nav2 behavior actions.
            if not args.skip_spin_behavior:
                cases = cases + [
                    {"behavior": "spin", "name": "spin_behavior_plus_90", "angle": math.pi / 2},
                    {"behavior": "spin", "name": "spin_behavior_minus_90", "angle": -math.pi / 2},
                    {"behavior": "spin", "name": "spin_behavior_180", "angle": math.pi},
                ]
            if not args.skip_backup_behavior:
                cases = cases + [
                    {"behavior": "backup", "name": "backup_0p5", "distance": 0.5, "speed": 0.20},
                    {"behavior": "backup", "name": "backup_1p0", "distance": 1.0, "speed": 0.20},
                ]

        print(f"Output: {outdir}")
        print(
            f"HOME map: x={home[0]:.3f} y={home[1]:.3f} "
            f"yaw={math.degrees(home[2]):.1f} deg"
        )
        print(f"Cases: {len(cases)}")
        print()

        consecutive_failures = 0

        for idx, case in enumerate(cases, 1):
            print("=" * 72)
            print(f"[{idx}/{len(cases)}] {case['name']}")
            print("=" * 72)

            if case.get("behavior") == "backup":
                result = node.run_backup_case(
                    case["name"], case["distance"], case["speed"]
                )
            elif case.get("behavior") == "spin":
                result = node.run_spin_case(case["name"], case["angle"])
            else:
                result = node.run_nav_case(case, home)

            results.append(result)
            write_reports(outdir, args, home, results)

            print(
                f"{result.get('status')}  "
                f"time={fmt(result.get('elapsed_sec'))}s  "
                f"err={fmt(result.get('final_position_error_m'))}m  "
                f"peak_wz={fmt(result.get('peak_nav2_wz'))}  "
                f"post_drift={fmt(result.get('post_goal_drift_m'))}m"
            )

            failed = result.get("status") not in (
                "SUCCEEDED",
                "SKIPPED_SERVER_UNAVAILABLE",
            )
            if failed:
                consecutive_failures += 1
            else:
                consecutive_failures = 0

            if consecutive_failures >= 2:
                print("Stopping suite after 2 consecutive failures.")
                break

            if failed and not args.continue_on_failure:
                print("Stopping after failure. Use --continue-on-failure to continue.")
                break

            # Return to the fixed home pose after position tests so each direction
            # is measured from the same geometric reference and the robot stays
            # within the bounded test area.
            if (
                args.home_between
                and case.get("behavior") is None
                and (
                    abs(case.get("x", 0.0)) > 1e-6
                    or abs(case.get("y", 0.0)) > 1e-6
                    or abs(case.get("yaw", 0.0)) > 1e-6
                )
                and result.get("status") == "SUCCEEDED"
            ):
                home_case = {
                    "name": f"return_home_after__{case['name']}",
                    "x": 0.0,
                    "y": 0.0,
                    "yaw": 0.0,
                    "category": "return_home",
                }
                print(f"Returning home after {case['name']} ...")
                ret = node.run_nav_case(home_case, home)
                results.append(ret)
                write_reports(outdir, args, home, results)
                if ret.get("status") != "SUCCEEDED":
                    print("Return-home failed; stopping suite for safety.")
                    break

            node.spin_for(args.settle)

        write_reports(outdir, args, home, results)
        print()
        print("BENCHMARK COMPLETE")
        print(f"Report: {outdir / 'report.md'}")
        print(f"CSV:    {outdir / 'results.csv'}")
        print(f"JSON:   {outdir / 'results.json'}")
        print(f"Raw:    {outdir / 'samples'}")
        return 0

    except KeyboardInterrupt:
        print("\nCtrl+C: canceling active Nav2 goal.")
        node.cancel_active()
        if "home" in locals():
            write_reports(outdir, args, home, results)
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
