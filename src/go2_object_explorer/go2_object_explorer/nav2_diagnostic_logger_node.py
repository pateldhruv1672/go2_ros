#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
import math
import os
import re
import time
from pathlib import Path
from typing import Optional

import rclpy
import tf2_ros

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from rcl_interfaces.msg import Log
from std_msgs.msg import String


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Nav2DiagnosticLogger(Node):
    NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"

    EXPLORER_GOAL_RE = re.compile(
        rf"sent_nav_goal.*?"
        rf"kind=(\S+).*?"
        rf"x=({NUMBER}).*?"
        rf"y=({NUMBER}).*?"
        rf"yaw=({NUMBER})"
    )

    BT_GOAL_RE = re.compile(
        rf"Begin navigating from current location\s*"
        rf"\(({NUMBER}),\s*({NUMBER})\)\s*"
        rf"to\s*\(({NUMBER}),\s*({NUMBER})\)"
    )

    STATUS_NAMES = {
        0: "UNKNOWN",
        1: "ACCEPTED",
        2: "EXECUTING",
        3: "CANCELING",
        4: "SUCCEEDED",
        5: "CANCELED",
        6: "ABORTED",
    }

    def __init__(self) -> None:
        super().__init__("nav2_diagnostic_logger_node")

        self.declare_parameter(
            "log_dir",
            "~/.ros/go2_object_explorer/nav2_diagnostics",
        )
        self.declare_parameter("sample_hz", 5.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        log_root = Path(
            os.path.expanduser(
                str(self.get_parameter("log_dir").value)
            )
        )
        log_root.mkdir(parents=True, exist_ok=True)

        session_name = (
            time.strftime("%Y%m%d_%H%M%S")
            + f"_{os.getpid()}"
        )

        self.session_dir = log_root / session_name
        self.session_dir.mkdir(parents=True)

        (log_root / "LATEST").write_text(
            str(self.session_dir),
            encoding="utf-8",
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_map_pose: Optional[
            tuple[float, float, float]
        ] = None
        self.active_goal: Optional[
            tuple[float, float, float, str]
        ] = None

        self.goal_number = 0
        self.last_status = {}

        self.pose_handle, self.pose_writer = self.open_csv(
            "robot_pose.csv",
            [
                "unix_time",
                "ros_time",
                "map_x",
                "map_y",
                "map_yaw",
                "odom_x",
                "odom_y",
                "odom_yaw",
                "odom_linear_x",
                "odom_linear_y",
                "odom_angular_z",
                "goal_x",
                "goal_y",
                "goal_yaw",
                "goal_source",
                "distance_to_goal",
            ],
        )

        self.goal_handle, self.goal_writer = self.open_csv(
            "goals.csv",
            [
                "unix_time",
                "ros_time",
                "goal_number",
                "source",
                "kind",
                "x",
                "y",
                "yaw",
                "raw_message",
            ],
        )

        self.plan_handle, self.plan_writer = self.open_csv(
            "plans.csv",
            [
                "unix_time",
                "ros_time",
                "topic",
                "frame_id",
                "pose_count",
                "path_length_m",
                "start_x",
                "start_y",
                "end_x",
                "end_y",
            ],
        )

        self.cmd_handle, self.cmd_writer = self.open_csv(
            "cmd_vel.csv",
            [
                "unix_time",
                "ros_time",
                "topic",
                "linear_x",
                "linear_y",
                "angular_z",
            ],
        )

        self.status_handle, self.status_writer = self.open_csv(
            "nav_status.csv",
            [
                "unix_time",
                "ros_time",
                "goal_uuid",
                "status",
                "status_name",
            ],
        )

        self.trace_handle = open(
            self.session_dir / "events.jsonl",
            "w",
            encoding="utf-8",
            buffering=1,
        )

        self.rosout_handle = open(
            self.session_dir / "rosout.log",
            "w",
            encoding="utf-8",
            buffering=1,
        )

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=Duration(seconds=20.0)
        )
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.on_odom,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.on_goal_pose,
            10,
        )

        self.create_subscription(
            NavPath,
            "/plan",
            lambda msg: self.on_plan("/plan", msg),
            10,
        )

        self.create_subscription(
            NavPath,
            "/local_plan",
            lambda msg: self.on_plan("/local_plan", msg),
            10,
        )

        for topic in (
            "/cmd_vel_nav2",
            "/cmd_vel_nav",
            "/cmd_vel_out",
            "/cmd_vel_omi",
            "/cmd_vel_escape",
        ):
            self.create_subscription(
                Twist,
                topic,
                lambda msg, name=topic: self.on_cmd(name, msg),
                20,
            )

        self.create_subscription(
            String,
            "/object_explorer/state",
            lambda msg: self.on_json_topic(
                "/object_explorer/state",
                msg,
            ),
            20,
        )

        self.create_subscription(
            String,
            "/object_explorer/llm_decision",
            lambda msg: self.on_json_topic(
                "/object_explorer/llm_decision",
                msg,
            ),
            20,
        )

        rosout_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=500,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Log,
            "/rosout",
            self.on_rosout,
            rosout_qos,
        )

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.on_status,
            status_qos,
        )

        sample_hz = max(
            0.5,
            float(self.get_parameter("sample_hz").value),
        )

        self.create_timer(
            1.0 / sample_hz,
            self.sample_pose,
        )

        self.write_event(
            "logger_started",
            session_dir=str(self.session_dir),
            sample_hz=sample_hz,
        )

        self.get_logger().info(
            f"Diagnostic logs: {self.session_dir}"
        )

    def open_csv(self, name, columns):
        handle = open(
            self.session_dir / name,
            "w",
            newline="",
            encoding="utf-8",
            buffering=1,
        )
        writer = csv.writer(handle)
        writer.writerow(columns)
        handle.flush()
        return handle, writer

    def ros_time(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def write_event(self, event: str, **data) -> None:
        record = {
            "unix_time": time.time(),
            "ros_time": self.ros_time(),
            "event": event,
            **data,
        }

        self.trace_handle.write(
            json.dumps(record, default=str) + "\n"
        )
        self.trace_handle.flush()

    def record_goal(
        self,
        source: str,
        kind: str,
        x: float,
        y: float,
        yaw: float,
        raw_message: str = "",
    ) -> None:
        self.goal_number += 1
        self.active_goal = (x, y, yaw, source)

        self.goal_writer.writerow(
            [
                time.time(),
                self.ros_time(),
                self.goal_number,
                source,
                kind,
                x,
                y,
                yaw,
                raw_message,
            ]
        )
        self.goal_handle.flush()

        self.write_event(
            "goal",
            goal_number=self.goal_number,
            source=source,
            kind=kind,
            x=x,
            y=y,
            yaw=yaw,
        )

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def on_goal_pose(self, msg: PoseStamped) -> None:
        self.record_goal(
            source="/goal_pose",
            kind="pose_goal",
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            yaw=yaw_from_quaternion(msg.pose.orientation),
        )

    def on_cmd(self, topic: str, msg: Twist) -> None:
        self.cmd_writer.writerow(
            [
                time.time(),
                self.ros_time(),
                topic,
                float(msg.linear.x),
                float(msg.linear.y),
                float(msg.angular.z),
            ]
        )
        self.cmd_handle.flush()

    def on_plan(self, topic: str, msg: NavPath) -> None:
        if not msg.poses:
            self.write_event(
                "empty_plan",
                topic=topic,
            )
            return

        points = [
            (
                float(item.pose.position.x),
                float(item.pose.position.y),
            )
            for item in msg.poses
        ]

        path_length = sum(
            math.hypot(
                points[index][0] - points[index - 1][0],
                points[index][1] - points[index - 1][1],
            )
            for index in range(1, len(points))
        )

        self.plan_writer.writerow(
            [
                time.time(),
                self.ros_time(),
                topic,
                msg.header.frame_id,
                len(points),
                path_length,
                points[0][0],
                points[0][1],
                points[-1][0],
                points[-1][1],
            ]
        )
        self.plan_handle.flush()

        self.write_event(
            "plan",
            topic=topic,
            path_length_m=path_length,
            pose_count=len(points),
            end_x=points[-1][0],
            end_y=points[-1][1],
        )

    def on_json_topic(
        self,
        topic: str,
        msg: String,
    ) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"raw": msg.data}

        self.write_event(
            "topic_message",
            topic=topic,
            payload=payload,
        )

    def on_rosout(self, msg: Log) -> None:
        line = (
            f"{time.time():.6f} "
            f"level={msg.level} "
            f"name={msg.name} "
            f"message={msg.msg}\n"
        )

        self.rosout_handle.write(line)
        self.rosout_handle.flush()

        explorer_match = self.EXPLORER_GOAL_RE.search(msg.msg)

        if explorer_match:
            self.record_goal(
                source="frontier_object_explorer",
                kind=explorer_match.group(1),
                x=float(explorer_match.group(2)),
                y=float(explorer_match.group(3)),
                yaw=float(explorer_match.group(4)),
                raw_message=msg.msg,
            )

        bt_match = self.BT_GOAL_RE.search(msg.msg)

        if bt_match:
            self.record_goal(
                source="bt_navigator",
                kind="accepted_goal",
                x=float(bt_match.group(3)),
                y=float(bt_match.group(4)),
                yaw=math.nan,
                raw_message=msg.msg,
            )

    def on_status(self, msg: GoalStatusArray) -> None:
        for item in msg.status_list:
            goal_uuid = "".join(
                f"{value:02x}"
                for value in item.goal_info.goal_id.uuid
            )
            status = int(item.status)

            if self.last_status.get(goal_uuid) == status:
                continue

            self.last_status[goal_uuid] = status

            status_name = self.STATUS_NAMES.get(
                status,
                "INVALID",
            )

            self.status_writer.writerow(
                [
                    time.time(),
                    self.ros_time(),
                    goal_uuid,
                    status,
                    status_name,
                ]
            )
            self.status_handle.flush()

            self.write_event(
                "navigation_status",
                goal_uuid=goal_uuid,
                status=status,
                status_name=status_name,
            )

            if status in (4, 5, 6):
                self.active_goal = None

    def sample_pose(self) -> None:
        map_x = math.nan
        map_y = math.nan
        map_yaw = math.nan

        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                Time(),
            )

            map_x = float(transform.transform.translation.x)
            map_y = float(transform.transform.translation.y)
            map_yaw = yaw_from_quaternion(
                transform.transform.rotation
            )

            self.latest_map_pose = (
                map_x,
                map_y,
                map_yaw,
            )

        except Exception as exc:
            self.write_event(
                "tf_lookup_error",
                error=str(exc),
            )

        odom_x = math.nan
        odom_y = math.nan
        odom_yaw = math.nan
        odom_vx = math.nan
        odom_vy = math.nan
        odom_wz = math.nan

        if self.latest_odom is not None:
            pose = self.latest_odom.pose.pose
            twist = self.latest_odom.twist.twist

            odom_x = float(pose.position.x)
            odom_y = float(pose.position.y)
            odom_yaw = yaw_from_quaternion(
                pose.orientation
            )
            odom_vx = float(twist.linear.x)
            odom_vy = float(twist.linear.y)
            odom_wz = float(twist.angular.z)

        goal_x = math.nan
        goal_y = math.nan
        goal_yaw = math.nan
        goal_source = ""
        distance_to_goal = math.nan

        if self.active_goal is not None:
            (
                goal_x,
                goal_y,
                goal_yaw,
                goal_source,
            ) = self.active_goal

            if math.isfinite(map_x) and math.isfinite(map_y):
                distance_to_goal = math.hypot(
                    goal_x - map_x,
                    goal_y - map_y,
                )

        self.pose_writer.writerow(
            [
                time.time(),
                self.ros_time(),
                map_x,
                map_y,
                map_yaw,
                odom_x,
                odom_y,
                odom_yaw,
                odom_vx,
                odom_vy,
                odom_wz,
                goal_x,
                goal_y,
                goal_yaw,
                goal_source,
                distance_to_goal,
            ]
        )
        self.pose_handle.flush()

    def close(self) -> None:
        self.write_event("logger_stopping")

        for handle in (
            self.pose_handle,
            self.goal_handle,
            self.plan_handle,
            self.cmd_handle,
            self.status_handle,
            self.trace_handle,
            self.rosout_handle,
        ):
            try:
                handle.flush()
                handle.close()
            except Exception:
                pass



def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2DiagnosticLogger()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        # Normal exit after Ctrl+C, timeout, SIGTERM, or launch shutdown.
        pass

    finally:
        try:
            # Support either version of the logger created earlier.
            if hasattr(node, "close_files"):
                node.close_files()
            elif hasattr(node, "close"):
                node.close()
        except Exception as exc:
            print(f"Logger file-close warning: {exc}")

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
