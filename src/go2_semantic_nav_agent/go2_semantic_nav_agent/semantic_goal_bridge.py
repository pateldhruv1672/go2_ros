from __future__ import annotations

from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from go2_semantic_nav_agent.semantic_demo_utils import (
    SemanticPose,
    best_pose_match,
    load_semantic_poses,
)


STOP_WORDS = (
    "stop", "halt", "freeze", "cancel", "cancel navigation", "emergency stop",
    "abort", "pause navigation",
)


class SemanticGoalBridge(Node):
    def __init__(self) -> None:
        super().__init__("semantic_goal_bridge")
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "latest")
        self.declare_parameter("text_command_topic", "/semantic_nav/text_command")
        self.declare_parameter("omi_transcript_topic", "/omi/transcript_raw")
        self.declare_parameter("voice_transcript_topic", "/go2_voice/transcript")
        self.declare_parameter("status_topic", "/semantic_nav/goal_bridge_status")
        self.declare_parameter("target_marker_topic", "/semantic_nav/target_marker")
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("require_nav2_server", True)
        self.declare_parameter("server_wait_timeout_sec", 2.0)
        self.declare_parameter("ignore_commands_without_sparky", False)

        self.status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.get_parameter("target_marker_topic").value, 10)
        self.nav_client = ActionClient(self, NavigateToPose, self.get_parameter("navigate_action").value)
        self.goal_handle = None

        for topic in (
            self.get_parameter("text_command_topic").value,
            self.get_parameter("omi_transcript_topic").value,
            self.get_parameter("voice_transcript_topic").value,
        ):
            if topic:
                self.create_subscription(String, str(topic), self._on_command, 10)
                self.get_logger().info(f"Listening for semantic commands on {topic}")

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _load_poses(self):
        root = str(self.get_parameter("session_root").value)
        name = str(self.get_parameter("session_name").value)
        return load_semantic_poses(root, name)

    def _is_stop(self, command: str) -> bool:
        c = command.strip().lower()
        return any(word in c for word in STOP_WORDS)

    def _on_command(self, msg: String) -> None:
        command = (msg.data or "").strip()
        if not command:
            return
        if bool(self.get_parameter("ignore_commands_without_sparky").value):
            if "sparky" not in command.lower():
                return

        if self._is_stop(command):
            self._cancel_goal(command)
            return

        try:
            session_dir, poses = self._load_poses()
        except Exception as exc:
            self._publish_status(f"ERROR failed_to_load_session error={exc}")
            return

        target = best_pose_match(command, poses)
        if target is None:
            names = ", ".join(sorted({p.name for p in poses})[:12])
            self._publish_status(f"NO_MATCH command='{command}' known=[{names}] session={session_dir}")
            return

        self._send_nav_goal(command, target)

    def _cancel_goal(self, command: str) -> None:
        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(lambda _: self._publish_status(f"CANCEL requested command='{command}'"))
        else:
            self._publish_status(f"CANCEL ignored no_active_goal command='{command}'")

    def _send_nav_goal(self, command: str, target: SemanticPose) -> None:
        timeout = float(self.get_parameter("server_wait_timeout_sec").value)
        require_server = bool(self.get_parameter("require_nav2_server").value)
        if not self.nav_client.wait_for_server(timeout_sec=timeout):
            text = f"NAV2_UNAVAILABLE action={self.get_parameter('navigate_action').value} command='{command}'"
            if require_server:
                self._publish_status("ERROR " + text)
                return
            self._publish_status("WARN " + text)

        goal = NavigateToPose.Goal()
        goal.pose = target.to_pose_stamped(self.get_clock().now().to_msg())
        goal.behavior_tree = ""
        self._publish_target_marker(target)
        self._publish_status(
            f"SENDING_GOAL name='{target.name}' x={target.x:.2f} y={target.y:.2f} yaw={target.yaw:.2f} command='{command}'"
        )

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_goal_response(fut, target))

    def _on_goal_response(self, future, target: SemanticPose) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status(f"GOAL_REJECTED name='{target.name}'")
            return
        self.goal_handle = goal_handle
        self._publish_status(f"GOAL_ACCEPTED name='{target.name}'")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_result(fut, target))

    def _on_result(self, future, target: SemanticPose) -> None:
        result = future.result()
        status = result.status if result is not None else -1
        label = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"STATUS_{status}")
        self._publish_status(f"GOAL_{label} name='{target.name}'")
        self.goal_handle = None

    def _publish_target_marker(self, target: SemanticPose) -> None:
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        sphere = Marker()
        sphere.header.frame_id = target.frame_id or "map"
        sphere.header.stamp = now
        sphere.ns = "semantic_target"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = target.x
        sphere.pose.position.y = target.y
        sphere.pose.position.z = 0.55
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.42
        sphere.scale.y = 0.42
        sphere.scale.z = 0.42
        sphere.color.r = 1.0
        sphere.color.g = 0.15
        sphere.color.b = 0.1
        sphere.color.a = 0.95
        arr.markers.append(sphere)

        text = Marker()
        text.header.frame_id = target.frame_id or "map"
        text.header.stamp = now
        text.ns = "semantic_target_label"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = target.x
        text.pose.position.y = target.y
        text.pose.position.z = 1.05
        text.pose.orientation.w = 1.0
        text.scale.z = 0.28
        text.color.r = 1.0
        text.color.g = 0.25
        text.color.b = 0.15
        text.color.a = 1.0
        text.text = f"TARGET: {target.name}"
        arr.markers.append(text)

        self.marker_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticGoalBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
