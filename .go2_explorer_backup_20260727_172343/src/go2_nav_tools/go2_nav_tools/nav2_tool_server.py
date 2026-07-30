from __future__ import annotations

import json
from typing import Any, Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _pose_from_dict(node: Node, pose_data: Dict[str, Any]) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = str(pose_data.get("frame_id", "map"))
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(pose_data.get("x", 0.0))
    pose.pose.position.y = float(pose_data.get("y", 0.0))
    pose.pose.position.z = float(pose_data.get("z", 0.0))
    pose.pose.orientation.x = float(pose_data.get("qx", 0.0))
    pose.pose.orientation.y = float(pose_data.get("qy", 0.0))
    pose.pose.orientation.z = float(pose_data.get("qz", 0.0))
    pose.pose.orientation.w = float(pose_data.get("qw", 1.0))
    return pose


def _extract_pose_list(command: Dict[str, Any]) -> List[Dict[str, Any]]:
    if isinstance(command.get("poses"), list):
        return [p for p in command["poses"] if isinstance(p, dict)]
    if isinstance(command.get("pose"), dict) and command["pose"]:
        return [command["pose"]]
    route = command.get("route") or []
    poses: List[Dict[str, Any]] = []
    for item in route:
        if not isinstance(item, dict):
            continue
        pose = item.get("pose") if isinstance(item.get("pose"), dict) else item
        if isinstance(pose, dict) and "x" in pose and "y" in pose:
            poses.append(pose)
    selected = command.get("selected_goal") or {}
    if isinstance(selected, dict) and isinstance(selected.get("pose"), dict):
        poses.insert(0, selected["pose"])
    return poses


class Nav2ToolServer(Node):
    def __init__(self) -> None:
        super().__init__("go2_nav2_tool_server")
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_out")
        self.status_pub = self.create_publisher(String, "/go2_nav/status", 10)
        self.recovery_pub = self.create_publisher(String, "/go2_nav/recovery_request", 10)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.create_subscription(String, "/go2_nav/command", self._on_command, 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.through_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self.get_logger().info("Nav2 tool server ready; motion disabled unless enable_motion:=true")

    @property
    def motion_enabled(self) -> bool:
        return _as_bool(self.get_parameter("enable_motion").value)

    def _publish_status(self, payload: dict) -> None:
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _on_command(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
            action = command.get("action")
            if action == "stop_robot":
                self._stop_robot(command)
            elif action in {"navigate_to_pose", "frontier_explore", "coverage_explore"}:
                self._navigate_to_pose(command)
            elif action in {"navigate_through_poses", "navigate_tour_route"}:
                self._navigate_through_poses(command)
            elif action == "return_to_spawn":
                self._navigate_to_pose({"action": "navigate_to_pose", "pose": self._spawn_pose_from_command(command), "source_action": action})
            elif action == "navigate_to_place":
                self._navigate_to_place(command)
            elif action in {"recover_localization", "recover_nav_failure"}:
                self._request_recovery(command)
            elif action == "manual_assisted_explore":
                self._publish_status({"success": True, "action": action, "message": "manual mode; memory stack should keep recording"})
            else:
                self._publish_status({"success": False, "message": f"unknown nav action {action}", "command": command})
        except Exception as exc:
            self._publish_status({"success": False, "message": str(exc)})

    def _stop_robot(self, command: Dict[str, Any] | None = None) -> None:
        self.cmd_pub.publish(Twist())
        self._publish_status({"success": True, "action": "stop_robot", "motion_enabled": self.motion_enabled, "reason": (command or {}).get("reason")})

    def _spawn_pose_from_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        if isinstance(command.get("spawn_pose"), dict):
            return command["spawn_pose"]
        poses = _extract_pose_list(command)
        return poses[0] if poses else {}

    def _navigate_to_place(self, command: Dict[str, Any]) -> None:
        poses = _extract_pose_list(command)
        if poses:
            self._navigate_to_pose({"action": "navigate_to_pose", "pose": poses[0], "source_action": "navigate_to_place", "destination": command.get("destination")})
        else:
            self._publish_status({"success": False, "action": "navigate_to_place", "message": "destination resolved but no pose available", "destination": command.get("destination")})

    def _navigate_to_pose(self, command: dict) -> None:
        poses = _extract_pose_list(command)
        pose_data = poses[0] if poses else command.get("pose", {})
        if not pose_data:
            self._publish_status({"success": False, "action": command.get("action"), "message": "no pose available"})
            return
        if not self.motion_enabled:
            self._publish_status({"success": True, "dry_run": True, "action": command.get("action", "navigate_to_pose"), "pose": pose_data, "selected_goal": command.get("selected_goal")})
            return
        goal = NavigateToPose.Goal()
        goal.pose = _pose_from_dict(self, pose_data)
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self._publish_status({"success": False, "message": "navigate_to_pose action server unavailable"})
            return
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_goal_response(f, command.get("action", "navigate_to_pose")))
        self._publish_status({"success": True, "action": "navigate_to_pose_sent", "source_action": command.get("action"), "pose": pose_data})

    def _navigate_through_poses(self, command: dict) -> None:
        poses = _extract_pose_list(command)
        if not poses:
            self._publish_status({"success": False, "action": command.get("action"), "message": "no route poses available"})
            return
        if not self.motion_enabled:
            self._publish_status({"success": True, "dry_run": True, "action": command.get("action", "navigate_through_poses"), "pose_count": len(poses), "poses": poses[:8]})
            return
        goal = NavigateThroughPoses.Goal()
        goal.poses = [_pose_from_dict(self, p) for p in poses]
        if not self.through_client.wait_for_server(timeout_sec=1.0):
            self._publish_status({"success": False, "message": "navigate_through_poses action server unavailable"})
            return
        fut = self.through_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_goal_response(f, command.get("action", "navigate_through_poses")))
        self._publish_status({"success": True, "action": "navigate_through_poses_sent", "pose_count": len(poses)})

    def _request_recovery(self, command: Dict[str, Any]) -> None:
        self._stop_robot(command)
        self.recovery_pub.publish(String(data=json.dumps(command, sort_keys=True)))
        self._publish_status({"success": True, "action": "recovery_requested", "issue": command.get("issue") or command.get("reason")})

    def _on_goal_response(self, future, source_action: str) -> None:
        try:
            handle = future.result()
            accepted = bool(handle.accepted)
            self._publish_status({"success": accepted, "action": "goal_response", "source_action": source_action, "accepted": accepted})
            if accepted:
                result_future = handle.get_result_async()
                result_future.add_done_callback(lambda f: self._on_goal_result(f, source_action))
        except Exception as exc:
            self._publish_status({"success": False, "action": "goal_response_error", "source_action": source_action, "message": str(exc)})

    def _on_goal_result(self, future, source_action: str) -> None:
        try:
            result = future.result()
            status = getattr(result, "status", None)
            self._publish_status({"success": status == 4, "action": "goal_result", "source_action": source_action, "status": status})
        except Exception as exc:
            self._publish_status({"success": False, "action": "goal_result_error", "source_action": source_action, "message": str(exc)})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2ToolServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
