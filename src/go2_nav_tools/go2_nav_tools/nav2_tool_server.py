from __future__ import annotations

import json
import math
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def pose_from_dict(node: Node, data: Dict[str, Any]) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = str(data.get("frame_id", "map"))
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(data["x"])
    pose.pose.position.y = float(data["y"])
    pose.pose.position.z = float(data.get("z", 0.0))
    if finite(data.get("yaw")):
        yaw = float(data["yaw"])
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
    else:
        pose.pose.orientation.x = float(data.get("qx", 0.0))
        pose.pose.orientation.y = float(data.get("qy", 0.0))
        pose.pose.orientation.z = float(data.get("qz", 0.0))
        pose.pose.orientation.w = float(data.get("qw", 1.0))
    return pose


def pose_list(command: Dict[str, Any]) -> List[Dict[str, Any]]:
    if isinstance(command.get("poses"), list):
        return [p for p in command["poses"] if isinstance(p, dict)]
    if isinstance(command.get("pose"), dict):
        return [command["pose"]]
    return []


class Nav2ToolServer(Node):
    def __init__(self) -> None:
        super().__init__("go2_nav2_tool_server")
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("preflight_path", True)
        self.declare_parameter("allowed_goal_frame", "map")
        self.declare_parameter("stop_topic", "/go2_motion/stop")
        self.status_pub = self.create_publisher(String, "/go2_nav/status", 20)
        self.recovery_pub = self.create_publisher(String, "/go2_nav/recovery_request", 10)
        self.stop_pub = self.create_publisher(Bool, str(self.get_parameter("stop_topic").value), 10)
        self.create_subscription(String, "/go2_nav/command", self.on_command, 20)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.through_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self.path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.active_handle = None
        self.command_seq = 0
        self.get_logger().info("safe Nav2 tool server ready")

    @property
    def motion_enabled(self) -> bool:
        return as_bool(self.get_parameter("enable_motion").value)

    def publish(self, payload: Dict[str, Any]) -> None:
        payload.setdefault("motion_enabled", self.motion_enabled)
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def valid_pose(self, data: Dict[str, Any]) -> Optional[str]:
        if not finite(data.get("x")) or not finite(data.get("y")):
            return "pose requires finite x and y"
        frame = str(data.get("frame_id", "map"))
        allowed = str(self.get_parameter("allowed_goal_frame").value)
        if frame != allowed:
            return f"goal frame {frame!r} is not allowed; expected {allowed!r}"
        return None

    def stop(self, reason: str) -> None:
        if self.active_handle is not None:
            try:
                self.active_handle.cancel_goal_async()
            except Exception:
                pass
            self.active_handle = None
        self.stop_pub.publish(Bool(data=True))
        self.publish({"success": True, "action": "stop_robot", "reason": reason})

    def on_command(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
            if not isinstance(command, dict):
                raise ValueError("command must be a JSON object")
        except Exception as exc:
            self.publish({"success": False, "action": "parse_error", "message": str(exc)})
            return
        action = str(command.get("action", ""))
        if action in {"stop", "stop_robot", "pause"}:
            self.stop(str(command.get("reason", action)))
        elif action in {"navigate_to_pose", "frontier_explore", "navigate_to_object"}:
            poses = pose_list(command)
            if not poses:
                self.publish({"success": False, "action": action, "message": "missing pose"})
                return
            self.navigate_one(command, poses[0])
        elif action in {"navigate_through_poses", "navigate_tour_route"}:
            self.navigate_many(command, pose_list(command))
        elif action in {"recover_nav_failure", "recover_localization"}:
            self.stop(action)
            self.recovery_pub.publish(String(data=json.dumps(command, sort_keys=True)))
        else:
            self.publish({"success": False, "action": action, "message": "unsupported action"})

    def navigate_one(self, command: Dict[str, Any], data: Dict[str, Any]) -> None:
        error = self.valid_pose(data)
        if error:
            self.publish({"success": False, "action": command.get("action"), "message": error})
            return
        if not self.motion_enabled:
            self.publish({"success": True, "dry_run": True, "action": command.get("action"), "pose": data})
            return
        self.command_seq += 1
        seq = self.command_seq
        pose = pose_from_dict(self, data)
        if as_bool(self.get_parameter("preflight_path").value):
            if not self.path_client.wait_for_server(timeout_sec=2.0):
                self.publish({"success": False, "action": "preflight", "message": "compute_path_to_pose unavailable", "seq": seq})
                return
            goal = ComputePathToPose.Goal()
            goal.goal = pose
            future = self.path_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.on_preflight_response(f, pose, command, seq))
            self.publish({"success": True, "action": "preflight_sent", "seq": seq, "pose": data})
        else:
            self.send_nav(pose, command, seq)

    def on_preflight_response(self, future, pose: PoseStamped, command: Dict[str, Any], seq: int) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                self.publish({"success": False, "action": "preflight_rejected", "seq": seq})
                return
            handle.get_result_async().add_done_callback(lambda f: self.on_preflight_result(f, pose, command, seq))
        except Exception as exc:
            self.publish({"success": False, "action": "preflight_error", "seq": seq, "message": str(exc)})

    def on_preflight_result(self, future, pose: PoseStamped, command: Dict[str, Any], seq: int) -> None:
        try:
            wrapped = future.result()
            path = wrapped.result.path
            if int(wrapped.status) != 4 or not path.poses:
                self.publish({"success": False, "action": "preflight_no_path", "seq": seq, "status": int(wrapped.status)})
                return
        except Exception as exc:
            self.publish({"success": False, "action": "preflight_error", "seq": seq, "message": str(exc)})
            return
        self.send_nav(pose, command, seq)

    def send_nav(self, pose: PoseStamped, command: Dict[str, Any], seq: int) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.publish({"success": False, "action": "navigate_to_pose", "message": "action server unavailable", "seq": seq})
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.on_goal_response(f, command, seq))
        self.publish({"success": True, "action": "navigate_to_pose_sent", "seq": seq})

    def navigate_many(self, command: Dict[str, Any], poses_data: List[Dict[str, Any]]) -> None:
        if not poses_data:
            self.publish({"success": False, "action": command.get("action"), "message": "missing poses"})
            return
        for item in poses_data:
            error = self.valid_pose(item)
            if error:
                self.publish({"success": False, "action": command.get("action"), "message": error})
                return
        if not self.motion_enabled:
            self.publish({"success": True, "dry_run": True, "action": command.get("action"), "pose_count": len(poses_data)})
            return
        if not self.through_client.wait_for_server(timeout_sec=2.0):
            self.publish({"success": False, "action": command.get("action"), "message": "action server unavailable"})
            return
        goal = NavigateThroughPoses.Goal()
        goal.poses = [pose_from_dict(self, p) for p in poses_data]
        future = self.through_client.send_goal_async(goal)
        self.command_seq += 1
        seq = self.command_seq
        future.add_done_callback(lambda f: self.on_goal_response(f, command, seq))
        self.publish({"success": True, "action": "navigate_through_poses_sent", "seq": seq, "pose_count": len(poses_data)})

    def on_goal_response(self, future, command: Dict[str, Any], seq: int) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                self.publish({"success": False, "action": "goal_rejected", "seq": seq})
                return
            self.active_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.on_goal_result(f, command, seq))
            self.publish({"success": True, "action": "goal_accepted", "seq": seq, "source_action": command.get("action")})
        except Exception as exc:
            self.publish({"success": False, "action": "goal_response_error", "seq": seq, "message": str(exc)})

    def on_goal_result(self, future, command: Dict[str, Any], seq: int) -> None:
        self.active_handle = None
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            self.publish({"success": status == 4, "action": "goal_result", "seq": seq, "status": status, "source_action": command.get("action")})
        except Exception as exc:
            self.publish({"success": False, "action": "goal_result_error", "seq": seq, "message": str(exc)})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2ToolServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
