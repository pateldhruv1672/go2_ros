from __future__ import annotations

import json
from typing import Any, Dict, List

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String


def recovery_plan(reason: str, localization_confidence: float = 0.0, nav_status: Dict[str, Any] | None = None) -> Dict[str, Any]:
    nav_status = nav_status or {}
    steps: List[str] = ["stop_robot"]
    if reason == "safety_blocked":
        steps += ["hold_position", "wait_for_dynamic_obstacles_to_clear", "ask_human_before_retry"]
    elif localization_confidence < 0.55 or reason == "low_localization":
        steps += ["publish_initial_pose_from_saved_spawn", "request_scan_map_match", "wait_for_amcl_confidence", "ask_human_if_needed"]
    elif nav_status.get("last_result") == "failed" or reason in {"nav_failure", "recover_nav_failure"}:
        steps += ["clear_nav2_costmaps", "try_nearby_safe_anchor", "reroute_or_return_to_spawn"]
    else:
        steps += ["clear_nav2_costmaps", "retry_once", "return_to_spawn_if_retry_fails"]
    return {"reason": reason, "steps": steps, "requires_human_interrupt": localization_confidence < 0.35 or reason == "safety_blocked"}


class RecoveryManager(Node):
    def __init__(self) -> None:
        super().__init__("go2_recovery_manager")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_out")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("enable_initial_pose_publish", False)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, str(self.get_parameter("initial_pose_topic").value), 10)
        self.status_pub = self.create_publisher(String, "/go2_nav/recovery_status", 10)
        self.create_subscription(String, "/go2_nav/recovery_request", self._on_request, 10)
        self.get_logger().info("Recovery manager ready: stop, initial-pose, costmap-clear plan publishing")

    def _publish(self, payload: Dict[str, Any]) -> None:
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _on_request(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
        except Exception:
            req = {"reason": msg.data}
        reason = str(req.get("reason") or req.get("issue") or "unknown")
        loc = float(req.get("localization_confidence", 0.0) or 0.0)
        plan = recovery_plan(reason, loc, req.get("nav_status") if isinstance(req.get("nav_status"), dict) else {})
        self.cmd_pub.publish(Twist())
        spawn = req.get("spawn_pose") or req.get("pose") or {}
        if self.get_parameter("enable_initial_pose_publish").value and isinstance(spawn, dict) and spawn:
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = str(spawn.get("frame_id", "map"))
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.pose.position.x = float(spawn.get("x", 0.0))
            pose.pose.pose.position.y = float(spawn.get("y", 0.0))
            pose.pose.pose.orientation.z = float(spawn.get("qz", 0.0))
            pose.pose.pose.orientation.w = float(spawn.get("qw", 1.0))
            pose.pose.covariance[0] = 0.25
            pose.pose.covariance[7] = 0.25
            pose.pose.covariance[35] = 0.1
            self.initial_pose_pub.publish(pose)
            plan["initial_pose_published"] = True
        self._publish({"success": True, "plan": plan})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
