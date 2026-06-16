from __future__ import annotations

import json
from typing import Any, Dict

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from go2_langgraph_agent.graphs.agent_orchestrator import AgentOrchestrator
from go2_langgraph_agent.persistence import JsonCheckpointer
from go2_langgraph_agent.tools.memory_tools import MemoryTools
from go2_langgraph_agent.tools.nav_tools import publish_nav_command


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _json_or_text(msg: String) -> Any:
    try:
        return json.loads(msg.data)
    except Exception:
        return msg.data


class MainSupervisor(Node):
    """Agentic supervisor that fuses live robot context into the task graph."""

    def __init__(self) -> None:
        super().__init__("go2_langgraph_main_supervisor")
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "default")
        self.declare_parameter("enable_debate_layer", True)
        self.declare_parameter("enable_tour_mode", False)
        self.declare_parameter("enable_explore_mode", False)
        self.declare_parameter("enable_nav_publish", True)
        self.declare_parameter("odom_topic", "/odom")
        self.session_root = str(self.get_parameter("session_root").value)
        self.session_name = str(self.get_parameter("session_name").value)
        self.memory = MemoryTools(self.session_root, self.session_name)
        self.checkpointer = JsonCheckpointer(self.session_root, self.session_name)
        self.latest: Dict[str, Any] = {}
        self.nav_pub = self.create_publisher(String, "/go2_nav/command", 10)
        self.status_pub = self.create_publisher(String, "/go2_agent/status", 10)
        self.speech_pub = self.create_publisher(String, "/go2_agent/speech", 10)
        for topic in ("/go2_agent/command", "/go2_agent/user_command", "/semantic_nav/command"):
            self.create_subscription(String, topic, self._on_command, 10)
        self.create_subscription(String, "/go2_perception/scan_summary", lambda m: self._store_json("scan_summary", m), 10)
        self.create_subscription(String, "/go2_perception/pointcloud_summary", lambda m: self._store_json("pointcloud_summary", m), 10)
        self.create_subscription(String, "/go2_perception/traversability_summary", lambda m: self._store_json("traversability", m), 10)
        self.create_subscription(String, "/go2_perception/open_vocab_detections", lambda m: self._store_json("open_vocab_detections", m), 10)
        self.create_subscription(String, "/go2_perception/dynamic_obstacles", lambda m: self._store_json("dynamic_obstacles", m), 10)
        self.create_subscription(String, "/go2_nav/frontier_candidates", lambda m: self._store_json("frontier_candidates", m), 10)
        self.create_subscription(String, "/go2_nav/coverage_plan", lambda m: self._store_json("coverage_waypoints", m), 10)
        self.create_subscription(String, "/go2_nav/status", lambda m: self._store_json("nav_status", m), 10)
        self.create_subscription(String, "/go2_vlm_checkpoint/status", lambda m: self._store_vlm(m), 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10)
        self.orchestrator = AgentOrchestrator(
            self.memory,
            self.checkpointer,
            {
                "enable_debate_layer": _as_bool(self.get_parameter("enable_debate_layer").value),
                "enable_tour_mode": _as_bool(self.get_parameter("enable_tour_mode").value),
                "enable_explore_mode": _as_bool(self.get_parameter("enable_explore_mode").value),
            },
            live_context_provider=self._live_context,
        )
        self.get_logger().info("Go2 agent supervisor ready: memory + multimodal goal selection + Nav2 command routing")

    def _store_json(self, key: str, msg: String) -> None:
        self.latest[key] = _json_or_text(msg)

    def _store_vlm(self, msg: String) -> None:
        value = _json_or_text(msg)
        self.latest["vlm_status"] = value
        if isinstance(value, dict) and value.get("summary"):
            self.latest["live_observation"] = value.get("summary")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        self.latest["odom"] = {
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "pose": {"x": p.x, "y": p.y, "z": p.z, "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w},
            "velocity": {"linear_x": t.linear.x, "linear_y": t.linear.y, "angular_z": t.angular.z},
        }

    def _live_context(self) -> Dict[str, Any]:
        return dict(self.latest)

    def _publish_status(self, payload: Dict[str, Any]) -> None:
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _on_command(self, msg: String) -> None:
        state = self.orchestrator.invoke(msg.data)
        command = state.get("nav_command") or {}
        if command and _as_bool(self.get_parameter("enable_nav_publish").value):
            publish_nav_command(self.nav_pub, command.get("action", "none"), command)
        speech = state.get("speech_response", "")
        if speech:
            self.speech_pub.publish(String(data=speech))
        self._publish_status(state.get("status", {}))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MainSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
