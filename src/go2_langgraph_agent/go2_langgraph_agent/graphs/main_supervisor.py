from __future__ import annotations

import json
from typing import Any, Dict

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from go2_langgraph_agent.graphs.agent_orchestrator import AgentOrchestrator
from go2_langgraph_agent.graphs.agent_state import LangGraphDependencyError
from go2_langgraph_agent.persistence import LangGraphSQLitePersistence
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
    """ROS adapter for the real LangGraph Go2 agent runtime."""

    def __init__(self) -> None:
        super().__init__("go2_langgraph_main_supervisor")
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "default")
        self.declare_parameter("thread_id", "")
        self.declare_parameter("enable_debate_layer", True)
        self.declare_parameter("enable_tour_mode", False)
        self.declare_parameter("enable_explore_mode", False)
        self.declare_parameter("enable_nav_publish", True)
        self.declare_parameter("enable_human_interrupts", False)
        self.declare_parameter("enable_langgraph_streaming", True)
        self.declare_parameter("langgraph_checkpoint_db", "")
        self.declare_parameter("odom_topic", "/odom")
        self.session_root = str(self.get_parameter("session_root").value)
        self.session_name = str(self.get_parameter("session_name").value)
        thread_param = str(self.get_parameter("thread_id").value or "")
        self.thread_id = thread_param or self.session_name
        self.memory = MemoryTools(self.session_root, self.session_name)
        checkpoint_db = str(self.get_parameter("langgraph_checkpoint_db").value or "") or None
        self.persistence = LangGraphSQLitePersistence(self.session_root, self.session_name, checkpoint_db)
        self.latest: Dict[str, Any] = {}
        self.nav_pub = self.create_publisher(String, "/go2_nav/command", 10)
        self.status_pub = self.create_publisher(String, "/go2_agent/status", 10)
        self.speech_pub = self.create_publisher(String, "/go2_agent/speech", 10)
        self.event_pub = self.create_publisher(String, "/go2_agent/events", 10)
        self.interrupt_pub = self.create_publisher(String, "/go2_agent/interrupts", 10)
        for topic in ("/go2_agent/command", "/go2_agent/user_command", "/semantic_nav/command"):
            self.create_subscription(String, topic, self._on_command, 10)
        self.create_subscription(String, "/go2_agent/resume", self._on_resume, 10)
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
        try:
            self.orchestrator = AgentOrchestrator(
                self.memory,
                self.persistence,
                {
                    "enable_debate_layer": _as_bool(self.get_parameter("enable_debate_layer").value),
                    "enable_tour_mode": _as_bool(self.get_parameter("enable_tour_mode").value),
                    "enable_explore_mode": _as_bool(self.get_parameter("enable_explore_mode").value),
                    "enable_human_interrupts": _as_bool(self.get_parameter("enable_human_interrupts").value),
                },
                live_context_provider=self._live_context,
                thread_id=self.thread_id,
            )
        except LangGraphDependencyError as exc:
            self.get_logger().error(str(exc))
            raise
        self.get_logger().info(
            "Real LangGraph Go2 supervisor ready: StateGraph + SQLite checkpoints + conditional subgraphs + ROS tool routing"
        )

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

    def _publish_events(self, payload: Dict[str, Any]) -> None:
        self.event_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))

    def _publish_result(self, state: Dict[str, Any]) -> None:
        command = state.get("nav_command") or {}
        if command and _as_bool(self.get_parameter("enable_nav_publish").value):
            publish_nav_command(self.nav_pub, command.get("action", "none"), command)
        speech = state.get("speech_response", "")
        if speech:
            self.speech_pub.publish(String(data=speech))
        status = state.get("status", {})
        self._publish_status(status)
        pending = state.get("pending_interrupt") or {}
        if pending:
            self.interrupt_pub.publish(String(data=json.dumps(pending, sort_keys=True, default=str)))

    def _on_command(self, msg: String) -> None:
        try:
            state = self.orchestrator.invoke(msg.data)
            self._publish_result(state)
            if _as_bool(self.get_parameter("enable_langgraph_streaming").value):
                self._publish_events({"type": "graph_result", "run_id": state.get("run_id"), "events": state.get("events", [])[-25:]})
        except Exception as exc:
            self.get_logger().exception(f"LangGraph invocation failed: {exc}")
            command = {"action": "stop_robot", "reason": "langgraph_invocation_failed", "error": str(exc)}
            publish_nav_command(self.nav_pub, "stop_robot", command)
            self.speech_pub.publish(String(data="The LangGraph agent failed internally, so I am stopping instead of moving."))
            self._publish_status({"error": str(exc), "nav_action": "stop_robot"})

    def _on_resume(self, msg: String) -> None:
        payload = _json_or_text(msg)
        try:
            state = self.orchestrator.resume_interrupt(payload)
            self._publish_result(state)
            self._publish_events({"type": "resume_completed", "thread_id": self.thread_id, "payload": payload})
        except Exception as exc:
            self.get_logger().exception(f"LangGraph resume failed: {exc}")
            self._publish_events({"type": "resume_failed", "thread_id": self.thread_id, "error": str(exc), "payload": payload})
            self.speech_pub.publish(String(data="I could not resume the interrupted LangGraph task, so I am staying stopped."))

    def destroy_node(self) -> bool:
        try:
            self.persistence.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MainSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
