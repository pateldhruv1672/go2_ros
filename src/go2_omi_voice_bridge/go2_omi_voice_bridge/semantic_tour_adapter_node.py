from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class SemanticTourAdapterNode(Node):
    """Thin Tour UI/voice adapter. semantic_nav_node is the only route-state owner."""

    def __init__(self) -> None:
        super().__init__("go2_semantic_tour_adapter")
        self.status_pub = self.create_publisher(String, "/go2_tour/status", 10)
        self.narration_pub = self.create_publisher(String, "/go2_tour/narration", 10)
        self.semantic_pub = self.create_publisher(String, "/semantic_nav/command", 10)
        self.create_subscription(String, "/go2_tour/start", self._on_start, 10)
        self.create_subscription(String, "/go2_tour/continue", self._on_continue, 10)
        self.create_subscription(String, "/go2_tour/skip", self._on_skip, 10)
        self.create_subscription(String, "/go2_tour/cancel", self._on_cancel, 10)
        self.create_subscription(String, "/semantic_nav/event", self._on_semantic_event, 20)
        self.create_subscription(String, "/semantic_nav/status", self._on_semantic_status, 20)
        self.latest_event: dict[str, Any] = {}
        self.latest_status = ""
        self.get_logger().info("Semantic Tour adapter ready; no independent stop index is maintained.")

    def _command(self, command_type: str, **extra: Any) -> None:
        payload = {"type": command_type, "source": "semantic_tour_adapter", **extra}
        self.semantic_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))

    def _publish_status(self, **extra: Any) -> None:
        event = self.latest_event
        payload = {
            "state": str(event.get("status") or self.latest_status or "idle"),
            "route_name": str(event.get("route_name") or ""),
            "current_checkpoint": str(event.get("stop_name") or ""),
            "completion": event.get("completion"),
            "semantic_nav_status": self.latest_status,
            "source_of_truth": "semantic_nav_node",
            **extra,
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))

    def _on_start(self, msg: String) -> None:
        self._command("start_tour", reset_index=True)
        self._publish_status(request="start_tour")

    def _on_continue(self, msg: String) -> None:
        self._command("resume_tour", speech="tour: Resuming the saved route.")
        self._publish_status(request="resume_tour")

    def _on_skip(self, msg: String) -> None:
        self._command("advance_tour")
        self._publish_status(request="advance_tour")

    def _on_cancel(self, msg: String) -> None:
        self._command("pause_tour", speech="tour: Tour canceled. I am stopping safely.")
        self._publish_status(request="pause_tour")

    def _on_semantic_status(self, msg: String) -> None:
        self.latest_status = msg.data.strip()
        self._publish_status()

    def _on_semantic_event(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.latest_event = payload if isinstance(payload, dict) else {}
        except Exception:
            self.latest_event = {}
            return
        # semantic_nav_node already emits event speech on /agent/reply, and TTS subscribes
        # there directly. Mirroring it to /go2_tour/narration would speak every stop twice.
        self._publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticTourAdapterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
