from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class SemanticTourAdapterNode(Node):
    # Thin Tour adapter; semantic_nav_node remains the only route-state owner.

    def __init__(self) -> None:
        super().__init__("go2_semantic_tour_adapter")
        self.declare_parameter("speech_topic", "/go2_speech/request")
        self.declare_parameter("speech_status_topic", "/go2_speech/status")
        self.declare_parameter("motion_topic", "/motion_skills/command")
        self.declare_parameter("auto_tour_gestures", True)
        self.declare_parameter("hello_skill", "hello")
        self.declare_parameter("heart_skill", "heart")

        self.status_pub = self.create_publisher(String, "/go2_tour/status", 10)
        # Retained as an observation topic for compatibility. Narration is sent
        # through /go2_speech/request so there is exactly one speech queue owner.
        self.narration_pub = self.create_publisher(String, "/go2_tour/narration", 10)
        self.semantic_pub = self.create_publisher(String, "/semantic_nav/command", 10)
        self.speech_pub = self.create_publisher(String, str(self.get_parameter("speech_topic").value), 10)
        self.motion_pub = self.create_publisher(String, str(self.get_parameter("motion_topic").value), 10)

        self.create_subscription(String, "/go2_tour/start", self._on_start, 10)
        self.create_subscription(String, "/go2_tour/continue", self._on_continue, 10)
        self.create_subscription(String, "/go2_tour/skip", self._on_skip, 10)
        self.create_subscription(String, "/go2_tour/cancel", self._on_cancel, 10)
        self.create_subscription(String, "/semantic_nav/event", self._on_semantic_event, 20)
        self.create_subscription(String, "/semantic_nav/status", self._on_semantic_status, 20)
        self.create_subscription(String, str(self.get_parameter("speech_status_topic").value), self._on_speech_status, 20)

        self.latest_event: dict[str, Any] = {}
        self.latest_status = ""
        self._pending_narration: dict[str, dict[str, Any]] = {}
        self._recent_speech: dict[str, float] = {}
        # /semantic_nav/event is transient-local. Ignore a retained event briefly
        # after startup/respawn so an old completed stop is never narrated again.
        self._accept_speech_after = time.monotonic() + 2.0
        self.get_logger().info(
            "Semantic Tour adapter ready; semantic_nav owns route state; "
            "route narration -> speech arbiter with correlated tour gestures."
        )

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
            "stamp_sec": time.time(),
            **extra,
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))

    def _motion(self, skill: str, request_id: str, phase: str) -> None:
        if not bool(self.get_parameter("auto_tour_gestures").value):
            return
        self.motion_pub.publish(String(data=json.dumps({
            "command": skill,
            "skill": skill,
            "source": "semantic_tour_adapter",
            "request_id": request_id,
            "tour_phase": phase,
            "verified": True,
            "safety_checked": True,
        }, sort_keys=True)))
        self._publish_status(gesture=skill, gesture_phase=phase, request_id=request_id)

    def _queue_route_speech(self, payload: dict[str, Any]) -> None:
        speech = str(payload.get("speech") or "").strip()
        route_name = str(payload.get("route_name") or "").strip()
        stop_name = str(payload.get("stop_name") or "").strip()
        # Do not turn unrelated semantic/object-nav chatter into tour narration.
        if not speech or not (route_name or stop_name):
            return
        key = "|".join((route_name, stop_name, speech))
        now = time.monotonic()
        if now < self._accept_speech_after:
            self._publish_status(event="startup_retained_narration_ignored", route_name=route_name, current_checkpoint=stop_name)
            return
        if now - self._recent_speech.get(key, -1e9) < 2.0:
            return
        self._recent_speech[key] = now
        self._recent_speech = {k: v for k, v in self._recent_speech.items() if now - v < 30.0}

        request_id = f"route_tour_{time.time_ns()}"
        self._pending_narration[request_id] = {
            "route_name": route_name,
            "stop_name": stop_name,
            "hello_sent": False,
            "speech": speech,
        }
        self.speech_pub.publish(String(data=json.dumps({
            "text": speech,
            "source": "semantic_tour_adapter",
            "category": "tour_route_narration",
            "priority": "normal",
            "request_id": request_id,
        }, sort_keys=True)))
        self._publish_status(
            event="route_narration_forwarded", request_id=request_id,
            route_name=route_name, current_checkpoint=stop_name,
        )

    def _on_speech_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        request_id = str(payload.get("request_id") or "")
        pending = self._pending_narration.get(request_id)
        if pending is None:
            return
        event = str(payload.get("event") or "")
        if event == "speaking" and not bool(pending.get("hello_sent")):
            pending["hello_sent"] = True
            self._motion(str(self.get_parameter("hello_skill").value), request_id, "speech_start")
        elif event == "speech_done":
            self._motion(str(self.get_parameter("heart_skill").value), request_id, "speech_done")
            self._pending_narration.pop(request_id, None)
            self._publish_status(event="route_narration_done", request_id=request_id)
        elif event in {"speech_timeout_release", "queue_full_drop"}:
            self._pending_narration.pop(request_id, None)
            self._publish_status(event="route_narration_failed", request_id=request_id, reason=event)

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
        self._queue_route_speech(self.latest_event)
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
