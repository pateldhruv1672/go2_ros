from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from go2_omi_voice_bridge.intent import (
    INTENT_CANCEL,
    INTENT_CONTINUE_TOUR,
    INTENT_FUN_FACT,
    INTENT_SKIP_CHECKPOINT,
    INTENT_START_TOUR,
    decode_json_or_text,
    speech_payload,
)


SAFE_FUN_FACTS = [
    "Applied Data Science combines statistics, machine learning, and computing to extract useful insights from data.",
    "Many applied data science projects include collecting data, cleaning it, modeling it, evaluating it, and communicating results.",
    "A robot tour guide is itself an applied data science system: it uses sensors, mapping, planning, language, and safety checks.",
    "Digital twin systems let teams simulate and analyze environments before deploying changes in the physical world.",
    "Data visualization is often as important as model accuracy because people need to understand what the system found.",
]


class TourVoiceCommandRouter(Node):
    """Small tour state adapter for verified voice commands.

    This node prepares tour status/narration messages. It does not send movement
    commands directly; the LangGraph agent and Nav2 tool server remain the motion
    path.
    """

    def __init__(self) -> None:
        super().__init__("go2_tour_voice_command_router")
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "default")
        self.declare_parameter("default_tour_id", "sjsu_ads_department_tour")
        self.status_pub = self.create_publisher(String, "/go2_tour/status", 10)
        self.narration_pub = self.create_publisher(String, "/go2_tour/narration", 10)
        self.create_subscription(String, "/go2_tour/start", self._on_start, 10)
        self.create_subscription(String, "/go2_tour/continue", self._on_continue, 10)
        self.create_subscription(String, "/go2_tour/skip", self._on_skip, 10)
        self.create_subscription(String, "/go2_tour/cancel", self._on_cancel, 10)
        self.create_subscription(String, "/go2_agent/user_command", self._on_agent_command, 10)
        self.tour: dict[str, Any] | None = None
        self.index = 0
        self.state = "IDLE"
        self.get_logger().info("Tour voice command router ready.")

    def _session_dir(self) -> Path:
        root = Path(os.path.expanduser(str(self.get_parameter("session_root").value)))
        session = str(self.get_parameter("session_name").value)
        return root / session

    def _tour_path(self, tour_id: str) -> Path:
        return self._session_dir() / "tours" / f"{tour_id}.json"

    def _load_tour(self, tour_id: str) -> dict[str, Any] | None:
        path = self._tour_path(tour_id)
        if not path.is_file():
            return None
        try:
            with path.open("r", encoding="utf-8") as f:
                tour = json.load(f)
            if isinstance(tour, dict) and isinstance(tour.get("checkpoints"), list):
                return tour
        except Exception as exc:
            self.get_logger().warn(f"Failed to load tour route {path}: {exc}")
        return None

    def _publish_status(self, extra: dict[str, Any] | None = None) -> None:
        checkpoints = (self.tour or {}).get("checkpoints") or []
        current = checkpoints[self.index].get("checkpoint_id") if checkpoints and self.index < len(checkpoints) else None
        payload = {
            "tour_id": (self.tour or {}).get("tour_id"),
            "state": self.state,
            "current_checkpoint": current,
            "checkpoint_index": self.index,
            "total_checkpoints": len(checkpoints),
        }
        if extra:
            payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))

    def _say(self, text: str, category: str = "narration") -> None:
        payload = speech_payload(text, category=category)
        self.narration_pub.publish(String(data=payload))

    def _on_agent_command(self, msg: String) -> None:
        payload = decode_json_or_text(msg.data)
        intent = str(payload.get("intent") or payload.get("intent_hint") or "")
        if intent == INTENT_FUN_FACT:
            self._say("Here is a fun fact: " + SAFE_FUN_FACTS[self.index % len(SAFE_FUN_FACTS)], "fun_fact")

    def _on_start(self, msg: String) -> None:
        payload = decode_json_or_text(msg.data)
        tour_id = str(payload.get("tour_id") or self.get_parameter("default_tour_id").value)
        tour = self._load_tour(tour_id)
        if tour is None:
            self.state = "FAILED"
            self._publish_status({"error": f"tour route not found: {self._tour_path(tour_id)}"})
            self._say("I need a saved tour route before I can start the tour.", "warning")
            return
        self.tour = tour
        self.index = 0
        self.state = "WAITING_TO_START"
        checkpoints = tour.get("checkpoints", [])
        self._publish_status({"title": tour.get("title"), "requires_resume_mode": tour.get("requires_resume_mode", True)})
        self._say(
            f"Welcome to {tour.get('title', 'the saved tour')}. I found {len(checkpoints)} saved checkpoints. "
            "I will move one checkpoint at a time through the agent and Nav2 safety path.",
            "narration",
        )

    def _on_continue(self, _msg: String) -> None:
        if not self.tour:
            self.state = "FAILED"
            self._publish_status({"error": "no active tour"})
            self._say("There is no active tour loaded yet.", "warning")
            return
        checkpoints = self.tour.get("checkpoints", [])
        if self.index >= len(checkpoints):
            self.state = "COMPLETED"
            self._publish_status()
            self._say("The tour is complete.", "narration")
            return
        cp = checkpoints[self.index]
        self.state = "WAITING_TO_CONTINUE"
        self._publish_status({"next_checkpoint": cp})
        narration = cp.get("narration") or f"Next checkpoint: {cp.get('name', cp.get('checkpoint_id', 'checkpoint'))}."
        fun_fact = cp.get("fun_fact")
        if fun_fact:
            narration = f"{narration} Here is a fun fact: {fun_fact}"
        self._say(narration, "narration")
        self.index += 1

    def _on_skip(self, _msg: String) -> None:
        self.index += 1
        self.state = "WAITING_TO_CONTINUE"
        self._publish_status({"skipped": True})
        self._say("Skipping this checkpoint.", "status")

    def _on_cancel(self, _msg: String) -> None:
        self.state = "CANCELED"
        self._publish_status()
        self._say("Tour canceled.", "status")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TourVoiceCommandRouter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
