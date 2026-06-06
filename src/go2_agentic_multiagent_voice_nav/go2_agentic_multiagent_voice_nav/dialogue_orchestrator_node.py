from __future__ import annotations

import json
import re
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from go2_agentic_system.openrouter_client import OpenRouterClient

from .place_resolver import PlaceResolver, slugify, norm


class DialogueOrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__("dialogue_orchestrator_node")
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "")
        self.declare_parameter("wake_words", ["sparky", "hey sparky", "okay sparky", "robot"])
        self.declare_parameter("use_chat_fallback", True)
        self.declare_parameter("openrouter_model", "google/gemini-2.5-flash")
        self.declare_parameter("openrouter_base_url", "https://openrouter.ai/api/v1/chat/completions")

        self.resolver = PlaceResolver(
            str(self.get_parameter("session_root").value),
            str(self.get_parameter("session_name").value),
        )
        self.openrouter = OpenRouterClient(
            model=str(self.get_parameter("openrouter_model").value),
            base_url=str(self.get_parameter("openrouter_base_url").value),
        )
        self.pending_camera = False

        self.transcript_sub = self.create_subscription(String, "/agent/transcript", self.transcript_cb, 20)
        self.status_sub = self.create_subscription(String, "/semantic_nav/status", self.semantic_status_cb, 20)
        self.route_event_sub = self.create_subscription(String, "/semantic_nav/event", self.route_event_cb, 20)
        self.cam_resp_sub = self.create_subscription(String, "/agent/camera/response", self.camera_response_cb, 20)

        self.reply_pub = self.create_publisher(String, "/agent/reply", 20)
        self.command_pub = self.create_publisher(String, "/semantic_nav/command", 20)
        self.camera_req_pub = self.create_publisher(String, "/agent/camera/request", 20)
        self.motion_pub = self.create_publisher(String, "/motion_skills/command", 20)

        self.create_timer(5.0, self._reload_places)
        self.get_logger().info(
            f"ready | session={self.resolver.session_name or 'latest'} | places={len(self.resolver.places)}"
        )

    def _reload_places(self) -> None:
        old_count = len(self.resolver.places)
        self.resolver.reload()
        new_count = len(self.resolver.places)
        if new_count != old_count:
            self.get_logger().info(f"places reloaded | count={new_count}")

    def transcript_cb(self, msg: String) -> None:
        raw = (msg.data or '').strip()
        if not raw:
            return
        text = self.strip_wake_word(raw)
        if text is None:
            return
        self.get_logger().info(f"heard: {text}")
        self.handle_text(text, raw)

    def strip_wake_word(self, raw: str) -> Optional[str]:
        text = norm(raw)
        wake_words = [norm(x) for x in list(self.get_parameter("wake_words").value)]
        for wake in sorted(wake_words, key=len, reverse=True):
            if text == wake:
                return ""
            if text.startswith(wake + " "):
                return text[len(wake):].strip()
        # also accept direct command phrases without wake word
        return text

    def semantic_status_cb(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text:
            return
        self.get_logger().info(f"semantic_status: {text}")

    def route_event_cb(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        try:
            event = json.loads(raw)
        except Exception:
            self.get_logger().info(f"route_event_raw: {raw}")
            return
        speech = str(event.get("speech") or "").strip()
        if speech:
            self.reply(speech)
        else:
            self.get_logger().info(f"route_event: {event}")

    def camera_response_cb(self, msg: String) -> None:
        self.pending_camera = False
        self.reply(msg.data)

    def reply(self, text: str) -> None:
        out = String()
        out.data = text
        self.reply_pub.publish(out)
        self.get_logger().info(f"reply: {text}")

    def publish_command(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.command_pub.publish(msg)
        self.get_logger().info(f"nav_command: {text}")

    def publish_event_command(self, payload: dict) -> None:
        self.publish_command(json.dumps(payload, ensure_ascii=False))

    def publish_motion(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.motion_pub.publish(msg)
        self.get_logger().info(f"motion_command: {text}")

    def request_camera(self, text: str) -> None:
        self.pending_camera = True
        msg = String()
        msg.data = text
        self.camera_req_pub.publish(msg)
        self.get_logger().info(f"camera_request: {text}")

    def handle_text(self, text: str, raw: str) -> None:
        if text in {"", "hello", "hi"}:
            self.reply("Hi, I am Sparky. You can ask me to navigate, describe the scene, or do a motion skill.")
            return

        if any(p in text for p in ["start tour", "begin tour", "tour mode on", "give a tour"]):
            self.publish_event_command({"type": "start_tour", "reset_index": True, "route_name": self.resolver.session_name or ""})
            self.publish_motion("tour_greet")
            self.reply("Starting the tour.")
            return

        if any(p in text for p in ["pause tour", "pause the tour", "stop tour for a moment"]):
            self.publish_event_command({"type": "pause_tour", "speech": "tour: Pausing the tour for a moment."})
            self.publish_motion("tour_pause")
            self.reply("Pausing the tour.")
            return

        if any(p in text for p in ["resume tour", "continue tour", "keep going on the tour"]):
            self.publish_event_command({"type": "resume_tour", "speech": "tour: Resuming the tour."})
            self.publish_motion("tour_resume")
            self.reply("Resuming the tour.")
            return

        if any(p in text for p in ["handoff tour", "tour handoff", "take over the tour"]):
            self.publish_event_command({"type": "handoff_tour", "speech": "tour: I am handing off the tour."})
            self.publish_motion("tour_handoff")
            self.reply("Handing off the tour.")
            return

        if any(p in text for p in ["what is this stop", "tell me about this stop", "explain this stop", "fact about this place", "tour facts", "tell us about this place"]):
            self.publish_event_command({"type": "tour_fact_request", "query": raw})
            self.reply("Let me explain this stop.")
            return

        if any(p in text for p in ["blocked route", "stuck", "route failed", "recover route", "why did you stop"]):
            self.publish_event_command({"type": "recover_route", "reason": text})
            self.publish_motion("blocked_route_recovery")
            self.reply("I am diagnosing the route failure.")
            return

        if text in {"stop", "cancel", "halt"}:
            self.publish_command("cancel")
            self.reply("Stopping current navigation.")
            return

        if "list places" in text or text == "places" or "what places" in text:
            self.publish_command("places")
            self.reply("Listing saved places.")
            return

        if any(p in text for p in ["what do you see", "describe the room", "describe the scene", "where am i", "look around"]):
            self.request_camera(raw)
            self.reply("Let me look around.")
            return

        if text.startswith("save spawn") or text == "save spawn":
            self.publish_command("save spawn room=digital_twin_lab category=spawn tags=spawn,home,start,safe")
            self.reply("Saving spawn.")
            return

        m = re.search(r"save this place as (.+)", text)
        if m:
            name = slugify(m.group(1))
            self.publish_command(f"save {name}")
            self.reply(f"Saving this place as {name.replace('_', ' ')}.")
            return

        motion_phrases = {
            "dance": "dance",
            "dance one": "dance1",
            "dance two": "dance2",
            "jump": "jump",
            "sit": "sit",
            "stand": "stand",
            "wave": "wave",
            "hello": "hello",
            "stretch": "stretch",
            "front flip": "front_flip",
            "back flip": "back_flip",
            "front jump": "front_jump",
            "front pounce": "front_pounce",
            "hand stand": "hand_stand",
            "free walk": "free_walk",
            "free bound": "free_bound",
            "free jump": "free_jump",
            "free avoid": "free_avoid",
            "walk upright": "walk_upright",
            "cross step": "cross_step",
            "static walk": "static_walk",
            "trot run": "trot_run",
            "classic walk": "classic_walk",
            "balance stand": "balance_stand",
            "recover": "recovery_stand",
            "auto recovery on": "auto_recovery_on",
            "auto recovery off": "auto_recovery_off",
        }
        if text in motion_phrases:
            motion = motion_phrases[text]
            self.publish_motion(motion)
            self.reply(f"Okay, doing {motion.replace('_', ' ')}.")
            return

        nav_match = re.search(r"(go to|navigate to|take me to|go)\s+(.+)", text)
        if nav_match:
            target = nav_match.group(2).strip()
            place = self.resolver.resolve(target)
            if place is None:
                self.reply(f"I could not find a saved place matching {target}.")
                return
            self.publish_event_command({"type": "go", "place": place.name})
            self.reply(f"Heading to {place.name.replace('_', ' ')}.")
            return

        # bare place utterance
        place = self.resolver.resolve(text)
        if place is not None:
            self.publish_event_command({"type": "go", "place": place.name})
            self.reply(f"Heading to {place.name.replace('_', ' ')}.")
            return

        if bool(self.get_parameter("use_chat_fallback").value):
            reply = self.chat_fallback(raw)
            self.reply(reply)
        else:
            self.reply("I can navigate, describe what I see, save spawn, list places, or do motion skills.")

    def chat_fallback(self, raw: str) -> str:
        if not self.openrouter.available:
            return "I heard you, but I do not have chat fallback configured."
        places = ", ".join(self.resolver.list_names()[:30])
        prompt = (
            "You are Sparky, a helpful robot dog assistant. "
            "Be concise, factual, and route-aware. "
            "If a tour is underway, speak like a guide and preserve the route state. "
            f"Known saved places: {places}. "
            f"User said: {raw}"
        )
        try:
            result = self.openrouter.complete_text(prompt, temperature=0.4, max_tokens=350)
            if not result.get("ok"):
                return f"Chat fallback failed: {result.get('error')}"
            return str(result.get("text", "")).strip() or "I am not sure how to answer that yet."
        except Exception as exc:
            return f"Chat fallback failed: {exc}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DialogueOrchestratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
