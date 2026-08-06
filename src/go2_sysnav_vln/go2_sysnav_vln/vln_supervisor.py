from __future__ import annotations

import base64
import json
import math
import time
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .common import angle_wrap, clean_label, labels_match, quat_to_yaw, target_from_text
from .ollama_structured import StructuredOllamaClient, StructuredOllamaError


INSTRUCTION_SCHEMA = {
    "type": "object",
    "properties": {
        "target_object": {"type": "string"},
        "room_condition": {"type": "string"},
        "spatial_condition": {"type": "string"},
        "attribute_condition": {"type": "string"},
        "anchor_object": {"type": "string"},
        "attribute_condition_anchor": {"type": "string"},
    },
    "required": [
        "target_object", "room_condition", "spatial_condition", "attribute_condition",
        "anchor_object", "attribute_condition_anchor",
    ],
    "additionalProperties": False,
}


class VLNSupervisor(Node):
    """Deterministic SysNav-style semantic supervisor for the Go2 Nav2 stack.

    The VLM never emits velocity, coordinates, or generic tool calls. It performs
    only schema-constrained instruction decomposition, room classification,
    room-ID selection, and target verification. Geometric frontier and approach
    poses are selected by deterministic code and sent to the safe Nav2 tool server.
    """

    def __init__(self) -> None:
        super().__init__("go2_vln_supervisor")
        self.declare_parameter("goal_topic", "/go2_vln/goal")
        self.declare_parameter("command_topic", "/go2_vln/command")
        self.declare_parameter("state_topic", "/go2_vln/state")
        self.declare_parameter("decision_topic", "/go2_vln/decision")
        self.declare_parameter("target_spec_topic", "/go2_vln/target_spec")
        self.declare_parameter("frontier_topic", "/go2_vln/frontier_candidates")
        self.declare_parameter("room_graph_topic", "/go2_vln/room_graph")
        self.declare_parameter("room_label_update_topic", "/go2_vln/room_label_update")
        self.declare_parameter("object_map_topic", "/go2_vln/object_map")
        self.declare_parameter("blacklist_topic", "/go2_vln/frontier_blacklist")
        self.declare_parameter("nav_command_topic", "/go2_nav/command")
        self.declare_parameter("nav_status_topic", "/go2_nav/status")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ollama_url", "http://127.0.0.1:11434")
        self.declare_parameter("ollama_model", "gemma4:e4b")
        self.declare_parameter("ollama_timeout_sec", 90.0)
        self.declare_parameter("ollama_think", "false")
        self.declare_parameter("ollama_keep_alive", "10m")
        self.declare_parameter("ollama_debug_topic", "/go2_vln/ollama_debug")
        self.declare_parameter("ollama_debug_jsonl", "~/.ros/go2_sysnav_vln/ollama_calls.jsonl")
        self.declare_parameter("send_image_to_vlm", True)
        self.declare_parameter("image_jpeg_quality", 65)
        self.declare_parameter("image_max_age_sec", 2.0)
        self.declare_parameter("room_types", [
            "classroom", "laboratory", "office room", "meeting room", "computer lab",
            "restroom", "storage room", "copy room", "student lounge", "reception",
            "corridor", "kitchen", "living room", "unknown",
        ])
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("decision_period_sec", 0.75)
        self.declare_parameter("object_approach_distance_m", 0.90)
        self.declare_parameter("object_approach_search_width_m", 0.70)
        self.declare_parameter("object_approach_samples", 48)
        self.declare_parameter("object_goal_clearance_m", 0.52)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("target_recent_age_sec", 12.0)
        self.declare_parameter("max_room_visits", 2)
        self.declare_parameter("frontier_blacklist_ttl_sec", 120.0)
        self.declare_parameter("max_mission_duration_sec", 420.0)
        self.declare_parameter("max_goals", 30)
        self.declare_parameter("max_failures", 6)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ollama = StructuredOllamaClient(
            str(self.get_parameter("ollama_url").value),
            str(self.get_parameter("ollama_model").value),
            float(self.get_parameter("ollama_timeout_sec").value),
            str(self.get_parameter("ollama_keep_alive").value),
            str(self.get_parameter("ollama_debug_jsonl").value),
        )
        self._llm_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="go2-vln")
        self.pending: Optional[Future] = None
        self.pending_kind = ""
        self.pending_context: Dict[str, Any] = {}

        self.state = "IDLE"
        self.reason = ""
        self.running = False
        self.instruction = ""
        self.spec: Dict[str, str] = {}
        self.mission_id = 0
        self.started_at = 0.0
        self.goal_count = 0
        self.failure_count = 0
        self.active_semantic_action = ""
        self.active_context: Dict[str, Any] = {}
        self.waiting_nav = False
        self.selected_room_id = ""
        self.room_visits: Dict[str, int] = {}
        self.exhausted_rooms: set[str] = set()
        self.rejected_objects: set[int] = set()
        self.frontiers: List[Dict[str, Any]] = []
        self.rooms: List[Dict[str, Any]] = []
        self.current_room_id: Optional[str] = None
        self.objects: List[Dict[str, Any]] = []
        self.latest_image: Optional[Image] = None
        self.latest_image_wall_sec = 0.0
        self.map_msg: Optional[OccupancyGrid] = None
        self.last_decision: Dict[str, Any] = {}
        self.ollama_request_seq = 0

        self.state_pub = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), 20
        )
        self.decision_pub = self.create_publisher(
            String, str(self.get_parameter("decision_topic").value), 20
        )
        self.target_spec_pub = self.create_publisher(
            String, str(self.get_parameter("target_spec_topic").value), 10
        )
        self.room_label_pub = self.create_publisher(
            String, str(self.get_parameter("room_label_update_topic").value), 10
        )
        self.blacklist_pub = self.create_publisher(
            String, str(self.get_parameter("blacklist_topic").value), 10
        )
        self.ollama_debug_pub = self.create_publisher(
            String, str(self.get_parameter("ollama_debug_topic").value), 20
        )
        self.nav_pub = self.create_publisher(
            String, str(self.get_parameter("nav_command_topic").value), 20
        )
        self.create_subscription(
            String, str(self.get_parameter("goal_topic").value), self.on_goal, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("command_topic").value), self.on_command, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("frontier_topic").value), self.on_frontiers, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("room_graph_topic").value), self.on_rooms, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("object_map_topic").value), self.on_objects, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("nav_status_topic").value), self.on_nav_status, 20
        )
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self.on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map_topic").value), self.on_map, 10
        )
        self.create_timer(float(self.get_parameter("decision_period_sec").value), self.tick)
        self.get_logger().info(
            "structured SysNav supervisor ready; explicit start required unless auto_start=true"
        )
        self.get_logger().info(
            "Ollama contract url=%s model=%s timeout=%.1fs think=%s keep_alive=%s audit=%s"
            % (
                self.ollama.base_url, self.ollama.model, self.ollama.timeout_sec,
                str(self.get_parameter("ollama_think").value), self.ollama.keep_alive,
                self.ollama.debug_jsonl_path,
            )
        )

    @property
    def motion_enabled(self) -> bool:
        value = self.get_parameter("enable_motion").value
        return str(value).lower() in {"1", "true", "yes", "on"} if isinstance(value, str) else bool(value)

    def publish_json(self, publisher: Any, payload: Dict[str, Any]) -> None:
        publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def publish_state(self, reason: Optional[str] = None) -> None:
        if reason is not None:
            self.reason = reason
        self.publish_json(self.state_pub, {
            "stamp_sec": time.time(), "mission_id": self.mission_id,
            "state": self.state, "reason": self.reason, "running": self.running,
            "motion_enabled": self.motion_enabled, "instruction": self.instruction,
            "target_spec": self.spec, "selected_room_id": self.selected_room_id,
            "current_room_id": self.current_room_id, "goal_count": self.goal_count,
            "failure_count": self.failure_count, "waiting_nav": self.waiting_nav,
        })

    def decision(self, payload: Dict[str, Any]) -> None:
        payload = dict(payload)
        payload.setdefault("stamp_sec", time.time())
        payload.setdefault("mission_id", self.mission_id)
        payload.setdefault("state", self.state)
        self.last_decision = payload
        self.publish_json(self.decision_pub, payload)

    def on_goal(self, msg: String) -> None:
        self.mission_id += 1
        raw = msg.data.strip()
        try:
            payload = json.loads(raw) if raw.startswith("{") else {"instruction": raw}
        except Exception:
            payload = {"instruction": raw}
        self.instruction = str(payload.get("instruction", payload.get("text", raw))).strip()
        direct_target = str(payload.get("target", payload.get("target_object", ""))).strip()
        self.spec = {
            "target_object": clean_label(direct_target) if direct_target else "",
            "room_condition": str(payload.get("room_condition", "")).strip().lower(),
            "spatial_condition": str(payload.get("spatial_condition", "")).strip().lower(),
            "attribute_condition": str(payload.get("attribute_condition", "")).strip().lower(),
            "anchor_object": clean_label(str(payload.get("anchor_object", ""))),
            "attribute_condition_anchor": str(payload.get("attribute_condition_anchor", "")).strip().lower(),
        }
        self.started_at = time.time()
        self.goal_count = 0; self.failure_count = 0
        self.room_visits.clear(); self.exhausted_rooms.clear(); self.rejected_objects.clear()
        self.selected_room_id = ""; self.waiting_nav = False
        self.state = "DECOMPOSE"
        auto = self.get_parameter("auto_start").value
        self.running = str(auto).lower() in {"1", "true", "yes", "on"} if isinstance(auto, str) else bool(auto)
        if not self.running:
            self.state = "PAUSED"
            self.reason = "goal_loaded_send_start"
        else:
            self.reason = "new_goal"
        self.publish_state()

    def on_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data) if msg.data.strip().startswith("{") else {"action": msg.data}
        except Exception:
            payload = {"action": msg.data}
        action = str(payload.get("action", "")).strip().lower()
        if action in {"start", "resume"}:
            if not self.instruction:
                self.publish_state("no_goal_loaded")
                return
            self.running = True
            if self.state in {"PAUSED", "IDLE", "FAILED"}:
                self.state = "DECOMPOSE" if not self.spec.get("target_object") else "CHECK_OBJECTS"
            self.publish_state("operator_start")
        elif action in {"stop", "pause", "cancel"}:
            self.running = False
            self.state = "PAUSED"
            self.waiting_nav = False
            self.publish_json(self.nav_pub, {"action": "stop_robot", "reason": "operator_stop"})
            self.publish_state("operator_stop")
        elif action == "reset":
            self.running = False; self.state = "IDLE"; self.instruction = ""; self.spec = {}
            self.publish_state("reset")
        elif action in {"ollama_test", "ollama_test_text", "ollama_test_vision"}:
            use_image = action == "ollama_test_vision"
            schema = {
                "type": "object",
                "properties": {
                    "ok": {"type": "boolean"},
                    "mode": {"type": "string", "enum": ["text", "vision"]},
                    "summary": {"type": "string"},
                },
                "required": ["ok", "mode", "summary"],
                "additionalProperties": False,
            }
            self.submit_llm(
                "diagnostic",
                "You are a diagnostics endpoint. Follow the schema exactly.",
                "Return ok=true, mode=%s, and a short summary of %s."
                % ("vision" if use_image else "text", "the supplied image" if use_image else "this text-only test"),
                schema, {"diagnostic": True, "use_image": use_image}, use_image=use_image,
            )

    def on_frontiers(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.frontiers = [x for x in payload.get("candidates", []) if isinstance(x, dict)]
            self.current_room_id = payload.get("current_room_id", self.current_room_id)
        except Exception:
            pass

    def on_rooms(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.rooms = [x for x in payload.get("rooms", []) if isinstance(x, dict)]
            self.current_room_id = payload.get("current_room_id", self.current_room_id)
        except Exception:
            pass

    def on_objects(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.objects = [x for x in payload.get("objects", []) if isinstance(x, dict)]
        except Exception:
            pass

    def on_image(self, msg: Image) -> None:
        self.latest_image = msg
        self.latest_image_wall_sec = time.time()

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def on_nav_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except Exception:
            return
        action = str(status.get("action", ""))
        if action in {"preflight_no_path", "preflight_rejected", "preflight_error", "goal_rejected", "goal_response_error"}:
            self.waiting_nav = False
            self.failure_count += 1
            self.handle_nav_failure(action)
            return
        if action == "goal_result":
            self.waiting_nav = False
            success = bool(status.get("success", False))
            if success:
                self.handle_nav_success()
            else:
                self.failure_count += 1
                self.handle_nav_failure(f"goal_status_{status.get('status')}")
        elif action == "parse_error":
            self.waiting_nav = False
            self.failure_count += 1
            self.handle_nav_failure("nav_command_parse_error")

    def handle_nav_success(self) -> None:
        semantic = self.active_semantic_action
        context = dict(self.active_context)
        self.active_semantic_action = ""; self.active_context = {}
        if semantic == "navigate_room":
            room_id = str(context.get("room_id", ""))
            self.selected_room_id = room_id
            self.room_visits[room_id] = self.room_visits.get(room_id, 0) + 1
            self.state = "CHECK_OBJECTS"
            self.reason = "arrived_room_anchor"
        elif semantic == "explore_frontier":
            self.state = "CHECK_OBJECTS"
            self.reason = "frontier_reached"
        elif semantic == "approach_object":
            self.state = "DONE"
            self.running = False
            self.reason = "target_reached"
        else:
            self.state = "CHECK_OBJECTS"
            self.reason = "navigation_succeeded"
        self.publish_state()

    def handle_nav_failure(self, reason: str) -> None:
        if self.active_semantic_action == "explore_frontier":
            fid = str(self.active_context.get("frontier_id", ""))
            if fid:
                self.publish_json(self.blacklist_pub, {
                    "frontier_id": fid,
                    "ttl_sec": float(self.get_parameter("frontier_blacklist_ttl_sec").value),
                    "reason": reason,
                })
        self.active_semantic_action = ""; self.active_context = {}
        if self.failure_count >= int(self.get_parameter("max_failures").value):
            self.state = "FAILED"; self.running = False
            self.publish_state(f"failure_budget_exhausted:{reason}")
        else:
            self.state = "SELECT_ROOM" if not self.selected_room_id else "EXPLORE_ROOM"
            self.publish_state(f"navigation_failed:{reason}")

    def robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                rclpy.time.Time(), timeout=Duration(seconds=0.15),
            )
            return (
                float(tf.transform.translation.x), float(tf.transform.translation.y),
                quat_to_yaw(tf.transform.rotation),
            )
        except Exception:
            return None

    def image_base64(self) -> Optional[str]:
        if self.latest_image is None or not bool(self.get_parameter("send_image_to_vlm").value):
            return None
        max_age = float(self.get_parameter("image_max_age_sec").value)
        if self.latest_image_wall_sec <= 0.0 or time.time() - self.latest_image_wall_sec > max_age:
            self.publish_json(self.ollama_debug_pub, {
                "event": "image_rejected", "reason": "stale_or_missing",
                "age_sec": None if self.latest_image_wall_sec <= 0.0 else round(time.time() - self.latest_image_wall_sec, 3),
                "max_age_sec": max_age,
            })
            return None
        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
            quality = int(self.get_parameter("image_jpeg_quality").value)
            ok, data = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
            return base64.b64encode(data.tobytes()).decode("ascii") if ok else None
        except Exception:
            return None

    def think_value(self) -> Any:
        value = self.get_parameter("ollama_think").value
        text = str(value).lower()
        if text in {"false", "off", "none", "0"}:
            return False
        if text in {"true", "on", "1"}:
            return True
        return text

    def submit_llm(
        self, kind: str, system: str, user: str, schema: Dict[str, Any],
        context: Optional[Dict[str, Any]] = None, use_image: bool = False,
    ) -> None:
        if self.pending is not None:
            return
        images = [self.image_base64()] if use_image else []
        images = [x for x in images if x]
        if use_image and not images:
            error = StructuredOllamaError("fresh image required but unavailable")
            self.publish_json(self.ollama_debug_pub, {
                "event": "error", "kind": kind, "error": str(error),
                "reason": "image_required_missing",
            })
            self.apply_llm_error(kind, error, dict(context or {}))
            return
        self.ollama_request_seq += 1
        request_id = f"m{self.mission_id}-{kind}-{self.ollama_request_seq}"
        self.pending_kind = kind
        self.pending_context = dict(context or {})
        self.pending_context["request_id"] = request_id
        self.publish_json(self.ollama_debug_pub, {
            "event": "request_queued", "request_id": request_id, "kind": kind,
            "model": self.ollama.model, "url": self.ollama.base_url + "/api/chat",
            "think": self.think_value(), "image_count": len(images),
            "image_base64_chars": sum(len(x) for x in images),
            "system_preview": system[:500], "user_preview": user[:1000],
            "schema": schema,
        })
        self.pending = self._llm_executor.submit(
            self.ollama.chat, system, user, schema, images or None, self.think_value(), request_id, kind
        )
        self.publish_state(f"vlm_query:{kind}")

    def poll_llm(self) -> bool:
        if self.pending is None or not self.pending.done():
            return False
        future = self.pending; kind = self.pending_kind; context = self.pending_context
        self.pending = None; self.pending_kind = ""; self.pending_context = {}
        try:
            result = future.result()
            answer = result.get("answer", result) if isinstance(result, dict) else result
            meta = result.get("meta", {}) if isinstance(result, dict) else {}
            self.publish_json(self.ollama_debug_pub, {
                "event": "response", "request_id": context.get("request_id", ""),
                "kind": kind, "meta": meta, "answer": answer,
            })
        except Exception as exc:
            self.publish_json(self.ollama_debug_pub, {
                "event": "error", "request_id": context.get("request_id", ""),
                "kind": kind, "error": str(exc),
            })
            self.apply_llm_error(kind, exc, context)
            return True
        self.apply_llm_answer(kind, answer, context)
        return True

    def apply_llm_error(self, kind: str, exc: Exception, context: Dict[str, Any]) -> None:
        self.decision({"decision_type": kind, "success": False, "error": str(exc)})
        if kind == "decompose":
            self.spec["target_object"] = self.spec.get("target_object") or target_from_text(self.instruction)
            self.finish_decomposition("heuristic_fallback")
        elif kind == "classify_room":
            self.state = "SELECT_ROOM"; self.publish_state("room_classification_unavailable")
        elif kind == "select_room":
            room = self.deterministic_room_choice(context.get("rooms", []))
            if room:
                self.select_room(room, "deterministic_fallback")
            else:
                self.fail("no_room_candidate")
        elif kind == "verify_object":
            candidate = context.get("candidate")
            if candidate and not any(self.spec.get(k) for k in ("attribute_condition", "spatial_condition", "room_condition")):
                self.accept_object(candidate, "confirmed_object_fallback")
            elif candidate:
                self.rejected_objects.add(int(candidate.get("object_id", -1)))
                self.state = "EXPLORE_ROOM" if self.selected_room_id else "SELECT_ROOM"
                self.publish_state("target_verification_unavailable")
        elif kind == "diagnostic":
            self.publish_state(f"ollama_diagnostic_failed:{exc}")

    def apply_llm_answer(self, kind: str, answer: Dict[str, Any], context: Dict[str, Any]) -> None:
        self.decision({"decision_type": kind, "success": True, "answer": answer})
        if kind == "decompose":
            for key in INSTRUCTION_SCHEMA["required"]:
                value = str(answer.get(key, "")).strip().lower()
                self.spec[key] = clean_label(value) if key in {"target_object", "anchor_object"} else value
            self.finish_decomposition("structured_ollama")
        elif kind == "classify_room":
            room_id = str(context.get("room_id", ""))
            allowed = [str(x).lower() for x in self.get_parameter("room_types").value]
            label = str(answer.get("room_type", "unknown")).strip().lower()
            if label not in allowed:
                label = "unknown"
            if room_id:
                self.publish_json(self.room_label_pub, {"room_id": room_id, "label": label})
                for room in self.rooms:
                    if str(room.get("room_id")) == room_id:
                        room["label"] = label
            self.state = "SELECT_ROOM"
            self.publish_state("room_classified")
        elif kind == "select_room":
            room_id = str(answer.get("room_id", ""))
            room = next((r for r in context.get("rooms", []) if str(r.get("room_id")) == room_id), None)
            if room is None:
                room = self.deterministic_room_choice(context.get("rooms", []))
            if room:
                self.select_room(room, str(answer.get("reason", "structured_selection")))
            else:
                self.fail("invalid_room_selection")
        elif kind == "verify_object":
            candidate = context.get("candidate")
            if candidate is None:
                self.state = "CHECK_OBJECTS"
                return
            if bool(answer.get("is_target", False)):
                self.accept_object(candidate, str(answer.get("reason", "verified")))
            else:
                self.rejected_objects.add(int(candidate.get("object_id", -1)))
                self.state = "EXPLORE_ROOM" if self.selected_room_id else "SELECT_ROOM"
                self.publish_state("candidate_rejected")
        elif kind == "diagnostic":
            self.publish_state("ollama_diagnostic_ok" if bool(answer.get("ok", False)) else "ollama_diagnostic_returned_false")

    def finish_decomposition(self, source: str) -> None:
        if not self.spec.get("target_object"):
            self.fail("instruction_has_no_target_object")
            return
        self.publish_json(self.target_spec_pub, {
            "instruction": self.instruction, **self.spec, "source": source,
        })
        # The detector accepts this topic as well as the raw goal topic.
        self.state = "CHECK_OBJECTS"
        self.publish_state("instruction_decomposed")

    def room_for_object(self, obj: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        ox, oy = float(obj.get("x", 0.0)), float(obj.get("y", 0.0))
        best = None; best_dist = float("inf")
        for room in self.rooms:
            centroid = room.get("centroid", {})
            dist = math.hypot(ox - float(centroid.get("x", 0.0)), oy - float(centroid.get("y", 0.0)))
            if dist < best_dist:
                best, best_dist = room, dist
        return best

    def anchor_relation_ok(self, candidate: Dict[str, Any]) -> bool:
        anchor_label = self.spec.get("anchor_object", "")
        spatial = self.spec.get("spatial_condition", "")
        if not anchor_label or not spatial:
            return True
        cx, cy = float(candidate.get("x", 0.0)), float(candidate.get("y", 0.0))
        anchors = [o for o in self.objects if bool(o.get("confirmed")) and labels_match(str(o.get("label", "")), anchor_label)]
        if not anchors:
            return False
        distance = min(math.hypot(cx - float(a.get("x", 0.0)), cy - float(a.get("y", 0.0))) for a in anchors)
        if any(term in spatial for term in ("next to", "near", "beside", "by")):
            return distance <= 1.5
        return True  # VLM performs the final check for relations not inferable in 2D.

    def matching_objects(self) -> List[Dict[str, Any]]:
        target = self.spec.get("target_object", "")
        now = time.time()
        candidates = []
        for obj in self.objects:
            object_id = int(obj.get("object_id", -1))
            if object_id in self.rejected_objects or not bool(obj.get("confirmed", False)):
                continue
            if not labels_match(str(obj.get("label", "")), target):
                continue
            if now - float(obj.get("last_seen", 0.0)) > float(self.get_parameter("target_recent_age_sec").value):
                continue
            room = self.room_for_object(obj)
            room_condition = self.spec.get("room_condition", "")
            if room_condition and room and room.get("label") not in {"unknown", ""}:
                if clean_label(str(room.get("label"))) not in clean_label(room_condition):
                    continue
            if not self.anchor_relation_ok(obj):
                continue
            item = dict(obj)
            item["room_id"] = room.get("room_id") if room else None
            item["room_label"] = room.get("label") if room else "unknown"
            candidates.append(item)
        candidates.sort(
            key=lambda o: (int(o.get("confirmations", 0)), float(o.get("confidence", 0.0)), float(o.get("last_seen", 0.0))),
            reverse=True,
        )
        return candidates

    def classify_current_room_if_needed(self) -> bool:
        room = next((r for r in self.rooms if str(r.get("room_id")) == str(self.current_room_id)), None)
        if room is None or str(room.get("label", "unknown")) not in {"", "unknown"}:
            return False
        options = [str(x) for x in self.get_parameter("room_types").value]
        schema = {
            "type": "object",
            "properties": {
                "room_type": {"type": "string", "enum": options},
                "reason": {"type": "string"},
            },
            "required": ["room_type", "reason"], "additionalProperties": False,
        }
        user = json.dumps({
            "room_id": room.get("room_id"), "area_m2": room.get("area_m2"),
            "known_objects": [o.get("label") for o in room.get("objects", [])],
            "allowed_room_types": options,
        })
        self.submit_llm(
            "classify_room",
            "Classify the current indoor room from the RGB view and structured map context. "
            "Choose exactly one allowed room type. Do not invent a room not visible or supported.",
            user, schema, {"room_id": room.get("room_id")}, use_image=True,
        )
        return True

    def room_candidates(self) -> List[Dict[str, Any]]:
        max_visits = int(self.get_parameter("max_room_visits").value)
        result = []
        for room in self.rooms:
            room_id = str(room.get("room_id", ""))
            if not room_id or room_id in self.exhausted_rooms:
                continue
            if self.room_visits.get(room_id, 0) >= max_visits:
                continue
            if bool(room.get("explored", False)) and not room.get("objects"):
                continue
            result.append(room)
        return result

    def deterministic_room_choice(self, rooms: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if not rooms:
            return None
        target = self.spec.get("target_object", "")
        room_condition = clean_label(self.spec.get("room_condition", ""))
        def score(room: Dict[str, Any]) -> float:
            labels = [clean_label(str(o.get("label", ""))) for o in room.get("objects", [])]
            value = 0.0
            if any(labels_match(label, target) for label in labels):
                value += 50.0
            if room_condition and clean_label(str(room.get("label", ""))) in room_condition:
                value += 20.0
            value += 2.0 * float(room.get("stable_frontier_count", 0))
            value += float(room.get("frontier_count", 0))
            value -= float(room.get("distance_m", 0.0))
            value -= 5.0 * self.room_visits.get(str(room.get("room_id")), 0)
            return value
        return max(rooms, key=score)

    def request_room_selection(self) -> None:
        rooms = self.room_candidates()
        if not self.rooms:
            self.publish_state("waiting_for_room_graph")
            return
        if not rooms:
            self.fail("all_rooms_exhausted")
            return
        schema = {
            "type": "object",
            "properties": {
                "room_id": {"type": "string", "enum": [str(r["room_id"]) for r in rooms]},
                "reason": {"type": "string"},
            },
            "required": ["room_id", "reason"], "additionalProperties": False,
        }
        compact = [{
            "room_id": r.get("room_id"), "label": r.get("label", "unknown"),
            "objects": [o.get("label") for o in r.get("objects", [])],
            "distance_m": r.get("distance_m"), "frontier_count": r.get("frontier_count"),
            "stable_frontier_count": r.get("stable_frontier_count"),
            "visit_count": self.room_visits.get(str(r.get("room_id")), 0),
        } for r in rooms]
        self.submit_llm(
            "select_room",
            "You are the high-level room selector in a hierarchical object-navigation system. "
            "Select one supplied room_id likely to satisfy the instruction while minimizing travel "
            "and repeated exploration. You may not output coordinates or an unlisted ID.",
            json.dumps({"instruction": self.instruction, "target_spec": self.spec, "rooms": compact}),
            schema, {"rooms": rooms}, use_image=False,
        )

    def select_room(self, room: Dict[str, Any], reason: str) -> None:
        self.selected_room_id = str(room.get("room_id", ""))
        anchor = room.get("anchor", {})
        pose = {
            "frame_id": str(self.get_parameter("map_frame").value),
            "x": float(anchor.get("x", 0.0)), "y": float(anchor.get("y", 0.0)),
            "yaw": self.heading_to(float(anchor.get("x", 0.0)), float(anchor.get("y", 0.0))),
        }
        self.decision({
            "decision_type": "room_navigation", "room_id": self.selected_room_id,
            "room_label": room.get("label", "unknown"), "reason": reason, "pose": pose,
        })
        self.send_nav("navigate_room", pose, {"room_id": self.selected_room_id})

    def heading_to(self, x: float, y: float) -> float:
        pose = self.robot_pose()
        return math.atan2(y - pose[1], x - pose[0]) if pose else 0.0

    def verify_candidate(self, candidate: Dict[str, Any]) -> None:
        schema = {
            "type": "object",
            "properties": {
                "is_target": {"type": "boolean"},
                "reason": {"type": "string"},
            },
            "required": ["is_target", "reason"], "additionalProperties": False,
        }
        context = {
            "instruction": self.instruction, "target_spec": self.spec,
            "candidate": {
                "object_id": candidate.get("object_id"), "label": candidate.get("label"),
                "confidence": candidate.get("confidence"), "confirmations": candidate.get("confirmations"),
                "room_label": candidate.get("room_label", "unknown"),
            },
        }
        self.submit_llm(
            "verify_object",
            "Verify whether the mapped candidate satisfies the complete object-navigation instruction. "
            "Require the object type and every stated room, attribute, and spatial condition. Return "
            "false when a condition cannot be established. Use only the supplied metadata and image.",
            json.dumps(context), schema, {"candidate": candidate}, use_image=True,
        )

    def accept_object(self, candidate: Dict[str, Any], reason: str) -> None:
        goal = self.safe_object_approach(candidate)
        if goal is None:
            self.rejected_objects.add(int(candidate.get("object_id", -1)))
            self.state = "EXPLORE_ROOM" if self.selected_room_id else "SELECT_ROOM"
            self.publish_state("no_safe_object_approach")
            return
        self.decision({
            "decision_type": "target_verified", "object_id": candidate.get("object_id"),
            "label": candidate.get("label"), "reason": reason, "pose": goal,
        })
        self.send_nav("approach_object", goal, {"object_id": candidate.get("object_id")})

    def safe_object_approach(self, obj: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if self.map_msg is None:
            return None
        robot = self.robot_pose()
        if robot is None:
            return None
        info = self.map_msg.info
        w, h = int(info.width), int(info.height)
        if len(self.map_msg.data) != w * h:
            return None
        grid = np.asarray(self.map_msg.data, dtype=np.int16).reshape((h, w))
        occ_thr = int(self.get_parameter("occupied_threshold").value)
        free = (grid >= 0) & (grid < occ_thr)
        clearance = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 3) * float(info.resolution)
        required = float(self.get_parameter("object_goal_clearance_m").value)
        ox, oy = float(obj.get("x", 0.0)), float(obj.get("y", 0.0))
        desired = float(self.get_parameter("object_approach_distance_m").value)
        width = float(self.get_parameter("object_approach_search_width_m").value)
        samples = int(self.get_parameter("object_approach_samples").value)
        best = None; best_score = float("inf")
        for radius in np.linspace(max(0.35, desired - width), desired + width, 5):
            for idx in range(samples):
                theta = 2.0 * math.pi * idx / max(1, samples)
                x, y = ox + radius * math.cos(theta), oy + radius * math.sin(theta)
                mx = int((x - float(info.origin.position.x)) / float(info.resolution))
                my = int((y - float(info.origin.position.y)) / float(info.resolution))
                if not (0 <= mx < w and 0 <= my < h and free[my, mx] and clearance[my, mx] >= required):
                    continue
                score = math.hypot(x - robot[0], y - robot[1]) + 0.3 * abs(radius - desired) - 0.2 * clearance[my, mx]
                if score < best_score:
                    best_score = score
                    best = {"frame_id": str(self.get_parameter("map_frame").value), "x": round(x, 3), "y": round(y, 3), "yaw": round(math.atan2(oy - y, ox - x), 3)}
        return best

    def choose_frontier(self) -> Optional[Dict[str, Any]]:
        room_frontiers = [f for f in self.frontiers if str(f.get("room_id")) == self.selected_room_id]
        if not room_frontiers:
            return None
        stable = [f for f in room_frontiers if bool(f.get("stable", False))]
        pool = stable or room_frontiers
        return max(pool, key=lambda f: float(f.get("score", -1e9)))

    def send_nav(self, semantic_action: str, pose: Dict[str, Any], context: Dict[str, Any]) -> None:
        command = {
            "action": "navigate_to_pose",
            "pose": pose,
            "mission_id": self.mission_id,
            "semantic_action": semantic_action,
            **context,
        }
        self.goal_count += 1
        self.active_semantic_action = semantic_action
        self.active_context = dict(context)
        self.decision({"decision_type": "nav_command", "command": command, "dry_run": not self.motion_enabled})
        if not self.motion_enabled:
            self.state = "PAUSED"; self.running = False
            self.publish_state("dry_run_proposal_only")
            return
        self.waiting_nav = True
        self.state = "WAIT_NAV"
        self.publish_json(self.nav_pub, command)
        self.publish_state(f"sent:{semantic_action}")

    def fail(self, reason: str) -> None:
        self.state = "FAILED"; self.running = False; self.waiting_nav = False
        self.publish_state(reason)

    def tick(self) -> None:
        if self.poll_llm():
            return
        if not self.running or self.pending is not None or self.waiting_nav:
            return
        if not self.instruction:
            return
        if time.time() - self.started_at > float(self.get_parameter("max_mission_duration_sec").value):
            self.fail("mission_timeout"); return
        if self.goal_count >= int(self.get_parameter("max_goals").value):
            self.fail("goal_budget_exhausted"); return
        if self.failure_count >= int(self.get_parameter("max_failures").value):
            self.fail("failure_budget_exhausted"); return

        if self.state == "DECOMPOSE":
            self.submit_llm(
                "decompose",
                "Decompose an indoor object-navigation instruction into the exact requested fields. "
                "Use empty strings for absent conditions. Do not add unstated constraints.",
                self.instruction, INSTRUCTION_SCHEMA, use_image=False,
            )
            return
        if self.state == "CHECK_OBJECTS":
            matches = self.matching_objects()
            if matches:
                self.state = "VERIFY_OBJECT"
                self.verify_candidate(matches[0])
                return
            if self.classify_current_room_if_needed():
                return
            self.state = "EXPLORE_ROOM" if self.selected_room_id else "SELECT_ROOM"
            self.publish_state("target_not_in_confirmed_map")
            return
        if self.state == "SELECT_ROOM":
            self.request_room_selection(); return
        if self.state == "EXPLORE_ROOM":
            frontier = self.choose_frontier()
            if frontier is None:
                if self.selected_room_id:
                    self.exhausted_rooms.add(self.selected_room_id)
                self.selected_room_id = ""
                self.state = "SELECT_ROOM"
                self.publish_state("room_frontiers_exhausted")
                return
            pose = {
                "frame_id": str(self.get_parameter("map_frame").value),
                "x": float(frontier["x"]), "y": float(frontier["y"]),
                "yaw": float(frontier["yaw"]),
            }
            self.decision({
                "decision_type": "classical_in_room_frontier", "frontier_id": frontier.get("frontier_id"),
                "room_id": frontier.get("room_id"), "stable": frontier.get("stable"),
                "view_confirmations": frontier.get("view_confirmations"), "pose": pose,
            })
            self.send_nav("explore_frontier", pose, {
                "frontier_id": frontier.get("frontier_id"), "room_id": frontier.get("room_id"),
            })
            return

    def destroy_node(self) -> bool:
        try:
            if self.pending is not None:
                self.pending.cancel()
            self._llm_executor.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VLNSupervisor()
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
