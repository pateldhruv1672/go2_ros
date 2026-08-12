from __future__ import annotations

import json
from typing import Any, Dict

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

from go2_langgraph_agent.graphs.agent_orchestrator import AgentOrchestrator
from go2_langgraph_agent.graphs.agent_state import LangGraphDependencyError, classify_intent, extract_text_from_message
from go2_langgraph_agent.persistence import LangGraphSQLitePersistence
from go2_langgraph_agent.tools.memory_tools import MemoryTools
from go2_langgraph_agent.tools.nav_tools import publish_nav_command
from go2_langgraph_agent.tools.world_tools import WorldAgentTools, classify_world_intent
import traceback


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


def _bounded_json_string(value, max_chars=60000):
    import json
    try:
        s = json.dumps(value, default=str)
    except Exception:
        s = str(value)
    if len(s) > max_chars:
        s = s[:max_chars] + f"...<truncated {len(s) - max_chars} chars>"
    return s


class MainSupervisor(Node):
    """ROS adapter for the real LangGraph Go2 agent runtime."""

    def __init__(self) -> None:
        super().__init__("go2_langgraph_main_supervisor")
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "default")
        self.declare_parameter("thread_id", "")
        self.declare_parameter("enable_debate_layer", True)
        self.declare_parameter("enable_llm_debate", False)
        self.declare_parameter("debate_llm_provider", "openrouter")
        self.declare_parameter("debate_llm_model", "openai/gpt-4o-mini")
        self.declare_parameter("debate_llm_timeout_sec", 8.0)
        self.declare_parameter("enable_tour_mode", False)
        self.declare_parameter("enable_explore_mode", False)
        self.declare_parameter("enable_nav_publish", False)
        self.declare_parameter("route_semantic_resume_commands", True)
        self.declare_parameter("enable_human_interrupts", False)
        self.declare_parameter("enable_langgraph_streaming", True)
        self.declare_parameter("langgraph_checkpoint_db", "")
        self.declare_parameter("langgraph_store_db", "")
        self.declare_parameter("require_native_langgraph_store", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("enable_object_navigation", False)
        self.declare_parameter("enable_web_search", False)
        self.declare_parameter("web_model", "gemini-2.5-flash")
        self.declare_parameter("enable_ollama_reasoner", True)
        self.declare_parameter("ollama_model", "gemma4:e4b")
        self.session_root = str(self.get_parameter("session_root").value)
        requested_session_name = str(self.get_parameter("session_name").value)
        thread_param = str(self.get_parameter("thread_id").value or "")
        self.memory = MemoryTools(self.session_root, requested_session_name)
        self.session_name = self.memory.session_name
        self.world_tools = WorldAgentTools(
            self.memory,
            enable_web_search=_as_bool(self.get_parameter("enable_web_search").value),
            web_model=str(self.get_parameter("web_model").value),
            enable_ollama_reasoner=_as_bool(self.get_parameter("enable_ollama_reasoner").value),
            ollama_model=str(self.get_parameter("ollama_model").value),
        )
        self.thread_id = thread_param or self.session_name
        if self.session_name != requested_session_name:
            self.get_logger().info(f"Resolved semantic session '{requested_session_name}' -> '{self.session_name}'")
        checkpoint_db = str(self.get_parameter("langgraph_checkpoint_db").value or "") or None
        store_db = str(self.get_parameter("langgraph_store_db").value or "") or None
        self.persistence = LangGraphSQLitePersistence(
            self.session_root,
            self.session_name,
            checkpoint_db,
            store_db,
            require_native_store=_as_bool(self.get_parameter("require_native_langgraph_store").value),
        )
        self.latest: Dict[str, Any] = {}
        self.nav_pub = self.create_publisher(String, "/go2_nav/command", 10)
        self.semantic_nav_pub = self.create_publisher(String, "/semantic_nav/command", 10)
        self.status_pub = self.create_publisher(String, "/go2_agent/status", 10)
        self.speech_pub = self.create_publisher(String, "/go2_agent/speech", 10)
        self.event_pub = self.create_publisher(String, "/go2_agent/events", 10)
        self.stream_pub = self.create_publisher(String, "/go2_agent/stream", 10)
        self.checkpoint_pub = self.create_publisher(String, "/go2_agent/checkpoints", 10)
        self.store_pub = self.create_publisher(String, "/go2_agent/store_results", 10)
        self.interrupt_pub = self.create_publisher(String, "/go2_agent/interrupts", 10)
        for topic in ("/go2_agent/command", "/go2_agent/user_command"):
            self.create_subscription(String, topic, self._on_command, 10)
        self.create_subscription(String, "/go2_agent/query", self._on_query, 10)
        self.create_subscription(String, "/go2_agent/resume", self._on_resume, 10)
        self.create_subscription(String, "/go2_agent/checkpoints/request", self._on_checkpoint_request, 10)
        self.create_subscription(String, "/go2_agent/time_travel", self._on_time_travel, 10)
        self.create_subscription(String, "/go2_agent/store/query", self._on_store_query, 10)
        self.create_subscription(String, "/go2_agent/store/write", self._on_store_write, 10)
        self.create_subscription(String, "/go2_perception/scan_summary", lambda m: self._store_json("scan_summary", m), 10)
        self.create_subscription(String, "/go2_perception/pointcloud_summary", lambda m: self._store_json("pointcloud_summary", m), 10)
        self.create_subscription(String, "/go2_perception/traversability_summary", lambda m: self._store_json("traversability", m), 10)
        self.create_subscription(String, "/go2_perception/open_vocab_detections", lambda m: self._store_json("open_vocab_detections", m), 10)
        self.create_subscription(String, "/go2_perception/dynamic_obstacles", lambda m: self._store_json("dynamic_obstacles", m), 10)
        self.create_subscription(String, "/go2_memory/object_inventory", lambda m: self._store_json("object_inventory", m), 10)
        self.create_subscription(String, "/go2_vln/object_map", lambda m: self._store_json("object_map", m), 10)
        self.create_subscription(String, "/go2_nav/frontier_candidates", lambda m: self._store_json("frontier_candidates", m), 10)
        self.create_subscription(String, "/go2_nav/coverage_plan", lambda m: self._store_json("coverage_waypoints", m), 10)
        self.create_subscription(String, "/go2_nav/status", lambda m: self._store_json("nav_status", m), 10)
        self.create_subscription(String, "/semantic_nav/status", lambda m: self._store_json("semantic_nav_status", m), 10)
        self.create_subscription(String, "/semantic_nav/event", lambda m: self._store_json("semantic_nav_event", m), 10)
        self.create_subscription(String, "/go2_vlm_checkpoint/status", lambda m: self._store_vlm(m), 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10)
        try:
            self.orchestrator = AgentOrchestrator(
                self.memory,
                self.persistence,
                {
                    "enable_debate_layer": _as_bool(self.get_parameter("enable_debate_layer").value),
                    "enable_llm_debate": _as_bool(self.get_parameter("enable_llm_debate").value),
                    "debate_llm_provider": str(self.get_parameter("debate_llm_provider").value),
                    "debate_llm_model": str(self.get_parameter("debate_llm_model").value),
                    "debate_llm_timeout_sec": float(self.get_parameter("debate_llm_timeout_sec").value),
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
            "Real LangGraph Go2 supervisor ready: StateGraph + SQLite checkpoints + conditional subgraphs "
            f"+ ROS tool routing; session={self.session_name}"
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

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest["map_pose"] = {
            "frame_id": msg.header.frame_id or "map",
            "x": float(p.x), "y": float(p.y), "z": float(p.z),
            "qx": float(q.x), "qy": float(q.y), "qz": float(q.z), "qw": float(q.w),
            "stamp_sec": float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec),
        }

    def _live_context(self) -> Dict[str, Any]:
        return dict(self.latest)

    def _publish_status(self, payload: Dict[str, Any]) -> None:
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_events(self, payload: Dict[str, Any]) -> None:
        self.event_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))

    def _publish_result(self, state: Dict[str, Any]) -> None:
        command = state.get("nav_command") or {}
        semantic_command = self._semantic_command_from_nav_command(command)
        if semantic_command:
            self.semantic_nav_pub.publish(String(data=json.dumps(semantic_command, sort_keys=True, default=str)))
        elif command and _as_bool(self.get_parameter("enable_nav_publish").value):
            publish_nav_command(self.nav_pub, command.get("action", "none"), command)
        speech = state.get("speech_response", "")
        if speech:
            self.speech_pub.publish(String(data=speech))
        status = state.get("status", {})
        self._publish_status(status)
        pending = state.get("pending_interrupt") or {}
        if pending:
            self.interrupt_pub.publish(String(data=json.dumps(pending, sort_keys=True, default=str)))

    def _try_handle_world_intent(self, msg: String, raw_payload: Any = None, read_only: bool = False) -> bool:
        text = extract_text_from_message(msg.data)
        payload = raw_payload if isinstance(raw_payload, dict) else (_json_or_text(msg) if msg else {})
        hint = str((payload or {}).get("intent") or (payload or {}).get("intent_hint") or "") if isinstance(payload, dict) else ""
        parsed = classify_world_intent(text, hint)
        intent = parsed.get("intent") or ""
        if not intent:
            return False
        label = str(parsed.get("object_query") or (payload or {}).get("semantic_target") or "object") if isinstance(payload, dict) else str(parsed.get("object_query") or "object")
        nearest = bool(parsed.get("nearest"))
        robot_map_pose = self.latest.get("map_pose") if isinstance(self.latest.get("map_pose"), dict) else None
        room = str(parsed.get("room") or "")
        # SPARKY_VISIBLE_MEMORY_SUPERVISOR_V2
        if intent == "visible_objects":
            result = self.world_tools.visible_objects(self.latest.get("object_inventory") if isinstance(self.latest.get("object_inventory"), dict) else {})
        elif intent == "memory_summary":
            result = self.world_tools.memory_summary()
        elif intent == "count_objects":
            result = self.world_tools.count_objects(label, room, self.latest.get("object_inventory") if isinstance(self.latest.get("object_inventory"), dict) else {})
        elif intent == "where_object":
            result = self.world_tools.where_object(label, room, robot_map_pose=robot_map_pose, nearest=nearest)
        elif intent == "web_search":
            result = self.world_tools.web_search(text)
        elif intent == "find_object":
            if read_only:
                result = {"success": False, "speech": "Finding that object requires robot motion. Send it as a command so the confirmation gate can verify it."}
            elif not isinstance(payload, dict) or not bool(payload.get("verified")) or not bool(payload.get("safety_checked")):
                result = {"success": False, "speech": "I will not move for an unverified object-search request. Please send it through the phone command confirmation path."}
            elif not _as_bool(self.get_parameter("enable_object_navigation").value):
                result = {"success": False, "speech": "I found the object in memory, but object navigation is disabled on this run."}
            else:
                result = self.world_tools.find_object(label, room, robot_map_pose=robot_map_pose, nearest=nearest)
                nav = result.get("nav_command") or {}
                if nav:
                    # SPARKY_OBJECT_NAV_VIA_SEMANTIC_NAV_V1
                    pose = nav.get("pose") if isinstance(nav.get("pose"), dict) else {}
                    semantic_object_goal = {
                        "type": "go_pose",
                        "source": "langgraph_agent",
                        "name": f"object_{nav.get('object_label') or label}_{nav.get('object_id') or 'memory'}",
                        "object_id": nav.get("object_id"),
                        "object_label": nav.get("object_label") or label,
                        "reason": nav.get("reason") or "memory_first_object_search",
                        "verify_live_after_arrival": bool(nav.get("verify_live_after_arrival", True)),
                        "pose": pose,
                    }
                    self.semantic_nav_pub.publish(
                        String(data=json.dumps(semantic_object_goal, sort_keys=True, default=str))
                    )
        else:
            return False
        speech = str(result.get("speech") or "")
        if speech:
            self.speech_pub.publish(String(data=speech))
        self._publish_status({
            "world_intent": intent,
            "world_result": {k: v for k, v in result.items() if k not in {"object", "objects", "result", "web_result"}},
            "read_only": read_only,
            "session_name": self.session_name,
        })
        return True

    def _semantic_resume_command(self, msg: String) -> Dict[str, Any]:
        if not _as_bool(self.get_parameter("route_semantic_resume_commands").value):
            return {}
        payload = _json_or_text(msg)
        text = extract_text_from_message(msg.data)
        intent = ""
        destination = ""
        if isinstance(payload, dict):
            intent = str(payload.get("intent") or payload.get("intent_hint") or payload.get("type") or "")
            destination = str(payload.get("semantic_target") or payload.get("destination") or payload.get("place") or "")
        parsed = classify_intent(text)
        classified = str((parsed or {}).get("intent") or "")
        entities = (parsed or {}).get("entities") or {}
        destination = destination or str(entities.get("destination") or "")

        if intent == "start_tour" or classified == "start_tour":
            return {"type": "start_tour", "source": "langgraph_agent", "reset_index": True}
        if intent == "continue_tour" or classified == "continue_task":
            return {"type": "resume_tour", "source": "langgraph_agent", "speech": "tour: Resuming the saved route."}
        if intent == "skip_checkpoint":
            return {"type": "advance_tour", "source": "langgraph_agent"}
        if intent == "navigate_to_place" or classified == "navigate":
            if destination:
                return {"type": "go", "source": "langgraph_agent", "place": destination}
        return {}

    def _semantic_command_from_nav_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        if not command:
            return {}
        action = str(command.get("action") or "")
        if action in {"", "none"}:
            return {}
        if action == "stop_robot":
            return {
                "type": "cancel",
                "source": "langgraph_agent",
                "reason": command.get("reason") or "langgraph_stop",
            }
        if action in {"navigate_to_place", "go", "navigate"}:
            place = str(command.get("place") or command.get("destination") or command.get("semantic_target") or "").strip()
            if place:
                return {"type": "go", "source": "langgraph_agent", "place": place}
        if action == "return_to_spawn":
            return {"type": "go", "source": "langgraph_agent", "place": "spawn"}
        if action == "navigate_tour_route":
            return {
                "type": "resume_tour",
                "source": "langgraph_agent",
                "speech": "tour: Resuming the saved route.",
            }
        if action in {"recover_nav_failure", "recover_localization"}:
            return {
                "type": "recover_route",
                "source": "langgraph_agent",
                "reason": command.get("reason") or command.get("issue") or action,
            }
        return {}

    def _on_query(self, msg: String) -> None:
        """Run a conversational/memory query with a hard no-motion boundary."""
        try:
            raw_payload = _json_or_text(msg)
            if self._try_handle_world_intent(msg, raw_payload, read_only=True):
                return
            text = extract_text_from_message(msg.data)
            parsed = classify_intent(text)
            if bool((parsed or {}).get("requires_motion")):
                self.speech_pub.publish(String(data="That looks like a motion request. Send it as a command so the safety confirmation gate can verify it."))
                self._publish_status({
                    "read_only_query": "motion_refused",
                    "parsed_intent": parsed,
                    "session_name": self.session_name,
                })
                return
            if _as_bool(self.get_parameter("enable_langgraph_streaming").value):
                state, updates = self.orchestrator.run_with_stream(msg.data)
                for idx, update in enumerate(updates):
                    self.stream_pub.publish(String(data=json.dumps({"index": idx, "update": update, "read_only": True}, sort_keys=True, default=str)))
            else:
                state = self.orchestrator.invoke(msg.data)
            state = dict(state or {})
            blocked_command = dict(state.get("nav_command") or {})
            state["nav_command"] = {}
            self._publish_result(state)
            if blocked_command:
                self._publish_status({
                    "read_only_query": "motion_output_suppressed",
                    "suppressed_nav_command": blocked_command,
                    "session_name": self.session_name,
                })
            self._publish_events({"type": "read_only_query_result", "run_id": state.get("run_id"), "events": state.get("events", [])[-25:]})
        except Exception as exc:
            self.get_logger().error(f"Read-only LangGraph query failed: {exc}\n{traceback.format_exc()}")
            self.speech_pub.publish(String(data="I could not answer that query, and I did not send any motion command."))
            self._publish_status({"read_only_query": "failed", "error": str(exc)})

    def _on_command(self, msg: String) -> None:
        try:
            raw_payload = _json_or_text(msg)
            if self._try_handle_world_intent(msg, raw_payload, read_only=False):
                return
            if (
                isinstance(raw_payload, dict)
                and raw_payload.get("source") in {"langgraph_agent", "omi_tour_voice_router"}
                and raw_payload.get("type") in {"start_tour", "resume_tour", "advance_tour", "pause_tour", "go"}
            ):
                return
            if (
                isinstance(raw_payload, dict)
                and raw_payload.get("source") == "omi_voice"
                and str(raw_payload.get("intent") or raw_payload.get("intent_hint") or "")
                in {"start_tour", "continue_tour", "skip_checkpoint"}
            ):
                self._publish_status({"semantic_resume_command": "handled_by_tour_voice_router", "session_name": self.session_name})
                return
            semantic_command = self._semantic_resume_command(msg)
            if semantic_command:
                # SPARKY_FRONTDOOR_NO_DUPLICATE_ROUTING_SPEECH_V12_9
                self.semantic_nav_pub.publish(String(data=json.dumps(semantic_command, sort_keys=True)))
                metadata = (raw_payload.get("metadata") or {}) if isinstance(raw_payload, dict) else {}
                frontdoor_handled = bool((raw_payload or {}).get("frontdoor_handled")) if isinstance(raw_payload, dict) else False
                frontdoor_handled = frontdoor_handled or bool(metadata.get("frontdoor"))
                if not frontdoor_handled:
                    self.speech_pub.publish(String(data="Routing that through saved resume memory."))
                self._publish_status({"semantic_resume_command": semantic_command, "session_name": self.session_name, "frontdoor_handled": frontdoor_handled})
                return
            if _as_bool(self.get_parameter("enable_langgraph_streaming").value):
                state, updates = self.orchestrator.run_with_stream(msg.data)
                for idx, update in enumerate(updates):
                    self.stream_pub.publish(String(data=json.dumps({"index": idx, "update": update}, sort_keys=True, default=str)))
            else:
                state = self.orchestrator.invoke(msg.data)
            self._publish_result(state)
            self._publish_events({"type": "graph_result", "run_id": state.get("run_id"), "events": state.get("events", [])[-25:]})
        except Exception as exc:
            self.get_logger().error(f"LangGraph invocation failed: {exc}\n{traceback.format_exc()}")
            command = {"action": "stop_robot", "reason": "langgraph_invocation_failed", "error": str(exc)}
            semantic_command = self._semantic_command_from_nav_command(command)
            if semantic_command:
                self.semantic_nav_pub.publish(String(data=json.dumps(semantic_command, sort_keys=True, default=str)))
            elif _as_bool(self.get_parameter("enable_nav_publish").value):
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
            self.get_logger().error(f"LangGraph resume failed: {exc}\n{traceback.format_exc()}")
            self._publish_events({"type": "resume_failed", "thread_id": self.thread_id, "error": str(exc), "payload": payload})
            self.speech_pub.publish(String(data="I could not resume the interrupted LangGraph task, so I am staying stopped."))


    def _on_checkpoint_request(self, msg: String) -> None:
        payload = _json_or_text(msg)
        limit = 10
        checkpoint_id = None
        if isinstance(payload, dict):
            limit = int(payload.get("limit", 10))
            checkpoint_id = payload.get("checkpoint_id")
        try:
            if checkpoint_id:
                result = self.orchestrator.get_checkpoint_state(str(checkpoint_id))
            else:
                result = {"thread_id": self.thread_id, "checkpoints": self.orchestrator.list_checkpoints(limit=limit)}
            self.checkpoint_pub.publish(String(data=json.dumps(result, sort_keys=True, default=str)))
        except Exception as exc:
            self.checkpoint_pub.publish(String(data=json.dumps({"error": str(exc)}, sort_keys=True)))

    def _on_time_travel(self, msg: String) -> None:
        payload = _json_or_text(msg)
        if not isinstance(payload, dict):
            self._publish_events({"type": "time_travel_failed", "error": "payload must be JSON object"})
            return
        checkpoint_id = str(payload.get("checkpoint_id") or "")
        command = payload.get("command")
        if not checkpoint_id:
            self._publish_events({"type": "time_travel_failed", "error": "missing checkpoint_id"})
            return
        try:
            state = self.orchestrator.time_travel_invoke(checkpoint_id, str(command) if command else None)
            self._publish_result(state)
            self._publish_events({"type": "time_travel_completed", "checkpoint_id": checkpoint_id, "run_id": state.get("run_id")})
        except Exception as exc:
            self.get_logger().exception(f"LangGraph time travel failed: {exc}")
            self._publish_events({"type": "time_travel_failed", "checkpoint_id": checkpoint_id, "error": str(exc)})
            self.speech_pub.publish(String(data="I could not time-travel to that LangGraph checkpoint, so I am staying stopped."))

    def _on_store_query(self, msg: String) -> None:
        payload = _json_or_text(msg)
        namespace = ["threads", self.thread_id]
        query = ""
        limit = 20
        if isinstance(payload, dict):
            namespace = payload.get("namespace", namespace)
            query = str(payload.get("query", ""))
            limit = int(payload.get("limit", 20))
        try:
            result = {"namespace": namespace, "query": query, "results": self.persistence.store.search(namespace, query=query, limit=limit)}
            self.store_pub.publish(String(data=json.dumps(result, sort_keys=True, default=str)))
        except Exception as exc:
            self.store_pub.publish(String(data=json.dumps({"error": str(exc)}, sort_keys=True)))

    def _on_store_write(self, msg: String) -> None:
        payload = _json_or_text(msg)
        if not isinstance(payload, dict):
            self.store_pub.publish(String(data=json.dumps({"error": "payload must be JSON object"}, sort_keys=True)))
            return
        namespace = payload.get("namespace", ["threads", self.thread_id, "manual"] )
        key = str(payload.get("key") or payload.get("id") or "manual_write")
        value = payload.get("value", payload)
        try:
            self.persistence.store.put(namespace, key, value if isinstance(value, dict) else {"value": value})
            self.store_pub.publish(String(data=json.dumps({"ok": True, "namespace": namespace, "key": key}, sort_keys=True, default=str)))
        except Exception as exc:
            self.store_pub.publish(String(data=json.dumps({"error": str(exc)}, sort_keys=True)))

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
