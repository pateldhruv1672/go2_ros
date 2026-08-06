from __future__ import annotations

import base64
import json
import math
import time
from concurrent.futures import Future, ThreadPoolExecutor
from functools import partial
from typing import Any, Dict, List, Optional, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .common import quat_to_yaw
from .ollama_structured import StructuredOllamaClient


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


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class AutonomousExploreSupervisor(Node):
    """Bounded autonomous exploration with deterministic safety gates.

    Semantic perception and Ollama label rooms and veto visually unsafe
    frontier proposals. They never emit poses or velocities. Safe frontier poses
    come from hierarchical_frontier_planner and are dispatched only through the
    Nav2 tool server, which performs ComputePathToPose preflight before
    NavigateToPose. The visual guard is an additional veto, never a replacement
    for geometry, Nav2, scan checks, or collision monitoring.
    """

    def __init__(self) -> None:
        super().__init__("go2_autonomous_explore_supervisor")

        # Topic and frame contract.
        self.declare_parameter("command_topic", "/go2_vln/command")
        self.declare_parameter("state_topic", "/go2_vln/explore_state")
        self.declare_parameter("decision_topic", "/go2_vln/decision")
        self.declare_parameter("frontier_topic", "/go2_vln/frontier_candidates")
        self.declare_parameter("room_graph_topic", "/go2_vln/room_graph")
        self.declare_parameter("room_label_update_topic", "/go2_vln/room_label_update")
        self.declare_parameter("object_map_topic", "/go2_vln/object_map")
        self.declare_parameter("blacklist_topic", "/go2_vln/frontier_blacklist")
        self.declare_parameter("nav_command_topic", "/go2_nav/command")
        self.declare_parameter("nav_status_topic", "/go2_nav/status")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan_nav")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        # Explicit arming and bounded mission policy.
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("decision_period_sec", 0.50)
        self.declare_parameter("startup_grace_sec", 8.0)
        self.declare_parameter("max_mission_duration_sec", 600.0)
        self.declare_parameter("max_goals", 12)
        self.declare_parameter("max_failures", 4)
        self.declare_parameter("max_travel_distance_m", 20.0)
        self.declare_parameter("post_goal_settle_sec", 3.0)
        self.declare_parameter("no_frontier_complete_sec", 20.0)
        self.declare_parameter("reached_frontier_blacklist_ttl_sec", 180.0)
        self.declare_parameter("failed_frontier_blacklist_ttl_sec", 180.0)
        self.declare_parameter("require_stable_frontier", True)
        self.declare_parameter("min_frontier_confirmations", 2)
        self.declare_parameter("health_loss_stop_sec", 2.0)

        # Runtime health contract.
        self.declare_parameter(
            "required_lifecycle_nodes",
            [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "collision_monitor",
            ],
        )
        self.declare_parameter("lifecycle_state_max_age_sec", 4.0)
        self.declare_parameter("scan_receive_max_age_sec", 1.0)
        self.declare_parameter("scan_stamp_max_age_sec", 1.5)
        self.declare_parameter("scan_future_tolerance_sec", 0.20)
        self.declare_parameter("min_valid_scan_points", 25)
        self.declare_parameter("tf_lookup_timeout_sec", 0.15)
        self.declare_parameter("frontier_feed_max_age_sec", 3.0)

        # Ollama is advisory: it labels rooms after safe navigation succeeds.
        self.declare_parameter("ollama_url", "http://127.0.0.1:11434")
        self.declare_parameter("ollama_model", "gemma4:e4b")
        self.declare_parameter("ollama_timeout_sec", 90.0)
        self.declare_parameter("ollama_think", "false")
        self.declare_parameter("ollama_keep_alive", "10m")
        self.declare_parameter("ollama_debug_topic", "/go2_vln/ollama_debug")
        self.declare_parameter(
            "ollama_debug_jsonl", "~/.ros/go2_sysnav_vln/ollama_calls.jsonl"
        )
        self.declare_parameter("enable_room_labeling", True)
        self.declare_parameter("image_max_age_sec", 2.0)
        self.declare_parameter("image_jpeg_quality", 65)

        # Optional visual veto before each frontier dispatch. A current frame is
        # considered relevant only when the frontier bearing lies inside the
        # assumed forward camera field of view. Out-of-view frontiers are never
        # assessed using an unrelated image.
        self.declare_parameter("enable_frontier_vision_guard", False)
        self.declare_parameter("frontier_camera_half_fov_deg", 40.0)
        self.declare_parameter("frontier_vision_min_confidence", 0.65)
        self.declare_parameter("frontier_vision_cache_sec", 3.0)
        self.declare_parameter("frontier_vision_reject_ttl_sec", 20.0)
        self.declare_parameter("frontier_vision_unknown_ttl_sec", 5.0)
        self.declare_parameter(
            "room_types",
            [
                "classroom",
                "laboratory",
                "office room",
                "meeting room",
                "computer lab",
                "restroom",
                "storage room",
                "copy room",
                "student lounge",
                "reception",
                "corridor",
                "kitchen",
                "living room",
                "unknown",
            ],
        )

        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ollama = StructuredOllamaClient(
            str(self.get_parameter("ollama_url").value),
            str(self.get_parameter("ollama_model").value),
            float(self.get_parameter("ollama_timeout_sec").value),
            str(self.get_parameter("ollama_keep_alive").value),
            str(self.get_parameter("ollama_debug_jsonl").value),
        )
        self._llm_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="go2-explore-vlm"
        )
        self._llm_future: Optional[Future] = None
        self._llm_context: Dict[str, Any] = {}
        self._request_seq = 0

        self.running = as_bool(self.get_parameter("auto_start").value)
        self.state = "WAIT_HEALTH" if self.running else "PAUSED"
        self.reason = "auto_start" if self.running else "operator_start_required"
        self.mission_id = 1 if self.running else 0
        self.started_mono = time.monotonic() if self.running else 0.0
        self.goal_count = 0
        self.failure_count = 0
        self.distance_travelled_m = 0.0
        self.last_distance_pose: Optional[Tuple[float, float, float]] = None
        self.waiting_nav = False
        self.active_frontier: Optional[Dict[str, Any]] = None
        self.active_nav_seq: Optional[int] = None
        self.post_goal_until = 0.0
        self.health_bad_since: Optional[float] = None
        self.last_state_publish = 0.0
        self.last_no_candidate_since: Optional[float] = None

        self.frontiers: List[Dict[str, Any]] = []
        self.frontier_planner_status: Dict[str, Any] = {"reason": "not_received"}
        self.rooms: List[Dict[str, Any]] = []
        self.objects: List[Dict[str, Any]] = []
        self.current_room_id: Optional[str] = None
        self.local_blacklist: Dict[str, float] = {}
        self.last_frontier_wall = 0.0
        self.map_received = False
        self.last_map_wall = 0.0
        self.last_scan_wall = 0.0
        self.scan_stamp_age_sec: Optional[float] = None
        self.valid_scan_points = 0
        self.latest_image: Optional[Image] = None
        self.latest_image_wall = 0.0
        self.frontier_vision_cache: Dict[str, Dict[str, Any]] = {}
        self.pending_frontier: Optional[Dict[str, Any]] = None
        self.last_selection_reason = "not_selected"

        self.required_lifecycle_nodes = [
            str(name).strip("/")
            for name in self.get_parameter("required_lifecycle_nodes").value
        ]
        self.lifecycle_states: Dict[str, Dict[str, Any]] = {
            name: {"id": None, "label": "unknown", "wall": 0.0}
            for name in self.required_lifecycle_nodes
        }
        self.lifecycle_pending: set[str] = set()
        self.lifecycle_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in self.required_lifecycle_nodes
        }

        self.state_pub = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), 20
        )
        self.decision_pub = self.create_publisher(
            String, str(self.get_parameter("decision_topic").value), 20
        )
        self.nav_pub = self.create_publisher(
            String, str(self.get_parameter("nav_command_topic").value), 20
        )
        self.blacklist_pub = self.create_publisher(
            String, str(self.get_parameter("blacklist_topic").value), 20
        )
        self.room_label_pub = self.create_publisher(
            String, str(self.get_parameter("room_label_update_topic").value), 10
        )
        self.ollama_debug_pub = self.create_publisher(
            String, str(self.get_parameter("ollama_debug_topic").value), 20
        )

        self.create_subscription(
            String, str(self.get_parameter("command_topic").value), self.on_command, 20
        )
        self.create_subscription(
            String, str(self.get_parameter("frontier_topic").value), self.on_frontiers, 20
        )
        self.create_subscription(
            String, str(self.get_parameter("room_graph_topic").value), self.on_rooms, 20
        )
        self.create_subscription(
            String, str(self.get_parameter("object_map_topic").value), self.on_objects, 20
        )
        self.create_subscription(
            String, str(self.get_parameter("nav_status_topic").value), self.on_nav_status, 20
        )
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map_topic").value), self.on_map, 10
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            qos_profile_sensor_data,
        )

        self.create_timer(1.0, self.refresh_lifecycle_states)
        self.create_timer(
            float(self.get_parameter("decision_period_sec").value), self.tick
        )
        self.get_logger().info(
            "bounded autonomous explorer ready; auto_start=%s enable_motion=%s"
            % (self.running, self.motion_enabled)
        )
        self.publish_state(force=True)

    @property
    def motion_enabled(self) -> bool:
        return as_bool(self.get_parameter("enable_motion").value)

    def publish_json(self, publisher: Any, payload: Dict[str, Any]) -> None:
        publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def decision(self, payload: Dict[str, Any]) -> None:
        record = dict(payload)
        record.setdefault("stamp_sec", time.time())
        record.setdefault("mission_id", self.mission_id)
        record.setdefault("state", self.state)
        record.setdefault("mode", "autonomous_explore")
        self.publish_json(self.decision_pub, record)

    def publish_state(self, reason: Optional[str] = None, force: bool = False) -> None:
        if reason is not None:
            self.reason = reason
        now = time.monotonic()
        if not force and now - self.last_state_publish < 0.5:
            return
        self.last_state_publish = now
        health_ok, health = self.health_snapshot()
        self.publish_json(
            self.state_pub,
            {
                "stamp_sec": time.time(),
                "mission_id": self.mission_id,
                "mode": "autonomous_explore",
                "state": self.state,
                "reason": self.reason,
                "running": self.running,
                "motion_enabled": self.motion_enabled,
                "waiting_nav": self.waiting_nav,
                "goal_count": self.goal_count,
                "failure_count": self.failure_count,
                "distance_travelled_m": round(self.distance_travelled_m, 3),
                "frontier_count": len(self.frontiers),
                "frontier_planner_status": self.frontier_planner_status,
                "room_count": len(self.rooms),
                "object_count": len(self.objects),
                "current_room_id": self.current_room_id,
                "active_frontier_id": None
                if self.active_frontier is None
                else self.active_frontier.get("frontier_id"),
                "frontier_vision_guard_enabled": as_bool(
                    self.get_parameter("enable_frontier_vision_guard").value
                ),
                "pending_frontier_vision_id": None
                if self.pending_frontier is None
                else self.pending_frontier.get("frontier_id"),
                "last_selection_reason": self.last_selection_reason,
                "health_ok": health_ok,
                "health": health,
            },
        )

    def on_command(self, msg: String) -> None:
        try:
            payload = (
                json.loads(msg.data)
                if msg.data.strip().startswith("{")
                else {"action": msg.data}
            )
        except Exception:
            payload = {"action": msg.data}
        action = str(payload.get("action", "")).strip().lower()
        if action in {"start", "resume", "start_explore", "explore"}:
            self.mission_id += 1
            self.running = True
            self.state = "WAIT_HEALTH"
            self.reason = "operator_start"
            self.started_mono = time.monotonic()
            self.goal_count = 0
            self.failure_count = 0
            self.distance_travelled_m = 0.0
            self.last_distance_pose = self.robot_pose()
            self.waiting_nav = False
            self.active_frontier = None
            self.active_nav_seq = None
            self.local_blacklist.clear()
            self.frontier_vision_cache.clear()
            self.pending_frontier = None
            self.last_selection_reason = "mission_started"
            self.last_no_candidate_since = None
            self.publish_state(force=True)
        elif action in {"stop", "pause", "cancel", "emergency_stop"}:
            self.stop_robot(f"operator_{action}")
            self.running = False
            self.waiting_nav = False
            self.state = "PAUSED"
            self.publish_state(f"operator_{action}", force=True)
        elif action == "reset":
            self.stop_robot("operator_reset")
            self.running = False
            self.state = "PAUSED"
            self.goal_count = 0
            self.failure_count = 0
            self.distance_travelled_m = 0.0
            self.local_blacklist.clear()
            self.frontier_vision_cache.clear()
            self.pending_frontier = None
            self.last_selection_reason = "reset"
            self.active_frontier = None
            self.active_nav_seq = None
            self.publish_state("reset", force=True)

    def on_frontiers(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.frontiers = [
                item for item in payload.get("candidates", []) if isinstance(item, dict)
            ]
            status = payload.get("planner_status", {})
            self.frontier_planner_status = (
                status if isinstance(status, dict) else {"reason": "invalid_status"}
            )
            self.current_room_id = payload.get(
                "current_room_id", self.current_room_id
            )
            self.last_frontier_wall = time.time()
        except Exception as exc:
            self.get_logger().warning(f"invalid frontier payload: {exc}")

    def on_rooms(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.rooms = [
                item for item in payload.get("rooms", []) if isinstance(item, dict)
            ]
            self.current_room_id = payload.get(
                "current_room_id", self.current_room_id
            )
        except Exception:
            pass

    def on_objects(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.objects = [
                item for item in payload.get("objects", []) if isinstance(item, dict)
            ]
        except Exception:
            pass

    def on_map(self, _msg: OccupancyGrid) -> None:
        self.map_received = True
        self.last_map_wall = time.time()

    def on_image(self, msg: Image) -> None:
        self.latest_image = msg
        self.latest_image_wall = time.time()

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan_wall = time.time()
        minimum = max(float(msg.range_min), 0.0)
        maximum = float(msg.range_max)
        self.valid_scan_points = sum(
            1
            for value in msg.ranges
            if finite(value) and minimum <= float(value) <= maximum
        )
        try:
            stamp = Time.from_msg(msg.header.stamp)
            self.scan_stamp_age_sec = (
                self.get_clock().now().nanoseconds - stamp.nanoseconds
            ) / 1e9
        except Exception:
            self.scan_stamp_age_sec = None

    def refresh_lifecycle_states(self) -> None:
        for name, client in self.lifecycle_clients.items():
            if name in self.lifecycle_pending or not client.service_is_ready():
                continue
            self.lifecycle_pending.add(name)
            future = client.call_async(GetState.Request())
            future.add_done_callback(partial(self.on_lifecycle_state, name))

    def on_lifecycle_state(self, name: str, future: Future) -> None:
        self.lifecycle_pending.discard(name)
        try:
            response = future.result()
            self.lifecycle_states[name] = {
                "id": int(response.current_state.id),
                "label": str(response.current_state.label),
                "wall": time.time(),
            }
        except Exception as exc:
            self.lifecycle_states[name] = {
                "id": None,
                "label": f"error:{exc}",
                "wall": time.time(),
            }

    def robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                Time(),
                timeout=Duration(
                    seconds=float(self.get_parameter("tf_lookup_timeout_sec").value)
                ),
            )
            return (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                quat_to_yaw(transform.transform.rotation),
            )
        except Exception:
            return None

    def health_snapshot(self) -> Tuple[bool, Dict[str, Any]]:
        now_wall = time.time()
        reasons: List[str] = []
        lifecycle_max_age = float(
            self.get_parameter("lifecycle_state_max_age_sec").value
        )
        lifecycle = {}
        for name, data in self.lifecycle_states.items():
            age = None if data["wall"] <= 0.0 else now_wall - float(data["wall"])
            active = data["id"] == 3 and age is not None and age <= lifecycle_max_age
            lifecycle[name] = {
                "active": active,
                "id": data["id"],
                "label": data["label"],
                "age_sec": None if age is None else round(age, 3),
            }
            if not active:
                reasons.append(f"lifecycle:{name}")

        if not self.map_received:
            reasons.append("map_missing")

        scan_receive_age = (
            None if self.last_scan_wall <= 0.0 else now_wall - self.last_scan_wall
        )
        if scan_receive_age is None or scan_receive_age > float(
            self.get_parameter("scan_receive_max_age_sec").value
        ):
            reasons.append("scan_stale")
        if self.valid_scan_points < int(
            self.get_parameter("min_valid_scan_points").value
        ):
            reasons.append("scan_too_sparse")
        if self.scan_stamp_age_sec is None:
            reasons.append("scan_stamp_missing")
        else:
            if self.scan_stamp_age_sec > float(
                self.get_parameter("scan_stamp_max_age_sec").value
            ):
                reasons.append("scan_stamp_old")
            if self.scan_stamp_age_sec < -float(
                self.get_parameter("scan_future_tolerance_sec").value
            ):
                reasons.append("scan_stamp_future")

        pose = self.robot_pose()
        if pose is None:
            reasons.append("map_to_base_tf_missing")

        frontier_age = (
            None
            if self.last_frontier_wall <= 0.0
            else now_wall - self.last_frontier_wall
        )
        if frontier_age is None or frontier_age > float(
            self.get_parameter("frontier_feed_max_age_sec").value
        ):
            reasons.append("frontier_feed_stale")

        return (
            not reasons,
            {
                "reasons": reasons,
                "lifecycle": lifecycle,
                "map_received": self.map_received,
                "scan_receive_age_sec": None
                if scan_receive_age is None
                else round(scan_receive_age, 3),
                "scan_stamp_age_sec": None
                if self.scan_stamp_age_sec is None
                else round(self.scan_stamp_age_sec, 3),
                "valid_scan_points": self.valid_scan_points,
                "tf_available": pose is not None,
                "frontier_feed_age_sec": None
                if frontier_age is None
                else round(frontier_age, 3),
            },
        )

    def on_nav_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except Exception:
            return
        action = str(status.get("action", ""))
        seq = status.get("seq")
        if action == "preflight_sent" and self.waiting_nav and seq is not None:
            self.active_nav_seq = int(seq)
            return
        if (
            self.active_nav_seq is not None
            and seq is not None
            and int(seq) != self.active_nav_seq
        ):
            return
        failure_actions = {
            "preflight_no_path",
            "preflight_rejected",
            "preflight_error",
            "goal_rejected",
            "goal_response_error",
            "goal_result_error",
            "parse_error",
        }
        generic_failure = (
            self.waiting_nav
            and status.get("success") is False
            and action not in {"goal_result", "stop_robot"}
        )
        if action in failure_actions or generic_failure:
            self.navigation_failed(action or "nav_status_failure")
            return
        if action == "goal_result" and self.waiting_nav:
            if bool(status.get("success", False)):
                self.navigation_succeeded()
            else:
                self.navigation_failed(f"goal_status_{status.get('status')}")

    def blacklist(self, frontier_id: str, ttl_sec: float, reason: str) -> None:
        if not frontier_id:
            return
        self.local_blacklist[frontier_id] = time.monotonic() + ttl_sec
        self.publish_json(
            self.blacklist_pub,
            {"frontier_id": frontier_id, "ttl_sec": ttl_sec, "reason": reason},
        )

    def navigation_failed(self, reason: str) -> None:
        frontier = self.active_frontier or {}
        frontier_id = str(frontier.get("frontier_id", ""))
        self.blacklist(
            frontier_id,
            float(self.get_parameter("failed_frontier_blacklist_ttl_sec").value),
            reason,
        )
        self.failure_count += 1
        self.waiting_nav = False
        self.active_nav_seq = None
        self.active_frontier = None
        self.state = "SELECT_FRONTIER"
        self.publish_state(f"navigation_failed:{reason}", force=True)

    def navigation_succeeded(self) -> None:
        frontier = self.active_frontier or {}
        frontier_id = str(frontier.get("frontier_id", ""))
        self.blacklist(
            frontier_id,
            float(self.get_parameter("reached_frontier_blacklist_ttl_sec").value),
            "frontier_reached",
        )
        reached_room = str(frontier.get("room_id", ""))
        self.waiting_nav = False
        self.active_nav_seq = None
        self.active_frontier = None
        self.post_goal_until = time.monotonic() + float(
            self.get_parameter("post_goal_settle_sec").value
        )
        self.state = "OBSERVE"
        self.publish_state("frontier_reached", force=True)
        self.maybe_label_room(reached_room)

    def stop_robot(self, reason: str) -> None:
        self.publish_json(
            self.nav_pub,
            {"action": "stop_robot", "reason": reason, "mission_id": self.mission_id},
        )

    def update_distance(self) -> None:
        pose = self.robot_pose()
        if pose is None:
            return
        if self.last_distance_pose is not None:
            step = math.hypot(
                pose[0] - self.last_distance_pose[0],
                pose[1] - self.last_distance_pose[1],
            )
            if 0.0 <= step <= 1.0:
                self.distance_travelled_m += step
        self.last_distance_pose = pose

    def frontier_alignment_deg(self, candidate: Dict[str, Any]) -> Optional[float]:
        pose = self.robot_pose()
        if pose is None:
            return None
        bearing = math.atan2(float(candidate["y"]) - pose[1], float(candidate["x"]) - pose[0])
        return abs(math.degrees(wrap_angle(bearing - pose[2])))

    def vision_cache_entry(self, frontier_id: str) -> Optional[Dict[str, Any]]:
        entry = self.frontier_vision_cache.get(frontier_id)
        if not isinstance(entry, dict):
            return None
        if float(entry.get("expires_mono", 0.0)) <= time.monotonic():
            self.frontier_vision_cache.pop(frontier_id, None)
            return None
        return entry

    def choose_frontier(self) -> Optional[Dict[str, Any]]:
        now = time.monotonic()
        expired = [key for key, until in self.local_blacklist.items() if until <= now]
        for key in expired:
            self.local_blacklist.pop(key, None)
        require_stable = as_bool(self.get_parameter("require_stable_frontier").value)
        min_confirmations = int(
            self.get_parameter("min_frontier_confirmations").value
        )
        guard_enabled = as_bool(
            self.get_parameter("enable_frontier_vision_guard").value
        )
        half_fov = float(self.get_parameter("frontier_camera_half_fov_deg").value)
        usable: List[Dict[str, Any]] = []
        saw_geometric_candidate = False
        for source in self.frontiers:
            candidate = dict(source)
            frontier_id = str(candidate.get("frontier_id", ""))
            if not frontier_id or frontier_id in self.local_blacklist:
                continue
            if not all(finite(candidate.get(key)) for key in ("x", "y", "yaw")):
                continue
            if require_stable and not bool(candidate.get("stable", False)):
                continue
            if int(candidate.get("view_confirmations", 0)) < min_confirmations:
                continue
            saw_geometric_candidate = True

            cached = self.vision_cache_entry(frontier_id)
            if cached is not None and cached.get("gate_decision") in {"reject", "unknown"}:
                continue

            alignment = self.frontier_alignment_deg(candidate)
            candidate["camera_alignment_deg"] = alignment
            if guard_enabled:
                if alignment is None or alignment > half_fov:
                    continue
            usable.append(candidate)
        if not usable:
            self.last_selection_reason = (
                "no_frontier_in_camera_fov"
                if guard_enabled and saw_geometric_candidate
                else "no_usable_frontier"
            )
            return None

        def utility(candidate: Dict[str, Any]) -> float:
            base_score = float(candidate.get("score", 0.0))
            info_gain = float(candidate.get("information_gain", candidate.get("info_gain", 0.0)))
            distance = float(candidate.get("distance_m", 0.0))
            confirmations = float(candidate.get("view_confirmations", 0.0))
            room_bonus = 0.5 if str(candidate.get("room_id", "")) == str(self.current_room_id) else 0.0
            alignment = float(candidate.get("camera_alignment_deg") or 0.0)
            alignment_bonus = max(0.0, 1.0 - alignment / max(1.0, half_fov)) if guard_enabled else 0.0
            cached = self.vision_cache_entry(str(candidate.get("frontier_id", "")))
            approved_bonus = 2.0 if cached and cached.get("gate_decision") == "approve" else 0.0
            return (
                base_score + 0.002 * info_gain + 0.25 * confirmations + room_bonus
                + 0.25 * alignment_bonus + approved_bonus - 0.10 * distance
            )

        selected = max(usable, key=utility)
        self.last_selection_reason = "frontier_selected"
        return selected

    def dispatch_frontier(self, frontier: Dict[str, Any]) -> None:
        command = {
            "action": "navigate_to_pose",
            "mission_id": self.mission_id,
            "semantic_action": "autonomous_explore_frontier",
            "frontier_id": frontier.get("frontier_id"),
            "room_id": frontier.get("room_id"),
            "pose": {
                "frame_id": str(self.get_parameter("map_frame").value),
                "x": float(frontier["x"]),
                "y": float(frontier["y"]),
                "yaw": float(frontier["yaw"]),
            },
        }
        visual = self.vision_cache_entry(str(frontier.get("frontier_id", "")))
        self.decision(
            {
                "decision_type": "autonomous_frontier_goal",
                "frontier": frontier,
                "visual_assessment": None if visual is None else visual.get("answer"),
                "visual_gate_decision": None if visual is None else visual.get("gate_decision"),
                "command": command,
                "dry_run": not self.motion_enabled,
            }
        )
        if not self.motion_enabled:
            self.running = False
            self.state = "PAUSED"
            self.publish_state("dry_run_frontier_proposal", force=True)
            return
        self.goal_count += 1
        self.active_frontier = dict(frontier)
        self.active_nav_seq = None
        self.waiting_nav = True
        self.state = "WAIT_NAV"
        self.publish_json(self.nav_pub, command)
        self.publish_state("frontier_goal_sent", force=True)

    def think_value(self) -> Any:
        value = str(self.get_parameter("ollama_think").value).strip().lower()
        if value in {"false", "off", "none", "0"}:
            return False
        if value in {"true", "on", "1"}:
            return True
        return value

    def image_base64(self) -> Optional[str]:
        if self.latest_image is None:
            return None
        if time.time() - self.latest_image_wall > float(
            self.get_parameter("image_max_age_sec").value
        ):
            return None
        try:
            image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
            quality = int(self.get_parameter("image_jpeg_quality").value)
            ok, encoded = cv2.imencode(
                ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality]
            )
            if not ok:
                return None
            return base64.b64encode(encoded.tobytes()).decode("ascii")
        except Exception:
            return None

    def frontier_vision_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "image_relevance": {
                    "type": "string",
                    "enum": [
                        "frontier_centered",
                        "frontier_partial",
                        "frontier_not_visible",
                        "uncertain",
                    ],
                },
                "path_surface": {
                    "type": "string",
                    "enum": ["clear", "partially_blocked", "blocked", "uncertain"],
                },
                "dropoff_or_stairs": {"type": "boolean"},
                "low_overhead_hazard": {"type": "boolean"},
                "dynamic_obstacle": {"type": "boolean"},
                "doorway_or_passage": {"type": "boolean"},
                "hazards": {"type": "array", "items": {"type": "string"}},
                "decision": {
                    "type": "string",
                    "enum": ["approve", "reject", "unknown"],
                },
                "confidence": {"type": "number"},
                "reason": {"type": "string"},
            },
            "required": [
                "image_relevance",
                "path_surface",
                "dropoff_or_stairs",
                "low_overhead_hazard",
                "dynamic_obstacle",
                "doorway_or_passage",
                "hazards",
                "decision",
                "confidence",
                "reason",
            ],
            "additionalProperties": False,
        }

    def frontier_visual_gate(self, answer: Dict[str, Any]) -> str:
        confidence = float(answer.get("confidence", 0.0))
        minimum = float(
            self.get_parameter("frontier_vision_min_confidence").value
        )
        hard_hazard = any(
            bool(answer.get(key, False))
            for key in (
                "dropoff_or_stairs",
                "low_overhead_hazard",
                "dynamic_obstacle",
            )
        )
        relevant = str(answer.get("image_relevance", "uncertain")) in {
            "frontier_centered",
            "frontier_partial",
        }
        clear = str(answer.get("path_surface", "uncertain")) == "clear"
        if (
            str(answer.get("decision", "unknown")) == "approve"
            and relevant
            and clear
            and not hard_hazard
            and confidence >= minimum
        ):
            return "approve"
        if hard_hazard or str(answer.get("decision", "unknown")) == "reject":
            return "reject"
        return "unknown"

    def request_frontier_vision(self, frontier: Dict[str, Any]) -> bool:
        if not as_bool(self.get_parameter("enable_frontier_vision_guard").value):
            return True
        frontier_id = str(frontier.get("frontier_id", ""))
        cached = self.vision_cache_entry(frontier_id)
        if cached is not None:
            return cached.get("gate_decision") == "approve"
        if self._llm_future is not None:
            return False

        image = self.image_base64()
        if image is None:
            self.pending_frontier = dict(frontier)
            self.state = "WAIT_FRONTIER_VISION"
            self.publish_state("frontier_vision_waiting_for_fresh_image", force=True)
            return False

        alignment = self.frontier_alignment_deg(frontier)
        half_fov = float(self.get_parameter("frontier_camera_half_fov_deg").value)
        if alignment is None or alignment > half_fov:
            self.last_selection_reason = "no_frontier_in_camera_fov"
            self.state = "WAIT_FRONTIER_VIEW"
            self.publish_state(self.last_selection_reason, force=True)
            return False

        schema = self.frontier_vision_schema()
        user = json.dumps(
            {
                "frontier_id": frontier_id,
                "frontier_distance_m": frontier.get("distance_m"),
                "frontier_information_gain": frontier.get(
                    "information_gain", frontier.get("info_gain")
                ),
                "camera_alignment_deg": round(float(alignment), 2),
                "instruction": (
                    "Assess only the visible floor and passage in the direction of this "
                    "frontier. Be conservative. Use unknown when the image does not show "
                    "enough of the proposed route."
                ),
            },
            sort_keys=True,
        )
        system = (
            "You are an advisory visual safety checker for an indoor quadruped robot. "
            "The image was captured from the robot before dispatching a geometric frontier. "
            "Approve only when the relevant route is visibly clear and traversable. Reject "
            "visible drop-offs, stairs, blocking people or objects, low overhead hazards, "
            "and clearly blocked passages. Return unknown when relevance or depth is unclear. "
            "Never output coordinates, velocities, or navigation commands."
        )
        self._request_seq += 1
        request_id = f"explore-{self.mission_id}-frontier-{self._request_seq}"
        self.pending_frontier = dict(frontier)
        self._llm_context = {
            "kind": "explore_frontier_safety",
            "frontier": dict(frontier),
            "frontier_id": frontier_id,
            "request_id": request_id,
            "image_wall": self.latest_image_wall,
        }
        self.publish_json(
            self.ollama_debug_pub,
            {
                "event": "request_queued",
                "request_id": request_id,
                "kind": "explore_frontier_safety",
                "model": self.ollama.model,
                "frontier_id": frontier_id,
                "camera_alignment_deg": round(float(alignment), 2),
                "image_count": 1,
            },
        )
        self._llm_future = self._llm_executor.submit(
            self.ollama.chat,
            system,
            user,
            schema,
            [image],
            self.think_value(),
            request_id,
            "explore_frontier_safety",
        )
        self.state = "WAIT_FRONTIER_VISION"
        self.publish_state("ollama_frontier_safety", force=True)
        return False

    def maybe_label_room(self, room_id: str) -> None:
        if not as_bool(self.get_parameter("enable_room_labeling").value):
            return
        if self._llm_future is not None or not room_id:
            return
        room = next(
            (item for item in self.rooms if str(item.get("room_id")) == room_id),
            None,
        )
        if room is None or str(room.get("label", "unknown")) not in {"", "unknown"}:
            return
        image = self.image_base64()
        if image is None:
            self.decision(
                {
                    "decision_type": "room_classification_skipped",
                    "room_id": room_id,
                    "reason": "fresh_image_unavailable",
                }
            )
            return
        room_types = [str(value) for value in self.get_parameter("room_types").value]
        schema = {
            "type": "object",
            "properties": {
                "room_type": {"type": "string", "enum": room_types},
                "reason": {"type": "string"},
            },
            "required": ["room_type", "reason"],
            "additionalProperties": False,
        }
        known_objects = [
            str(item.get("label", ""))
            for item in self.objects
            if str(item.get("room_id", "")) == room_id
            and bool(item.get("confirmed", False))
        ]
        user = json.dumps(
            {
                "room_id": room_id,
                "area_m2": room.get("area_m2"),
                "known_objects": known_objects,
                "allowed_room_types": room_types,
            }
        )
        system = (
            "Classify the current indoor room from the RGB image and supplied map context. "
            "Choose exactly one allowed room type. Semantic output is advisory and must not "
            "contain coordinates, navigation actions, or velocities."
        )
        self._request_seq += 1
        request_id = f"explore-{self.mission_id}-room-{self._request_seq}"
        self._llm_context = {
            "kind": "explore_classify_room",
            "room_id": room_id,
            "request_id": request_id,
        }
        self.publish_json(
            self.ollama_debug_pub,
            {
                "event": "request_queued",
                "request_id": request_id,
                "kind": "explore_classify_room",
                "model": self.ollama.model,
                "image_count": 1,
            },
        )
        self._llm_future = self._llm_executor.submit(
            self.ollama.chat,
            system,
            user,
            schema,
            [image],
            self.think_value(),
            request_id,
            "explore_classify_room",
        )
        self.state = "SEMANTIC_OBSERVE"
        self.publish_state("ollama_room_classification", force=True)

    def poll_llm(self) -> None:
        if self._llm_future is None or not self._llm_future.done():
            return
        future = self._llm_future
        context = dict(self._llm_context)
        self._llm_future = None
        self._llm_context = {}
        kind = str(context.get("kind", "unknown"))
        try:
            result = future.result()
            answer = result.get("answer", result)
            meta = result.get("meta", {})
            self.publish_json(
                self.ollama_debug_pub,
                {
                    "event": "response",
                    "request_id": context.get("request_id"),
                    "kind": kind,
                    "answer": answer,
                    "meta": meta,
                },
            )

            if kind == "explore_frontier_safety":
                frontier_id = str(context.get("frontier_id", ""))
                gate = self.frontier_visual_gate(answer)
                ttl_name = (
                    "frontier_vision_cache_sec"
                    if gate == "approve"
                    else "frontier_vision_reject_ttl_sec"
                    if gate == "reject"
                    else "frontier_vision_unknown_ttl_sec"
                )
                entry = {
                    "gate_decision": gate,
                    "answer": answer,
                    "meta": meta,
                    "request_id": context.get("request_id"),
                    "image_wall": context.get("image_wall"),
                    "expires_mono": time.monotonic()
                    + float(self.get_parameter(ttl_name).value),
                }
                self.frontier_vision_cache[frontier_id] = entry
                if gate == "reject":
                    self.local_blacklist[frontier_id] = time.monotonic() + float(
                        self.get_parameter("frontier_vision_reject_ttl_sec").value
                    )
                self.decision(
                    {
                        "decision_type": "frontier_visual_assessment",
                        "frontier_id": frontier_id,
                        "frontier": context.get("frontier"),
                        "visual_gate_decision": gate,
                        "answer": answer,
                        "meta": meta,
                    }
                )
                self.pending_frontier = None
                if self.running:
                    self.state = "SELECT_FRONTIER"
                    self.publish_state(
                        f"frontier_visual_assessment:{gate}", force=True
                    )
                return

            label = str(answer.get("room_type", "unknown"))
            room_id = str(context.get("room_id", ""))
            if room_id:
                self.publish_json(
                    self.room_label_pub, {"room_id": room_id, "label": label}
                )
            self.decision(
                {
                    "decision_type": "room_classified",
                    "room_id": room_id,
                    "label": label,
                    "reason": answer.get("reason", ""),
                }
            )
        except Exception as exc:
            self.publish_json(
                self.ollama_debug_pub,
                {
                    "event": "error",
                    "request_id": context.get("request_id"),
                    "kind": kind,
                    "error": str(exc),
                },
            )
            if kind == "explore_frontier_safety":
                frontier_id = str(context.get("frontier_id", ""))
                self.frontier_vision_cache[frontier_id] = {
                    "gate_decision": "unknown",
                    "answer": {"reason": f"ollama_error:{exc}"},
                    "request_id": context.get("request_id"),
                    "expires_mono": time.monotonic()
                    + float(
                        self.get_parameter("frontier_vision_unknown_ttl_sec").value
                    ),
                }
                self.pending_frontier = None
                self.decision(
                    {
                        "decision_type": "frontier_visual_assessment_failed",
                        "frontier_id": frontier_id,
                        "error": str(exc),
                    }
                )
            else:
                self.decision(
                    {
                        "decision_type": "room_classification_failed",
                        "room_id": context.get("room_id"),
                        "error": str(exc),
                    }
                )
        if self.running:
            self.state = "SELECT_FRONTIER"
            self.publish_state("semantic_observation_complete", force=True)

    def finish(self, reason: str, failed: bool = False) -> None:
        self.stop_robot(reason)
        self.running = False
        self.waiting_nav = False
        self.state = "FAILED" if failed else "DONE"
        self.publish_state(reason, force=True)

    def tick(self) -> None:
        self.poll_llm()
        self.update_distance()
        self.publish_state()
        if not self.running:
            return

        elapsed = time.monotonic() - self.started_mono
        if elapsed > float(self.get_parameter("max_mission_duration_sec").value):
            self.finish("mission_time_budget_reached")
            return
        if self.goal_count >= int(self.get_parameter("max_goals").value):
            self.finish("goal_budget_reached")
            return
        if self.failure_count >= int(self.get_parameter("max_failures").value):
            self.finish("failure_budget_reached", failed=True)
            return
        if self.distance_travelled_m >= float(
            self.get_parameter("max_travel_distance_m").value
        ):
            self.finish("distance_budget_reached")
            return

        health_ok, health = self.health_snapshot()
        now = time.monotonic()
        if not health_ok:
            if self.health_bad_since is None:
                self.health_bad_since = now
            self.state = "WAIT_HEALTH"
            self.publish_state("health_gate:" + ",".join(health["reasons"]), force=True)
            if self.waiting_nav and now - self.health_bad_since >= float(
                self.get_parameter("health_loss_stop_sec").value
            ):
                self.stop_robot("runtime_health_lost")
                self.waiting_nav = False
                self.running = False
                self.state = "SAFETY_STOPPED"
                self.publish_state("runtime_health_lost", force=True)
            return
        self.health_bad_since = None

        if self._llm_future is not None or self.waiting_nav:
            return
        if now < self.post_goal_until:
            self.state = "OBSERVE"
            return

        candidate = self.choose_frontier()
        if candidate is None:
            planner_reason = str(
                self.frontier_planner_status.get("reason", "unknown")
            )
            planner_not_ready = {
                "not_received",
                "not_computed",
                "computing",
                "invalid_map",
                "tf_missing",
                "robot_outside_raw_map",
                "no_traversable_start",
            }
            if planner_reason in planner_not_ready:
                self.last_no_candidate_since = None
                self.state = "WAIT_FRONTIERS"
                self.publish_state(
                    f"planner_not_ready:{planner_reason}", force=True
                )
                return

            if self.last_selection_reason == "no_frontier_in_camera_fov":
                self.last_no_candidate_since = None
                self.state = "WAIT_FRONTIER_VIEW"
                self.publish_state(self.last_selection_reason, force=True)
                return

            if self.last_no_candidate_since is None:
                self.last_no_candidate_since = now
            self.state = "WAIT_FRONTIERS"
            self.publish_state(
                f"waiting_for_frontier:{planner_reason}", force=True
            )
            if now - self.last_no_candidate_since >= float(
                self.get_parameter("no_frontier_complete_sec").value
            ):
                self.finish(f"no_safe_frontiers_remaining:{planner_reason}")
            return

        self.last_no_candidate_since = None
        self.state = "SELECT_FRONTIER"
        if not self.request_frontier_vision(candidate):
            return
        self.dispatch_frontier(candidate)

    def destroy_node(self) -> bool:
        try:
            self.stop_robot("supervisor_shutdown")
            if self._llm_future is not None:
                self._llm_future.cancel()
            self._llm_executor.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomousExploreSupervisor()
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
