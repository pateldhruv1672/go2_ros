from __future__ import annotations

import json
import math
import os
import re
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any, Optional

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


def clean_label(text: str) -> str:
    value = (text or "").lower().strip()
    value = re.sub(r"[^a-z0-9\s_]", " ", value)
    value = re.sub(r"\s+", " ", value)
    return value.strip().replace(" ", "_")


def label_match(target: str, label: str) -> bool:
    t = clean_label(target)
    l = clean_label(label)
    if not t or not l:
        return False
    if t == l:
        return True
    if t.endswith("s") and t[:-1] == l:
        return True
    if l.endswith("s") and l[:-1] == t:
        return True
    return t in l or l in t


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class ObjectObservation:
    label: str
    confidence: float
    map_x: float
    map_y: float
    range_m: float
    bearing_rad: float
    seen_count: int
    last_seen: float
    source: str = "yolov8_scan_tf"


@dataclass
class Frontier:
    frontier_id: int
    map_x: float
    map_y: float
    cell_count: int
    distance_m: float
    score: float


class OptionalSam2Refiner:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.enabled = bool(node.get_parameter("enable_sam2").value)
        self.predictor = None

        if not self.enabled:
            return

        try:
            from sam2.sam2_image_predictor import SAM2ImagePredictor

            hf_model = str(node.get_parameter("sam2_hf_model").value).strip()
            if hf_model:
                self.predictor = SAM2ImagePredictor.from_pretrained(hf_model)
                node.get_logger().info(f"SAM2 enabled from HF model={hf_model}")
                return

            from sam2.build_sam import build_sam2

            cfg = str(node.get_parameter("sam2_model_cfg").value).strip()
            ckpt = os.path.expanduser(str(node.get_parameter("sam2_checkpoint").value).strip())

            if cfg and os.path.isfile(ckpt):
                self.predictor = SAM2ImagePredictor(build_sam2(cfg, ckpt))
                node.get_logger().info(f"SAM2 enabled checkpoint={ckpt}")
            else:
                node.get_logger().warn("SAM2 requested but checkpoint/cfg invalid. Running YOLO-only.")

        except Exception as exc:
            node.get_logger().warn(f"SAM2 disabled: {type(exc).__name__}: {exc}")
            self.predictor = None

    def refine_box(self, image_rgb: np.ndarray, box_xyxy: list[float]) -> dict[str, Any]:
        if self.predictor is None:
            return {"sam2_used": False}

        try:
            import torch

            box = np.array(box_xyxy, dtype=np.float32)
            with torch.inference_mode():
                self.predictor.set_image(image_rgb)
                masks, scores, _ = self.predictor.predict(box=box, multimask_output=False)

            mask = masks[0].astype(bool)
            ys, xs = np.where(mask)
            if len(xs) == 0:
                return {"sam2_used": True, "mask_area": 0}

            return {
                "sam2_used": True,
                "mask_area": int(mask.sum()),
                "mask_cx": float(xs.mean()),
                "mask_cy": float(ys.mean()),
                "sam2_score": float(scores[0]) if len(scores) else 0.0,
            }

        except Exception as exc:
            return {"sam2_used": False, "sam2_error": f"{type(exc).__name__}: {exc}"}


class FrontierObjectExplorerNode(Node):
    def __init__(self) -> None:
        super().__init__("frontier_object_explorer_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/object_explorer/goal")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_out")
        self.declare_parameter("tts_topic", "/go2_tts/say")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_yaw_offset_rad", 0.0)
        self.declare_parameter("fallback_hfov_deg", 70.0)

        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("yolo_conf", 0.35)
        self.declare_parameter("yolo_imgsz", 640)
        self.declare_parameter("detect_period_sec", 0.55)

        self.declare_parameter("enable_sam2", False)
        self.declare_parameter("sam2_hf_model", "")
        self.declare_parameter("sam2_checkpoint", "~/Dhruv/sparky/models/sam2/sam2.1_hiera_tiny.pt")
        self.declare_parameter("sam2_model_cfg", "configs/sam2.1/sam2.1_hiera_t.yaml")

        self.declare_parameter("memory_path", "~/.ros/go2_object_explorer/object_memory.yaml")
        self.declare_parameter("merge_radius_m", 0.65)
        self.declare_parameter("max_object_age_sec", 86400.0)

        self.declare_parameter("auto_navigate", True)
        self.declare_parameter("approach_distance_m", 0.85)

        self.declare_parameter("local_scan_duration_sec", 8.0)
        self.declare_parameter("search_timeout_sec", 180.0)
        self.declare_parameter("search_spin_speed", 0.26)

        self.declare_parameter("frontier_min_cluster_cells", 8)
        self.declare_parameter("frontier_blacklist_radius_m", 0.65)
        self.declare_parameter("frontier_goal_timeout_sec", 45.0)
        self.declare_parameter("frontier_min_distance_m", 0.75)
        self.declare_parameter("frontier_max_distance_m", 8.0)

        self.declare_parameter("frontier_distance_weight", 1.0)
        self.declare_parameter("frontier_size_weight", 0.08)
        self.declare_parameter("frontier_heading_weight", 0.35)

        # Camera + map frontier fusion.
        # fallback_and: prefer map_frontier AND camera_condition, but fallback to map-only if none exist.
        # strict_and: always require map_frontier AND camera_condition.
        # soft_score: allow all map frontiers, but boost camera-qualified frontiers.
        self.declare_parameter("frontier_camera_map_mode", "fallback_and")
        self.declare_parameter("visual_frontier_fov_deg", 80.0)
        self.declare_parameter("visual_frontier_max_range_m", 9.0)
        self.declare_parameter("visual_sector_memory_sec", 70.0)
        self.declare_parameter("visual_sector_sample_period_sec", 0.75)
        self.declare_parameter("visual_semantic_radius_m", 2.0)
        self.declare_parameter("visual_semantic_memory_sec", 90.0)
        self.declare_parameter("visual_frontier_bonus", 3.0)
        self.declare_parameter("visual_los_block_occupied", True)

        # LLM frontier selector. The LLM is only allowed to choose among
        # validated frontier candidates. It never directly publishes cmd_vel.
        self.declare_parameter("use_ollama_frontier_selector", False)
        self.declare_parameter("ollama_url", "http://127.0.0.1:11434")
        self.declare_parameter("ollama_model", "gemma3:4b")
        self.declare_parameter("ollama_timeout_sec", 4.0)
        self.declare_parameter("llm_top_k_frontiers", 8)
        self.declare_parameter("llm_decision_topic", "/object_explorer/llm_decision")
        self.declare_parameter("min_nav_goal_separation_sec", 2.0)

        self.bridge = CvBridge()
        self.latest_image_bgr: Optional[np.ndarray] = None
        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_map: Optional[OccupancyGrid] = None
        self.image_lock = threading.Lock()

        self.active_target = ""
        self.state = "IDLE"
        self.state_started_at = time.time()
        self.mission_started_at = 0.0
        self.navigation_started_for_target = False
        self.current_goal_handle = None
        self.current_frontier: Optional[Frontier] = None
        self.nav_goal_seq = 0
        self.active_nav_goal_seq = -1
        self.active_nav_kind = ""
        self.last_nav_goal_sent_at = 0.0
        self.blacklisted_frontiers: list[tuple[float, float]] = []

        self.memory: dict[str, list[ObjectObservation]] = {}
        self.frontiers: list[Frontier] = []

        # Camera-view memory. Each sector is robot pose when a camera frame was seen.
        # This lets frontier planning use map_frontier AND camera-observed direction.
        self.visual_sectors: list[tuple[float, float, float, float]] = []
        self.visual_context: list[dict[str, Any]] = []
        self.latest_image_seen_at = 0.0
        self.last_visual_sector_at = 0.0

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self.tts_pub = self.create_publisher(
            String,
            str(self.get_parameter("tts_topic").value),
            10,
        )
        self.detections_pub = self.create_publisher(String, "/object_explorer/detections", 10)
        self.memory_pub = self.create_publisher(String, "/object_explorer/memory", 10)
        self.state_pub = self.create_publisher(String, "/object_explorer/state", 10)
        self.frontier_pub = self.create_publisher(String, "/object_explorer/frontiers", 10)
        self.llm_decision_pub = self.create_publisher(
            String,
            str(self.get_parameter("llm_decision_topic").value),
            10,
        )
        self.marker_pub = self.create_publisher(MarkerArray, "/object_explorer/markers", 10)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.on_camera_info,
            sensor_qos,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.on_scan,
            sensor_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.on_map,
            map_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("goal_topic").value),
            self.on_goal,
            10,
        )

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.model = None
        self.sam2 = None

        self.load_memory()
        self.load_detector()

        self.create_timer(float(self.get_parameter("detect_period_sec").value), self.detect_once)
        self.create_timer(0.25, self.tick)
        self.create_timer(1.0, self.publish_state_memory_markers)

        self.say("Frontier object explorer is ready.")
        self.get_logger().info("Frontier ObjectNav explorer ready")

    def load_detector(self) -> None:
        from ultralytics import YOLO

        model_name = str(self.get_parameter("yolo_model").value)
        self.model = YOLO(model_name)
        self.sam2 = OptionalSam2Refiner(self)
        self.get_logger().info(f"YOLO loaded model={model_name}")

    def say(self, text: str) -> None:
        payload = {"text": text, "category": "object_explorer", "priority": "normal"}
        self.tts_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def set_state(self, state: str, reason: str = "") -> None:
        if self.state != state:
            self.get_logger().info(f"state {self.state} -> {state} reason={reason}")
        self.state = state
        self.state_started_at = time.time()
        self.publish_state(reason=reason)

    def publish_state(self, reason: str = "") -> None:
        payload = {
            "state": self.state,
            "target": self.active_target,
            "reason": reason,
            "mission_age_sec": time.time() - self.mission_started_at if self.mission_started_at else 0.0,
            "frontier": asdict(self.current_frontier) if self.current_frontier else None,
        }
        self.state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def on_image(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.image_lock:
                self.latest_image_bgr = bgr
            self.latest_image_seen_at = time.time()
            self.record_visual_sector()
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def parse_goal_text(self, raw: str) -> str:
        raw = (raw or "").strip()
        if not raw:
            return ""

        if raw.startswith("{"):
            try:
                payload = json.loads(raw)
                for key in ("target", "object", "label", "query", "text"):
                    value = payload.get(key)
                    if isinstance(value, str) and value.strip():
                        raw = value
                        break
            except Exception:
                pass

        text = raw.lower()
        text = re.sub(r"[^a-z0-9\s_]", " ", text)
        text = re.sub(r"\s+", " ", text).strip()

        patterns = [
            r"^explore\s+and\s+find\s+(?:a|an|the)?\s*(.+)$",
            r"^find\s+(?:a|an|the)?\s*(.+)$",
            r"^locate\s+(?:a|an|the)?\s*(.+)$",
            r"^search\s+for\s+(?:a|an|the)?\s*(.+)$",
            r"^where\s+is\s+(?:a|an|the)?\s*(.+)$",
        ]

        for pattern in patterns:
            m = re.match(pattern, text)
            if m:
                return clean_label(m.group(1))

        return clean_label(text)

    def on_goal(self, msg: String) -> None:
        target = self.parse_goal_text(msg.data)
        if not target:
            self.say("I did not understand which object to find.")
            return

        self.cancel_nav_goal()
        self.stop_robot()

        self.active_target = target
        self.mission_started_at = time.time()
        self.navigation_started_for_target = False
        self.current_frontier = None
        self.blacklisted_frontiers = []

        remembered = self.best_match(target)
        if remembered is not None:
            self.say(f"I remember a {remembered.label.replace('_', ' ')}. Navigating near it.")
            self.navigate_to_object(remembered)
            return

        self.say(f"I will explore and search for {target.replace('_', ' ')}.")
        self.set_state("LOCAL_SCAN", "new_object_goal")

    def detect_once(self) -> None:
        if self.model is None:
            return

        with self.image_lock:
            if self.latest_image_bgr is None:
                return
            image_bgr = self.latest_image_bgr.copy()

        try:
            conf = float(self.get_parameter("yolo_conf").value)
            imgsz = int(self.get_parameter("yolo_imgsz").value)
            results = self.model.predict(image_bgr, conf=conf, imgsz=imgsz, verbose=False)
        except Exception as exc:
            self.get_logger().warn(f"YOLO predict failed: {exc}")
            return

        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        detections = []

        for result in results:
            names = result.names or {}
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue

            for box in boxes:
                try:
                    xyxy = box.xyxy[0].detach().cpu().numpy().astype(float).tolist()
                    cls_id = int(box.cls[0].detach().cpu().item())
                    label = clean_label(str(names.get(cls_id, cls_id)))
                    score = float(box.conf[0].detach().cpu().item())
                except Exception:
                    continue

                bearing = self.bbox_to_bearing(xyxy, image_bgr.shape[1])
                rng = self.range_from_scan(bearing)
                pose = self.project_to_map(bearing, rng) if rng is not None else None

                sam_info = {}
                if self.sam2 is not None:
                    sam_info = self.sam2.refine_box(image_rgb, xyxy)

                det = {
                    "label": label,
                    "confidence": score,
                    "bbox_xyxy": [round(float(x), 2) for x in xyxy],
                    "bearing_rad": bearing,
                    "range_m": rng,
                    "map_pose": pose,
                    "stamp": time.time(),
                    **sam_info,
                }
                detections.append(det)

                if pose is not None:
                    obs = ObjectObservation(
                        label=label,
                        confidence=score,
                        map_x=float(pose["x"]),
                        map_y=float(pose["y"]),
                        range_m=float(rng),
                        bearing_rad=float(bearing),
                        seen_count=1,
                        last_seen=time.time(),
                    )
                    self.merge_observation(obs)
                    self.add_visual_context(label, float(pose["x"]), float(pose["y"]), score)

                    if self.active_target and label_match(self.active_target, label):
                        self.on_target_found(obs)

        if detections:
            self.detections_pub.publish(String(data=json.dumps(detections, sort_keys=True)))

    def on_target_found(self, obs: ObjectObservation) -> None:
        if self.navigation_started_for_target:
            return

        self.navigation_started_for_target = True
        self.cancel_nav_goal()
        self.stop_robot()
        self.say(f"I found {obs.label.replace('_', ' ')}.")
        self.navigate_to_object(obs)

    def bbox_to_bearing(self, xyxy: list[float], image_width: int) -> float:
        cx = 0.5 * (xyxy[0] + xyxy[2])

        if self.latest_camera_info is not None and self.latest_camera_info.k[0] > 1.0:
            fx = float(self.latest_camera_info.k[0])
            c_x = float(self.latest_camera_info.k[2])
            bearing = math.atan2(cx - c_x, fx)
        else:
            hfov = math.radians(float(self.get_parameter("fallback_hfov_deg").value))
            bearing = ((cx / max(1.0, image_width)) - 0.5) * hfov

        return bearing + float(self.get_parameter("camera_yaw_offset_rad").value)

    def range_from_scan(self, bearing: float) -> Optional[float]:
        scan = self.latest_scan
        if scan is None:
            return None

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        if len(ranges) == 0:
            return None

        idx = int(round((bearing - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(ranges):
            return None

        win = max(2, int(math.radians(4.0) / max(1e-6, abs(scan.angle_increment))))
        lo = max(0, idx - win)
        hi = min(len(ranges), idx + win + 1)

        vals = ranges[lo:hi]
        vals = vals[np.isfinite(vals)]
        vals = vals[(vals >= scan.range_min) & (vals <= scan.range_max)]
        if len(vals) == 0:
            return None

        return float(np.nanmedian(vals))

    def current_base_pose(self) -> Optional[tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                Time(),
                timeout=Duration(seconds=0.25),
            )
            x = float(tf.transform.translation.x)
            y = float(tf.transform.translation.y)
            yaw = quat_to_yaw(tf.transform.rotation)
            return x, y, yaw
        except Exception:
            return None

    def project_to_map(self, bearing: float, rng: Optional[float]) -> Optional[dict[str, float]]:
        if rng is None:
            return None

        base = self.current_base_pose()
        if base is None:
            return None

        bx, by, byaw = base
        theta = byaw + bearing
        ox = bx + rng * math.cos(theta)
        oy = by + rng * math.sin(theta)
        return {"x": ox, "y": oy, "yaw": theta}

    def tick(self) -> None:
        self.publish_state()

        if not self.active_target:
            return

        if self.mission_started_at and time.time() - self.mission_started_at > float(self.get_parameter("search_timeout_sec").value):
            self.cancel_nav_goal()
            self.stop_robot()
            self.say(f"I could not find {self.active_target.replace('_', ' ')} before timeout.")
            self.active_target = ""
            self.set_state("IDLE", "mission_timeout")
            return

        if self.state == "LOCAL_SCAN":
            self.local_scan_tick()
            return

        if self.state == "PLANNING_FRONTIER":
            self.plan_next_frontier()
            return

        if self.state == "NAV_TO_FRONTIER":
            if time.time() - self.state_started_at > float(self.get_parameter("frontier_goal_timeout_sec").value):
                self.blacklist_current_frontier()
                self.cancel_nav_goal()
                self.stop_robot()
                self.set_state("PLANNING_FRONTIER", "frontier_timeout")
            return

    def local_scan_tick(self) -> None:
        if time.time() - self.state_started_at <= float(self.get_parameter("local_scan_duration_sec").value):
            twist = Twist()
            twist.angular.z = float(self.get_parameter("search_spin_speed").value)
            self.cmd_pub.publish(twist)
            return

        self.stop_robot()
        self.set_state("PLANNING_FRONTIER", "local_scan_done")

    def plan_next_frontier(self) -> None:
        pose = self.current_base_pose()
        if pose is None:
            self.say("I am waiting for map localization from SLAM.")
            return

        frontiers = self.compute_frontiers(pose)
        self.frontiers = frontiers

        self.frontier_pub.publish(String(data=json.dumps([asdict(f) for f in frontiers], sort_keys=True)))

        if not frontiers:
            self.say("I do not see any useful frontiers yet. I will scan again.")
            self.set_state("LOCAL_SCAN", "no_frontiers")
            return

        chosen = frontiers[0]
        selector = "heuristic"

        if bool(self.get_parameter("use_ollama_frontier_selector").value):
            llm_choice = self.ollama_select_frontier(frontiers, pose)
            if llm_choice is not None:
                chosen = llm_choice
                selector = "ollama_tool_call"
            else:
                selector = "heuristic_fallback"

        self.current_frontier = chosen

        self.llm_decision_pub.publish(
            String(
                data=json.dumps(
                    {
                        "event": "frontier_selected",
                        "selector": selector,
                        "target": self.active_target,
                        "chosen_frontier": asdict(chosen),
                        "candidate_count": len(frontiers),
                    },
                    sort_keys=True,
                )
            )
        )

        self.navigate_to_frontier(chosen)

    def ollama_select_frontier(
        self,
        frontiers: list[Frontier],
        pose: tuple[float, float, float],
    ) -> Optional[Frontier]:
        top_k = int(self.get_parameter("llm_top_k_frontiers").value)
        candidates = frontiers[:max(1, top_k)]

        bx, by, byaw = pose

        tool_schema = {
            "allowed_tools": [
                {
                    "name": "navigate_to_frontier",
                    "description": "Choose one validated frontier waypoint by frontier_id. The robot will navigate there using Nav2.",
                    "arguments": {
                        "frontier_id": "integer id from candidates",
                        "reason": "brief reason",
                    },
                },
                {
                    "name": "scan_here",
                    "description": "Do another local scan instead of navigating when all frontiers are unsafe or irrelevant.",
                    "arguments": {
                        "reason": "brief reason",
                    },
                },
            ]
        }

        prompt = {
            "task": "Choose the next exploration waypoint for a Unitree Go2 object-search mission.",
            "target_object": self.active_target,
            "robot_pose": {"x": bx, "y": by, "yaw": byaw},
            "policy": [
                "Prefer frontiers that are close enough to reach safely.",
                "Prefer larger frontier clusters because they may reveal more new space.",
                "Prefer camera-supported/semantically useful frontiers when scores are similar.",
                "Never invent coordinates.",
                "Only select a frontier_id from the provided candidates.",
                "Return JSON only.",
            ],
            "tools": tool_schema,
            "candidate_frontiers": [asdict(f) for f in candidates],
            "required_response_format": {
                "tool": "navigate_to_frontier",
                "frontier_id": 0,
                "reason": "why this frontier is best",
            },
        }

        messages = [
            {
                "role": "system",
                "content": (
                    "You are the high-level exploration planner for a quadruped robot. "
                    "You must choose a safe tool call from the allowed tools. "
                    "You are not allowed to directly control velocity. "
                    "Return strict JSON only."
                ),
            },
            {
                "role": "user",
                "content": json.dumps(prompt, sort_keys=True),
            },
        ]

        try:
            response = self.call_ollama_json(messages)
        except Exception as exc:
            self.get_logger().warn(f"Ollama frontier selection failed: {type(exc).__name__}: {exc}")
            self.llm_decision_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "event": "ollama_failed",
                            "error": f"{type(exc).__name__}: {exc}",
                            "fallback": "heuristic",
                        },
                        sort_keys=True,
                    )
                )
            )
            return None

        if not isinstance(response, dict):
            self.get_logger().warn(f"Ollama returned non-dict response: {response}")
            return None

        tool = str(response.get("tool", "")).strip()

        if tool == "scan_here":
            self.llm_decision_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "event": "ollama_tool_call",
                            "tool": "scan_here",
                            "reason": str(response.get("reason", "")),
                        },
                        sort_keys=True,
                    )
                )
            )
            return None

        if tool != "navigate_to_frontier":
            self.get_logger().warn(f"Ollama returned unsupported tool={tool}; using heuristic")
            return None

        try:
            frontier_id = int(response.get("frontier_id"))
        except Exception:
            self.get_logger().warn(f"Ollama returned invalid frontier_id: {response}")
            return None

        valid = {f.frontier_id: f for f in candidates}
        if frontier_id not in valid:
            self.get_logger().warn(f"Ollama selected frontier_id={frontier_id}, not in candidates; using heuristic")
            return None

        chosen = valid[frontier_id]

        self.llm_decision_pub.publish(
            String(
                data=json.dumps(
                    {
                        "event": "ollama_tool_call",
                        "tool": "navigate_to_frontier",
                        "frontier_id": frontier_id,
                        "chosen_frontier": asdict(chosen),
                        "reason": str(response.get("reason", "")),
                        "raw_response": response,
                    },
                    sort_keys=True,
                )
            )
        )

        return chosen

    def call_ollama_json(self, messages: list[dict[str, str]]) -> dict[str, Any]:
        base_url = str(self.get_parameter("ollama_url").value).rstrip("/")
        model = str(self.get_parameter("ollama_model").value)
        timeout = float(self.get_parameter("ollama_timeout_sec").value)

        payload = {
            "model": model,
            "messages": messages,
            "stream": False,
            "format": "json",
            "options": {
                "temperature": 0.1,
                "num_predict": 256,
            },
        }

        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            base_url + "/api/chat",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8")

        outer = json.loads(raw)
        content = outer.get("message", {}).get("content", "")
        if not content:
            raise RuntimeError(f"empty Ollama response: {outer}")

        try:
            return json.loads(content)
        except json.JSONDecodeError:
            # Some models wrap JSON in text despite format=json.
            m = re.search(r"\{.*\}", content, flags=re.DOTALL)
            if not m:
                raise
            return json.loads(m.group(0))


    def _get_float_param(self, name: str, default: float) -> float:
        try:
            if self.has_parameter(name):
                return float(self.get_parameter(name).value)
        except Exception:
            pass
        return float(default)

    def _get_bool_param(self, name: str, default: bool) -> bool:
        try:
            if self.has_parameter(name):
                return bool(self.get_parameter(name).value)
        except Exception:
            pass
        return bool(default)

    def record_visual_sector(self) -> None:
        now = time.time()
        period = self._get_float_param("visual_sector_sample_period_sec", 0.75)
        if now - getattr(self, "last_visual_sector_at", 0.0) < period:
            return

        pose = self.current_base_pose()
        if pose is None:
            return

        if not hasattr(self, "visual_sectors"):
            self.visual_sectors = []

        bx, by, byaw = pose
        self.visual_sectors.append((float(bx), float(by), float(byaw), now))
        self.last_visual_sector_at = now
        self.prune_visual_memory()

    def add_visual_context(self, label: str, x: float, y: float, confidence: float) -> None:
        if not hasattr(self, "visual_context"):
            self.visual_context = []

        self.visual_context.append(
            {
                "label": clean_label(label),
                "x": float(x),
                "y": float(y),
                "confidence": float(confidence),
                "stamp": time.time(),
            }
        )
        self.prune_visual_memory()

    def prune_visual_memory(self) -> None:
        now = time.time()
        sector_age = self._get_float_param("visual_sector_memory_sec", 70.0)
        semantic_age = self._get_float_param("visual_semantic_memory_sec", 90.0)

        if not hasattr(self, "visual_sectors"):
            self.visual_sectors = []
        if not hasattr(self, "visual_context"):
            self.visual_context = []

        self.visual_sectors = [
            item for item in self.visual_sectors
            if now - float(item[3]) <= sector_age
        ]

        self.visual_context = [
            item for item in self.visual_context
            if now - float(item.get("stamp", 0.0)) <= semantic_age
        ]

    def angle_error(self, a: float, b: float) -> float:
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    def world_to_map_cell(self, grid: OccupancyGrid, wx: float, wy: float) -> Optional[tuple[int, int]]:
        res = float(grid.info.resolution)
        if res <= 0.0:
            return None

        ox = float(grid.info.origin.position.x)
        oy = float(grid.info.origin.position.y)

        cx = int((wx - ox) / res)
        cy = int((wy - oy) / res)

        if cx < 0 or cy < 0 or cx >= int(grid.info.width) or cy >= int(grid.info.height):
            return None

        return cx, cy

    def line_cells(self, x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
        cells: list[tuple[int, int]] = []

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        x = x0
        y = y0

        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

        return cells

    def map_line_is_camera_traversable(
        self,
        grid: OccupancyGrid,
        data: np.ndarray,
        sx: float,
        sy: float,
        gx: float,
        gy: float,
    ) -> bool:
        if not self._get_bool_param("visual_los_block_occupied", True):
            return True

        start_cell = self.world_to_map_cell(grid, sx, sy)
        goal_cell = self.world_to_map_cell(grid, gx, gy)
        if start_cell is None or goal_cell is None:
            return False

        width = int(grid.info.width)
        height = int(grid.info.height)

        cells = self.line_cells(start_cell[0], start_cell[1], goal_cell[0], goal_cell[1])

        if len(cells) > 6:
            cells = cells[2:-2]

        for cx, cy in cells:
            if cx < 0 or cy < 0 or cx >= width or cy >= height:
                continue

            value = int(data[cy, cx])
            if value >= 65:
                return False

        return True

    def frontier_seen_by_camera_sector(
        self,
        grid: OccupancyGrid,
        data: np.ndarray,
        wx: float,
        wy: float,
    ) -> tuple[bool, float]:
        self.prune_visual_memory()

        if not getattr(self, "visual_sectors", []):
            return False, 0.0

        fov = math.radians(self._get_float_param("visual_frontier_fov_deg", 80.0))
        max_range = self._get_float_param("visual_frontier_max_range_m", 9.0)

        best_score = 0.0

        for sx, sy, syaw, stamp in self.visual_sectors:
            dx = wx - sx
            dy = wy - sy
            dist = math.hypot(dx, dy)

            if dist <= 0.05 or dist > max_range:
                continue

            bearing = math.atan2(dy, dx)
            heading_err = self.angle_error(bearing, syaw)

            if heading_err > 0.5 * fov:
                continue

            if not self.map_line_is_camera_traversable(grid, data, sx, sy, wx, wy):
                continue

            age = max(0.0, time.time() - stamp)
            age_weight = max(0.15, 1.0 - age / max(1.0, self._get_float_param("visual_sector_memory_sec", 70.0)))
            heading_weight = max(0.0, 1.0 - heading_err / max(1e-6, 0.5 * fov))
            distance_weight = max(0.1, 1.0 - dist / max(1e-6, max_range))

            score = age_weight * (0.65 * heading_weight + 0.35 * distance_weight)
            best_score = max(best_score, score)

        return best_score > 0.05, best_score

    def frontier_has_visual_context(self, wx: float, wy: float) -> tuple[bool, float]:
        self.prune_visual_memory()

        radius = self._get_float_param("visual_semantic_radius_m", 2.0)
        if radius <= 0.0:
            return False, 0.0

        best_score = 0.0

        for item in getattr(self, "visual_context", []):
            ix = float(item.get("x", 0.0))
            iy = float(item.get("y", 0.0))
            dist = math.hypot(wx - ix, wy - iy)

            if dist > radius:
                continue

            conf = float(item.get("confidence", 0.0))
            proximity = max(0.0, 1.0 - dist / radius)
            score = 0.5 * proximity + 0.5 * conf
            best_score = max(best_score, score)

        return best_score > 0.05, best_score

    def camera_map_frontier_condition(
        self,
        grid: OccupancyGrid,
        data: np.ndarray,
        wx: float,
        wy: float,
    ) -> tuple[bool, float, str]:
        seen_by_camera, camera_score = self.frontier_seen_by_camera_sector(grid, data, wx, wy)
        has_context, context_score = self.frontier_has_visual_context(wx, wy)

        passed = bool(seen_by_camera or has_context)
        score = max(camera_score, context_score)

        if seen_by_camera and has_context:
            reason = "camera_sector_and_visual_context"
        elif seen_by_camera:
            reason = "camera_sector"
        elif has_context:
            reason = "visual_context"
        else:
            reason = "map_only"

        return passed, score, reason


    def compute_frontiers(self, pose: tuple[float, float, float]) -> list[Frontier]:
        grid = self.latest_map
        if grid is None:
            return []

        width = int(grid.info.width)
        height = int(grid.info.height)
        if width <= 0 or height <= 0:
            return []

        data = np.asarray(grid.data, dtype=np.int16).reshape((height, width))

        free = data == 0
        unknown = data < 0

        frontier_mask = np.zeros_like(free, dtype=bool)

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if not free[y, x]:
                    continue

                nb_unknown = unknown[y - 1:y + 2, x - 1:x + 2]
                if np.any(nb_unknown):
                    frontier_mask[y, x] = True

        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters: list[list[tuple[int, int]]] = []

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if not frontier_mask[y, x] or visited[y, x]:
                    continue

                q = deque([(x, y)])
                visited[y, x] = True
                cluster = []

                while q:
                    cx, cy = q.popleft()
                    cluster.append((cx, cy))

                    for nx in (cx - 1, cx, cx + 1):
                        for ny in (cy - 1, cy, cy + 1):
                            if nx < 1 or nx >= width - 1 or ny < 1 or ny >= height - 1:
                                continue
                            if visited[ny, nx] or not frontier_mask[ny, nx]:
                                continue
                            visited[ny, nx] = True
                            q.append((nx, ny))

                clusters.append(cluster)

        min_cells = int(self.get_parameter("frontier_min_cluster_cells").value)
        bx, by, byaw = pose

        mode = str(self.get_parameter("frontier_camera_map_mode").value).strip().lower()
        visual_bonus = float(self.get_parameter("visual_frontier_bonus").value)

        all_map_frontiers: list[Frontier] = []
        camera_map_frontiers: list[Frontier] = []

        fid = 0

        for cluster in clusters:
            if len(cluster) < min_cells:
                continue

            xs = np.asarray([c[0] for c in cluster], dtype=np.float32)
            ys = np.asarray([c[1] for c in cluster], dtype=np.float32)

            mx_cell = float(xs.mean())
            my_cell = float(ys.mean())
            wx, wy = self.map_cell_to_world(grid, mx_cell, my_cell)

            dist = math.hypot(wx - bx, wy - by)

            if dist < float(self.get_parameter("frontier_min_distance_m").value):
                continue
            if dist > float(self.get_parameter("frontier_max_distance_m").value):
                continue
            if self.is_frontier_blacklisted(wx, wy):
                continue

            heading = math.atan2(wy - by, wx - bx)
            heading_err = self.angle_error(heading, byaw)

            size_w = float(self.get_parameter("frontier_size_weight").value)
            dist_w = float(self.get_parameter("frontier_distance_weight").value)
            heading_w = float(self.get_parameter("frontier_heading_weight").value)

            camera_ok, camera_score, camera_reason = self.camera_map_frontier_condition(grid, data, wx, wy)

            base_score = size_w * len(cluster) - dist_w * dist - heading_w * heading_err
            fused_score = base_score + visual_bonus * camera_score

            frontier = Frontier(
                frontier_id=fid,
                map_x=wx,
                map_y=wy,
                cell_count=len(cluster),
                distance_m=dist,
                score=fused_score,
            )
            fid += 1

            all_map_frontiers.append(frontier)

            if camera_ok:
                camera_map_frontiers.append(frontier)

            self.get_logger().debug(
                f"frontier_candidate x={wx:.2f} y={wy:.2f} "
                f"cells={len(cluster)} dist={dist:.2f} "
                f"camera_ok={camera_ok} camera_reason={camera_reason} "
                f"score={fused_score:.2f}"
            )

        all_map_frontiers.sort(key=lambda f: f.score, reverse=True)
        camera_map_frontiers.sort(key=lambda f: f.score, reverse=True)

        if mode == "strict_and":
            return camera_map_frontiers

        if mode == "soft_score":
            return all_map_frontiers

        # Default: use AND when possible, but avoid deadlock if the camera sector memory
        # is empty or no frontier passed the visual condition.
        if camera_map_frontiers:
            return camera_map_frontiers

        return all_map_frontiers

    def map_cell_to_world(self, grid: OccupancyGrid, cell_x: float, cell_y: float) -> tuple[float, float]:
        res = float(grid.info.resolution)
        ox = float(grid.info.origin.position.x)
        oy = float(grid.info.origin.position.y)
        wx = ox + (cell_x + 0.5) * res
        wy = oy + (cell_y + 0.5) * res
        return wx, wy

    def is_frontier_blacklisted(self, x: float, y: float) -> bool:
        radius = float(self.get_parameter("frontier_blacklist_radius_m").value)
        for bx, by in self.blacklisted_frontiers:
            if math.hypot(x - bx, y - by) <= radius:
                return True
        return False

    def blacklist_current_frontier(self) -> None:
        if self.current_frontier is not None:
            self.blacklisted_frontiers.append((self.current_frontier.map_x, self.current_frontier.map_y))

    def navigate_to_frontier(self, frontier: Frontier) -> None:
        if not bool(self.get_parameter("auto_navigate").value):
            self.say(f"Next frontier is at x {frontier.map_x:.1f}, y {frontier.map_y:.1f}.")
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.say("Nav2 is not ready yet.")
            return

        pose = self.current_base_pose()
        if pose is None:
            self.say("I cannot localize in the map yet.")
            return

        bx, by, _ = pose
        yaw = math.atan2(frontier.map_y - by, frontier.map_x - bx)
        self.send_nav_goal(frontier.map_x, frontier.map_y, yaw, kind="frontier")

    def navigate_to_object(self, obs: ObjectObservation) -> None:
        self.navigation_started_for_target = True
        self.set_state("APPROACH_OBJECT", "target_found")

        if not bool(self.get_parameter("auto_navigate").value):
            self.say(f"I found {obs.label.replace('_', ' ')} at map x {obs.map_x:.1f}, y {obs.map_y:.1f}.")
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.say("I found the object, but Nav2 is not ready.")
            return

        base = self.current_base_pose()
        if base is None:
            self.say("I found the object, but cannot localize myself.")
            return

        bx, by, _ = base
        dx = obs.map_x - bx
        dy = obs.map_y - by
        dist = max(0.001, math.hypot(dx, dy))
        ux = dx / dist
        uy = dy / dist

        approach = float(self.get_parameter("approach_distance_m").value)
        gx = obs.map_x - approach * ux
        gy = obs.map_y - approach * uy
        yaw = math.atan2(obs.map_y - gy, obs.map_x - gx)

        self.send_nav_goal(gx, gy, yaw, kind="object_approach")

    def send_nav_goal(self, x: float, y: float, yaw: float, kind: str) -> None:
        now = time.time()
        min_sep = float(self.get_parameter("min_nav_goal_separation_sec").value)

        # Do not spam Nav2 with repeated object goals from repeated detections.
        if kind == self.active_nav_kind and now - self.last_nav_goal_sent_at < min_sep:
            self.get_logger().info(f"suppressing duplicate nav goal kind={kind}")
            return

        qx, qy, qz, qw = yaw_to_quat(yaw)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(self.get_parameter("map_frame").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.stop_robot()

        self.nav_goal_seq += 1
        seq = self.nav_goal_seq
        self.active_nav_goal_seq = seq
        self.active_nav_kind = kind
        self.last_nav_goal_sent_at = now

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self.on_nav_goal_response(fut, kind=kind, seq=seq))

        if kind == "frontier":
            self.say("Navigating to the next frontier.")
            self.set_state("NAV_TO_FRONTIER", "nav_goal_sent")
        else:
            self.say("Navigating near the object.")
            self.set_state("APPROACH_OBJECT", "nav_goal_sent")

        self.get_logger().info(f"sent_nav_goal seq={seq} kind={kind} x={x:.2f} y={y:.2f} yaw={yaw:.2f}")

    def on_nav_goal_response(self, future, kind: str, seq: int) -> None:
        if seq != self.active_nav_goal_seq:
            self.get_logger().info(
                f"ignoring stale nav goal response seq={seq} active_seq={self.active_nav_goal_seq} kind={kind}"
            )
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Nav2 goal send failed: {exc}")
            if kind == "frontier":
                self.blacklist_current_frontier()
                self.set_state("PLANNING_FRONTIER", "goal_send_failed")
            elif self.active_target:
                self.set_state("LOCAL_SCAN", "object_goal_send_failed")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected")
            if kind == "frontier":
                self.blacklist_current_frontier()
                self.set_state("PLANNING_FRONTIER", "goal_rejected")
            elif self.active_target:
                self.set_state("LOCAL_SCAN", "object_goal_rejected")
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self.on_nav_result(fut, kind=kind, seq=seq))

    def on_nav_result(self, future, kind: str, seq: int) -> None:
        if seq != self.active_nav_goal_seq:
            self.get_logger().info(
                f"ignoring stale nav result seq={seq} active_seq={self.active_nav_goal_seq} kind={kind}"
            )
            return

        self.current_goal_handle = None

        try:
            result = future.result()
            status = int(result.status)
        except Exception as exc:
            self.get_logger().warn(f"Nav2 result failed: {exc}")
            status = -1

        self.get_logger().info(f"nav_result seq={seq} kind={kind} status={status}")

        # ROS 2 action status 4 means succeeded.
        if kind == "object_approach":
            if status == 4:
                self.say("I arrived near the object.")
                self.active_target = ""
                self.navigation_started_for_target = False
                self.active_nav_kind = ""
                self.set_state("IDLE", "object_arrived")
            else:
                self.say("I could not reach the object. I will keep exploring.")
                self.navigation_started_for_target = False
                if self.active_target:
                    self.set_state("PLANNING_FRONTIER", f"object_approach_failed_status_{status}")
                else:
                    self.set_state("IDLE", f"object_approach_failed_status_{status}")
            return

        if kind == "frontier":
            if status == 4:
                self.say("I reached a frontier. Scanning for the object.")
                self.active_nav_kind = ""
                self.set_state("LOCAL_SCAN", "frontier_reached")
            else:
                self.blacklist_current_frontier()
                self.active_nav_kind = ""
                self.set_state("PLANNING_FRONTIER", f"frontier_nav_failed_status_{status}")

    def cancel_nav_goal(self) -> None:
        self.active_nav_goal_seq = -1
        self.active_nav_kind = ""
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None

    def stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())

    def merge_observation(self, obs: ObjectObservation) -> None:
        arr = self.memory.setdefault(obs.label, [])
        radius = float(self.get_parameter("merge_radius_m").value)

        for old in arr:
            d = math.hypot(obs.map_x - old.map_x, obs.map_y - old.map_y)
            if d <= radius:
                n = old.seen_count + 1
                old.map_x = (old.map_x * old.seen_count + obs.map_x) / n
                old.map_y = (old.map_y * old.seen_count + obs.map_y) / n
                old.confidence = max(old.confidence, obs.confidence)
                old.range_m = obs.range_m
                old.bearing_rad = obs.bearing_rad
                old.seen_count = n
                old.last_seen = obs.last_seen
                self.save_memory()
                return

        arr.append(obs)
        self.save_memory()

    def best_match(self, target: str) -> Optional[ObjectObservation]:
        now = time.time()
        max_age = float(self.get_parameter("max_object_age_sec").value)
        candidates: list[ObjectObservation] = []

        for label, obs_list in self.memory.items():
            if not label_match(target, label):
                continue
            for obs in obs_list:
                if now - obs.last_seen <= max_age:
                    candidates.append(obs)

        if not candidates:
            return None

        candidates.sort(key=lambda o: (o.seen_count, o.confidence, -abs(now - o.last_seen)), reverse=True)
        return candidates[0]

    def save_memory(self) -> None:
        path = os.path.expanduser(str(self.get_parameter("memory_path").value))
        os.makedirs(os.path.dirname(path), exist_ok=True)

        data = {label: [asdict(obs) for obs in obs_list] for label, obs_list in self.memory.items()}

        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=True)

    def load_memory(self) -> None:
        path = os.path.expanduser(str(self.get_parameter("memory_path").value))
        if not os.path.isfile(path):
            return

        try:
            with open(path, "r", encoding="utf-8") as f:
                raw = yaml.safe_load(f) or {}

            for label, obs_list in raw.items():
                self.memory[label] = [ObjectObservation(**obs) for obs in obs_list]
            self.get_logger().info(f"loaded object memory from {path}")
        except Exception as exc:
            self.get_logger().warn(f"failed to load object memory: {exc}")

    def publish_state_memory_markers(self) -> None:
        data = {label: [asdict(obs) for obs in obs_list] for label, obs_list in self.memory.items()}
        self.memory_pub.publish(String(data=json.dumps(data, sort_keys=True)))

        markers = MarkerArray()
        mid = 0
        stamp = self.get_clock().now().to_msg()

        for f in self.frontiers[:30]:
            m = Marker()
            m.header.frame_id = str(self.get_parameter("map_frame").value)
            m.header.stamp = stamp
            m.ns = "frontiers"
            m.id = mid
            mid += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(f.map_x)
            m.pose.position.y = float(f.map_y)
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = 0.22
            m.scale.y = 0.22
            m.scale.z = 0.08
            m.color.r = 0.1
            m.color.g = 0.3
            m.color.b = 1.0
            m.color.a = 0.75
            markers.markers.append(m)

        for label, obs_list in self.memory.items():
            for obs in obs_list:
                m = Marker()
                m.header.frame_id = str(self.get_parameter("map_frame").value)
                m.header.stamp = stamp
                m.ns = "objects"
                m.id = mid
                mid += 1
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(obs.map_x)
                m.pose.position.y = float(obs.map_y)
                m.pose.position.z = 0.35
                m.pose.orientation.w = 1.0
                m.scale.x = 0.28
                m.scale.y = 0.28
                m.scale.z = 0.28
                m.color.r = 0.0
                m.color.g = 0.9
                m.color.b = 0.2
                m.color.a = 0.9
                markers.markers.append(m)

                t = Marker()
                t.header.frame_id = str(self.get_parameter("map_frame").value)
                t.header.stamp = stamp
                t.ns = "object_labels"
                t.id = mid
                mid += 1
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = float(obs.map_x)
                t.pose.position.y = float(obs.map_y)
                t.pose.position.z = 0.75
                t.pose.orientation.w = 1.0
                t.scale.z = 0.25
                t.color.r = 1.0
                t.color.g = 1.0
                t.color.b = 1.0
                t.color.a = 1.0
                t.text = f"{label} ({obs.seen_count})"
                markers.markers.append(t)

        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierObjectExplorerNode()
    try:
        rclpy.spin(node)
    finally:
        node.cancel_nav_goal()
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
