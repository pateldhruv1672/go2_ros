#!/usr/bin/env python3

import json
import math
import time
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quat(yaw: float):
    import math
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def dist2(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def extract_json_list(raw: str) -> List[Dict[str, Any]]:
    try:
        data = json.loads(raw)
    except Exception:
        return []

    if isinstance(data, list):
        return [x for x in data if isinstance(x, dict)]

    if isinstance(data, dict):
        for k in ("detections", "objects", "items", "results", "frontiers"):
            if isinstance(data.get(k), list):
                return [x for x in data[k] if isinstance(x, dict)]
        return [data]

    return []


class MRKLExplorerAgent(Node):
    def declare_parameter_once(self, name, default_value):
        try:
            return self.declare_parameter(name, default_value)
        except ParameterAlreadyDeclaredException:
            return self.get_parameter(name)

    def __init__(self):
        super().__init__("mrkl_explorer_agent_node")

        self.declare_parameter_once("goal_topic", "/mrkl_explorer/goal")
        self.declare_parameter_once("object_explorer_goal_topic", "/object_explorer/goal")
        self.declare_parameter_once("state_topic", "/object_explorer/state")
        self.declare_parameter_once("detections_topic", "/object_explorer/detections")
        self.declare_parameter_once("sam2_detections_topic", "/object_explorer/sam2_detections")
        self.declare_parameter_once("frontier_marker_topic", "/object_explorer/frontiers")
        self.declare_parameter_once("cmd_vel_topic", "/cmd_vel_omi")
        self.declare_parameter_once("trace_topic", "/mrkl_explorer/trace")
        self.declare_parameter_once("status_topic", "/mrkl_explorer/status")
        self.declare_parameter_once("goal_marker_topic", "/mrkl_explorer/current_goal_marker")
        self.declare_parameter_once("map_topic", "/map")

        self.declare_parameter_once("ollama_url", "http://127.0.0.1:11434")
        self.declare_parameter_once("ollama_model", "gemma4:e4b")
        self.declare_parameter_once("ollama_timeout_sec", 45.0)
        self.declare_parameter_once("ollama_keep_alive", "30m")
        self.declare_parameter_once("ollama_num_ctx", 8192)
        self.declare_parameter_once("ollama_num_predict", 512)
        self.declare_parameter_once("ollama_temperature", 0.1)
        self.declare_parameter_once("ollama_retry_count", 2)

        self.declare_parameter_once("decision_period_sec", 3.0)
        self.declare_parameter_once("scan_duration_sec", 5.0)
        self.declare_parameter_once("scan_angular_speed", 0.45)
        self.declare_parameter_once("object_approach_distance_m", 0.85)
        self.declare_parameter_once("max_frontiers_for_llm", 10)
        self.declare_parameter_once("max_objects_for_llm", 12)
        self.declare_parameter_once("goal_free_search_radius_m", 0.8)
        self.declare_parameter_once("min_goal_separation_m", 0.35)
        self.declare_parameter_once("frontier_standoff_m", 0.65)
        self.declare_parameter_once("frontier_sample_step_m", 0.10)
        self.declare_parameter_once("frontier_min_distance_m", 0.75)
        self.declare_parameter_once("frontier_max_distance_m", 4.0)
        self.declare_parameter_once("frontier_footprint_radius_m", 0.32)
        self.declare_parameter_once("frontier_clutter_radius_m", 0.65)
        self.declare_parameter_once("frontier_max_clutter_fraction", 0.40)
        self.declare_parameter_once("frontier_occupied_threshold", 50)
        self.declare_parameter_once("frontier_required_ns_contains", "")
        self.declare_parameter_once("failed_goal_blacklist_radius_m", 0.75)
        self.declare_parameter_once("max_failed_goal_blacklist", 50)
        self.declare_parameter_once("frontier_standoff_m", 0.75)
        self.declare_parameter_once("frontier_sample_step_m", 0.10)
        self.declare_parameter_once("failed_goal_blacklist_radius_m", 0.75)
        self.declare_parameter_once("max_failed_goal_blacklist", 50)

        self.goal_text = ""
        self.active = False
        self.triggered_explorer = False
        self.nav_active = False
        self.scan_active_until = 0.0
        self.last_decision_t = 0.0
        self.last_tool_result = None
        self.last_nav_goal_xy = None
        self.failed_goal_blacklist = []

        self.explorer_state = "unknown"
        self.mapped_detections: List[Dict[str, Any]] = []
        self.sam2_detections: List[Dict[str, Any]] = []
        self.frontier_markers: List[Marker] = []
        self.map_msg: Optional[OccupancyGrid] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.goal_sub = self.create_subscription(
            String,
            self.get_parameter("goal_topic").value,
            self.on_goal,
            10,
        )
        self.state_sub = self.create_subscription(
            String,
            self.get_parameter("state_topic").value,
            self.on_state,
            10,
        )
        self.det_sub = self.create_subscription(
            String,
            self.get_parameter("detections_topic").value,
            self.on_detections,
            10,
        )
        self.sam2_sub = self.create_subscription(
            String,
            self.get_parameter("sam2_detections_topic").value,
            self.on_sam2_detections,
            10,
        )
        self.frontier_sub = self.create_subscription(
            MarkerArray,
            self.get_parameter("frontier_marker_topic").value,
            self.on_frontier_markers,
            10,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.on_map,
            10,
        )

        self.explorer_goal_pub = self.create_publisher(
            String,
            self.get_parameter("object_explorer_goal_topic").value,
            10,
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )
        self.trace_pub = self.create_publisher(
            String,
            self.get_parameter("trace_topic").value,
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter("status_topic").value,
            10,
        )
        self.goal_marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("goal_marker_topic").value,
            10,
        )

        self.timer = self.create_timer(0.2, self.tick)

        self.get_logger().info("MRKL explorer agent ready")

    # -----------------------
    # Topic callbacks
    # -----------------------

    def on_goal(self, msg: String):
        self.goal_text = msg.data.strip()
        self.active = bool(self.goal_text)
        self.triggered_explorer = False
        self.last_decision_t = 0.0
        self.last_tool_result = None
        self.publish_status({"event": "new_mission", "goal": self.goal_text})

    def on_state(self, msg: String):
        self.explorer_state = msg.data

    def on_detections(self, msg: String):
        self.mapped_detections = extract_json_list(msg.data)

    def on_sam2_detections(self, msg: String):
        self.sam2_detections = extract_json_list(msg.data)

    def on_frontier_markers(self, msg: MarkerArray):
        self.frontier_markers = list(msg.markers)

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    # -----------------------
    # Utilities
    # -----------------------

    def publish_json(self, pub, obj):
        m = String()
        m.data = json.dumps(obj)
        pub.publish(m)

    def publish_trace(self, obj):
        obj["stamp_sec"] = time.time()
        self.publish_json(self.trace_pub, obj)

    def publish_status(self, obj):
        obj["stamp_sec"] = time.time()
        self.publish_json(self.status_pub, obj)

    def current_pose_xy_yaw(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception as exc:
            self.publish_trace({"event": "pose_unavailable", "error": str(exc)})
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return x, y, yaw

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return mx, my

    def cell_value(self, mx: int, my: int) -> Optional[int]:
        if self.map_msg is None:
            return None
        idx = my * self.map_msg.info.width + mx
        if idx < 0 or idx >= len(self.map_msg.data):
            return None
        return int(self.map_msg.data[idx])

    def is_free_world(self, x: float, y: float) -> bool:
        c = self.world_to_cell(x, y)
        if c is None:
            return False
        v = self.cell_value(*c)
        return v is not None and 0 <= v < 50

    def nearest_free_world(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        if self.map_msg is None:
            return x, y

        if self.is_free_world(x, y):
            return x, y

        info = self.map_msg.info
        max_r_m = float(self.get_parameter("goal_free_search_radius_m").value)
        max_r_cells = max(1, int(max_r_m / info.resolution))
        start = self.world_to_cell(x, y)
        if start is None:
            return None

        sx, sy = start
        best = None
        best_d2 = 1e18

        for r in range(1, max_r_cells + 1):
            for dx in range(-r, r + 1):
                for dy in (-r, r):
                    mx, my = sx + dx, sy + dy
                    if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                        continue
                    v = self.cell_value(mx, my)
                    if v is not None and 0 <= v < 50:
                        wx = info.origin.position.x + (mx + 0.5) * info.resolution
                        wy = info.origin.position.y + (my + 0.5) * info.resolution
                        d2v = dist2((x, y), (wx, wy))
                        if d2v < best_d2:
                            best = (wx, wy)
                            best_d2 = d2v
            for dy in range(-r + 1, r):
                for dx in (-r, r):
                    mx, my = sx + dx, sy + dy
                    if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                        continue
                    v = self.cell_value(mx, my)
                    if v is not None and 0 <= v < 50:
                        wx = info.origin.position.x + (mx + 0.5) * info.resolution
                        wy = info.origin.position.y + (my + 0.5) * info.resolution
                        d2v = dist2((x, y), (wx, wy))
                        if d2v < best_d2:
                            best = (wx, wy)
                            best_d2 = d2v
            if best is not None:
                return best

        return None

    # -----------------------
    # Observation builders
    # -----------------------

    def object_xy(self, det: Dict[str, Any]) -> Optional[Tuple[float, float]]:
        pairs = [
            ("map_x", "map_y"),
            ("world_x", "world_y"),
            ("x_map", "y_map"),
            ("target_x", "target_y"),
            ("x", "y"),
        ]
        for kx, ky in pairs:
            if kx in det and ky in det:
                try:
                    return float(det[kx]), float(det[ky])
                except Exception:
                    pass
        pose = det.get("pose")
        if isinstance(pose, dict) and "x" in pose and "y" in pose:
            try:
                return float(pose["x"]), float(pose["y"])
            except Exception:
                pass
        return None

    def object_label(self, det: Dict[str, Any]) -> str:
        for k in ("label", "name", "class_name", "class", "object"):
            if k in det:
                return str(det[k])
        return "object"

    def object_conf(self, det: Dict[str, Any]) -> float:
        for k in ("confidence", "conf", "score", "probability"):
            if k in det:
                try:
                    return float(det[k])
                except Exception:
                    return 0.0
        return 0.0


    def is_blacklisted_world(self, x: float, y: float) -> bool:
        radius = float(self.get_parameter("failed_goal_blacklist_radius_m").value)
        r2 = radius * radius
        for g in self.failed_goal_blacklist:
            try:
                if dist2((x, y), (float(g["x"]), float(g["y"]))) <= r2:
                    return True
            except Exception:
                continue
        return False

    def map_value_world(self, x: float, y: float):
        c = self.world_to_cell(x, y)
        if c is None:
            return None
        return self.cell_value(*c)

    def footprint_is_clear_world(self, x: float, y: float) -> bool:
        """
        Candidate goal must be in known free space with enough footprint clearance.
        Unknown and occupied cells inside the footprint are rejected.
        """
        if self.map_msg is None:
            return False

        info = self.map_msg.info
        radius_m = float(self.get_parameter("frontier_footprint_radius_m").value)
        occupied_threshold = int(self.get_parameter("frontier_occupied_threshold").value)

        center = self.world_to_cell(x, y)
        if center is None:
            return False

        r_cells = max(1, int(radius_m / info.resolution))
        cx, cy = center

        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                wx = info.origin.position.x + (cx + dx + 0.5) * info.resolution
                wy = info.origin.position.y + (cy + dy + 0.5) * info.resolution
                if dist2((x, y), (wx, wy)) > radius_m * radius_m:
                    continue

                mx = cx + dx
                my = cy + dy
                if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                    return False

                v = self.cell_value(mx, my)
                if v is None or v < 0 or v >= occupied_threshold:
                    return False

        return True

    def clutter_fraction_world(self, x: float, y: float) -> float:
        """
        Measures how cluttered the local neighborhood is.
        Used to avoid navigating into dense furniture/object regions.
        """
        if self.map_msg is None:
            return 1.0

        info = self.map_msg.info
        radius_m = float(self.get_parameter("frontier_clutter_radius_m").value)
        occupied_threshold = int(self.get_parameter("frontier_occupied_threshold").value)

        center = self.world_to_cell(x, y)
        if center is None:
            return 1.0

        r_cells = max(1, int(radius_m / info.resolution))
        cx, cy = center

        total = 0
        bad = 0

        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                wx = info.origin.position.x + (cx + dx + 0.5) * info.resolution
                wy = info.origin.position.y + (cy + dy + 0.5) * info.resolution
                if dist2((x, y), (wx, wy)) > radius_m * radius_m:
                    continue

                mx = cx + dx
                my = cy + dy
                if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                    bad += 1
                    total += 1
                    continue

                v = self.cell_value(mx, my)
                total += 1
                if v is None or v < 0 or v >= occupied_threshold:
                    bad += 1

        if total <= 0:
            return 1.0
        return float(bad) / float(total)

    def frontier_goal_is_valid(self, x: float, y: float) -> bool:
        if self.is_blacklisted_world(x, y):
            return False
        if not self.footprint_is_clear_world(x, y):
            return False
        clutter = self.clutter_fraction_world(x, y)
        if clutter > float(self.get_parameter("frontier_max_clutter_fraction").value):
            return False
        return True

    def marker_is_frontier(self, m: Marker) -> bool:
        """
        Reject object labels/detection text markers.
        Only true frontier markers should become navigation candidates.
        """
        required = str(self.get_parameter("frontier_required_ns_contains").value).strip().lower()

        # Never use text labels as navigation frontiers.
        if m.type == Marker.TEXT_VIEW_FACING:
            return False

        # Only geometric markers.
        if m.type not in (Marker.SPHERE, Marker.CUBE, Marker.ARROW, Marker.CYLINDER):
            return False

        ns = str(m.ns or "").lower()

        # If required string is empty, accept geometric markers.
        if not required:
            return True

        return required in ns

    def frontier_approach_goal(self, raw_x: float, raw_y: float, pose) -> Optional[Tuple[float, float]]:
        """
        Convert raw frontier boundary into a reachable goal before the frontier.
        We walk from frontier back toward robot until we find a free, low-clutter footprint.
        """
        rx, ry = float(pose[0]), float(pose[1])
        vx = float(raw_x) - rx
        vy = float(raw_y) - ry
        d = math.sqrt(vx * vx + vy * vy)

        min_d = float(self.get_parameter("frontier_min_distance_m").value)
        max_d = float(self.get_parameter("frontier_max_distance_m").value)

        if d < min_d:
            return None

        ux = vx / d
        uy = vy / d

        standoff = float(self.get_parameter("frontier_standoff_m").value)
        step = float(self.get_parameter("frontier_sample_step_m").value)

        start_s = min(max_d, max(min_d, d - standoff))
        end_s = min_d

        s = start_s
        while s >= end_s:
            gx = rx + ux * s
            gy = ry + uy * s

            if self.frontier_goal_is_valid(gx, gy):
                return gx, gy

            s -= step

        return None

    def frontier_candidates(self, pose) -> List[Dict[str, Any]]:
        out = []
        seen = set()

        for m in self.frontier_markers:
            if m.action == Marker.DELETEALL:
                continue

            if not self.marker_is_frontier(m):
                continue

            raw_x = float(m.pose.position.x)
            raw_y = float(m.pose.position.y)

            key = (round(raw_x, 2), round(raw_y, 2))
            if key in seen:
                continue
            seen.add(key)

            if abs(raw_x) < 1e-6 and abs(raw_y) < 1e-6:
                continue

            marker_ns = str(m.ns or "")

            # Safe frontier filter publishes already-validated goal cells.
            # Do not standoff again, or we move the goal too far backward.
            if "safe_frontier" in marker_ns:
                if not self.frontier_goal_is_valid(raw_x, raw_y):
                    self.publish_trace({
                        "event": "frontier_rejected",
                        "reason": "safe_frontier_failed_final_validation",
                        "raw_x": round(raw_x, 3),
                        "raw_y": round(raw_y, 3),
                        "marker_ns": marker_ns,
                    })
                    continue
                gx, gy = raw_x, raw_y
            else:
                goal_xy = self.frontier_approach_goal(raw_x, raw_y, pose)
                if goal_xy is None:
                    self.publish_trace({
                        "event": "frontier_rejected",
                        "reason": "no_safe_low_clutter_standoff_goal",
                        "raw_x": round(raw_x, 3),
                        "raw_y": round(raw_y, 3),
                        "marker_ns": marker_ns,
                    })
                    continue
                gx, gy = goal_xy

            clutter = self.clutter_fraction_world(gx, gy)

            out.append(
                {
                    "id": len(out),
                    "x": round(gx, 3),
                    "y": round(gy, 3),
                    "raw_x": round(raw_x, 3),
                    "raw_y": round(raw_y, 3),
                    "distance_m": round(math.sqrt(dist2((pose[0], pose[1]), (gx, gy))), 3),
                    "raw_frontier_distance_m": round(math.sqrt(dist2((pose[0], pose[1]), (raw_x, raw_y))), 3),
                    "frontier_standoff_m": round(math.sqrt(dist2((gx, gy), (raw_x, raw_y))), 3),
                    "local_clutter_fraction": round(clutter, 3),
                    "valid_goal_cell": True,
                    "source": m.ns or "frontier",
                }
            )

        # If marker frontier topic is empty/over-filtered, use the passive
        # explorer's current frontier from /object_explorer/state as a fallback.
        if not out:
            sf = self.state_frontier_candidate(pose)
            if sf is not None:
                sf["id"] = 0
                out.append(sf)
                self.publish_trace({
                    "event": "state_frontier_used",
                    "candidate": sf,
                })

        # Reassign compact candidate IDs so Ollama can only choose 0..N-1.
        out.sort(key=lambda f: (f.get("local_clutter_fraction", 1.0), f["distance_m"]))
        for i, f in enumerate(out):
            f["id"] = i

        return out[: int(self.get_parameter("max_frontiers_for_llm").value)]



    def state_frontier_candidate(self, pose) -> Optional[Dict[str, Any]]:
        """
        Fallback frontier source from /object_explorer/state.

        The passive explorer publishes a JSON state that may contain:
          frontier: {frontier_id, map_x, map_y, score, distance_m, cell_count}

        We do NOT trust its raw frontier_id as a MRKL candidate ID.
        We only use its map_x/map_y as a raw frontier boundary and generate
        our own safe standoff goal.
        """
        try:
            data = json.loads(self.explorer_state) if isinstance(self.explorer_state, str) else {}
        except Exception:
            return None

        f = data.get("frontier")
        if not isinstance(f, dict):
            return None

        try:
            raw_x = float(f.get("map_x"))
            raw_y = float(f.get("map_y"))
        except Exception:
            return None

        goal_xy = self.frontier_approach_goal(raw_x, raw_y, pose)
        if goal_xy is None:
            self.publish_trace({
                "event": "state_frontier_rejected",
                "reason": "no_safe_standoff_goal",
                "raw_x": round(raw_x, 3),
                "raw_y": round(raw_y, 3),
                "state_frontier_id": f.get("frontier_id"),
            })
            return None

        gx, gy = goal_xy
        return {
            "id": -1,
            "x": round(gx, 3),
            "y": round(gy, 3),
            "raw_x": round(raw_x, 3),
            "raw_y": round(raw_y, 3),
            "distance_m": round(math.sqrt(dist2((pose[0], pose[1]), (gx, gy))), 3),
            "raw_frontier_distance_m": round(math.sqrt(dist2((pose[0], pose[1]), (raw_x, raw_y))), 3),
            "frontier_standoff_m": round(math.sqrt(dist2((gx, gy), (raw_x, raw_y))), 3),
            "local_clutter_fraction": round(self.clutter_fraction_world(gx, gy), 3),
            "valid_goal_cell": True,
            "source": "object_explorer_state",
            "state_frontier_id": f.get("frontier_id"),
            "state_score": f.get("score"),
            "state_cell_count": f.get("cell_count"),
        }


    def object_candidates(self, pose) -> List[Dict[str, Any]]:
        mapped = []
        for i, det in enumerate(self.mapped_detections):
            xy = self.object_xy(det)
            label = self.object_label(det)
            conf = self.object_conf(det)
            item = {
                "id": len(mapped),
                "label": label,
                "confidence": round(conf, 3),
                "source": "mapped_yolo",
                "has_map_xy": xy is not None,
            }
            if xy is not None:
                item["x"] = round(xy[0], 3)
                item["y"] = round(xy[1], 3)
                item["distance_m"] = round(math.sqrt(dist2((pose[0], pose[1]), xy)), 3)
            mapped.append(item)

        # SAM2 detections are image-space unless another node adds map_x/map_y.
        for det in self.sam2_detections:
            label = self.object_label(det)
            conf = self.object_conf(det)
            xy = self.object_xy(det)
            item = {
                "id": len(mapped),
                "label": label,
                "confidence": round(conf, 3),
                "source": "sam2_track",
                "track_id": det.get("track_id"),
                "has_mask_polygon": bool(det.get("mask_polygon")),
                "has_map_xy": xy is not None,
            }
            if xy is not None:
                item["x"] = round(xy[0], 3)
                item["y"] = round(xy[1], 3)
                item["distance_m"] = round(math.sqrt(dist2((pose[0], pose[1]), xy)), 3)
            mapped.append(item)

        # Prefer target-ish labels, but do not hard-code the object class.
        goal = self.goal_text.lower()
        mapped.sort(
            key=lambda o: (
                0 if str(o.get("label", "")).lower() in goal else 1,
                0 if o.get("has_map_xy") else 1,
                -float(o.get("confidence", 0.0)),
                float(o.get("distance_m", 999.0)),
            )
        )
        return mapped[: int(self.get_parameter("max_objects_for_llm").value)]

    # -----------------------
    # Ollama MRKL routing
    # -----------------------

    def call_ollama(self, observation: Dict[str, Any]) -> Dict[str, Any]:
        url = str(self.get_parameter("ollama_url").value).rstrip("/") + "/api/chat"
        model = str(self.get_parameter("ollama_model").value)
        timeout = float(self.get_parameter("ollama_timeout_sec").value)

        system = (
            "You are the MRKL router for a Unitree Go2 object-exploration robot. "
            "You do not invent coordinates. You must choose exactly one tool call from the allowed tools. "
            "Use only valid_frontiers[].id and objects[].id from the observation. Ignore any frontier_id mentioned inside explorer_state because it is not a valid MRKL candidate ID. "
            "Prefer navigate_to_object only when an object has map coordinates. "
            "Prefer navigate_to_frontier when the target object is not yet mapped. "
            "Use scan_here when no useful valid frontier or object exists. "
            "Return JSON only."
        )

        allowed_tools = {
            "scan_here": {"reason": "string"},
            "navigate_to_frontier": {"frontier_id": "integer", "reason": "string"},
            "navigate_to_object": {"object_id": "integer", "reason": "string"},
            "finish": {"reason": "string"},
        }

        user = {
            "mrkl_rules": [
                "LLM routes only; tools execute.",
                "Never output raw target coordinates.",
                "Never choose ids not present in candidates.",
                "Do not issue motion directly.",
                "Return one JSON object: {tool: ..., args: {...}, reason: ...}.",
            ],
            "allowed_tools": allowed_tools,
            "observation": observation,
        }

        payload = {
            "model": model,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": json.dumps(user)},
            ],
            "stream": False,
            "format": "json",
            "keep_alive": str(self.get_parameter("ollama_keep_alive").value),
            "options": {
                "temperature": float(self.get_parameter("ollama_temperature").value),
                "num_ctx": int(self.get_parameter("ollama_num_ctx").value),
                "num_predict": int(self.get_parameter("ollama_num_predict").value),
            },
        }

        last_err = None
        retries = int(self.get_parameter("ollama_retry_count").value)

        for _ in range(retries + 1):
            try:
                req = urllib.request.Request(
                    url,
                    data=json.dumps(payload).encode("utf-8"),
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib.request.urlopen(req, timeout=timeout) as resp:
                    raw = resp.read().decode("utf-8")
                data = json.loads(raw)
                content = data.get("message", {}).get("content", "{}")
                decision = json.loads(content)
                return decision
            except urllib.error.HTTPError as exc:
                body = exc.read().decode("utf-8", errors="replace")
                last_err = f"Ollama HTTP {exc.code}: {body}"
            except Exception as exc:
                last_err = f"{type(exc).__name__}: {exc}"

        raise RuntimeError(last_err or "Ollama failed")

    def heuristic_fallback(self, objects, frontiers):
        goal = self.goal_text.lower()

        for obj in objects:
            if obj.get("has_map_xy") and str(obj.get("label", "")).lower() in goal:
                return {
                    "tool": "navigate_to_object",
                    "args": {"object_id": int(obj["id"])},
                    "reason": "heuristic target object with map position",
                }

        if frontiers:
            return {
                "tool": "navigate_to_frontier",
                "args": {"frontier_id": int(frontiers[0]["id"])},
                "reason": "heuristic nearest valid frontier",
            }

        return {
            "tool": "scan_here",
            "args": {},
            "reason": "heuristic no valid frontier or mapped object",
        }

    def validate_decision(self, decision, objects, frontiers):
        tool = decision.get("tool")
        args = decision.get("args", {}) or {}

        if tool == "scan_here":
            return True, decision

        if tool == "finish":
            return True, decision

        if tool == "navigate_to_frontier":
            fid = args.get("frontier_id")
            if not isinstance(fid, int):
                return False, "frontier_id must be integer"
            if fid not in {f["id"] for f in frontiers}:
                return False, f"invalid frontier_id={fid}"
            return True, decision

        if tool == "navigate_to_object":
            oid = args.get("object_id")
            if not isinstance(oid, int):
                return False, "object_id must be integer"
            candidates = {o["id"]: o for o in objects}
            if oid not in candidates:
                return False, f"invalid object_id={oid}"
            if not candidates[oid].get("has_map_xy"):
                return False, f"object_id={oid} has no map position"
            return True, decision

        return False, f"unknown tool={tool}"

    # -----------------------
    # Tool execution
    # -----------------------

    def execute_scan_here(self, reason: str):
        self.scan_active_until = time.time() + float(self.get_parameter("scan_duration_sec").value)
        self.last_tool_result = {"tool": "scan_here", "started": True, "reason": reason}
        self.publish_trace({"event": "tool_start", "tool": "scan_here", "reason": reason})

    def send_nav_goal(self, x: float, y: float, reason: str, tool: str):
        pose = self.current_pose_xy_yaw()
        if pose is None:
            self.last_tool_result = {"tool": tool, "accepted": False, "error": "no_current_pose"}
            return

        free_xy = self.nearest_free_world(x, y)
        if free_xy is None:
            self.last_tool_result = {"tool": tool, "accepted": False, "error": "no_nearest_free_cell"}
            self.publish_trace({"event": "tool_rejected", "tool": tool, "reason": "no_nearest_free_cell", "x": x, "y": y})
            return

        gx, gy = free_xy

        if self.last_nav_goal_xy is not None:
            sep = math.sqrt(dist2(self.last_nav_goal_xy, (gx, gy)))
            if sep < float(self.get_parameter("min_goal_separation_m").value):
                self.last_tool_result = {"tool": tool, "accepted": False, "error": "goal_too_similar"}
                self.publish_trace({"event": "tool_rejected", "tool": tool, "reason": "goal_too_similar", "x": gx, "y": gy})
                return

        yaw = math.atan2(gy - pose[1], gx - pose[0])

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = yaw_to_quat(yaw)

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.last_tool_result = {"tool": tool, "accepted": False, "error": "navigate_to_pose_unavailable"}
            self.publish_trace({"event": "tool_failed", "tool": tool, "error": "navigate_to_pose_unavailable"})
            return

        self.nav_active = True
        self.last_nav_goal_xy = (gx, gy)
        self.publish_goal_marker(gx, gy, tool, reason)
        self.publish_trace({"event": "tool_start", "tool": tool, "x": gx, "y": gy, "reason": reason})

        fut = self.nav_client.send_goal_async(goal_msg)
        fut.add_done_callback(lambda f: self.on_nav_goal_response(f, tool, gx, gy))

    def on_nav_goal_response(self, fut, tool, x, y):
        try:
            goal_handle = fut.result()
        except Exception as exc:
            self.nav_active = False
            self.last_tool_result = {"tool": tool, "accepted": False, "error": str(exc)}
            return

        if not goal_handle.accepted:
            self.nav_active = False
            self.last_tool_result = {"tool": tool, "accepted": False, "error": "goal_rejected"}
            self.publish_trace({"event": "tool_rejected", "tool": tool, "x": x, "y": y})
            return

        self.last_tool_result = {"tool": tool, "accepted": True, "x": x, "y": y}
        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(lambda f: self.on_nav_result(f, tool, x, y))

    def on_nav_result(self, fut, tool, x, y):
        self.nav_active = False
        try:
            result = fut.result()
            status = int(result.status)
        except Exception as exc:
            status = -1
            self.last_tool_result = {"tool": tool, "done": True, "status": status, "error": str(exc)}
            self.publish_trace({"event": "tool_result", "tool": tool, "status": status, "error": str(exc)})
            return

        self.last_tool_result = {"tool": tool, "done": True, "status": status, "x": x, "y": y}
        if status != 4:
            self.failed_goal_blacklist.append({"x": float(x), "y": float(y), "status": status, "time": time.time()})
            max_bad = int(self.get_parameter("max_failed_goal_blacklist").value)
            self.failed_goal_blacklist = self.failed_goal_blacklist[-max_bad:]
            self.publish_trace({"event": "goal_blacklisted", "tool": tool, "status": status, "x": x, "y": y})
        self.publish_trace({"event": "tool_result", "tool": tool, "status": status, "x": x, "y": y})

    def execute_navigate_to_frontier(self, decision, frontiers):
        fid = int(decision.get("args", {}).get("frontier_id"))
        f = next(x for x in frontiers if x["id"] == fid)
        self.send_nav_goal(float(f["x"]), float(f["y"]), decision.get("reason", ""), "navigate_to_frontier")

    def execute_navigate_to_object(self, decision, objects):
        oid = int(decision.get("args", {}).get("object_id"))
        obj = next(x for x in objects if x["id"] == oid)

        pose = self.current_pose_xy_yaw()
        ox, oy = float(obj["x"]), float(obj["y"])

        if pose is None:
            self.send_nav_goal(ox, oy, decision.get("reason", ""), "navigate_to_object")
            return

        approach = float(self.get_parameter("object_approach_distance_m").value)
        vx = ox - pose[0]
        vy = oy - pose[1]
        norm = math.sqrt(vx * vx + vy * vy)

        if norm > approach:
            gx = ox - (vx / norm) * approach
            gy = oy - (vy / norm) * approach
        else:
            gx, gy = ox, oy

        self.send_nav_goal(gx, gy, decision.get("reason", ""), "navigate_to_object")

    def publish_goal_marker(self, x, y, tool, reason):
        arr = MarkerArray()

        delete = Marker()
        delete.action = Marker.DELETEALL
        arr.markers.append(delete)

        sphere = Marker()
        sphere.header.frame_id = "map"
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = "mrkl_goal"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = x
        sphere.pose.position.y = y
        sphere.pose.position.z = 0.45
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.45
        sphere.scale.y = 0.45
        sphere.scale.z = 0.45
        sphere.color.r = 1.0
        sphere.color.g = 0.0
        sphere.color.b = 0.0
        sphere.color.a = 0.95
        arr.markers.append(sphere)

        text = Marker()
        text.header.frame_id = "map"
        text.header.stamp = sphere.header.stamp
        text.ns = "mrkl_goal_label"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 0.95
        text.pose.orientation.w = 1.0
        text.scale.z = 0.25
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f"{tool}\n{x:.2f}, {y:.2f}"
        arr.markers.append(text)

        self.goal_marker_pub.publish(arr)

    # -----------------------
    # Main tick
    # -----------------------

    def tick(self):
        now = time.time()

        if self.scan_active_until > now:
            tw = Twist()
            tw.angular.z = float(self.get_parameter("scan_angular_speed").value)
            self.cmd_pub.publish(tw)
            return
        elif self.scan_active_until != 0.0:
            self.scan_active_until = 0.0
            self.cmd_pub.publish(Twist())
            self.last_tool_result = {"tool": "scan_here", "done": True}
            self.publish_trace({"event": "tool_result", "tool": "scan_here", "done": True})

        if not self.active or self.nav_active:
            return

        if not self.triggered_explorer:
            msg = String()
            msg.data = self.goal_text
            self.explorer_goal_pub.publish(msg)
            self.triggered_explorer = True
            self.publish_trace({"event": "triggered_object_explorer", "goal": self.goal_text})
            return

        period = float(self.get_parameter("decision_period_sec").value)
        if now - self.last_decision_t < period:
            return
        self.last_decision_t = now

        pose = self.current_pose_xy_yaw()
        if pose is None:
            self.execute_scan_here("no valid robot pose yet")
            return

        frontiers = self.frontier_candidates(pose)
        objects = self.object_candidates(pose)

        observation = {
            "mission": self.goal_text,
            "robot_pose": {"x": round(pose[0], 3), "y": round(pose[1], 3), "yaw": round(pose[2], 3)},
            "explorer_state": self.explorer_state,
            "valid_frontiers": frontiers,
            "objects": objects,
            "last_tool_result": self.last_tool_result,
        }

        try:
            decision = self.call_ollama(observation)
            source = "ollama"
        except Exception as exc:
            decision = self.heuristic_fallback(objects, frontiers)
            source = "heuristic_fallback"
            self.publish_trace({"event": "ollama_failed", "error": str(exc), "fallback_decision": decision})

        valid, checked = self.validate_decision(decision, objects, frontiers)
        if not valid:
            self.publish_trace({"event": "invalid_llm_tool_call", "decision": decision, "error": checked})
            decision = self.heuristic_fallback(objects, frontiers)
            source = "validator_fallback"

        self.publish_trace({
            "event": "mrkl_decision",
            "source": source,
            "decision": decision,
            "observation_summary": {
                "frontier_count": len(frontiers),
                "object_count": len(objects),
                "state": self.explorer_state,
            },
        })

        tool = decision.get("tool")
        if tool == "scan_here":
            self.execute_scan_here(decision.get("reason", "MRKL selected scan_here"))
        elif tool == "navigate_to_frontier":
            self.execute_navigate_to_frontier(decision, frontiers)
        elif tool == "navigate_to_object":
            self.execute_navigate_to_object(decision, objects)
        elif tool == "finish":
            self.active = False
            self.publish_status({"event": "mission_finished", "reason": decision.get("reason", "")})
        else:
            self.execute_scan_here("unknown tool fallback")


def main():
    rclpy.init()
    node = MRKLExplorerAgent()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
