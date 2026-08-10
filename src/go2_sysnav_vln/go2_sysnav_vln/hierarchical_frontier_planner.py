from __future__ import annotations

import heapq
import json
import math
import time
import zlib
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .common import angle_wrap, labels_match, quat_to_yaw, stable_id, viewpoint_is_novel


@dataclass
class FrontierTrack:
    track_id: str
    x: float
    y: float
    room_id: str
    confirmations: int = 1
    last_seen: float = 0.0
    viewpoints: List[Tuple[float, float, float]] = field(default_factory=list)


class HierarchicalFrontierPlanner(Node):
    """Room graph + classical in-room frontier planning.

    This mirrors the SysNav division of labor: rooms are the semantic decision
    units, while goal generation within a room remains geometric. A frontier goal
    is always a known-free, reachable standoff cell; the raw boundary centroid is
    metadata only. Frontier persistence is updated only from novel robot poses.
    """

    def __init__(self) -> None:
        super().__init__("go2_hierarchical_frontier_planner")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("object_map_topic", "/go2_vln/object_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("candidate_topic", "/go2_vln/frontier_candidates")
        self.declare_parameter("room_graph_topic", "/go2_vln/room_graph")
        self.declare_parameter("room_label_update_topic", "/go2_vln/room_label_update")
        self.declare_parameter("marker_topic", "/go2_vln/frontier_markers")
        self.declare_parameter("room_marker_topic", "/go2_vln/room_markers")
        self.declare_parameter("blacklist_topic", "/go2_vln/frontier_blacklist")
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("robot_radius_m", 0.42)
        self.declare_parameter("clearance_margin_m", 0.10)
        self.declare_parameter("min_cluster_cells", 10)
        self.declare_parameter("max_candidates", 16)
        self.declare_parameter("standoff_min_m", 0.45)
        self.declare_parameter("standoff_max_m", 1.25)
        self.declare_parameter("min_goal_distance_m", 0.70)
        self.declare_parameter("max_goal_distance_m", 10.0)
        self.declare_parameter("info_gain_radius_m", 1.8)
        self.declare_parameter("recompute_translation_m", 0.20)
        self.declare_parameter("recompute_rotation_rad", 0.20)
        self.declare_parameter("room_core_clearance_m", 0.72)
        self.declare_parameter("room_min_core_area_m2", 0.75)
        self.declare_parameter("room_id_quantization_m", 1.0)
        self.declare_parameter("frontier_track_radius_m", 0.75)
        self.declare_parameter("frontier_view_angle_threshold_deg", 8.0)
        self.declare_parameter("frontier_view_range_threshold_m", 0.40)
        self.declare_parameter("frontier_view_translation_m", 0.20)
        self.declare_parameter("frontier_view_rotation_rad", 0.20)
        self.declare_parameter("frontier_stable_confirmations", 2)
        self.declare_parameter("frontier_track_ttl_sec", 90.0)
        self.declare_parameter("publish_period_sec", 0.8)

        self.map_msg: Optional[OccupancyGrid] = None
        self.objects: List[Dict[str, Any]] = []
        self.room_labels_by_id: Dict[str, str] = {}
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_compute_pose: Optional[Tuple[float, float, float]] = None
        self.last_map_signature: Optional[Tuple[int, int, int, int, int]] = None
        self.cached_candidates: List[Dict[str, Any]] = []
        self.cached_rooms: List[Dict[str, Any]] = []
        self.cached_status: Dict[str, Any] = {"reason": "not_computed"}
        self.last_status_reason = ""
        self.blacklist: Dict[str, float] = {}
        self.frontier_tracks: Dict[str, FrontierTrack] = {}

        self.candidate_pub = self.create_publisher(
            String, str(self.get_parameter("candidate_topic").value), 10
        )
        self.room_pub = self.create_publisher(
            String, str(self.get_parameter("room_graph_topic").value), 10
        )
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), marker_qos
        )
        self.room_marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("room_marker_topic").value), marker_qos
        )
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map_topic").value), self.on_map, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("object_map_topic").value), self.on_objects, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("blacklist_topic").value), self.on_blacklist, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("room_label_update_topic").value),
            self.on_room_label_update, 10,
        )
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self.tick)
        self.get_logger().info("SysNav-style room graph and frontier planner ready")

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def on_objects(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.objects = [x for x in payload.get("objects", []) if isinstance(x, dict)]
        except Exception:
            pass

    def on_room_label_update(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            room_id = str(payload.get("room_id", ""))
            label = str(payload.get("label", "")).strip()
            if room_id and label:
                self.room_labels_by_id[room_id] = label
        except Exception:
            pass

    def on_blacklist(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            fid = str(payload.get("frontier_id", ""))
            ttl = float(payload.get("ttl_sec", 120.0))
            if fid:
                self.blacklist[fid] = time.time() + max(1.0, ttl)
        except Exception:
            pass

    def robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                rclpy.time.Time(), timeout=Duration(seconds=0.15),
            )
            return (
                float(tf.transform.translation.x),
                float(tf.transform.translation.y),
                quat_to_yaw(tf.transform.rotation),
            )
        except Exception:
            return None

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        mx = int((x - float(info.origin.position.x)) / float(info.resolution))
        my = int((y - float(info.origin.position.y)) / float(info.resolution))
        if 0 <= mx < int(info.width) and 0 <= my < int(info.height):
            return mx, my
        return None

    def cell_to_world(self, x: float, y: float) -> Tuple[float, float]:
        assert self.map_msg is not None
        info = self.map_msg.info
        return (
            float(info.origin.position.x) + (x + 0.5) * float(info.resolution),
            float(info.origin.position.y) + (y + 0.5) * float(info.resolution),
        )

    def map_signature(self, grid: np.ndarray) -> Tuple[int, int, int, int, int]:
        sampled = np.ascontiguousarray(grid[::2, ::2], dtype=np.int16)
        checksum = zlib.crc32(sampled.tobytes()) & 0xFFFFFFFF
        return (
            int(grid.shape[1]), int(grid.shape[0]), int(np.count_nonzero(grid < 0)),
            int(np.count_nonzero(grid >= int(self.get_parameter("occupied_threshold").value))),
            int(checksum),
        )

    @staticmethod
    def geodesic_distances(mask: np.ndarray, start: Tuple[int, int]) -> np.ndarray:
        h, w = mask.shape
        dist = np.full((h, w), np.inf, dtype=np.float32)
        if not (0 <= start[0] < w and 0 <= start[1] < h and bool(mask[start[1], start[0]])):
            return dist
        dist[start[1], start[0]] = 0.0
        queue: List[Tuple[float, int, int]] = [(0.0, start[0], start[1])]
        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414),
        ]
        while queue:
            base, x, y = heapq.heappop(queue)
            if base > float(dist[y, x]) + 1e-6:
                continue
            for dx, dy, step in neighbors:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < w and 0 <= ny < h and bool(mask[ny, nx])):
                    continue
                candidate = base + step
                if candidate + 1e-6 < float(dist[ny, nx]):
                    dist[ny, nx] = candidate
                    heapq.heappush(queue, (candidate, nx, ny))
        return dist

    @staticmethod
    def nearest_mask(mask: np.ndarray, start: Tuple[int, int], max_radius: int = 60) -> Optional[Tuple[int, int]]:
        sx, sy = start
        h, w = mask.shape
        if 0 <= sx < w and 0 <= sy < h and bool(mask[sy, sx]):
            return sx, sy
        for radius in range(1, max_radius + 1):
            x0, x1 = max(0, sx - radius), min(w, sx + radius + 1)
            y0, y1 = max(0, sy - radius), min(h, sy + radius + 1)
            candidates = np.argwhere(mask[y0:y1, x0:x1])
            if len(candidates):
                y, x = candidates[0]
                return int(x + x0), int(y + y0)
        return None

    def segment_rooms(
        self, known_free: np.ndarray, clearance_m: np.ndarray, traversable: np.ndarray,
        robot_cell: Tuple[int, int], res: float,
    ) -> Tuple[np.ndarray, Dict[int, str], List[Dict[str, Any]]]:
        """Create stable room units from high-clearance cores and propagate labels.

        This avoids treating every frontier as a separate semantic choice while not
        requiring the full upstream TARE C++ stack. Narrow doorway/corridor cells do
        not form cores, so adjacent open regions become distinct room units.
        """
        core = known_free & (clearance_m >= float(self.get_parameter("room_core_clearance_m").value))
        # Remove one-pixel artifacts without erasing narrow frontier boundaries.
        core = cv2.morphologyEx(core.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(core.astype(np.uint8), 8)
        min_area_cells = max(4, int(round(
            float(self.get_parameter("room_min_core_area_m2").value) / max(res * res, 1e-6)
        )))
        valid_labels = [i for i in range(1, count) if int(stats[i, cv2.CC_STAT_AREA]) >= min_area_cells]
        if not valid_labels:
            labels = np.zeros_like(labels, dtype=np.int32)
            labels[known_free] = 1
            valid_labels = [1]
            centroids = np.asarray([[0.0, 0.0], [robot_cell[0], robot_cell[1]]])
            stats = np.zeros((2, 5), dtype=np.int32)
            stats[1, cv2.CC_STAT_AREA] = int(np.count_nonzero(known_free))
        else:
            labels[~np.isin(labels, valid_labels)] = 0

        # Multi-source geodesic propagation through known free space. This assigns
        # doorway and frontier-adjacent cells to a room instead of room 0.
        propagated = labels.copy().astype(np.int32)
        pq: List[Tuple[float, int, int, int]] = []
        ys, xs = np.where(propagated > 0)
        for x, y in zip(xs[::max(1, len(xs) // 20000 + 1)], ys[::max(1, len(ys) // 20000 + 1)]):
            heapq.heappush(pq, (0.0, int(x), int(y), int(propagated[y, x])))
        best = np.full(known_free.shape, np.inf, dtype=np.float32)
        best[propagated > 0] = 0.0
        while pq:
            d, x, y, lab = heapq.heappop(pq)
            if d > float(best[y, x]) + 1e-6:
                continue
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if not (0 <= nx < known_free.shape[1] and 0 <= ny < known_free.shape[0]):
                    continue
                if not bool(known_free[ny, nx]):
                    continue
                nd = d + 1.0
                if nd + 1e-6 < float(best[ny, nx]):
                    best[ny, nx] = nd
                    propagated[ny, nx] = lab
                    heapq.heappush(pq, (nd, nx, ny, lab))

        local_to_stable: Dict[int, str] = {}
        rooms: List[Dict[str, Any]] = []
        quant = float(self.get_parameter("room_id_quantization_m").value)
        robot_room_local = int(propagated[robot_cell[1], robot_cell[0]])
        for local_id in sorted(int(v) for v in np.unique(propagated) if int(v) > 0):
            ys, xs = np.where(propagated == local_id)
            if not len(xs):
                continue
            cx_cell, cy_cell = float(np.mean(xs)), float(np.mean(ys))
            cx, cy = self.cell_to_world(cx_cell, cy_cell)
            stable = "room_" + stable_id(round(cx / quant) * quant, round(cy / quant) * quant)
            local_to_stable[local_id] = stable
            safe_cells = np.argwhere((propagated == local_id) & traversable)
            if len(safe_cells):
                clearance_values = clearance_m[safe_cells[:, 0], safe_cells[:, 1]]
                anchor_y, anchor_x = safe_cells[int(np.argmax(clearance_values))]
            else:
                anchor_x, anchor_y = int(round(cx_cell)), int(round(cy_cell))
            anchor_xw, anchor_yw = self.cell_to_world(int(anchor_x), int(anchor_y))
            room_objects = []
            for obj in self.objects:
                cell = self.world_to_cell(float(obj.get("x", 0.0)), float(obj.get("y", 0.0)))
                if cell is not None and int(propagated[cell[1], cell[0]]) == local_id:
                    room_objects.append({
                        "object_id": int(obj.get("object_id", -1)),
                        "label": str(obj.get("label", "")),
                        "confirmed": bool(obj.get("confirmed", False)),
                        "confidence": float(obj.get("confidence", 0.0)),
                    })
            rooms.append({
                "room_id": stable,
                "local_id": local_id,
                "label": self.room_labels_by_id.get(stable, "unknown"),
                "centroid": {"x": round(cx, 3), "y": round(cy, 3)},
                "anchor": {"x": round(anchor_xw, 3), "y": round(anchor_yw, 3)},
                "area_m2": round(float(len(xs)) * res * res, 2),
                "is_current": local_id == robot_room_local,
                "objects": room_objects,
            })
        return propagated, local_to_stable, rooms

    def information_gain(self, unknown: np.ndarray, x: int, y: int, radius: int) -> int:
        h, w = unknown.shape
        y0, y1 = max(0, y - radius), min(h, y + radius + 1)
        x0, x1 = max(0, x - radius), min(w, x + radius + 1)
        patch = unknown[y0:y1, x0:x1]
        yy, xx = np.ogrid[y0:y1, x0:x1]
        circle = (xx - x) ** 2 + (yy - y) ** 2 <= radius * radius
        return int(np.count_nonzero(patch & circle))

    def match_frontier_track(
        self, x: float, y: float, room_id: str, pose: Tuple[float, float, float], now: float
    ) -> FrontierTrack:
        radius = float(self.get_parameter("frontier_track_radius_m").value)
        best: Optional[FrontierTrack] = None
        best_dist = float("inf")
        for track in self.frontier_tracks.values():
            if track.room_id != room_id:
                continue
            dist = math.hypot(track.x - x, track.y - y)
            if dist < radius and dist < best_dist:
                best, best_dist = track, dist
        if best is None:
            track_id = "frontier_" + stable_id(round(x, 1), round(y, 1), room_id, len(self.frontier_tracks))
            best = FrontierTrack(track_id, x, y, room_id, 1, now, [pose])
            self.frontier_tracks[track_id] = best
            return best
        novel = viewpoint_is_novel(
            (best.x, best.y), pose, best.viewpoints,
            math.radians(float(self.get_parameter("frontier_view_angle_threshold_deg").value)),
            float(self.get_parameter("frontier_view_range_threshold_m").value),
            float(self.get_parameter("frontier_view_translation_m").value),
            float(self.get_parameter("frontier_view_rotation_rad").value),
        )
        if novel:
            best.confirmations += 1
            best.viewpoints.append(pose)
            best.viewpoints = best.viewpoints[-32:]
        best.x = 0.75 * best.x + 0.25 * x
        best.y = 0.75 * best.y + 0.25 * y
        best.last_seen = now
        return best

    def recompute_needed(
        self, pose: Tuple[float, float, float], signature: Tuple[int, int, int, int, int]
    ) -> bool:
        if self.last_compute_pose is None or self.last_map_signature != signature:
            return True
        moved = math.hypot(pose[0] - self.last_compute_pose[0], pose[1] - self.last_compute_pose[1])
        rotated = abs(angle_wrap(pose[2] - self.last_compute_pose[2]))
        return (
            moved >= float(self.get_parameter("recompute_translation_m").value)
            or rotated >= float(self.get_parameter("recompute_rotation_rad").value)
        )

    def tick(self) -> None:
        if self.map_msg is None:
            return
        pose = self.robot_pose()
        if pose is None:
            return
        info = self.map_msg.info
        w, h = int(info.width), int(info.height)
        if w <= 0 or h <= 0 or len(self.map_msg.data) != w * h:
            return
        grid = np.asarray(self.map_msg.data, dtype=np.int16).reshape((h, w))
        signature = self.map_signature(grid)
        now = time.time()
        self.blacklist = {k: v for k, v in self.blacklist.items() if v > now}
        ttl = float(self.get_parameter("frontier_track_ttl_sec").value)
        self.frontier_tracks = {
            k: v for k, v in self.frontier_tracks.items() if now - v.last_seen <= ttl or v.last_seen == 0.0
        }
        if not self.recompute_needed(pose, signature):
            self.publish(self.cached_candidates, self.cached_rooms, self.cached_status)
            return

        res = float(info.resolution)
        occ_thr = int(self.get_parameter("occupied_threshold").value)
        known_free = (grid >= 0) & (grid < occ_thr)
        unknown = grid < 0
        occupied = grid >= occ_thr
        adj_unknown = np.zeros_like(unknown, dtype=np.bool_)
        adj_unknown[1:, :] |= unknown[:-1, :]
        adj_unknown[:-1, :] |= unknown[1:, :]
        adj_unknown[:, 1:] |= unknown[:, :-1]
        adj_unknown[:, :-1] |= unknown[:, 1:]
        frontier_mask = known_free & adj_unknown

        clearance_px = cv2.distanceTransform(known_free.astype(np.uint8), cv2.DIST_L2, 3)
        clearance_m = clearance_px * res
        required_clearance = float(self.get_parameter("robot_radius_m").value) + float(
            self.get_parameter("clearance_margin_m").value
        )
        traversable = known_free & (clearance_m >= required_clearance)
        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)
        status: Dict[str, Any] = {
            "reason": "computing",
            "map_width_cells": w,
            "map_height_cells": h,
            "resolution_m": round(res, 4),
            "map_bounds": {
                "min_x": round(origin_x, 3),
                "min_y": round(origin_y, 3),
                "max_x": round(origin_x + w * res, 3),
                "max_y": round(origin_y + h * res, 3),
            },
            "robot_pose": {
                "x": round(pose[0], 3),
                "y": round(pose[1], 3),
                "yaw": round(pose[2], 3),
            },
            "known_free_cells": int(np.count_nonzero(known_free)),
            "unknown_cells": int(np.count_nonzero(unknown)),
            "filtered_unknown_cells": int(np.count_nonzero(filtered_unknown)),
            "occupied_cells": int(np.count_nonzero(occupied)),
            "frontier_cells": int(np.count_nonzero(frontier_mask)),
            "traversable_cells": int(np.count_nonzero(traversable)),
            "required_clearance_m": round(required_clearance, 3),
            "min_unknown_region_cells": min_unknown_region,
            "min_cluster_cells": int(self.get_parameter("min_cluster_cells").value),
            "min_goal_distance_m": round(float(self.get_parameter("min_goal_distance_m").value), 3),
            "standoff_min_m": round(float(self.get_parameter("standoff_min_m").value), 3),
            "standoff_max_m": round(float(self.get_parameter("standoff_max_m").value), 3),
        }

        robot_cell = self.world_to_cell(pose[0], pose[1])
        if robot_cell is None:
            status["reason"] = "robot_outside_raw_map"
            self.cached_candidates = []
            self.cached_rooms = []
            self.cached_status = status
            self.last_compute_pose = pose
            self.last_map_signature = signature
            self.publish([], [], status)
            return
        status["robot_cell"] = {"x": int(robot_cell[0]), "y": int(robot_cell[1])}

        start = self.nearest_mask(traversable, robot_cell)
        if start is None:
            status["reason"] = "no_traversable_start"
            self.cached_candidates = []
            self.cached_rooms = []
            self.cached_status = status
            self.last_compute_pose = pose
            self.last_map_signature = signature
            self.publish([], [], status)
            return
        status["start_cell"] = {"x": int(start[0]), "y": int(start[1])}
        status["start_offset_cells"] = round(
            math.hypot(start[0] - robot_cell[0], start[1] - robot_cell[1]), 3
        )
        status["start_offset_m"] = round(status["start_offset_cells"] * res, 3)

        geodesic = self.geodesic_distances(traversable, start)
        reachable = np.isfinite(geodesic)
        status["reachable_cells"] = int(np.count_nonzero(reachable))
        room_grid, local_to_stable, rooms = self.segment_rooms(
            known_free, clearance_m, traversable, start, res
        )

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            frontier_mask.astype(np.uint8), 8
        )
        status["frontier_cluster_count_raw"] = max(0, int(num_labels) - 1)
        min_cluster = int(self.get_parameter("min_cluster_cells").value)
        min_standoff = float(self.get_parameter("standoff_min_m").value)
        max_standoff = float(self.get_parameter("standoff_max_m").value)
        min_goal = float(self.get_parameter("min_goal_distance_m").value)
        max_goal = float(self.get_parameter("max_goal_distance_m").value)
        info_radius = max(1, int(round(float(self.get_parameter("info_gain_radius_m").value) / res)))
        candidates: List[Dict[str, Any]] = []

        for component in range(1, num_labels):
            count = int(stats[component, cv2.CC_STAT_AREA])
            if count < min_cluster:
                continue
            component_mask = labels == component
            raw_cx, raw_cy = centroids[component]
            raw_x, raw_y = self.cell_to_world(float(raw_cx), float(raw_cy))
            distance_to_frontier = cv2.distanceTransform(
                (~component_mask).astype(np.uint8), cv2.DIST_L2, 3
            )
            band = reachable & ((distance_to_frontier * res) >= min_standoff) & (
                (distance_to_frontier * res) <= max_standoff
            )
            ys, xs = np.where(band)
            if not len(xs):
                continue
            bx, by = int(stats[component, cv2.CC_STAT_LEFT]), int(stats[component, cv2.CC_STAT_TOP])
            bw, bh = int(stats[component, cv2.CC_STAT_WIDTH]), int(stats[component, cv2.CC_STAT_HEIGHT])
            pad = max(2, int(round(max_standoff / res)))
            near = (
                (xs >= max(0, bx - pad)) & (xs < min(w, bx + bw + pad))
                & (ys >= max(0, by - pad)) & (ys < min(h, by + bh + pad))
            )
            xs, ys = xs[near], ys[near]
            best: Optional[Dict[str, Any]] = None
            best_score = -1e18
            stride = max(1, len(xs) // 300)
            for x, y in zip(xs[::stride], ys[::stride]):
                gx, gy = self.cell_to_world(int(x), int(y))
                euclidean = math.hypot(gx - pose[0], gy - pose[1])
                path_distance = float(geodesic[int(y), int(x)]) * res
                if euclidean < min_goal or euclidean > max_goal or not math.isfinite(path_distance):
                    continue
                clearance = float(clearance_m[int(y), int(x)])
                gain = self.information_gain(unknown, int(x), int(y), info_radius)
                goal_heading = math.atan2(raw_y - gy, raw_x - gx)
                heading_error = abs(angle_wrap(goal_heading - pose[2]))
                local_room = int(room_grid[int(y), int(x)])
                room_id = local_to_stable.get(local_room, "room_unknown")
                score = (
                    0.040 * gain + 0.10 * count + 1.35 * min(clearance, 1.2)
                    - 0.55 * path_distance - 0.25 * heading_error
                )
                if score > best_score:
                    best_score = score
                    best = {
                        "room_id": room_id,
                        "x": round(gx, 3), "y": round(gy, 3), "yaw": round(goal_heading, 3),
                        "raw_x": round(raw_x, 3), "raw_y": round(raw_y, 3),
                        "cell_count": count, "information_gain_cells": gain,
                        "clearance_m": round(clearance, 3),
                        "path_distance_m": round(path_distance, 3),
                        "euclidean_distance_m": round(euclidean, 3),
                        "score": round(score, 3),
                    }
            if best is None:
                continue
            # Preserve a bounded sample of the actual unknown/free boundary for
            # RViz. The navigation goal remains the separately computed safe
            # standoff pose in known free space.
            frontier_ys, frontier_xs = np.where(component_mask)
            point_stride = max(1, len(frontier_xs) // 120)
            best["frontier_points"] = [
                {
                    "x": round(self.cell_to_world(int(px), int(py))[0], 3),
                    "y": round(self.cell_to_world(int(px), int(py))[1], 3),
                }
                for px, py in zip(
                    frontier_xs[::point_stride], frontier_ys[::point_stride]
                )
            ]
            track = self.match_frontier_track(raw_x, raw_y, str(best["room_id"]), pose, now)
            best["frontier_id"] = track.track_id
            best["view_confirmations"] = track.confirmations
            best["stable"] = track.confirmations >= int(
                self.get_parameter("frontier_stable_confirmations").value
            )
            if track.track_id not in self.blacklist:
                candidates.append(best)

        candidates.sort(
            key=lambda item: (bool(item.get("stable", False)), float(item["score"])), reverse=True
        )
        candidates = candidates[: int(self.get_parameter("max_candidates").value)]
        frontier_counts: Dict[str, int] = {}
        stable_counts: Dict[str, int] = {}
        for item in candidates:
            room_id = str(item["room_id"])
            frontier_counts[room_id] = frontier_counts.get(room_id, 0) + 1
            if item.get("stable"):
                stable_counts[room_id] = stable_counts.get(room_id, 0) + 1
        for room in rooms:
            room_id = str(room["room_id"])
            room["frontier_count"] = frontier_counts.get(room_id, 0)
            room["stable_frontier_count"] = stable_counts.get(room_id, 0)
            room["explored"] = room["frontier_count"] == 0
            room["distance_m"] = round(math.hypot(
                float(room["anchor"]["x"]) - pose[0], float(room["anchor"]["y"]) - pose[1]
            ), 3)

        status["room_count"] = len(rooms)
        status["candidate_count"] = len(candidates)
        if candidates:
            status["reason"] = "ok"
        elif status["frontier_cells"] == 0:
            status["reason"] = "no_frontier_cells"
        elif status["frontier_cluster_count_raw"] == 0:
            status["reason"] = "no_frontier_clusters"
        else:
            status["reason"] = "frontiers_filtered_out"

        self.cached_candidates = candidates
        self.cached_rooms = rooms
        self.cached_status = status
        self.last_compute_pose = pose
        self.last_map_signature = signature
        self.publish(candidates, rooms, status)

    def publish(
        self, candidates: List[Dict[str, Any]], rooms: List[Dict[str, Any]],
        status: Optional[Dict[str, Any]] = None,
    ) -> None:
        current = next((r.get("room_id") for r in rooms if r.get("is_current")), None)
        planner_status = dict(status or self.cached_status or {"reason": "unknown"})
        reason = str(planner_status.get("reason", "unknown"))
        if reason != self.last_status_reason:
            log = self.get_logger().info if reason == "ok" else self.get_logger().warning
            log(
                "frontier planner status=%s candidates=%d rooms=%d free=%s traversable=%s frontier_cells=%s"
                % (
                    reason, len(candidates), len(rooms),
                    planner_status.get("known_free_cells"),
                    planner_status.get("traversable_cells"),
                    planner_status.get("frontier_cells"),
                )
            )
            self.last_status_reason = reason
        self.candidate_pub.publish(String(data=json.dumps({
            "stamp_sec": time.time(),
            "planner_contract": "room_level_semantics_classical_in_room_v3",
            "planner_status": planner_status,
            "current_room_id": current,
            "candidates": candidates,
        }, sort_keys=True)))
        self.room_pub.publish(String(data=json.dumps({
            "stamp_sec": time.time(),
            "current_room_id": current,
            "rooms": rooms,
        }, sort_keys=True)))
        self.publish_markers(candidates, rooms)

    def publish_markers(self, candidates: List[Dict[str, Any]], rooms: List[Dict[str, Any]]) -> None:
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray(); delete = Marker(); delete.action = Marker.DELETEALL
        markers.markers.append(delete)
        for idx, item in enumerate(candidates):
            stable = bool(item.get("stable"))
            boundary = Marker()
            boundary.header.frame_id = str(self.get_parameter("map_frame").value)
            boundary.header.stamp = stamp
            boundary.ns = "sysnav_frontier_boundaries"
            boundary.id = idx * 3
            boundary.type = Marker.POINTS
            boundary.action = Marker.ADD
            boundary.pose.orientation.w = 1.0
            boundary.scale.x = 0.07
            boundary.scale.y = 0.07
            boundary.color.r = 0.10 if stable else 1.0
            boundary.color.g = 0.75 if stable else 0.45
            boundary.color.b = 1.0 if stable else 0.05
            boundary.color.a = 0.95
            for xy in item.get("frontier_points", []):
                point = Point()
                point.x = float(xy["x"])
                point.y = float(xy["y"])
                point.z = 0.06
                boundary.points.append(point)
            markers.markers.append(boundary)

            goal = Marker()
            goal.header = boundary.header
            goal.ns = "sysnav_safe_frontier_goals"
            goal.id = idx * 3 + 1
            goal.type = Marker.ARROW
            goal.action = Marker.ADD
            goal.pose.position.x = float(item["x"])
            goal.pose.position.y = float(item["y"])
            goal.pose.position.z = 0.12
            yaw = float(item["yaw"])
            goal.pose.orientation.z = math.sin(yaw * 0.5)
            goal.pose.orientation.w = math.cos(yaw * 0.5)
            goal.scale.x = 0.48
            goal.scale.y = 0.13
            goal.scale.z = 0.13
            goal.color.r = 0.05 if stable else 1.0
            goal.color.g = 1.0 if stable else 0.70
            goal.color.b = 0.20
            goal.color.a = 1.0
            markers.markers.append(goal)

            text = Marker()
            text.header = boundary.header
            text.ns = "sysnav_frontier_labels"
            text.id = idx * 3 + 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(item["x"])
            text.pose.position.y = float(item["y"])
            text.pose.position.z = 0.48
            text.pose.orientation.w = 1.0
            text.scale.z = 0.16
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = (
                f"F:{str(item['frontier_id'])[-5:]} R:{str(item['room_id'])[-5:]} "
                f"views={item['view_confirmations']} IG={item['information_gain_cells']}"
            )
            markers.markers.append(text)
        self.marker_pub.publish(markers)

        room_markers = MarkerArray(); d2 = Marker(); d2.action = Marker.DELETEALL
        room_markers.markers.append(d2)
        for idx, room in enumerate(rooms):
            m = Marker(); m.header.frame_id = str(self.get_parameter("map_frame").value); m.header.stamp = stamp
            m.ns = "sysnav_rooms"; m.id = idx; m.type = Marker.TEXT_VIEW_FACING; m.action = Marker.ADD
            m.pose.position.x = float(room["centroid"]["x"]); m.pose.position.y = float(room["centroid"]["y"])
            m.pose.position.z = 1.0; m.pose.orientation.w = 1.0; m.scale.z = 0.28
            m.color.r = 0.2; m.color.g = 1.0 if room.get("is_current") else 0.75; m.color.b = 0.3
            m.color.a = 1.0
            m.text = (
                f"{room.get('label','unknown')} {str(room['room_id'])[-5:]} "
                f"F={room.get('frontier_count',0)} O={len(room.get('objects',[]))}"
            )
            room_markers.markers.append(m)
        self.room_marker_pub.publish(room_markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HierarchicalFrontierPlanner()
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
