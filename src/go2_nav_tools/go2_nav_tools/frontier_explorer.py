from __future__ import annotations

import json
import math
from collections import deque
from typing import Dict, Iterable, List, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import String

Grid = OccupancyGrid
Cell = Tuple[int, int]


def _idx(width: int, x: int, y: int) -> int:
    return y * width + x


def _neighbors4(x: int, y: int, width: int, height: int) -> Iterable[Cell]:
    for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
        if 0 <= nx < width and 0 <= ny < height:
            yield nx, ny


def _neighbors8(x: int, y: int, width: int, height: int) -> Iterable[Cell]:
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                yield nx, ny


def _world_from_cell(msg: Grid, cx: float, cy: float) -> Tuple[float, float]:
    return (
        msg.info.origin.position.x + (cx + 0.5) * msg.info.resolution,
        msg.info.origin.position.y + (cy + 0.5) * msg.info.resolution,
    )


def _robot_xy_from_odom(odom: Dict[str, float] | None) -> Tuple[float, float]:
    if not odom:
        return 0.0, 0.0
    return float(odom.get("x", 0.0)), float(odom.get("y", 0.0))


def frontier_candidates(msg: Grid, robot_xy: Tuple[float, float] = (0.0, 0.0), max_candidates: int = 12, min_cluster_cells: int = 4) -> Dict[str, object]:
    width, height = int(msg.info.width), int(msg.info.height)
    data = list(msg.data)
    if width <= 0 or height <= 0 or len(data) != width * height:
        return {"map_frame": msg.header.frame_id, "candidates": [], "summary": {"error": "invalid_map"}}
    frontier_cells: set[Cell] = set()
    for y in range(height):
        for x in range(width):
            value = data[_idx(width, x, y)]
            if value != 0:
                continue
            if any(data[_idx(width, nx, ny)] == -1 for nx, ny in _neighbors8(x, y, width, height)):
                frontier_cells.add((x, y))
    visited: set[Cell] = set()
    clusters: List[List[Cell]] = []
    for cell in sorted(frontier_cells):
        if cell in visited:
            continue
        q: deque[Cell] = deque([cell])
        visited.add(cell)
        cluster: List[Cell] = []
        while q:
            cx, cy = q.popleft()
            cluster.append((cx, cy))
            for nb in _neighbors8(cx, cy, width, height):
                if nb in frontier_cells and nb not in visited:
                    visited.add(nb)
                    q.append(nb)
        if len(cluster) >= min_cluster_cells:
            clusters.append(cluster)
    rx, ry = robot_xy
    candidates: List[Dict[str, object]] = []
    for i, cluster in enumerate(clusters):
        mx = sum(c[0] for c in cluster) / len(cluster)
        my = sum(c[1] for c in cluster) / len(cluster)
        wx, wy = _world_from_cell(msg, mx, my)
        dist = math.hypot(wx - rx, wy - ry)
        unknown_neighbors = 0
        for cx, cy in cluster:
            unknown_neighbors += sum(1 for nx, ny in _neighbors8(cx, cy, width, height) if data[_idx(width, nx, ny)] == -1)
        info_gain = min(1.0, (unknown_neighbors / max(1, len(cluster))) / 5.0 + len(cluster) / 250.0)
        distance_pref = max(0.0, min(1.0, 1.0 - abs(dist - 2.5) / 10.0))
        score = round(0.65 * info_gain + 0.35 * distance_pref, 4)
        yaw = math.atan2(wy - ry, wx - rx)
        candidates.append({
            "id": f"frontier_{i:03d}",
            "source": "occupancy_grid_frontier",
            "pose": {"frame_id": msg.header.frame_id or "map", "x": wx, "y": wy, "yaw": yaw, "qz": math.sin(yaw / 2.0), "qw": math.cos(yaw / 2.0)},
            "cluster_cells": len(cluster),
            "unknown_cells": unknown_neighbors,
            "information_gain": round(info_gain, 4),
            "distance_m": round(dist, 3),
            "score": score,
        })
    candidates.sort(key=lambda c: float(c["score"]), reverse=True)
    return {
        "map_frame": msg.header.frame_id or "map",
        "resolution": msg.info.resolution,
        "robot_xy": {"x": rx, "y": ry},
        "candidate_count": len(candidates),
        "candidates": candidates[:max_candidates],
        "summary": frontier_summary(msg),
    }


def frontier_summary(msg: Grid) -> dict:
    data = list(msg.data)
    unknown = data.count(-1)
    free = data.count(0)
    occupied = sum(1 for v in data if v > 50)
    total = len(data) or 1
    return {
        "map_frame": msg.header.frame_id,
        "width": msg.info.width,
        "height": msg.info.height,
        "resolution": msg.info.resolution,
        "unknown_ratio": unknown / total,
        "free_ratio": free / total,
        "occupied_ratio": occupied / total,
        "frontier_candidate": unknown > 0 and free > 0,
    }


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("go2_frontier_explorer")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("max_candidates", 12)
        self.declare_parameter("min_cluster_cells", 4)
        self.odom_xy: Tuple[float, float] = (0.0, 0.0)
        self.pub = self.create_publisher(String, "/go2_nav/frontier_summary", 10)
        self.candidate_pub = self.create_publisher(String, "/go2_nav/frontier_candidates", 10)
        self.create_subscription(OccupancyGrid, str(self.get_parameter("map_topic").value), self._on_map, 5)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10)
        self.get_logger().info("Frontier explorer ready: publishes scored frontier goal candidates")

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def _on_map(self, msg: Grid) -> None:
        summary = frontier_summary(msg)
        candidates = frontier_candidates(msg, self.odom_xy, int(self.get_parameter("max_candidates").value), int(self.get_parameter("min_cluster_cells").value))
        self.pub.publish(String(data=json.dumps(summary, sort_keys=True)))
        self.candidate_pub.publish(String(data=json.dumps(candidates, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
