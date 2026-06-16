from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


@dataclass
class Track:
    track_id: int
    x: float
    y: float
    last_x: float
    last_y: float
    last_t: float
    speed_mps: float = 0.0
    age: int = 1

    def update(self, x: float, y: float, now: float) -> None:
        dt = max(1e-3, now - self.last_t)
        self.speed_mps = math.hypot(x - self.x, y - self.y) / dt
        self.last_x = self.x
        self.last_y = self.y
        self.x = x
        self.y = y
        self.last_t = now
        self.age += 1

    def to_dict(self) -> Dict[str, float | int]:
        return {"track_id": self.track_id, "x": round(self.x, 3), "y": round(self.y, 3), "speed_mps": round(self.speed_mps, 3), "age": self.age}


def scan_clusters(msg: LaserScan, jump_threshold_m: float = 0.35, min_points: int = 3, max_range_m: float = 4.0) -> List[Tuple[float, float, int]]:
    pts: List[Tuple[float, float]] = []
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r) or r < msg.range_min or r > min(msg.range_max, max_range_m):
            pts.append((float("nan"), float("nan")))
            continue
        angle = msg.angle_min + i * msg.angle_increment
        pts.append((r * math.cos(angle), r * math.sin(angle)))
    clusters: List[List[Tuple[float, float]]] = []
    cur: List[Tuple[float, float]] = []
    last: Tuple[float, float] | None = None
    for p in pts:
        if not math.isfinite(p[0]):
            if len(cur) >= min_points:
                clusters.append(cur)
            cur = []
            last = None
            continue
        if last is not None and math.hypot(p[0] - last[0], p[1] - last[1]) > jump_threshold_m:
            if len(cur) >= min_points:
                clusters.append(cur)
            cur = []
        cur.append(p)
        last = p
    if len(cur) >= min_points:
        clusters.append(cur)
    centroids: List[Tuple[float, float, int]] = []
    for cluster in clusters:
        x = sum(p[0] for p in cluster) / len(cluster)
        y = sum(p[1] for p in cluster) / len(cluster)
        centroids.append((x, y, len(cluster)))
    return centroids


class DynamicObstacleTracker(Node):
    def __init__(self) -> None:
        super().__init__("go2_dynamic_obstacle_tracker")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("associate_radius_m", 0.55)
        self.declare_parameter("dynamic_speed_threshold_mps", 0.12)
        self.declare_parameter("front_block_distance_m", 0.8)
        self.next_id = 1
        self.tracks: Dict[int, Track] = {}
        self.pub = self.create_publisher(String, "/go2_perception/dynamic_obstacles", 10)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self._on_scan, 10)
        self.get_logger().info("Dynamic obstacle tracker ready")

    def _on_scan(self, msg: LaserScan) -> None:
        now = time.time()
        clusters = scan_clusters(msg)
        assigned: set[int] = set()
        for x, y, count in clusters:
            best_id = None
            best_dist = 999.0
            for tid, tr in self.tracks.items():
                d = math.hypot(x - tr.x, y - tr.y)
                if d < best_dist and d < float(self.get_parameter("associate_radius_m").value):
                    best_dist = d
                    best_id = tid
            if best_id is None:
                best_id = self.next_id
                self.next_id += 1
                self.tracks[best_id] = Track(best_id, x, y, x, y, now)
            else:
                self.tracks[best_id].update(x, y, now)
            assigned.add(best_id)
        # Drop stale tracks.
        self.tracks = {tid: tr for tid, tr in self.tracks.items() if now - tr.last_t < 2.0 or tid in assigned}
        speed_thr = float(self.get_parameter("dynamic_speed_threshold_mps").value)
        front_dist = float(self.get_parameter("front_block_distance_m").value)
        dynamic = [tr for tr in self.tracks.values() if tr.speed_mps >= speed_thr and tr.age >= 2]
        front_blocked = any(0.0 < tr.x < front_dist and abs(tr.y) < 0.45 for tr in self.tracks.values())
        payload = {
            "track_count": len(self.tracks),
            "dynamic_count": len(dynamic),
            "front_blocked": front_blocked,
            "tracks": [tr.to_dict() for tr in sorted(dynamic, key=lambda t: -t.speed_mps)[:12]],
        }
        self.pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacleTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
