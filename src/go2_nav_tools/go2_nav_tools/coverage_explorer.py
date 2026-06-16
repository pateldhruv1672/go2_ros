from __future__ import annotations

import json
import math
from typing import Dict, List, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import String


def simple_lawnmower_waypoints(x_min: float, x_max: float, y_min: float, y_max: float, step_m: float) -> List[Dict[str, float]]:
    waypoints = []
    y = y_min
    direction = 1
    while y <= y_max + 1e-6:
        xs = [x_min, x_max] if direction > 0 else [x_max, x_min]
        for x in xs:
            waypoints.append({"x": float(x), "y": float(y), "yaw": 0.0})
        direction *= -1
        y += max(step_m, 0.1)
    return waypoints


def coverage_waypoints_from_map(msg: OccupancyGrid, robot_xy: Tuple[float, float], step_m: float = 1.2, max_waypoints: int = 40, inflation_cells: int = 1) -> Dict[str, object]:
    width, height = int(msg.info.width), int(msg.info.height)
    data = list(msg.data)
    if width <= 0 or height <= 0:
        return {"waypoints": [], "summary": {"error": "invalid_map"}}
    free_cells: List[Tuple[int, int]] = []
    for y in range(height):
        for x in range(width):
            idx = y * width + x
            if data[idx] != 0:
                continue
            safe = True
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height and data[ny * width + nx] > 50:
                        safe = False
            if safe:
                free_cells.append((x, y))
    if not free_cells:
        return {"waypoints": [], "summary": {"error": "no_safe_free_cells"}}
    xs = [c[0] for c in free_cells]
    ys = [c[1] for c in free_cells]
    resolution = float(msg.info.resolution)
    x_min = msg.info.origin.position.x + min(xs) * resolution
    x_max = msg.info.origin.position.x + max(xs) * resolution
    y_min = msg.info.origin.position.y + min(ys) * resolution
    y_max = msg.info.origin.position.y + max(ys) * resolution
    raw = simple_lawnmower_waypoints(x_min, x_max, y_min, y_max, step_m)
    rx, ry = robot_xy
    raw.sort(key=lambda p: math.hypot(float(p["x"]) - rx, float(p["y"]) - ry))
    waypoints: List[Dict[str, object]] = []
    for idx, pose in enumerate(raw[:max_waypoints]):
        dist = math.hypot(float(pose["x"]) - rx, float(pose["y"]) - ry)
        waypoints.append({"goal_id": f"coverage_{idx:03d}", "frame_id": msg.header.frame_id or "map", **pose, "distance_m": round(dist, 3)})
    return {
        "map_frame": msg.header.frame_id or "map",
        "step_m": step_m,
        "bounds": {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
        "waypoints": waypoints,
        "summary": {"safe_free_cells": len(free_cells), "waypoint_count": len(waypoints)},
    }


class CoverageExplorer(Node):
    def __init__(self) -> None:
        super().__init__("go2_coverage_explorer")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("step_m", 1.2)
        self.declare_parameter("max_waypoints", 40)
        self.odom_xy = (0.0, 0.0)
        self.pub = self.create_publisher(String, "/go2_nav/coverage_plan", 10)
        self.create_subscription(OccupancyGrid, str(self.get_parameter("map_topic").value), self._on_map, 5)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10)
        self.get_logger().info("Coverage explorer ready: publishes map-derived lawnmower coverage waypoints")

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def _on_map(self, msg: OccupancyGrid) -> None:
        plan = coverage_waypoints_from_map(msg, self.odom_xy, float(self.get_parameter("step_m").value), int(self.get_parameter("max_waypoints").value))
        self.pub.publish(String(data=json.dumps(plan, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
