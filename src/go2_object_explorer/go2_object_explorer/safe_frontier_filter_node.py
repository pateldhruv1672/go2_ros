#!/usr/bin/env python3

import json
import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class SafeFrontierFilterNode(Node):
    def __init__(self):
        super().__init__("safe_frontier_filter_node")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("marker_topic", "/mrkl_explorer/safe_frontiers")
        self.declare_parameter("json_topic", "/mrkl_explorer/safe_frontier_candidates")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("min_cluster_cells", 12)
        self.declare_parameter("max_frontiers", 10)

        self.declare_parameter("footprint_radius_m", 0.34)
        self.declare_parameter("frontier_goal_min_dist_m", 0.45)
        self.declare_parameter("frontier_goal_max_dist_m", 1.35)
        self.declare_parameter("robot_min_goal_dist_m", 0.65)
        self.declare_parameter("robot_max_goal_dist_m", 4.5)

        self.declare_parameter("clutter_radius_m", 0.75)
        self.declare_parameter("max_clutter_fraction", 0.35)

        self.map_msg = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.on_map,
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("marker_topic").value,
            10,
        )

        self.json_pub = self.create_publisher(
            String,
            self.get_parameter("json_topic").value,
            10,
        )

        self.timer = self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self.tick,
        )

        self.get_logger().info("safe frontier filter ready")

    def on_map(self, msg):
        self.map_msg = msg

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter("global_frame").value,
                self.get_parameter("base_frame").value,
                rclpy.time.Time(),
            )
            return float(tf.transform.translation.x), float(tf.transform.translation.y)
        except Exception as exc:
            self.get_logger().warn(f"no robot pose yet: {exc}")
            return None

    def world_to_cell(self, x, y):
        info = self.map_msg.info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return mx, my

    def cell_to_world(self, mx, my):
        info = self.map_msg.info
        return (
            info.origin.position.x + (mx + 0.5) * info.resolution,
            info.origin.position.y + (my + 0.5) * info.resolution,
        )

    def clutter_fraction(self, grid, mx, my, radius_cells):
        h, w = grid.shape
        occ_thr = int(self.get_parameter("occupied_threshold").value)

        total = 0
        bad = 0

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue

                x = mx + dx
                y = my + dy
                total += 1

                if x < 0 or y < 0 or x >= w or y >= h:
                    bad += 1
                    continue

                v = int(grid[y, x])
                if v < 0 or v >= occ_thr:
                    bad += 1

        return 1.0 if total <= 0 else float(bad) / float(total)

    def nearest_clear_start(self, clear_mask, start):
        sx, sy = start
        h, w = clear_mask.shape

        if 0 <= sx < w and 0 <= sy < h and clear_mask[sy, sx]:
            return sx, sy

        for r in range(1, 20):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    x = sx + dx
                    y = sy + dy
                    if 0 <= x < w and 0 <= y < h and clear_mask[y, x]:
                        return x, y

        return None

    def reachable_mask(self, clear_mask, start_cell):
        h, w = clear_mask.shape
        reachable = np.zeros_like(clear_mask, dtype=np.bool_)

        start = self.nearest_clear_start(clear_mask, start_cell)
        if start is None:
            return reachable

        q = deque([start])
        reachable[start[1], start[0]] = True

        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while q:
            x, y = q.popleft()
            for dx, dy in neighbors:
                nx = x + dx
                ny = y + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                if reachable[ny, nx]:
                    continue
                if not clear_mask[ny, nx]:
                    continue
                reachable[ny, nx] = True
                q.append((nx, ny))

        return reachable

    def publish(self, candidates):
        arr = MarkerArray()

        delete = Marker()
        delete.action = Marker.DELETEALL
        arr.markers.append(delete)

        for c in candidates:
            m = Marker()
            m.header.frame_id = self.get_parameter("global_frame").value
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "safe_frontier_goal"
            m.id = int(c["id"])
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(c["x"])
            m.pose.position.y = float(c["y"])
            m.pose.position.z = 0.12
            m.pose.orientation.w = 1.0
            m.scale.x = 0.28
            m.scale.y = 0.28
            m.scale.z = 0.16
            m.color.r = 0.0
            m.color.g = 0.35
            m.color.b = 1.0
            m.color.a = 0.95
            arr.markers.append(m)

            t = Marker()
            t.header.frame_id = m.header.frame_id
            t.header.stamp = m.header.stamp
            t.ns = "safe_frontier_label"
            t.id = 1000 + int(c["id"])
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(c["x"])
            t.pose.position.y = float(c["y"])
            t.pose.position.z = 0.45
            t.pose.orientation.w = 1.0
            t.scale.z = 0.18
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 1.0
            t.text = f"safe F{c['id']}"
            arr.markers.append(t)

        self.marker_pub.publish(arr)

        msg = String()
        msg.data = json.dumps(
            {
                "stamp": time.time(),
                "count": len(candidates),
                "candidates": candidates,
            }
        )
        self.json_pub.publish(msg)

    def tick(self):
        if self.map_msg is None:
            return

        pose = self.robot_pose()
        if pose is None:
            return

        info = self.map_msg.info
        w = int(info.width)
        h = int(info.height)
        res = float(info.resolution)

        if w <= 0 or h <= 0 or len(self.map_msg.data) != w * h:
            return

        grid = np.array(self.map_msg.data, dtype=np.int16).reshape((h, w))
        occ_thr = int(self.get_parameter("occupied_threshold").value)

        known_free = (grid >= 0) & (grid < occ_thr)
        unknown = grid < 0

        # Raw frontier cells: known-free cells touching unknown cells.
        adj_unknown = np.zeros_like(unknown, dtype=np.bool_)
        adj_unknown[1:, :] |= unknown[:-1, :]
        adj_unknown[:-1, :] |= unknown[1:, :]
        adj_unknown[:, 1:] |= unknown[:, :-1]
        adj_unknown[:, :-1] |= unknown[:, 1:]

        raw_frontier = known_free & adj_unknown

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            raw_frontier.astype(np.uint8),
            8,
        )

        # Clearance mask: known-free cells far enough from unknown/occupied.
        free_u8 = known_free.astype(np.uint8) * 255
        dist_px = cv2.distanceTransform(free_u8, cv2.DIST_L2, 3)
        footprint_radius_m = float(self.get_parameter("footprint_radius_m").value)
        clear_mask = (dist_px * res) >= footprint_radius_m

        robot_cell = self.world_to_cell(pose[0], pose[1])
        if robot_cell is None:
            return

        reachable = self.reachable_mask(clear_mask, robot_cell)

        min_cluster = int(self.get_parameter("min_cluster_cells").value)
        max_frontiers = int(self.get_parameter("max_frontiers").value)

        band_min = float(self.get_parameter("frontier_goal_min_dist_m").value)
        band_max = float(self.get_parameter("frontier_goal_max_dist_m").value)
        robot_min = float(self.get_parameter("robot_min_goal_dist_m").value)
        robot_max = float(self.get_parameter("robot_max_goal_dist_m").value)

        clutter_radius_cells = max(1, int(float(self.get_parameter("clutter_radius_m").value) / res))
        max_clutter = float(self.get_parameter("max_clutter_fraction").value)

        candidates = []

        reachable_indices = np.argwhere(reachable)
        if reachable_indices.size == 0:
            self.publish([])
            return

        for label_id in range(1, num_labels):
            cell_count = int(stats[label_id, cv2.CC_STAT_AREA])
            if cell_count < min_cluster:
                continue

            cx, cy = centroids[label_id]
            raw_x, raw_y = self.cell_to_world(cx, cy)

            best = None
            best_score = -1e18

            # Search reachable clear cells around this frontier cluster.
            search_radius_cells = max(2, int(band_max / res))
            center_x = int(round(cx))
            center_y = int(round(cy))

            for yy in range(max(0, center_y - search_radius_cells), min(h, center_y + search_radius_cells + 1)):
                for xx in range(max(0, center_x - search_radius_cells), min(w, center_x + search_radius_cells + 1)):
                    if not reachable[yy, xx]:
                        continue

                    gx, gy = self.cell_to_world(xx, yy)

                    d_frontier = math.hypot(gx - raw_x, gy - raw_y)
                    if d_frontier < band_min or d_frontier > band_max:
                        continue

                    d_robot = math.hypot(gx - pose[0], gy - pose[1])
                    if d_robot < robot_min or d_robot > robot_max:
                        continue

                    clutter = self.clutter_fraction(grid, xx, yy, clutter_radius_cells)
                    if clutter > max_clutter:
                        continue

                    # Prefer high-info clusters, low clutter, reachable, close-ish, and about 0.65m behind frontier.
                    target_standoff = 0.65
                    score = (
                        0.08 * cell_count
                        - 1.20 * d_robot
                        - 8.00 * clutter
                        - 1.50 * abs(d_frontier - target_standoff)
                    )

                    if score > best_score:
                        best_score = score
                        best = {
                            "x": gx,
                            "y": gy,
                            "raw_x": raw_x,
                            "raw_y": raw_y,
                            "cell_count": cell_count,
                            "distance_m": d_robot,
                            "frontier_standoff_m": d_frontier,
                            "local_clutter_fraction": clutter,
                            "score": score,
                        }

            if best is not None:
                candidates.append(best)

        candidates.sort(key=lambda c: -c["score"])
        candidates = candidates[:max_frontiers]

        for i, c in enumerate(candidates):
            c["id"] = i
            for k in ("x", "y", "raw_x", "raw_y", "distance_m", "frontier_standoff_m", "local_clutter_fraction", "score"):
                c[k] = round(float(c[k]), 3)

        self.publish(candidates)


def main():
    rclpy.init()
    node = SafeFrontierFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
