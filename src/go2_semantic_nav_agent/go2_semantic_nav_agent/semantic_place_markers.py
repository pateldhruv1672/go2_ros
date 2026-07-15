#!/usr/bin/env python3

import math
from pathlib import Path
from typing import Any, Dict, List

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def load_places(path: str) -> List[Dict[str, Any]]:
    p = Path(path).expanduser()
    if not p.exists():
        return []
    data = yaml.safe_load(p.read_text()) or {}
    if isinstance(data, dict):
        places = data.get("places", [])
    elif isinstance(data, list):
        places = data
    else:
        places = []
    return [x for x in places if isinstance(x, dict)]


def color_for_category(category: str):
    c = (category or "").lower()
    if "education" in c or "class" in c or "lecture" in c:
        return 0.1, 0.45, 1.0
    if "passage" in c or "hall" in c or "corridor" in c:
        return 0.1, 0.9, 0.2
    if "furniture" in c:
        return 1.0, 0.55, 0.1
    return 1.0, 0.2, 0.8


class SemanticPlaceMarkers(Node):
    def __init__(self):
        super().__init__("semantic_place_markers")

        self.declare_parameter("places_file", "")
        self.declare_parameter("marker_topic", "/semantic_nav/place_markers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("text_height", 0.22)
        self.declare_parameter("marker_scale", 0.22)

        self.places_file = str(self.get_parameter("places_file").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(MarkerArray, self.marker_topic, qos)

        period = float(self.get_parameter("publish_period_sec").value)
        self.timer = self.create_timer(period, self.publish_markers)

        self.get_logger().info(
            f"semantic place markers ready: places_file={self.places_file} topic={self.marker_topic}"
        )

    def publish_markers(self):
        places = load_places(self.places_file)

        arr = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        now = self.get_clock().now().to_msg()
        marker_scale = float(self.get_parameter("marker_scale").value)
        text_height = float(self.get_parameter("text_height").value)

        mid = 1
        for idx, place in enumerate(places):
            try:
                x = float(place.get("x", 0.0))
                y = float(place.get("y", 0.0))
                yaw = float(place.get("yaw", 0.0))
            except Exception:
                continue

            name = str(place.get("name", f"place_{idx}"))
            category = str(place.get("category", "place"))
            summary = str(place.get("summary", "") or place.get("description", ""))
            r, g, b = color_for_category(category)

            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = now
            sphere.ns = "semantic_places"
            sphere.id = mid
            mid += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.12
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = marker_scale
            sphere.scale.y = marker_scale
            sphere.scale.z = marker_scale
            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            sphere.color.a = 0.95
            arr.markers.append(sphere)

            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = now
            arrow.ns = "semantic_place_arrows"
            arrow.id = mid
            mid += 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.06
            arrow.scale.y = 0.12
            arrow.scale.z = 0.12
            arrow.color.r = r
            arrow.color.g = g
            arrow.color.b = b
            arrow.color.a = 0.9
            start = Point()
            start.x = x
            start.y = y
            start.z = 0.22
            end = Point()
            end.x = x + 0.55 * math.cos(yaw)
            end.y = y + 0.55 * math.sin(yaw)
            end.z = 0.22
            arrow.points = [start, end]
            arr.markers.append(arrow)

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = now
            text.ns = "semantic_place_labels"
            text.id = mid
            mid += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.65
            text.pose.orientation.w = 1.0
            text.scale.z = text_height
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            if summary:
                text.text = f"{idx + 1}. {name}\n{summary[:70]}"
            else:
                text.text = f"{idx + 1}. {name}"
            arr.markers.append(text)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPlaceMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
