from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from .storage import MemoryStore


class SemanticMapVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__('semantic_map_visualizer_node')
        self.declare_parameter('storage_root', '~/.ros/go2_agent_memory')
        self.declare_parameter('marker_topic', '/agent/semantic_markers')
        self.declare_parameter('refresh_sec', 1.0)
        self.store = MemoryStore(self.get_parameter('storage_root').value)
        self.marker_pub = self.create_publisher(MarkerArray, self.get_parameter('marker_topic').value, 10)
        self.timer = self.create_timer(float(self.get_parameter('refresh_sec').value), self.publish_markers)

    def publish_markers(self) -> None:
        state = self.store.read_state()
        active_map = state.get('active_map')
        markers = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)
        if not active_map:
            self.marker_pub.publish(markers)
            return
        places = self.store.list_places(active_map).get('places', {})
        marker_id = 1
        for name, pose in places.items():
            frame = pose.get('frame_id', 'map')
            now = self.get_clock().now().to_msg()
            sphere = Marker()
            sphere.header.frame_id = frame
            sphere.header.stamp = now
            sphere.ns = 'places'
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(pose['x'])
            sphere.pose.position.y = float(pose['y'])
            sphere.pose.position.z = 0.15
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.22
            sphere.scale.y = 0.22
            sphere.scale.z = 0.22
            sphere.color.r = 0.1
            sphere.color.g = 0.9
            sphere.color.b = 0.3
            sphere.color.a = 0.9
            markers.markers.append(sphere)
            marker_id += 1
            text = Marker()
            text.header.frame_id = frame
            text.header.stamp = now
            text.ns = 'labels'
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(pose['x'])
            text.pose.position.y = float(pose['y'])
            text.pose.position.z = 0.55
            text.pose.orientation.w = 1.0
            text.scale.z = 0.22
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = name
            markers.markers.append(text)
            marker_id += 1
            arrow = Marker()
            arrow.header.frame_id = frame
            arrow.header.stamp = now
            arrow.ns = 'headings'
            arrow.id = marker_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = float(pose['x'])
            arrow.pose.position.y = float(pose['y'])
            arrow.pose.position.z = 0.1
            yaw = float(pose.get('yaw', 0.0))
            arrow.pose.orientation.z = math.sin(yaw / 2.0)
            arrow.pose.orientation.w = math.cos(yaw / 2.0)
            arrow.scale.x = 0.45
            arrow.scale.y = 0.08
            arrow.scale.z = 0.08
            arrow.color.r = 0.0
            arrow.color.g = 0.5
            arrow.color.b = 1.0
            arrow.color.a = 0.9
            markers.markers.append(arrow)
            marker_id += 1
        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMapVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
