from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .command_utils import parse_event
from .storage import MemoryStore, sanitize_name, utc_now, yaw_from_quaternion


class MemoryManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('memory_manager_node')
        self.declare_parameter('storage_root', '~/.ros/go2_agent_memory')
        self.declare_parameter('command_topic', '/agent/commands')
        self.declare_parameter('status_topic', '/agent/status')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/initialpose')
        self.declare_parameter('pose_log_interval_sec', 2.0)

        self.store = MemoryStore(self.get_parameter('storage_root').value)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.command_sub = self.create_subscription(String, self.get_parameter('command_topic').value, self.on_command, 10)
        self.odom_sub = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 20)
        self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('pose_topic').value, self.on_initialpose, 10)
        self.last_pose_log_ns: Optional[int] = None
        self.pose_log_interval_ns = int(float(self.get_parameter('pose_log_interval_sec').value) * 1e9)
        self.latest_pose = None
        self.get_logger().info(f'memory storage root: {self.store.paths.root}')

    def status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        payload = {
            'frame_id': msg.header.frame_id or 'map',
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'yaw': yaw_from_quaternion(pose.orientation.z, pose.orientation.w),
            'stamp': utc_now(),
            'source': 'initialpose',
        }
        self.store.update_last_pose(payload)
        self.latest_pose = payload

    def on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        payload = {
            'frame_id': msg.header.frame_id or 'odom',
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'yaw': yaw_from_quaternion(pose.orientation.z, pose.orientation.w),
            'stamp': utc_now(),
            'source': 'odom',
        }
        self.latest_pose = payload
        self.store.update_last_pose(payload)
        now_ns = self.get_clock().now().nanoseconds
        if self.last_pose_log_ns is None or now_ns - self.last_pose_log_ns >= self.pose_log_interval_ns:
            self.store.log_event('pose_snapshot', payload)
            self.last_pose_log_ns = now_ns

    def on_command(self, msg: String) -> None:
        event = parse_event(msg.data)
        event_type = event.get('type')

        if event_type == 'start_survey':
            survey_name = sanitize_name(event.get('survey_name', 'survey'))
            self.store.set_active_survey(survey_name)
            self.store.log_event('survey_started', {'survey_name': survey_name})
            self.status(f'memory: survey active = {survey_name}')
            return

        if event_type == 'stop_survey':
            active = self.store.read_state().get('active_survey')
            self.store.log_event('survey_stopped', {'survey_name': active})
            self.store.set_active_survey(None)
            self.status(f'memory: survey stopped ({active})')
            return

        if event_type == 'set_active_map':
            map_name = sanitize_name(event['map_name'])
            self.store.set_active_map(map_name)
            self.store.log_event('active_map_changed', {'map_name': map_name})
            self.status(f'memory: active map = {map_name}')
            return

        if event_type == 'save_survey_map':
            self.store.log_event('save_map_requested', {'map_name': sanitize_name(event['map_name'])})
            return

        if event_type == 'remember_current_pose':
            state = self.store.read_state()
            map_name = state.get('active_map') or 'unspecified_map'
            latest_pose = self.latest_pose or state.get('last_pose')
            if latest_pose is None:
                self.status('memory: cannot save place, no pose available yet')
                return
            place_name = sanitize_name(event['place'])
            self.store.remember_place(
                map_name,
                place_name,
                {
                    'frame_id': 'map',
                    'x': latest_pose['x'],
                    'y': latest_pose['y'],
                    'yaw': latest_pose['yaw'],
                    'saved_at': utc_now(),
                    'source': latest_pose.get('source', 'unknown'),
                },
            )
            self.store.log_event('place_saved', {'map_name': map_name, 'place_name': place_name})
            self.status(f'memory: saved place {place_name} in map {map_name}')
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MemoryManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
