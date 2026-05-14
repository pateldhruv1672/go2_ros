from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from std_msgs.msg import String

from .command_utils import parse_event
from .storage import MemoryStore


class NavigationAgentNode(Node):
    def __init__(self) -> None:
        super().__init__('navigation_agent_node')
        self.declare_parameter('storage_root', '~/.ros/go2_agent_memory')
        self.declare_parameter('command_topic', '/agent/commands')
        self.declare_parameter('status_topic', '/agent/status')
        self.declare_parameter('active_map_override', '')
        self.declare_parameter('autostart_nav2_wait', False)
        self.declare_parameter('feedback_period_sec', 1.0)

        self.store = MemoryStore(self.get_parameter('storage_root').value)
        self.command_sub = self.create_subscription(String, self.get_parameter('command_topic').value, self.on_command, 20)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.navigator = BasicNavigator()
        self.feedback_timer: Optional[rclpy.timer.Timer] = None
        self._task_active = False

        if self.get_parameter('autostart_nav2_wait').value:
            self.get_logger().info('waiting for Nav2 to become active...')
            self.navigator.waitUntilNav2Active()

        period = float(self.get_parameter('feedback_period_sec').value)
        self.feedback_timer = self.create_timer(period, self.check_feedback)

    def status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def active_map(self) -> Optional[str]:
        override = self.get_parameter('active_map_override').value
        if override:
            return override
        return self.store.read_state().get('active_map')

    def make_pose(self, frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose

    def on_command(self, msg: String) -> None:
        event = parse_event(msg.data)
        event_type = event.get('type')

        if event_type == 'cancel_navigation':
            self.navigator.cancelTask()
            self._task_active = False
            self.status('nav: cancel requested')
            return

        if event_type != 'navigate_named_place':
            return

        map_name = self.active_map()
        if not map_name:
            self.status('nav: no active map configured')
            return

        place = event['place']
        target = self.store.get_place(map_name, place)
        if target is None:
            self.status(f'nav: unknown place "{place}" in map {map_name}')
            return

        goal = self.make_pose(target.get('frame_id', 'map'), target['x'], target['y'], target.get('yaw', 0.0))
        self.navigator.goToPose(goal)
        self._task_active = True
        self.store.log_event('navigate_named_place', {'map_name': map_name, 'place': place, 'target': target})
        self.status(f'nav: goal sent for {place} on map {map_name}')

    def check_feedback(self) -> None:
        if not self._task_active:
            return
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.status('nav: goal succeeded')
                self.store.log_event('navigation_result', {'result': 'succeeded'})
            elif result == TaskResult.CANCELED:
                self.status('nav: goal canceled')
                self.store.log_event('navigation_result', {'result': 'canceled'})
            else:
                self.status('nav: goal failed')
                self.store.log_event('navigation_result', {'result': 'failed'})
            self._task_active = False
            return

        feedback = self.navigator.getFeedback()
        if feedback is None:
            return
        distance = getattr(feedback, 'distance_remaining', float('nan'))
        eta = getattr(feedback, 'estimated_time_remaining', None)
        eta_sec = getattr(eta, 'sec', None)
        self.status(f'nav: progress distance_remaining={distance:.2f}m eta={eta_sec}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.destroyNode()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
