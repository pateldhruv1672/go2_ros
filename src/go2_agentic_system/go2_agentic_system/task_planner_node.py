from __future__ import annotations

from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .command_utils import extract_destination, extract_label, make_event, normalize_text


class TaskPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('task_planner_node')
        self.declare_parameter('input_topic', '/agent/voice_text')
        self.declare_parameter('command_topic', '/agent/commands')
        self.declare_parameter('status_topic', '/agent/status')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value

        self.command_pub = self.create_publisher(String, command_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.subscription = self.create_subscription(String, input_topic, self.handle_voice_text, 10)

    def publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def handle_voice_text(self, msg: String) -> None:
        text = normalize_text(msg.data)
        if not text:
            return

        destination = extract_destination(text)
        label = extract_label(text)

        if destination:
            self.command_pub.publish(String(data=make_event('navigate_named_place', place=destination)))
            self.publish_status(f'planner: navigate request for {destination}')
            return

        if 'stop survey mode' in text or 'exit survey mode' in text:
            self.command_pub.publish(String(data=make_event('stop_survey')))
            self.publish_status('planner: stop survey request')
            return

        if 'start survey mode' in text or 'start mapping' in text or 'survey mode' in text:
            survey_name = datetime.now().strftime('survey_%Y%m%d_%H%M%S')
            self.command_pub.publish(String(data=make_event('start_survey', survey_name=survey_name)))
            self.publish_status(f'planner: start survey {survey_name}')
            return

        if text.startswith('stop') or text == 'cancel':
            self.command_pub.publish(String(data=make_event('cancel_navigation')))
            self.publish_status('planner: cancel navigation request')
            return

        if label and ('save this place as' in text or 'remember this place as' in text or 'mark this location as' in text or 'name this location' in text):
            self.command_pub.publish(String(data=make_event('remember_current_pose', place=label)))
            self.publish_status(f'planner: save current pose as {label}')
            return

        if label and ('save map as' in text or 'save the map as' in text or 'store map as' in text):
            self.command_pub.publish(String(data=make_event('save_survey_map', map_name=label)))
            self.publish_status(f'planner: save map as {label}')
            return

        if text.startswith('set active map to '):
            map_name = text.replace('set active map to ', '', 1).strip()
            self.command_pub.publish(String(data=make_event('set_active_map', map_name=map_name)))
            self.publish_status(f'planner: switch active map to {map_name}')
            return

        self.publish_status(f'planner: no rule matched for "{text}"')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
