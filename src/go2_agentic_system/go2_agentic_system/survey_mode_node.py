from __future__ import annotations

from pathlib import Path
from typing import Optional

import rclpy
from nav2_msgs.srv import SaveMap
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .command_utils import parse_event
from .storage import MemoryStore, sanitize_name


class SurveyModeNode(Node):
    def __init__(self) -> None:
        super().__init__('survey_mode_node')
        self.declare_parameter('storage_root', '~/.ros/go2_agent_memory')
        self.declare_parameter('command_topic', '/agent/commands')
        self.declare_parameter('status_topic', '/agent/status')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('image_format', 'pgm')
        self.declare_parameter('map_mode', 'trinary')
        self.declare_parameter('free_thresh', 0.25)
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('save_map_service', '/map_saver/save_map')
        self.declare_parameter('default_map_name', 'building_map')
        self.declare_parameter('default_posegraph_prefix', '')

        self.store = MemoryStore(self.get_parameter('storage_root').value)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.command_sub = self.create_subscription(String, self.get_parameter('command_topic').value, self.on_command, 10)
        self.save_cli = self.create_client(SaveMap, self.get_parameter('save_map_service').value)

        self.start_srv = self.create_service(Trigger, '/agent/survey/start', self.handle_start)
        self.stop_srv = self.create_service(Trigger, '/agent/survey/stop', self.handle_stop)
        self.save_srv = self.create_service(Trigger, '/agent/survey/save_map', self.handle_save)

    def status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def start_survey(self, survey_name: str) -> None:
        survey_name = sanitize_name(survey_name)
        self.store.set_active_survey(survey_name)
        self.store.log_event('survey_started', {'survey_name': survey_name})
        self.status(f'survey: active survey {survey_name}')

    def stop_survey(self) -> None:
        active = self.store.read_state().get('active_survey')
        self.store.set_active_survey(None)
        self.store.log_event('survey_stopped', {'survey_name': active})
        self.status(f'survey: stopped {active}')

    def save_map(self, requested_name: Optional[str]) -> None:
        map_name = sanitize_name(requested_name or self.get_parameter('default_map_name').value)
        map_dir = self.store.paths.maps / map_name
        map_dir.mkdir(parents=True, exist_ok=True)
        map_stem = map_dir / map_name

        if not self.save_cli.wait_for_service(timeout_sec=2.0):
            self.status('survey: /map_saver/save_map not available. Launch map_saver_server or save via the SlamToolbox RViz plugin.')
            return

        request = SaveMap.Request()
        request.map_topic = self.get_parameter('map_topic').value
        request.map_url = f'file://{map_stem}'
        request.image_format = self.get_parameter('image_format').value
        request.map_mode = self.get_parameter('map_mode').value
        request.free_thresh = float(self.get_parameter('free_thresh').value)
        request.occupied_thresh = float(self.get_parameter('occupied_thresh').value)

        self.status(f'survey: saving map to {map_stem}')
        future = self.save_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done() or future.result() is None:
            self.status('survey: save_map timed out or returned no result')
            return

        result = future.result()
        if not result.result:
            self.status('survey: save_map service returned failure')
            return

        map_yaml = str(Path(f'{map_stem}.yaml'))
        posegraph_prefix = self.get_parameter('default_posegraph_prefix').value or str(map_stem)
        self.store.register_map_artifacts(map_name, map_yaml, posegraph_prefix)
        self.store.log_event('map_saved', {'map_name': map_name, 'map_yaml': map_yaml, 'posegraph_prefix': posegraph_prefix})
        self.status(f'survey: map saved as {map_name}')

    def handle_start(self, request, response):
        self.start_survey('manual_triggered_survey')
        response.success = True
        response.message = 'survey started'
        return response

    def handle_stop(self, request, response):
        self.stop_survey()
        response.success = True
        response.message = 'survey stopped'
        return response

    def handle_save(self, request, response):
        self.save_map(None)
        response.success = True
        response.message = 'save requested'
        return response

    def on_command(self, msg: String) -> None:
        event = parse_event(msg.data)
        event_type = event.get('type')
        if event_type == 'start_survey':
            self.start_survey(event.get('survey_name', 'survey'))
        elif event_type == 'stop_survey':
            self.stop_survey()
        elif event_type == 'save_survey_map':
            self.save_map(event.get('map_name'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SurveyModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
