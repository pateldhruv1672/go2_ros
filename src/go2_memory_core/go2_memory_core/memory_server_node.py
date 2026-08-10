from __future__ import annotations

import json
from typing import Callable

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from go2_memory_core_interfaces.srv import (
    ConnectLocations,
    ForgetMemory,
    GetResumeContext,
    PromoteMemory,
    QueryGraph,
    QueryMemory,
    WriteCheckpoint,
    WritePlace,
    WriteSpawn,
)

from .memory_api import UnifiedMemoryAPI


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'on'}
    return bool(value)


class MemoryServerNode(Node):
    def __init__(self) -> None:
        super().__init__('go2_memory_server')
        self.declare_parameter('session_root', '~/.ros/go2_semantic_nav_sessions')
        self.declare_parameter('enable_graph_memory', False)
        self.declare_parameter('enable_voxel_memory', False)
        self.declare_parameter('enable_vector_memory', False)
        self.declare_parameter('enable_legacy_exports', True)
        self.api = UnifiedMemoryAPI(
            session_root=self.get_parameter('session_root').value,
            enable_graph_memory=_as_bool(self.get_parameter('enable_graph_memory').value),
            enable_voxel_memory=_as_bool(self.get_parameter('enable_voxel_memory').value),
            enable_vector_memory=_as_bool(self.get_parameter('enable_vector_memory').value),
            enable_legacy_exports=_as_bool(self.get_parameter('enable_legacy_exports').value),
        )
        self.status_pub = self.create_publisher(String, '/go2_memory/status', 10)
        self.command_sub = self.create_subscription(String, '/go2_memory/command', self._on_command, 10)
        self.create_service(WriteSpawn, '/go2_memory/write_spawn', self._write_spawn)
        self.create_service(WriteCheckpoint, '/go2_memory/write_checkpoint', self._write_checkpoint)
        self.create_service(WritePlace, '/go2_memory/write_place', self._write_place)
        self.create_service(ConnectLocations, '/go2_memory/connect_locations', self._connect_locations)
        self.create_service(QueryMemory, '/go2_memory/query_memory', self._query_memory)
        self.create_service(QueryGraph, '/go2_memory/query_graph', self._query_graph)
        self.create_service(PromoteMemory, '/go2_memory/promote_memory', self._promote_memory)
        self.create_service(ForgetMemory, '/go2_memory/forget_memory', self._forget_memory)
        self.create_service(GetResumeContext, '/go2_memory/get_resume_context', self._get_resume_context)
        self.get_logger().info('Go2 memory core server ready')

    def _respond(self, response, call: Callable[[], dict], id_field: str = 'id'):
        try:
            result = call()
            response.success = bool(result.get('success', True))
            response.message = result.get('message', 'ok')
            if hasattr(response, 'checkpoint_id'):
                response.checkpoint_id = result.get(id_field, '')
            if hasattr(response, 'place_id'):
                response.place_id = result.get(id_field, '')
            if hasattr(response, 'spawn_id'):
                response.spawn_id = result.get(id_field, '')
            if hasattr(response, 'edge_id'):
                response.edge_id = result.get(id_field, '')
            if hasattr(response, 'result_json'):
                response.result_json = json.dumps(result, sort_keys=True)
            if hasattr(response, 'context_json'):
                response.context_json = json.dumps(result, sort_keys=True)
        except Exception as exc:  # pragma: no cover - ROS callback safety
            response.success = False
            response.message = str(exc)
            self.get_logger().error(str(exc))
        return response

    def _write_spawn(self, request, response):
        return self._respond(response, lambda: self.api.write_spawn(request.session_name, request.spawn_json))

    def _write_checkpoint(self, request, response):
        return self._respond(response, lambda: self.api.write_checkpoint(request.session_name, request.checkpoint_json))

    def _write_place(self, request, response):
        return self._respond(response, lambda: self.api.write_place(request.session_name, request.place_json))

    def _connect_locations(self, request, response):
        return self._respond(response, lambda: self.api.connect_locations(request.session_name, request.from_id, request.to_id, request.edge_json))

    def _query_memory(self, request, response):
        return self._respond(response, lambda: self.api.query_memory(request.session_name, request.query_json))

    def _query_graph(self, request, response):
        return self._respond(response, lambda: self.api.query_graph(request.session_name, request.query_json))

    def _promote_memory(self, request, response):
        return self._respond(response, lambda: self.api.promote_memory(request.session_name, request.memory_id, request.reason))

    def _forget_memory(self, request, response):
        return self._respond(response, lambda: self.api.forget_memory(request.session_name, request.memory_id, request.reason))

    def _get_resume_context(self, request, response):
        return self._respond(response, lambda: self.api.get_resume_context(request.session_name, request.request_json))

    def _on_command(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
            action = command.get('action')
            session_name = command.get('session_name', 'default')
            if action == 'write_checkpoint':
                result = self.api.write_checkpoint(session_name, command.get('data', {}))
            elif action == 'write_place':
                result = self.api.write_place(session_name, command.get('data', {}))
            elif action == 'write_spawn':
                result = self.api.write_spawn(session_name, command.get('data', {}))
            elif action == 'resume_context':
                result = self.api.get_resume_context(session_name, command.get('data', {}))
            elif action == 'write_room':
                result = self.api.write_room(session_name, command.get('data', {}))
            elif action == 'upsert_object':
                result = self.api.upsert_object_instance(session_name, command.get('data', {}))
            elif action == 'write_object_observation':
                result = self.api.write_object_observation(session_name, command.get('data', {}))
            elif action == 'query_objects':
                data = command.get('data', {}) or {}
                result = self.api.query_objects(session_name, label=str(data.get('label', '')), room=str(data.get('room', '')), confirmed_only=bool(data.get('confirmed_only', True)), limit=int(data.get('limit', 100)))
            elif action == 'count_objects':
                data = command.get('data', {}) or {}
                result = self.api.count_objects(session_name, label=str(data.get('label', '')), room=str(data.get('room', '')), confirmed_only=bool(data.get('confirmed_only', True)))
            elif action == 'write_fact':
                result = self.api.write_fact(session_name, command.get('data', {}))
            elif action == 'write_tour_stop':
                result = self.api.write_tour_stop(session_name, command.get('data', {}))
            elif action == 'world_snapshot':
                result = self.api.world_snapshot(session_name, limit=int((command.get('data', {}) or {}).get('limit', 100)))
            else:
                result = {'success': False, 'message': f'unknown action: {action}'}
        except Exception as exc:
            result = {'success': False, 'message': str(exc)}
        self.status_pub.publish(String(data=json.dumps(result, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MemoryServerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
