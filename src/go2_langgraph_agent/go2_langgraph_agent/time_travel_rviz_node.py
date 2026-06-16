from __future__ import annotations

from typing import Any, Dict, List, Optional
from datetime import datetime, timezone
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def _json_or_text(value: str) -> Any:
    try:
        return json.loads(value)
    except Exception:
        return value


class TimeTravelRvizNode(Node):
    """RViz-friendly checkpoint selector and replay bridge.

    This is not a custom Qt RViz panel. It is a ROS/RViz tool node that publishes
    checkpoint markers and accepts selection/replay commands over ROS topics.
    If interactive_markers is installed, the markers also become clickable in
    RViz's InteractiveMarkers display; otherwise normal MarkerArray labels are
    still shown.

    Topics:
      /go2_agent/checkpoints/request  String JSON request emitted periodically
      /go2_agent/checkpoints          String JSON checkpoint list input
      /go2_agent/time_travel_markers  MarkerArray visual checkpoint timeline
      /go2_agent/time_travel_select   String checkpoint id, index, or JSON
      /go2_agent/time_travel          String JSON replay command to supervisor
      /go2_agent/time_travel_selected String selected checkpoint status
    """

    def __init__(self) -> None:
        super().__init__("go2_time_travel_rviz")
        self.declare_parameter("request_period_sec", 5.0)
        self.declare_parameter("checkpoint_limit", 25)
        self.declare_parameter("marker_frame", "map")
        self.declare_parameter("timeline_origin_x", 0.0)
        self.declare_parameter("timeline_origin_y", -2.0)
        self.declare_parameter("timeline_spacing_m", 0.35)
        self.declare_parameter("auto_replay_on_select", False)
        self.declare_parameter("default_replay_command", "")

        self.checkpoints: List[Dict[str, Any]] = []
        self.selected_checkpoint_id: str = ""
        self.request_pub = self.create_publisher(String, "/go2_agent/checkpoints/request", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/go2_agent/time_travel_markers", 10)
        self.time_travel_pub = self.create_publisher(String, "/go2_agent/time_travel", 10)
        self.selected_pub = self.create_publisher(String, "/go2_agent/time_travel_selected", 10)
        self.create_subscription(String, "/go2_agent/checkpoints", self._on_checkpoints, 10)
        self.create_subscription(String, "/go2_agent/time_travel_select", self._on_select, 10)
        self.create_timer(float(self.get_parameter("request_period_sec").value), self._request_checkpoints)
        self.interactive_server = self._make_interactive_server()
        self.get_logger().info("Time-travel RViz tool ready: publishes checkpoint markers and replay bridge topics")

    def _make_interactive_server(self) -> Any:
        try:
            from interactive_markers.interactive_marker_server import InteractiveMarkerServer  # type: ignore
            return InteractiveMarkerServer(self, "go2_time_travel_checkpoints")
        except Exception:
            return None

    def _request_checkpoints(self) -> None:
        payload = {"limit": int(self.get_parameter("checkpoint_limit").value), "source": "time_travel_rviz"}
        self.request_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _on_checkpoints(self, msg: String) -> None:
        payload = _json_or_text(msg.data)
        if not isinstance(payload, dict):
            return
        checkpoints = payload.get("checkpoints") or []
        if not isinstance(checkpoints, list):
            return
        self.checkpoints = checkpoints
        self._publish_markers()
        self._publish_interactive_markers()

    def _checkpoint_id(self, item: Dict[str, Any]) -> str:
        if item.get("checkpoint_id"):
            return str(item.get("checkpoint_id"))
        cfg = item.get("config") or {}
        if isinstance(cfg, dict):
            return str(((cfg.get("configurable") or {}).get("checkpoint_id")) or "")
        return ""

    def _label(self, index: int, item: Dict[str, Any]) -> str:
        checkpoint_id = self._checkpoint_id(item)
        metadata = item.get("metadata") or {}
        step = metadata.get("step") or metadata.get("source") or "checkpoint"
        return f"{index}: {step}\n{checkpoint_id[:10]}"

    def _marker_position(self, index: int) -> Point:
        origin_x = float(self.get_parameter("timeline_origin_x").value)
        origin_y = float(self.get_parameter("timeline_origin_y").value)
        spacing = float(self.get_parameter("timeline_spacing_m").value)
        p = Point()
        p.x = origin_x + spacing * index
        p.y = origin_y + 0.15 * math.sin(index * 0.5)
        p.z = 0.25
        return p

    def _publish_markers(self) -> None:
        frame = str(self.get_parameter("marker_frame").value)
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = frame
        clear.header.stamp = now
        clear.ns = "go2_time_travel"
        clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, item in enumerate(self.checkpoints):
            checkpoint_id = self._checkpoint_id(item)
            sphere = Marker()
            sphere.header.frame_id = frame
            sphere.header.stamp = now
            sphere.ns = "go2_time_travel_nodes"
            sphere.id = i + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = self._marker_position(i)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.16
            sphere.scale.y = 0.16
            sphere.scale.z = 0.16
            selected = checkpoint_id and checkpoint_id == self.selected_checkpoint_id
            sphere.color.r = 0.1 if not selected else 1.0
            sphere.color.g = 0.7 if not selected else 0.5
            sphere.color.b = 1.0 if not selected else 0.0
            sphere.color.a = 0.9
            arr.markers.append(sphere)

            text = Marker()
            text.header.frame_id = frame
            text.header.stamp = now
            text.ns = "go2_time_travel_labels"
            text.id = 1000 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = self._marker_position(i)
            text.pose.position.z += 0.25
            text.pose.orientation.w = 1.0
            text.scale.z = 0.12
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.95
            text.text = self._label(i, item)
            arr.markers.append(text)
        self.marker_pub.publish(arr)

    def _publish_interactive_markers(self) -> None:
        if self.interactive_server is None:
            return
        try:
            from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl  # type: ignore
            self.interactive_server.clear()
            frame = str(self.get_parameter("marker_frame").value)
            for i, item in enumerate(self.checkpoints):
                checkpoint_id = self._checkpoint_id(item)
                if not checkpoint_id:
                    continue
                marker = InteractiveMarker()
                marker.header.frame_id = frame
                marker.name = checkpoint_id
                marker.description = self._label(i, item)
                marker.pose.position = self._marker_position(i)
                marker.scale = 0.35
                control = InteractiveMarkerControl()
                control.interaction_mode = InteractiveMarkerControl.BUTTON
                control.always_visible = True
                control.name = "select_checkpoint"
                visual = Marker()
                visual.type = Marker.SPHERE
                visual.scale.x = 0.16
                visual.scale.y = 0.16
                visual.scale.z = 0.16
                visual.color.r = 0.2
                visual.color.g = 0.9
                visual.color.b = 1.0
                visual.color.a = 0.9
                control.markers.append(visual)
                marker.controls.append(control)
                self.interactive_server.insert(marker, self._on_interactive_feedback)
            self.interactive_server.applyChanges()
        except Exception as exc:
            self.get_logger().warn(f"Interactive marker update failed, normal MarkerArray remains available: {exc}")

    def _on_interactive_feedback(self, feedback: Any) -> None:
        checkpoint_id = str(getattr(feedback, "marker_name", ""))
        if checkpoint_id:
            self._select(checkpoint_id, source="interactive_marker")

    def _on_select(self, msg: String) -> None:
        payload = _json_or_text(msg.data)
        command = ""
        replay = bool(self.get_parameter("auto_replay_on_select").value)
        checkpoint_id = ""
        if isinstance(payload, dict):
            checkpoint_id = str(payload.get("checkpoint_id") or "")
            if not checkpoint_id and "index" in payload:
                checkpoint_id = self._checkpoint_id(self.checkpoints[int(payload["index"])])
            command = str(payload.get("command") or self.get_parameter("default_replay_command").value or "")
            replay = bool(payload.get("replay", replay))
        else:
            text = str(payload).strip()
            if text.isdigit():
                idx = int(text)
                if 0 <= idx < len(self.checkpoints):
                    checkpoint_id = self._checkpoint_id(self.checkpoints[idx])
            else:
                checkpoint_id = text
        if checkpoint_id:
            self._select(checkpoint_id, command=command, replay=replay, source="topic")

    def _select(self, checkpoint_id: str, command: str = "", replay: bool = False, source: str = "unknown") -> None:
        self.selected_checkpoint_id = checkpoint_id
        selected = {
            "checkpoint_id": checkpoint_id,
            "selected_at": datetime.now(timezone.utc).isoformat(),
            "source": source,
            "replay": replay,
            "command": command,
        }
        self.selected_pub.publish(String(data=json.dumps(selected, sort_keys=True)))
        self._publish_markers()
        if replay:
            payload = {"checkpoint_id": checkpoint_id}
            if command:
                payload["command"] = command
            self.time_travel_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TimeTravelRvizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
