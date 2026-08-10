#!/usr/bin/env python3

import time
from collections import Counter

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node


class Nav2ReadinessActivator(Node):
    def __init__(self):
        super().__init__("nav2_readiness_activator")

        self.declare_parameter("startup_timeout_sec", 90.0)
        self.timeout_sec = float(
            self.get_parameter("startup_timeout_sec").value
        )

        self.core_nodes = [
            "/controller_server",
            "/planner_server",
            "/behavior_server",
            "/bt_navigator",
            "/waypoint_follower",
        ]

        self.critical_duplicate_names = {
            "/go2_driver_node",
            "/go2_robot_state_publisher",
            "/go2_pointcloud_to_laserscan",
            "/lidar_to_pointcloud",
            "/slam_toolbox",
            "/controller_server",
            "/planner_server",
            "/behavior_server",
            "/bt_navigator",
            "/waypoint_follower",
            "/collision_monitor",
            "/lifecycle_manager_navigation",
        }

    @staticmethod
    def full_name(name, namespace):
        namespace = namespace.rstrip("/")
        return f"{namespace}/{name}" if namespace else f"/{name}"

    def detect_critical_duplicates(self):
        names = [
            self.full_name(name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
        ]
        counts = Counter(names)

        return {
            name: count
            for name, count in counts.items()
            if count > 1 and name in self.critical_duplicate_names
        }

    def call_service(self, client, request, timeout_sec=10.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_sec,
        )

        if not future.done():
            return None

        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f"Service call failed: {exc}")
            return None

    def get_state(self, node_name):
        client = self.create_client(
            GetState,
            f"{node_name}/get_state",
        )

        response = self.call_service(
            client,
            GetState.Request(),
            timeout_sec=5.0,
        )

        if response is None:
            return None, "unavailable"

        state = response.current_state
        return int(state.id), str(state.label)

    def change_state(self, node_name, transition_id):
        client = self.create_client(
            ChangeState,
            f"{node_name}/change_state",
        )

        request = ChangeState.Request()
        request.transition.id = int(transition_id)

        response = self.call_service(
            client,
            request,
            timeout_sec=15.0,
        )

        return bool(response and response.success)

    def wait_for_scan_nav(self):
        deadline = time.monotonic() + self.timeout_sec

        while rclpy.ok() and time.monotonic() < deadline:
            publishers = self.get_publishers_info_by_topic("/scan_nav")
            if publishers:
                self.get_logger().info(
                    f"/scan_nav has {len(publishers)} publisher(s)"
                )
                return True

            self.get_logger().info("Waiting for /scan_nav publisher...")
            rclpy.spin_once(self, timeout_sec=1.0)

        return False

    def start_nav2_manager(self):
        states = [self.get_state(node)[0] for node in self.core_nodes]

        if states and all(
            state == State.PRIMARY_STATE_ACTIVE for state in states
        ):
            self.get_logger().info("Nav2 core nodes already active")
            return True

        client = self.create_client(
            ManageLifecycleNodes,
            "/lifecycle_manager_navigation/manage_nodes",
        )

        request = ManageLifecycleNodes.Request()
        request.command = 0  # STARTUP

        response = self.call_service(
            client,
            request,
            timeout_sec=40.0,
        )

        if response is None:
            self.get_logger().error(
                "Lifecycle manager startup service unavailable"
            )
            return False

        # Some manager versions return false if nodes were partly configured.
        # Recheck actual states instead of trusting only the response.
        deadline = time.monotonic() + 30.0

        while rclpy.ok() and time.monotonic() < deadline:
            state_data = [self.get_state(node) for node in self.core_nodes]
            self.get_logger().info(
                "Nav2 states: "
                + ", ".join(
                    f"{node}={label}"
                    for node, (_, label) in zip(
                        self.core_nodes,
                        state_data,
                    )
                )
            )

            if all(
                state_id == State.PRIMARY_STATE_ACTIVE
                for state_id, _ in state_data
            ):
                return True

            rclpy.spin_once(self, timeout_sec=1.0)

        return False

    def activate_collision_monitor(self):
        state_id, label = self.get_state("/collision_monitor")
        self.get_logger().info(
            f"collision_monitor initial state: {label}"
        )

        if state_id == State.PRIMARY_STATE_UNCONFIGURED:
            if not self.change_state(
                "/collision_monitor",
                Transition.TRANSITION_CONFIGURE,
            ):
                self.get_logger().error(
                    "Could not configure collision_monitor"
                )
                return False

        state_id, label = self.get_state("/collision_monitor")

        if state_id == State.PRIMARY_STATE_INACTIVE:
            if not self.change_state(
                "/collision_monitor",
                Transition.TRANSITION_ACTIVATE,
            ):
                self.get_logger().error(
                    "Could not activate collision_monitor"
                )
                return False

        state_id, label = self.get_state("/collision_monitor")
        self.get_logger().info(
            f"collision_monitor final state: {label}"
        )

        return state_id == State.PRIMARY_STATE_ACTIVE

    def verify_scan_consumers(self):
        deadline = time.monotonic() + 20.0

        while rclpy.ok() and time.monotonic() < deadline:
            endpoints = self.get_subscriptions_info_by_topic("/scan_nav")

            names = [
                self.full_name(info.node_name, info.node_namespace)
                for info in endpoints
            ]

            has_collision_monitor = any(
                "collision_monitor" in name for name in names
            )

            # The local costmap subscription usually belongs to
            # controller_server or is named local_costmap.
            has_costmap = any(
                "controller_server" in name or "local_costmap" in name
                for name in names
            )

            if has_collision_monitor and has_costmap:
                self.get_logger().info(
                    "/scan_nav safety consumers connected: "
                    + ", ".join(names)
                )
                return True

            self.get_logger().info(
                "Waiting for /scan_nav consumers; currently: "
                + (", ".join(names) if names else "none")
            )
            rclpy.spin_once(self, timeout_sec=1.0)

        return False

    def run(self):
        if not self.wait_for_scan_nav():
            self.get_logger().error(
                "No /scan_nav publisher; refusing to activate navigation"
            )
            return 2

        duplicates = self.detect_critical_duplicates()
        if duplicates:
            self.get_logger().error(
                "Critical duplicate nodes detected; navigation will remain "
                f"inactive: {duplicates}"
            )
            return 3

        if not self.start_nav2_manager():
            self.get_logger().error(
                "Nav2 core nodes failed to reach active state"
            )
            return 4

        if not self.activate_collision_monitor():
            self.get_logger().error(
                "Collision monitor failed to reach active state"
            )
            return 5

        if not self.verify_scan_consumers():
            self.get_logger().error(
                "/scan_nav is not connected to both collision monitor "
                "and local costmap"
            )
            return 6

        self.get_logger().info(
            "NAVIGATION READY: lifecycle active, collision monitor active, "
            "scan consumers connected, no critical duplicates"
        )
        return 0


def main():
    rclpy.init()
    node = Nav2ReadinessActivator()

    try:
        result = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(result)


if __name__ == "__main__":
    main()
