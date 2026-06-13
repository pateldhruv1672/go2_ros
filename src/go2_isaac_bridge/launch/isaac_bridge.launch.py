from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("publish_clear_scan", default_value="false"),
        Node(
            package="go2_isaac_bridge",
            executable="go2_isaac_bridge_node",
            name="go2_isaac_bridge_node",
            output="screen",
            parameters=[{
                "publish_clear_scan": LaunchConfiguration("publish_clear_scan"),
                "cmd_topic": "/cmd_vel_out",
                "scan_topic": "/scan",
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
                "scan_frame_id": "base_scan",
                "camera_frame_id": "camera_link",
                "imu_frame_id": "imu_link",
            }],
        ),
    ])
