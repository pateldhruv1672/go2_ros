from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("state_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("state_port", default_value="15001"),
        DeclareLaunchArgument("cmd_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("cmd_port", default_value="15000"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("publish_tf", default_value="true"),

        Node(
            package="go2_isaac_bridge",
            executable="isaac_udp_ros_bridge",
            name="go2_isaac_udp_ros_bridge",
            output="screen",
            parameters=[{
                "state_host": LaunchConfiguration("state_host"),
                "state_port": LaunchConfiguration("state_port"),
                "cmd_host": LaunchConfiguration("cmd_host"),
                "cmd_port": LaunchConfiguration("cmd_port"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "publish_tf": LaunchConfiguration("publish_tf"),
            }],
        ),
    ])
