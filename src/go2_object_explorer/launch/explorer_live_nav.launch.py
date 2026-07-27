import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_robot_sdk_dir = get_package_share_directory("go2_robot_sdk")
    explorer_dir = get_package_share_directory("go2_object_explorer")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    nav2_launch = os.path.join(
        go2_robot_sdk_dir,
        "launch",
        "navigation_no_docking.launch.py",
    )

    slam_launch = os.path.join(
        slam_toolbox_dir,
        "launch",
        "online_async_launch.py",
    )

    slam_params = os.path.join(
        go2_robot_sdk_dir,
        "config",
        "mapper_params_online_async.yaml",
    )

    nav2_params = os.path.join(
        explorer_dir,
        "config",
        "nav2_params_live_explorer.yaml",
    )

    motion_arbiter = Node(
        package="go2_nav_tools",
        executable="motion_arbiter",
        name="go2_motion_arbiter",
        output="screen",
        parameters=[
            {
                "nav2_topic": "/cmd_vel_nav2",
                "omi_topic": "/cmd_vel_omi",
                "escape_topic": "/cmd_vel_escape",
                "output_topic": "/cmd_vel_nav",
                "source_timeout_sec": 0.40,
                "nav2_max_x": 0.75,
                "nav2_max_y": 0.0,
                "nav2_max_theta": 0.90,
                "omi_max_x": 0.25,
                "omi_max_y": 0.20,
                "omi_max_theta": 0.60,
                "escape_max_x": 0.30,
                "escape_max_y": 0.25,
                "escape_max_theta": 0.70,
            }
        ],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": "false",
        }.items(),
    )

    padded_map = Node(
        package="go2_object_explorer",
        executable="padded_map_node",
        name="padded_map_node",
        output="screen",
        parameters=[
            {
                "input_map_topic": "/map",
                "output_map_topic": "/map_padded",
                "padding_m": LaunchConfiguration("padding_m"),
            }
        ],
    )

    nav2 = TimerAction(
        period=LaunchConfiguration("nav2_start_delay_sec"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "params_file": nav2_params,
                    "use_sim_time": "false",
                    "autostart": "true",
                    "use_composition": "False",
                    "use_respawn": "False",
                    "log_level": "info",
                }.items(),
            )
        ],
    )

    scan_retimestamp = Node(

        package="go2_object_explorer",

        executable="scan_retimestamp_node",

        name="object_explorer_scan_retimestamp_node",

        output="screen",

        respawn=True,

        respawn_delay=2.0,

        parameters=[

            {

                "input_topic": "/scan",

                "output_topic": "/scan_nav",

                "frame_id": "",

                "stamp_offset_sec": -0.08,

            }

        ],

    )


    return LaunchDescription(
        [
            
        scan_retimestamp,
DeclareLaunchArgument("padding_m", default_value="3.0"),
            DeclareLaunchArgument("nav2_start_delay_sec", default_value="30.0"),
            motion_arbiter,
            slam,
            padded_map,
            nav2,
        ]
    )
