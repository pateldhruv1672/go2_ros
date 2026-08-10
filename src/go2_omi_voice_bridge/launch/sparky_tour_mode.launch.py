from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    session_root = LaunchConfiguration("session_root")
    session_name = LaunchConfiguration("session_name")
    rviz2 = LaunchConfiguration("rviz2")
    adapter_mode = LaunchConfiguration("adapter_mode")
    ble_device_address = LaunchConfiguration("ble_device_address")
    require_confirmation_for_motion = LaunchConfiguration("require_confirmation_for_motion")
    enable_phone_web = LaunchConfiguration("enable_phone_web")
    phone_web_bind_host = LaunchConfiguration("phone_web_bind_host")
    phone_web_port = LaunchConfiguration("phone_web_port")
    phone_web_require_token = LaunchConfiguration("phone_web_require_token")

    resume_voice = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("go2_omi_voice_bridge"),
                "launch",
                "sparky_voice_resume.launch.py",
            ])
        ),
        launch_arguments={
            "session_root": session_root,
            "session_name": session_name,
            "rviz2": rviz2,
            "adapter_mode": adapter_mode,
            "ble_device_address": ble_device_address,
            "require_confirmation_for_motion": require_confirmation_for_motion,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("session_root", default_value="~/.ros/go2_semantic_nav_sessions"),
        DeclareLaunchArgument("session_name", default_value="auto"),
        DeclareLaunchArgument("rviz2", default_value="true"),
        DeclareLaunchArgument("adapter_mode", default_value="ble_audio"),
        DeclareLaunchArgument("ble_device_address", default_value="EF:1C:34:C6:25:92"),
        DeclareLaunchArgument("require_confirmation_for_motion", default_value="true"),
        DeclareLaunchArgument("enable_phone_web", default_value="false"),
        DeclareLaunchArgument("phone_web_bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("phone_web_port", default_value="8765"),
        DeclareLaunchArgument("phone_web_require_token", default_value="true"),
        resume_voice,
        Node(
            package="go2_omi_voice_bridge",
            executable="phone_web_gateway_node",
            name="go2_phone_web_gateway",
            output="screen",
            condition=IfCondition(enable_phone_web),
            parameters=[{
                "bind_host": phone_web_bind_host,
                "port": ParameterValue(phone_web_port, value_type=int),
                "require_token": ParameterValue(phone_web_require_token, value_type=bool),
            }],
        ),
    ])
