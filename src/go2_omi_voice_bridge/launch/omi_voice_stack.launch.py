from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("go2_omi_voice_bridge")
    config_file = PathJoinSubstitution([package_share, "config", "omi_voice.yaml"])
    adapter_mode = LaunchConfiguration("adapter_mode")
    ble_device_name = LaunchConfiguration("ble_device_name")
    ble_device_address = LaunchConfiguration("ble_device_address")
    tts_enabled = LaunchConfiguration("tts_enabled")
    require_confirmation_for_motion = LaunchConfiguration("require_confirmation_for_motion")
    session_root = LaunchConfiguration("session_root")
    session_name = LaunchConfiguration("session_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument("adapter_mode", default_value="transcript_only"),
            DeclareLaunchArgument("ble_device_name", default_value="Omi"),
            DeclareLaunchArgument("ble_device_address", default_value="EF:1C:34:C6:25:92"),
            DeclareLaunchArgument("tts_enabled", default_value="true"),
            DeclareLaunchArgument("require_confirmation_for_motion", default_value="true"),
            DeclareLaunchArgument("session_root", default_value="~/.ros/go2_semantic_nav_sessions"),
            DeclareLaunchArgument("session_name", default_value="default"),
            Node(
                package="go2_omi_voice_bridge",
                executable="omi_ble_bridge_node",
                name="go2_omi_bridge",
                output="screen",
                parameters=[config_file, {"adapter_mode": adapter_mode, "ble_device_name": ble_device_name, "ble_device_address": ble_device_address}],
            ),
            Node(
                package="go2_omi_voice_bridge",
                executable="stt_node",
                name="go2_voice_stt_node",
                output="screen",
                parameters=[config_file, {"adapter_mode": adapter_mode, "ble_device_name": ble_device_name, "ble_device_address": ble_device_address}],
            ),
            Node(
                package="go2_omi_voice_bridge",
                executable="voice_intent_gate_node",
                name="go2_voice_intent_gate",
                output="screen",
                parameters=[
                    config_file,
                    {"require_confirmation_for_motion": require_confirmation_for_motion},
                ],
            ),
            Node(
                package="go2_omi_voice_bridge",
                executable="tts_node",
                name="go2_tts_node",
                output="screen",
                parameters=[config_file, {"tts_enabled": tts_enabled}],
            ),
            Node(
                package="go2_omi_voice_bridge",
                executable="tour_voice_command_router",
                name="go2_tour_voice_command_router",
                output="screen",
                parameters=[config_file, {"session_root": session_root, "session_name": session_name}],
            ),
        ]
    )
