from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    local_speaker_enabled = LaunchConfiguration("local_speaker_enabled")
    local_speaker_backend = LaunchConfiguration("local_speaker_backend")
    agent_enabled = LaunchConfiguration("agent_enabled")
    enable_llm_debate = LaunchConfiguration("enable_llm_debate")
    debate_llm_provider = LaunchConfiguration("debate_llm_provider")
    debate_llm_model = LaunchConfiguration("debate_llm_model")
    debate_llm_timeout_sec = LaunchConfiguration("debate_llm_timeout_sec")
    enable_vlm_checkpointing = LaunchConfiguration("enable_vlm_checkpointing")
    vlm_provider = LaunchConfiguration("vlm_provider")
    vlm_model = LaunchConfiguration("vlm_model")
    vlm_write_period_sec = LaunchConfiguration("vlm_write_period_sec")
    vlm_auto_write_checkpoints = LaunchConfiguration("vlm_auto_write_checkpoints")
    camera_topic = LaunchConfiguration("camera_topic")
    require_confirmation_for_motion = LaunchConfiguration("require_confirmation_for_motion")
    session_root = LaunchConfiguration("session_root")
    session_name = LaunchConfiguration("session_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument("adapter_mode", default_value="transcript_only"),
            DeclareLaunchArgument("ble_device_name", default_value="Omi"),
            DeclareLaunchArgument("ble_device_address", default_value="EF:1C:34:C6:25:92"),
            DeclareLaunchArgument("tts_enabled", default_value="true"),
            DeclareLaunchArgument("local_speaker_enabled", default_value="true"),
            DeclareLaunchArgument("local_speaker_backend", default_value="auto"),
            DeclareLaunchArgument("agent_enabled", default_value="true"),
            DeclareLaunchArgument("enable_llm_debate", default_value="true"),
            DeclareLaunchArgument("debate_llm_provider", default_value="ollama"),
            DeclareLaunchArgument("debate_llm_model", default_value="qwen3:14b"),
            DeclareLaunchArgument("debate_llm_timeout_sec", default_value="8.0"),
            DeclareLaunchArgument("enable_vlm_checkpointing", default_value="true"),
            DeclareLaunchArgument("vlm_provider", default_value="ollama"),
            DeclareLaunchArgument("vlm_model", default_value="qwen3-vl:8b"),
            DeclareLaunchArgument("vlm_write_period_sec", default_value="30.0"),
            DeclareLaunchArgument("vlm_auto_write_checkpoints", default_value="false"),
            DeclareLaunchArgument("camera_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("require_confirmation_for_motion", default_value="true"),
            DeclareLaunchArgument("session_root", default_value="~/.ros/go2_semantic_nav_sessions"),
            DeclareLaunchArgument("session_name", default_value="default"),
            Node(
                package="go2_langgraph_agent",
                executable="main_supervisor",
                name="go2_langgraph_main_supervisor",
                output="screen",
                condition=IfCondition(agent_enabled),
                parameters=[
                    {
                        "session_root": session_root,
                        "session_name": session_name,
                        "enable_llm_debate": enable_llm_debate,
                        "debate_llm_provider": debate_llm_provider,
                        "debate_llm_model": debate_llm_model,
                        "debate_llm_timeout_sec": debate_llm_timeout_sec,
                        "enable_nav_publish": True,
                    }
                ],
            ),
            Node(
                package="go2_memory_core",
                executable="vlm_checkpoint_node",
                name="go2_vlm_checkpoint_node",
                output="screen",
                condition=IfCondition(enable_vlm_checkpointing),
                parameters=[
                    {
                        "session_root": session_root,
                        "session_name": session_name,
                        "camera_topic": camera_topic,
                        "write_period_sec": vlm_write_period_sec,
                        "auto_write_checkpoints": vlm_auto_write_checkpoints,
                        "vlm_provider": vlm_provider,
                        "vlm_model": vlm_model,
                    }
                ],
            ),
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
                parameters=[config_file, {"tts_enabled": tts_enabled, "local_speaker_enabled": local_speaker_enabled, "local_speaker_backend": local_speaker_backend}],
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
