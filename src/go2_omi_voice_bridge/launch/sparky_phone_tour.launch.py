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
    restore_spawn_on_start = LaunchConfiguration("restore_spawn_on_start")
    nav2_start_delay_sec = LaunchConfiguration("nav2_start_delay_sec")
    scan_input_topic = LaunchConfiguration("scan_input_topic")
    scan_nav_topic = LaunchConfiguration("scan_nav_topic")
    scan_frame_id = LaunchConfiguration("scan_frame_id")
    scan_stamp_offset_sec = LaunchConfiguration("scan_stamp_offset_sec")

    require_confirmation_for_motion = LaunchConfiguration("require_confirmation_for_motion")
    enable_llm_debate = LaunchConfiguration("enable_llm_debate")
    debate_llm_provider = LaunchConfiguration("debate_llm_provider")
    debate_llm_model = LaunchConfiguration("debate_llm_model")
    debate_llm_timeout_sec = LaunchConfiguration("debate_llm_timeout_sec")
    enable_vlm_checkpointing = LaunchConfiguration("enable_vlm_checkpointing")
    vlm_provider = LaunchConfiguration("vlm_provider")
    vlm_model = LaunchConfiguration("vlm_model")
    vlm_write_period_sec = LaunchConfiguration("vlm_write_period_sec")
    vlm_auto_write_checkpoints = LaunchConfiguration("vlm_auto_write_checkpoints")
    vlm_prompt = LaunchConfiguration("vlm_prompt")
    camera_topic = LaunchConfiguration("camera_topic")
    tts_enabled = LaunchConfiguration("tts_enabled")
    local_speaker_enabled = LaunchConfiguration("local_speaker_enabled")
    local_speaker_backend = LaunchConfiguration("local_speaker_backend")

    enable_phone_web = LaunchConfiguration("enable_phone_web")
    phone_web_bind_host = LaunchConfiguration("phone_web_bind_host")
    phone_web_port = LaunchConfiguration("phone_web_port")
    phone_web_require_token = LaunchConfiguration("phone_web_require_token")

    enable_perception_tools = LaunchConfiguration("enable_perception_tools")
    enable_dynamic_obstacle_tracking = LaunchConfiguration("enable_dynamic_obstacle_tracking")
    enable_open_vocab_detector = LaunchConfiguration("enable_open_vocab_detector")
    open_vocab_backend = LaunchConfiguration("open_vocab_backend")
    open_vocab_model = LaunchConfiguration("open_vocab_model")

    config_file = PathJoinSubstitution([
        FindPackageShare("go2_omi_voice_bridge"), "config", "omi_voice.yaml"
    ])

    semantic_resume = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("go2_semantic_nav_agent"),
                "launch",
                "semantic_nav_resume.launch.py",
            ])
        ),
        launch_arguments={
            "session_root": session_root,
            "session_name": session_name,
            "rviz2": rviz2,
            "restore_spawn_on_start": restore_spawn_on_start,
            "nav2_start_delay_sec": nav2_start_delay_sec,
            "scan_input_topic": scan_input_topic,
            "scan_nav_topic": scan_nav_topic,
            "scan_frame_id": scan_frame_id,
            "scan_stamp_offset_sec": scan_stamp_offset_sec,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("session_root", default_value="~/.ros/go2_semantic_nav_sessions"),
        DeclareLaunchArgument("session_name", default_value="auto"),
        DeclareLaunchArgument("rviz2", default_value="true"),
        DeclareLaunchArgument("restore_spawn_on_start", default_value="true"),
        DeclareLaunchArgument("nav2_start_delay_sec", default_value="8.0"),
        DeclareLaunchArgument("scan_input_topic", default_value="/scan"),
        DeclareLaunchArgument("scan_nav_topic", default_value="/scan_nav"),
        DeclareLaunchArgument("scan_frame_id", default_value="base_link"),
        DeclareLaunchArgument("scan_stamp_offset_sec", default_value="0.25"),

        DeclareLaunchArgument("require_confirmation_for_motion", default_value="true"),
        DeclareLaunchArgument("enable_llm_debate", default_value="true"),
        DeclareLaunchArgument("debate_llm_provider", default_value="ollama"),
        DeclareLaunchArgument("debate_llm_model", default_value="gemma4:12b"),
        DeclareLaunchArgument("debate_llm_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("enable_vlm_checkpointing", default_value="true"),
        DeclareLaunchArgument("vlm_provider", default_value="ollama"),
        DeclareLaunchArgument("vlm_model", default_value="gemma4:12b"),
        DeclareLaunchArgument("vlm_write_period_sec", default_value="30.0"),
        DeclareLaunchArgument("vlm_auto_write_checkpoints", default_value="false"),
        DeclareLaunchArgument(
            "vlm_prompt",
            default_value=(
                "Answer this tour question in no more than two short sentences. "
                "Describe only important visible navigation cues, obstacles, signage, and uncertainty. "
                "Use plain text only."
            ),
        ),
        DeclareLaunchArgument("camera_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("tts_enabled", default_value="true"),
        DeclareLaunchArgument("local_speaker_enabled", default_value="true"),
        DeclareLaunchArgument("local_speaker_backend", default_value="auto"),

        DeclareLaunchArgument("enable_phone_web", default_value="true"),
        DeclareLaunchArgument("phone_web_bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("phone_web_port", default_value="8765"),
        DeclareLaunchArgument("phone_web_require_token", default_value="true"),

        DeclareLaunchArgument("enable_perception_tools", default_value="true"),
        DeclareLaunchArgument("enable_dynamic_obstacle_tracking", default_value="false"),
        DeclareLaunchArgument("enable_open_vocab_detector", default_value="false"),
        DeclareLaunchArgument("open_vocab_backend", default_value="grounding_dino"),
        DeclareLaunchArgument("open_vocab_model", default_value="IDEA-Research/grounding-dino-tiny"),

        semantic_resume,

        # Intentionally NO omi_ble_bridge_node and NO stt_node.
        # Phone Chrome performs speech recognition and sends transcript JSON directly here.
        Node(
            package="go2_langgraph_agent",
            executable="main_supervisor",
            name="go2_langgraph_main_supervisor",
            output="screen",
            parameters=[{
                "session_root": session_root,
                "session_name": session_name,
                "enable_llm_debate": enable_llm_debate,
                "enable_tour_mode": True,
                "debate_llm_provider": debate_llm_provider,
                "debate_llm_model": debate_llm_model,
                "debate_llm_timeout_sec": debate_llm_timeout_sec,
                "enable_nav_publish": False,
                "enable_object_navigation": True,
                "enable_web_search": True,
                "web_model": "gemini-2.5-flash",
                "enable_ollama_reasoner": True,
                "ollama_model": "gemma4:e4b",
                "route_semantic_resume_commands": True,
            }],
        ),
        Node(
            package="go2_memory_core",
            executable="vlm_checkpoint_node",
            name="go2_vlm_checkpoint_node",
            output="screen",
            condition=IfCondition(enable_vlm_checkpointing),
            parameters=[{
                "session_root": session_root,
                "session_name": session_name,
                "camera_topic": camera_topic,
                "write_period_sec": vlm_write_period_sec,
                "auto_write_checkpoints": vlm_auto_write_checkpoints,
                "vlm_provider": vlm_provider,
                "vlm_model": vlm_model,
                "vlm_prompt": ParameterValue(vlm_prompt, value_type=str),
            }],
        ),
        Node(
            package="go2_omi_voice_bridge",
            executable="voice_intent_gate_node",
            name="go2_voice_intent_gate",
            output="screen",
            parameters=[config_file, {
                "require_confirmation_for_motion": require_confirmation_for_motion,
            }],
        ),
        Node(
            package="go2_omi_voice_bridge",
            executable="tts_node",
            name="go2_tts_node",
            output="screen",
            parameters=[config_file, {
                "tts_enabled": tts_enabled,
                "local_speaker_enabled": local_speaker_enabled,
                "local_speaker_backend": local_speaker_backend,
            }],
        ),
        Node(
            package="go2_omi_voice_bridge",
            executable="tour_voice_command_router",
            name="go2_tour_voice_command_router",
            output="screen",
            parameters=[config_file, {
                "session_root": session_root,
                "session_name": session_name,
            }],
        ),
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
        Node(
            package="go2_perception_tools",
            executable="lidar_geometry_node",
            name="go2_lidar_geometry_node",
            output="screen",
            condition=IfCondition(enable_perception_tools),
        ),
        Node(
            package="go2_perception_tools",
            executable="pointcloud_analyzer_node",
            name="go2_pointcloud_analyzer_node",
            output="screen",
            condition=IfCondition(enable_perception_tools),
        ),
        Node(
            package="go2_perception_tools",
            executable="traversability_node",
            name="go2_traversability_node",
            output="screen",
            condition=IfCondition(enable_perception_tools),
        ),
        Node(
            package="go2_perception_tools",
            executable="dynamic_obstacle_tracker",
            name="go2_dynamic_obstacle_tracker",
            output="screen",
            condition=IfCondition(enable_dynamic_obstacle_tracking),
        ),
        Node(
            package="go2_perception_tools",
            executable="open_vocab_detector_node",
            name="go2_open_vocab_detector",
            output="screen",
            condition=IfCondition(enable_open_vocab_detector),
            parameters=[{
                "camera_topic": camera_topic,
                "backend": open_vocab_backend,
                "model_name": open_vocab_model,
            }],
        ),
    ])
