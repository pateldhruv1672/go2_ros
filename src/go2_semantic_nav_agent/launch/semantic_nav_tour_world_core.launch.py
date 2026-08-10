from __future__ import annotations

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    session_root = LaunchConfiguration('session_root')
    session_name = LaunchConfiguration('session_name')
    camera_topic = LaunchConfiguration('camera_topic')
    omi_config = PathJoinSubstitution([FindPackageShare('go2_omi_voice_bridge'), 'config', 'omi_voice.yaml'])
    tour_host_config = PathJoinSubstitution([FindPackageShare('go2_semantic_nav_agent'), 'config', 'tour_host.yaml'])
    rviz_config = PathJoinSubstitution([FindPackageShare('go2_semantic_nav_agent'), 'config', 'semantic_nav.rviz'])

    resume_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('go2_semantic_nav_agent'), 'launch', 'semantic_nav_resume_world.launch.py'
        ])),
        launch_arguments={
            'session_root': session_root,
            'session_name': session_name,
            'rviz2': 'false',
            'restore_spawn_on_start': LaunchConfiguration('restore_spawn_on_start'),
            'camera_topic': camera_topic,
            'device': LaunchConfiguration('device'),
            'yolo_model': LaunchConfiguration('yolo_model'),
            'sam2_model': LaunchConfiguration('sam2_model'),
            'enable_object_perception': LaunchConfiguration('enable_object_perception'),
            'enable_sam2': LaunchConfiguration('enable_sam2'),
            'enable_vlm_backup': LaunchConfiguration('enable_vlm_backup'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('session_name', default_value='auto'),
        DeclareLaunchArgument('rviz2', default_value='true'),
        DeclareLaunchArgument('restore_spawn_on_start', default_value='true'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('yolo_model', default_value=os.path.expanduser('~/.cache/sparky_models/yolov8n.pt')),
        DeclareLaunchArgument('sam2_model', default_value=os.path.expanduser('~/.cache/sparky_models/sam2_t.pt')),
        DeclareLaunchArgument('enable_object_perception', default_value='true'),
        DeclareLaunchArgument('enable_sam2', default_value='true'),
        DeclareLaunchArgument('enable_vlm_backup', default_value='false'),

        DeclareLaunchArgument('enable_agentic_voice', default_value='true'),
        DeclareLaunchArgument('ollama_url', default_value=os.environ.get('OLLAMA_CHAT_URL', 'http://127.0.0.1:11434/api/chat')),
        DeclareLaunchArgument('ollama_model', default_value=os.environ.get('GO2_AGENT_OLLAMA_MODEL', 'llama3.2:3b')),
        DeclareLaunchArgument('ollama_timeout_sec', default_value='12.0'),
        DeclareLaunchArgument('require_motion_skill_confirmation', default_value='false'),
        DeclareLaunchArgument('motion_confirmation_policy', default_value='risky_only'),

        DeclareLaunchArgument('enable_tour_adapter', default_value='true'),
        DeclareLaunchArgument('enable_tour_host', default_value='true'),
        DeclareLaunchArgument('enable_speech_arbiter', default_value='true'),
        DeclareLaunchArgument('enable_tts', default_value='true'),
        DeclareLaunchArgument('local_speaker_backend', default_value='auto'),

        DeclareLaunchArgument('enable_phone_web', default_value='true'),
        DeclareLaunchArgument('phone_web_bind_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('phone_web_port', default_value='8765'),
        DeclareLaunchArgument('phone_web_require_token', default_value='true'),
        DeclareLaunchArgument('phone_admin_pin', default_value=os.environ.get('SPARKY_ADMIN_PIN', '0000')),

        DeclareLaunchArgument('enable_motion_skills', default_value='true'),
        DeclareLaunchArgument('allow_motion_during_navigation', default_value='false'),
        DeclareLaunchArgument('allow_medium_risk_motion', default_value='true'),
        DeclareLaunchArgument('allow_high_risk_motion', default_value='false'),

        DeclareLaunchArgument('enable_omi_input', default_value='false'),

        resume_world,

        Node(
            package='rviz2',
            executable='rviz2',
            name='sparky_tour_rviz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz2')),
            arguments=['-d', rviz_config],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': __import__('os').environ.get('LIBGL_ALWAYS_SOFTWARE', '1')},
            respawn=True,
            respawn_delay=3.0,
        ),

        Node(
            package='go2_langgraph_agent', executable='agentic_voice_action_node',
            name='go2_agentic_voice_action', output='screen', respawn=True, respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_agentic_voice')),
            parameters=[{
                'session_root': session_root, 'session_name': session_name,
                'ollama_url': LaunchConfiguration('ollama_url'),
                'ollama_model': LaunchConfiguration('ollama_model'),
                'ollama_timeout_sec': ParameterValue(LaunchConfiguration('ollama_timeout_sec'), value_type=float),
                'require_motion_skill_confirmation': ParameterValue(LaunchConfiguration('require_motion_skill_confirmation'), value_type=bool),
                'motion_confirmation_policy': LaunchConfiguration('motion_confirmation_policy'),
            }],
        ),

        Node(
            package='go2_omi_voice_bridge', executable='semantic_tour_adapter_node',
            name='go2_semantic_tour_adapter', output='screen', respawn=True, respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_tour_adapter')),
        ),
        Node(
            package='go2_semantic_nav_agent', executable='tour_host_node',
            name='go2_tour_host', output='screen', respawn=True, respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_tour_host')),
            parameters=[{
                'session_root': session_root, 'session_name': session_name,
                'default_script_path': tour_host_config,
                'tts_topic': '/go2_speech/request',
                'motion_topic': '/motion_skills/command',
            }],
        ),
        Node(
            package='go2_omi_voice_bridge', executable='speech_arbiter_node',
            name='go2_speech_arbiter', output='screen', respawn=True, respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_speech_arbiter')),
        ),
        Node(
            package='go2_omi_voice_bridge', executable='tts_node', name='go2_tts_node',
            output='screen', respawn=True, respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_tts')),
            parameters=[omi_config, {
                'input_topics': ['/go2_tts/say'],
                'tts_enabled': True,
                'local_speaker_enabled': True,
                'local_speaker_backend': LaunchConfiguration('local_speaker_backend'),
                'speak_vlm_status': False,
                'interrupt_previous_by_default': False,
            }],
        ),
        Node(
            package='go2_omi_voice_bridge', executable='phone_web_gateway_node',
            name='go2_phone_web_gateway', output='screen', respawn=True, respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_phone_web')),
            parameters=[{
                'bind_host': ParameterValue(LaunchConfiguration('phone_web_bind_host'), value_type=str),
                'port': ParameterValue(LaunchConfiguration('phone_web_port'), value_type=int),
                'require_token': ParameterValue(LaunchConfiguration('phone_web_require_token'), value_type=bool),
                'session_root': session_root, 'session_name': session_name,
                'admin_pin': ParameterValue(LaunchConfiguration('phone_admin_pin'), value_type=str),
            }],
        ),
        Node(
            package='go2_agentic_motion_skills', executable='safe_webrtc_motion_skill_agent_node',
            name='go2_safe_webrtc_motion_skill_agent', output='screen', respawn=True, respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_motion_skills')),
            parameters=[{
                'allow_during_navigation': ParameterValue(LaunchConfiguration('allow_motion_during_navigation'), value_type=bool),
                'allow_medium_risk': ParameterValue(LaunchConfiguration('allow_medium_risk_motion'), value_type=bool),
                'allow_high_risk': ParameterValue(LaunchConfiguration('allow_high_risk_motion'), value_type=bool),
                'allow_parameter_commands': False, 'allow_query_commands': True,
            }],
        ),
        Node(
            package='go2_omi_voice_bridge', executable='omi_ble_bridge_node', name='go2_omi_bridge',
            output='screen', condition=IfCondition(LaunchConfiguration('enable_omi_input')), parameters=[omi_config],
        ),
        Node(
            package='go2_omi_voice_bridge', executable='stt_node', name='go2_voice_stt_node',
            output='screen', condition=IfCondition(LaunchConfiguration('enable_omi_input')), parameters=[omi_config],
        ),
    ])
