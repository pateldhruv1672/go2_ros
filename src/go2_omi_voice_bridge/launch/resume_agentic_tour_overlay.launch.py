from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def b(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    session_root = LaunchConfiguration('session_root')
    session_name = LaunchConfiguration('session_name')
    camera_topic = LaunchConfiguration('camera_topic')
    vlm_provider = LaunchConfiguration('vlm_provider')
    vlm_model = LaunchConfiguration('vlm_model')
    vlm_base_url = LaunchConfiguration('vlm_base_url')
    phone_port = LaunchConfiguration('phone_port')
    object_db = PathJoinSubstitution([session_root, session_name, 'object_mapper.sqlite3'])

    return LaunchDescription([
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('session_name', default_value='latest'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('vlm_provider', default_value='openrouter'),
        DeclareLaunchArgument('vlm_model', default_value='google/gemini-2.5-flash'),
        DeclareLaunchArgument('vlm_base_url', default_value=''),
        DeclareLaunchArgument('enable_rich_memory', default_value='false'),
        DeclareLaunchArgument('enable_object_perception', default_value='true'),
        DeclareLaunchArgument('enable_sam2', default_value='true'),
        DeclareLaunchArgument('annotated_publish_period_sec', default_value='0.05'),
        DeclareLaunchArgument('perception_period_sec', default_value='0.10'),
        DeclareLaunchArgument('sam2_every_n', default_value='4'),
        DeclareLaunchArgument('sam2_imgsz', default_value='384'),
        DeclareLaunchArgument('yolo_imgsz', default_value='416'),
        DeclareLaunchArgument('publish_object_overlay', default_value='false'),
        DeclareLaunchArgument('yolo_model', default_value='yolov8n.pt'),
        DeclareLaunchArgument('sam2_model', default_value='sam2_t.pt'),
        DeclareLaunchArgument('object_device', default_value='cuda:0'),
        DeclareLaunchArgument('enable_motion_skills', default_value='true'),
        DeclareLaunchArgument('enable_front_flip', default_value='true'),
        DeclareLaunchArgument('enable_llm_debate', default_value='false'),
        DeclareLaunchArgument('phone_port', default_value='8765'),
        DeclareLaunchArgument('local_speaker_backend', default_value='auto'),

        Node(
            package='go2_memory_core', executable='memory_server_node', name='go2_memory_server', output='screen',
            parameters=[{
                'session_root': session_root,
                'enable_graph_memory': b('enable_rich_memory'),
                'enable_voxel_memory': b('enable_rich_memory'),
                'enable_vector_memory': b('enable_rich_memory'),
                'enable_legacy_exports': True,
            }],
        ),
        Node(
            package='go2_memory_core', executable='vlm_checkpoint_node', name='go2_vlm_checkpoint_node', output='screen',
            parameters=[{
                'session_root': session_root, 'session_name': session_name,
                'camera_topic': camera_topic, 'auto_write_checkpoints': False,
                'vlm_provider': vlm_provider, 'vlm_model': vlm_model,
                'fresh_frame_wait_timeout_sec': 1.5, 'detector_context_wait_sec': 0.25, 'vlm_base_url': vlm_base_url,
                'enable_graph_memory': b('enable_rich_memory'),
                'enable_voxel_memory': b('enable_rich_memory'),
                'enable_vector_memory': b('enable_rich_memory'),
                'max_live_image_age_sec': 2.5,
            }],
        ),
        Node(
            package='go2_object_explorer', executable='fast_sam2_tracker_overlay_node',
            name='fast_sam2_tracker_overlay_node', output='screen', condition=IfCondition(LaunchConfiguration('enable_object_perception')),
            parameters=[{
                'image_topic': camera_topic,
                # SPARKY_SAM2_BALANCED_V13_15
                'yolo_imgsz': ParameterValue(LaunchConfiguration('yolo_imgsz'), value_type=int),
                'sam2_imgsz': ParameterValue(LaunchConfiguration('sam2_imgsz'), value_type=int),
                'sam2_every_n': ParameterValue(LaunchConfiguration('sam2_every_n'), value_type=int),
                'inference_period_sec': ParameterValue(LaunchConfiguration('perception_period_sec'), value_type=float),
                'publish_annotated_image': True,
                'annotated_publish_period_sec': ParameterValue(LaunchConfiguration('annotated_publish_period_sec'), value_type=float),
                'yolo_model': LaunchConfiguration('yolo_model'), 'sam2_model': LaunchConfiguration('sam2_model'),
                'device': LaunchConfiguration('object_device'), 'enable_sam2': b('enable_sam2'),
                'publish_annotated_image': b('publish_object_overlay'),
                'detections_topic': '/object_explorer/sam2_detections',
                'annotated_image_topic': '/object_explorer/annotated_image',
                'yolo_imgsz': 416, 'sam2_imgsz': 384, 'sam2_every_n': 3,
                'inference_period_sec': 0.12, 'max_detections': 15,
            }],
        ),
        Node(
            package='go2_sysnav_vln', executable='pose_aware_object_mapper', name='go2_pose_aware_object_mapper',
            output='screen', condition=IfCondition(LaunchConfiguration('enable_object_perception')),
            parameters=[{
                'detection_topics': ['/go2_vln/target_detections_3d'], 'scan_topic': '/scan_nav',
                'database_path': object_db, 'reset_database_on_start': False,
                'require_registered_3d': True, 'allow_scan_ray_fallback': False,
            }],
        ),
        Node(
            package='go2_memory_core', executable='world_object_memory_node', name='go2_world_object_memory',
            output='screen', condition=IfCondition(LaunchConfiguration('enable_object_perception')),
            parameters=[{
                'session_root': session_root, 'session_name': session_name,
                'enable_graph_memory': b('enable_rich_memory'),
                'enable_voxel_memory': b('enable_rich_memory'),
                'enable_vector_memory': b('enable_rich_memory'),
                'ingest_historical_mapper_objects': False,
            }],
        ),
        Node(
            package='go2_langgraph_agent', executable='main_supervisor', name='go2_langgraph_main_supervisor', output='screen',
            parameters=[{
                'session_root': session_root, 'session_name': session_name,
                'enable_debate_layer': True, 'enable_llm_debate': b('enable_llm_debate'),
                'enable_tour_mode': True, 'enable_explore_mode': False,
                'enable_nav_publish': False, 'route_semantic_resume_commands': True,
                'enable_object_navigation': True, 'enable_human_interrupts': False,
                'enable_langgraph_streaming': True, 'require_native_langgraph_store': True,
                'enable_web_search': False, 'enable_ollama_reasoner': True,
            }],
        ),
        Node(
            package='go2_omi_voice_bridge', executable='unified_intent_gate_node', name='go2_unified_intent_gate', output='screen',
            parameters=[{
                'require_confirmation_for_motion': False, 'emit_legacy_tour_topics': False,
                'require_wake_word': True, 'ignore_transcripts_during_tts': True,
                'agent_command_topic': '/go2_agent/user_command', 'agent_query_topic': '/go2_agent/query',
                'vlm_query_topic': '/go2_vlm/query', 'nav_command_topic': '/semantic_nav/command',
            }],
        ),
        Node(
            package='go2_omi_voice_bridge', executable='semantic_tour_adapter_node', name='go2_semantic_tour_adapter', output='screen',
            parameters=[{'auto_tour_gestures': False}],
        ),
        Node(
            package='go2_omi_voice_bridge', executable='tour_host_script_node', name='go2_tour_host_script', output='screen',
            parameters=[{'session_root': session_root, 'session_name': session_name}],
        ),
        Node(
            package='go2_omi_voice_bridge', executable='speech_arbiter_node', name='go2_speech_arbiter', output='screen',
        ),
        Node(
            package='go2_omi_voice_bridge', executable='tts_node', name='go2_tts_node', output='screen',
            parameters=[{
                'tts_enabled': True, 'local_speaker_enabled': True,
                'local_speaker_backend': LaunchConfiguration('local_speaker_backend'),
                'input_topics': ['/go2_tts/say'], 'speak_vlm_status': False,
            }],
        ),
        Node(
            package='go2_agentic_motion_skills', executable='safe_webrtc_motion_skill_agent_node',
            name='safe_webrtc_motion_skill_agent_node', output='screen', condition=IfCondition(LaunchConfiguration('enable_motion_skills')),
            parameters=[{
                'allow_during_navigation': False, 'allow_medium_risk': True,
                'allow_high_risk': b('enable_front_flip'), 'allow_parameter_commands': False, 'allow_query_commands': True,
            }],
        ),
        Node(
            package='go2_omi_voice_bridge', executable='phone_web_gateway_node', name='go2_phone_web_gateway', output='screen',
            parameters=[{
                'bind_host': '0.0.0.0', 'port': ParameterValue(phone_port, value_type=int),
                'require_token': True, 'session_root': session_root, 'session_name': session_name,
                # Intentionally do NOT pass api_token/admin_pin as ROS params: numeric PINs must stay env strings.
                'transcript_topic': '/go2_voice/transcript', 'query_topic': '/go2_agent/query', 'vlm_query_topic': '/go2_vlm/query',
            }],
        ),
    ])
