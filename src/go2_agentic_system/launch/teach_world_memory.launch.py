from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    session_root = LaunchConfiguration('session_root')
    camera_topic = LaunchConfiguration('camera_topic')
    return LaunchDescription([
        DeclareLaunchArgument('map_label', default_value='session'),
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('yolo_model', default_value='yolov8n.pt'),
        DeclareLaunchArgument('sam2_model', default_value='sam2_t.pt'),
        DeclareLaunchArgument('enable_sam2', default_value='true'),
        DeclareLaunchArgument('yolo_conf', default_value='0.25'),
        DeclareLaunchArgument('inference_period_sec', default_value='0.20'),
        DeclareLaunchArgument('min_confirmations', default_value='2'),
        DeclareLaunchArgument('object_map_db', default_value='~/.ros/go2_sysnav_vln/object_map.sqlite3'),
        DeclareLaunchArgument('vlm_provider', default_value='ollama'),
        DeclareLaunchArgument('vlm_model', default_value='gemma4:e4b'),
        DeclareLaunchArgument('vlm_base_url', default_value='http://127.0.0.1:11434/api/chat'),
        DeclareLaunchArgument('vlm_checkpoint_period_sec', default_value='8.0'),
        DeclareLaunchArgument('semantic_rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('go2_semantic_nav_agent'), 'launch', 'semantic_nav_teach.launch.py'
            ])),
            launch_arguments={
                'map_label': LaunchConfiguration('map_label'),
                'session_root': session_root,
                'vlm_provider': LaunchConfiguration('vlm_provider'),
                'vlm_model': LaunchConfiguration('vlm_model'),
                'vlm_base_url': LaunchConfiguration('vlm_base_url'),
                'semantic_rviz': LaunchConfiguration('semantic_rviz'),
            }.items(),
        ),
        Node(
            package='go2_object_explorer',
            executable='fast_sam2_tracker_overlay_node',
            name='go2_teach_yolo_sam2',
            output='screen',
            parameters=[{
                'image_topic': camera_topic,
                'device': LaunchConfiguration('device'),
                'yolo_model': LaunchConfiguration('yolo_model'),
                'sam2_model': LaunchConfiguration('sam2_model'),
                'enable_sam2': LaunchConfiguration('enable_sam2'),
                'yolo_conf': LaunchConfiguration('yolo_conf'),
                'inference_period_sec': LaunchConfiguration('inference_period_sec'),
                'detections_topic': '/object_explorer/sam2_detections',
                'annotated_image_topic': '/object_explorer/annotated_image',
            }],
        ),
        Node(
            package='go2_sysnav_vln',
            executable='pose_aware_object_mapper',
            name='go2_teach_pose_aware_object_mapper',
            output='screen',
            parameters=[{
                'detection_topics': ['/object_explorer/sam2_detections'],
                'scan_topic': LaunchConfiguration('scan_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'min_confirmations': LaunchConfiguration('min_confirmations'),
                'database_path': LaunchConfiguration('object_map_db'),
                'reset_database_on_start': True,
                'publish_topic': '/go2_vln/object_map',
                'marker_topic': '/go2_vln/object_markers',
            }],
        ),
        TimerAction(period=3.0, actions=[
            Node(
                package='go2_memory_core',
                executable='world_object_memory_node',
                name='go2_world_object_memory',
                output='screen',
                parameters=[{
                    'session_root': session_root,
                    'session_name': 'latest',
                    'camera_topic': camera_topic,
                    'detector_topic': '/object_explorer/sam2_detections',
                    'object_map_topic': '/go2_vln/object_map',
                    'ingest_historical_mapper_objects': False,
                    'enable_graph_memory': True,
                    'enable_vector_memory': True,
                    'enable_voxel_memory': True,
                }],
            ),
            Node(
                package='go2_memory_core',
                executable='vlm_checkpoint_node',
                name='go2_teach_vlm_checkpoint_node',
                output='screen',
                parameters=[{
                    'session_root': session_root,
                    'session_name': 'latest',
                    'camera_topic': camera_topic,
                    'write_period_sec': LaunchConfiguration('vlm_checkpoint_period_sec'),
                    'auto_write_checkpoints': True,
                    'vlm_provider': LaunchConfiguration('vlm_provider'),
                    'vlm_model': LaunchConfiguration('vlm_model'),
                    'vlm_base_url': LaunchConfiguration('vlm_base_url'),
                    'enable_graph_memory': True,
                    'enable_vector_memory': True,
                    'enable_voxel_memory': True,
                }],
            ),
        ]),
    ])
