from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('session_name', default_value='latest'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('yolo_model', default_value='yolov8n.pt'),
        DeclareLaunchArgument('sam2_model', default_value='sam2_t.pt'),
        DeclareLaunchArgument('enable_sam2', default_value='true'),
        DeclareLaunchArgument('yolo_conf', default_value='0.25'),
        DeclareLaunchArgument('inference_period_sec', default_value='0.20'),
        DeclareLaunchArgument('min_confirmations', default_value='2'),
        DeclareLaunchArgument('object_map_db', default_value='~/.ros/go2_sysnav_vln/object_map.sqlite3'),
        Node(
            package='go2_object_explorer', executable='fast_sam2_tracker_overlay_node',
            name='go2_world_yolo_sam2', output='screen', parameters=[{
                'image_topic': LaunchConfiguration('camera_topic'),
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
            package='go2_sysnav_vln', executable='pose_aware_object_mapper',
            name='go2_world_pose_aware_object_mapper', output='screen', parameters=[{
                'detection_topics': ['/object_explorer/sam2_detections'],
                'scan_topic': LaunchConfiguration('scan_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'min_confirmations': LaunchConfiguration('min_confirmations'),
                'database_path': LaunchConfiguration('object_map_db'),
                'reset_database_on_start': True,
                'publish_topic': '/go2_vln/object_map',
                'marker_topic': '/go2_vln/object_markers',
            }],
        ),
        Node(
            package='go2_memory_core', executable='world_object_memory_node',
            name='go2_world_object_memory', output='screen', parameters=[{
                'session_root': LaunchConfiguration('session_root'),
                'session_name': LaunchConfiguration('session_name'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'detector_topic': '/object_explorer/sam2_detections',
                'object_map_topic': '/go2_vln/object_map',
                'ingest_historical_mapper_objects': False,
                'enable_graph_memory': True,
                'enable_vector_memory': True,
                'enable_voxel_memory': True,
            }],
        ),
    ])
