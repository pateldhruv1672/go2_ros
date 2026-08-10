from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    session_name = LaunchConfiguration('session_name')
    session_root = LaunchConfiguration('session_root')
    camera_topic = LaunchConfiguration('camera_topic')
    return LaunchDescription([
        DeclareLaunchArgument('session_name', default_value='auto'),
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('yolo_model', default_value='yolov8n.pt'),
        DeclareLaunchArgument('sam2_model', default_value='sam2_t.pt'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('go2_omi_voice_bridge'), 'launch', 'sparky_phone_tour.launch.py'
            ])),
            launch_arguments={
                'session_name': session_name,
                'session_root': session_root,
                'camera_topic': camera_topic,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('go2_agentic_system'), 'launch', 'world_perception_overlay.launch.py'
            ])),
            launch_arguments={
                'session_name': session_name,
                'session_root': session_root,
                'camera_topic': camera_topic,
                'scan_topic': LaunchConfiguration('scan_topic'),
                'device': LaunchConfiguration('device'),
                'yolo_model': LaunchConfiguration('yolo_model'),
                'sam2_model': LaunchConfiguration('sam2_model'),
            }.items(),
        ),
        Node(
            package='go2_nav_tools',
            executable='nav2_tool_server',
            name='go2_object_find_nav2_tool_server',
            output='screen',
            parameters=[{'enable_motion': True}],
        ),
    ])
