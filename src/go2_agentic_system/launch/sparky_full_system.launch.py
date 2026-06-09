from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='',
        description='Network interface for Unitree SDK2 motion skills, e.g. eth0 or wlan0'
    )
    session_root_arg = DeclareLaunchArgument(
        'session_root',
        default_value='~/.ros/go2_semantic_nav_sessions',
        description='Semantic navigation session root'
    )
    session_name_arg = DeclareLaunchArgument(
        'session_name',
        default_value='',
        description='Semantic navigation session name'
    )
    semantic_rviz_arg = DeclareLaunchArgument(
        'semantic_rviz',
        default_value='true',
        description='Open the semantic navigation RViz session'
    )
    speaker_tts_enabled_arg = DeclareLaunchArgument(
        'speaker_tts_enabled',
        default_value='true',
        description='Enable on-device speaker TTS'
    )
    input_backend_arg = DeclareLaunchArgument(
        'input_backend',
        default_value='auto',
        description='Voice input backend: auto, omi, or mic'
    )

    semantic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('go2_semantic_nav_agent'),
                'launch',
                'semantic_nav_resume.launch.py',
            ])
        ),
        launch_arguments={
            'session_root': LaunchConfiguration('session_root'),
            'session_name': LaunchConfiguration('session_name'),
            'rviz': LaunchConfiguration('semantic_rviz'),
            'rviz2': LaunchConfiguration('semantic_rviz'),
        }.items()
    )

    voice_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('go2_agentic_multiagent_voice_nav'),
                'launch',
                'multiagent_resume.launch.py',
            ])
        ),
        launch_arguments={
            'session_root': LaunchConfiguration('session_root'),
            'session_name': LaunchConfiguration('session_name'),
            'input_backend': LaunchConfiguration('input_backend'),
            'speaker_tts_enabled': LaunchConfiguration('speaker_tts_enabled'),
        }.items()
    )

    motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('go2_agentic_motion_skills'),
                'launch',
                'motion_skills.launch.py',
            ])
        ),
        launch_arguments={
            'network_interface': LaunchConfiguration('network_interface'),
        }.items()
    )

    return LaunchDescription([
        network_interface_arg,
        session_root_arg,
        session_name_arg,
        semantic_rviz_arg,
        speaker_tts_enabled_arg,
        input_backend_arg,
        semantic_launch,
        voice_launch,
        motion_launch,
    ])
