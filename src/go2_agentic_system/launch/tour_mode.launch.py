from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('session_name', default_value='latest'),
        DeclareLaunchArgument('enable_tour_mode', default_value='true'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('enable_human_interrupts', default_value='false'),
    ]
    memory_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('go2_agentic_system'), 'launch', 'agentic_memory_stack.launch.py'])),
        launch_arguments={
            'session_root': LaunchConfiguration('session_root'),
            'session_name': LaunchConfiguration('session_name'),
            'enable_memory_core': 'true',
            'enable_graph_memory': 'true',
            'enable_voxel_memory': 'true',
            'enable_vector_memory': 'true',
            'enable_perception_tools': 'true',
            'enable_langgraph_agent': 'true',
            'enable_debate_layer': 'true',
            'enable_tour_mode': LaunchConfiguration('enable_tour_mode'),
            'enable_human_interrupts': LaunchConfiguration('enable_human_interrupts'),
        }.items(),
    )
    return LaunchDescription(args + [
        memory_stack,
        Node(package='go2_nav_tools', executable='nav2_tool_server', name='go2_nav2_tool_server', output='screen', parameters=[{'enable_motion': LaunchConfiguration('enable_motion')}]),
    ])
