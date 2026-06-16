from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('session_name', default_value='default'),
        DeclareLaunchArgument('enable_explore_mode', default_value='true'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('enable_open_vocab_detector', default_value='true'),
        DeclareLaunchArgument('enable_dynamic_obstacle_tracking', default_value='true'),
        DeclareLaunchArgument('voice_input_mode', default_value='text_topic'),
        DeclareLaunchArgument('enable_human_interrupts', default_value='false'),
        DeclareLaunchArgument('enable_llm_debate', default_value='false'),
        DeclareLaunchArgument('debate_llm_provider', default_value='openrouter'),
        DeclareLaunchArgument('debate_llm_model', default_value='openai/gpt-4o-mini'),
        DeclareLaunchArgument('enable_time_travel_rviz', default_value='false'),
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
            'enable_open_vocab_detector': LaunchConfiguration('enable_open_vocab_detector'),
            'enable_dynamic_obstacle_tracking': LaunchConfiguration('enable_dynamic_obstacle_tracking'),
            'enable_langgraph_agent': 'true',
            'enable_debate_layer': 'true',
            'enable_llm_debate': LaunchConfiguration('enable_llm_debate'),
            'debate_llm_provider': LaunchConfiguration('debate_llm_provider'),
            'debate_llm_model': LaunchConfiguration('debate_llm_model'),
            'enable_time_travel_rviz': LaunchConfiguration('enable_time_travel_rviz'),
            'enable_explore_mode': LaunchConfiguration('enable_explore_mode'),
            'enable_unified_voice': 'true',
            'enable_human_interrupts': LaunchConfiguration('enable_human_interrupts'),
            'voice_input_mode': LaunchConfiguration('voice_input_mode'),
        }.items(),
    )
    return LaunchDescription(args + [
        memory_stack,
        Node(package='go2_nav_tools', executable='nav2_tool_server', name='go2_nav2_tool_server', output='screen', parameters=[{'enable_motion': LaunchConfiguration('enable_motion')}]),
        Node(package='go2_nav_tools', executable='frontier_explorer', name='go2_frontier_explorer', output='screen'),
        Node(package='go2_nav_tools', executable='coverage_explorer', name='go2_coverage_explorer', output='screen'),
        Node(package='go2_nav_tools', executable='recovery_manager', name='go2_recovery_manager', output='screen'),
    ])
