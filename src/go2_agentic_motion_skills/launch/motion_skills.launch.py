from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('network_interface', default_value=''),
        DeclareLaunchArgument('motion_mode', default_value='normal'),
        DeclareLaunchArgument('allow_during_navigation', default_value='false'),
        Node(
            package='go2_agentic_motion_skills',
            executable='motion_skill_agent_node',
            name='motion_skill_agent_node',
            output='screen',
            parameters=[{
                'network_interface': LaunchConfiguration('network_interface'),
                'motion_mode': LaunchConfiguration('motion_mode'),
                'allow_during_navigation': LaunchConfiguration('allow_during_navigation'),
            }],
        ),
    ])
