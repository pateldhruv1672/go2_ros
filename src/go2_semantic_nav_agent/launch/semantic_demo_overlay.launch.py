from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    places_file = LaunchConfiguration('places_file')
    omi_text_topic = LaunchConfiguration('omi_text_topic')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('places_file', default_value=''),
        DeclareLaunchArgument('omi_text_topic', default_value='/omi/transcript'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(
            package='go2_semantic_nav_agent',
            executable='semantic_place_markers',
            name='semantic_place_markers',
            output='screen',
            parameters=[{
                'places_file': places_file,
                'marker_topic': '/semantic_nav/place_markers',
                'frame_id': 'map',
            }],
        ),

        Node(
            package='go2_semantic_nav_agent',
            executable='omi_demo_bridge',
            name='omi_demo_bridge',
            output='screen',
            parameters=[{
                'places_file': places_file,
                'omi_text_topic': omi_text_topic,
                'semantic_command_topic': '/semantic_nav/command',
                'cmd_vel_topic': '/cmd_vel_omi',
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='semantic_demo_rviz',
            output='screen',
            condition=IfCondition(rviz),
        ),
    ])
