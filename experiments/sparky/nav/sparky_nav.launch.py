from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node

def generate_launch_description():
    P = LC('params'); sim = {'use_sim_time': True}
    def nav(pkg, exe, name, extra=None, remap=None):
        return Node(package=pkg, executable=exe, name=name, output='screen',
                    parameters=[P, sim], remappings=remap or [])
    return LaunchDescription([
        DeclareLaunchArgument('params'),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node', name='pc2scan', output='screen',
             remappings=[('cloud_in', '/velodyne_points/points'), ('scan', '/scan')],
             parameters=[sim, {'target_frame': 'base_link', 'transform_tolerance': 0.5, 'min_height': -0.15, 'max_height': 0.6,
                               'angle_min': -3.14159, 'angle_max': 3.14159, 'angle_increment': 0.0087, 'scan_time': 0.1,
                               'range_min': 0.5, 'range_max': 10.0, 'use_inf': True}]),
        nav('nav2_map_server', 'map_server', 'map_server'),
        nav('nav2_amcl', 'amcl', 'amcl'),
        nav('nav2_controller', 'controller_server', 'controller_server', remap=[('cmd_vel', 'cmd_vel/smooth')]),
        nav('nav2_planner', 'planner_server', 'planner_server'),
        nav('nav2_behaviors', 'behavior_server', 'behavior_server', remap=[('cmd_vel', 'cmd_vel/smooth')]),
        nav('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', output='screen',
             parameters=[sim, {'autostart': True, 'node_names': ['map_server', 'amcl'], 'bond_timeout': 0.0}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen',
             parameters=[sim, {'autostart': True, 'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator'], 'bond_timeout': 0.0}]),
    ])
