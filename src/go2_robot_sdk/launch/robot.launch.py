# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource


class Go2LaunchConfig:
    """Configuration container for Go2 robot launch parameters"""
    
    def __init__(self):
        # Environment variables
        self.robot_token = os.getenv('ROBOT_TOKEN', '')
        self.robot_ip = os.getenv('ROBOT_IP', '')
        self.robot_ip_list = self._parse_ip_list(self.robot_ip)
        self.map_name = os.getenv('MAP_NAME', '3d_map')
        self.save_map = os.getenv('MAP_SAVE', 'false')
        self.conn_type = os.getenv('CONN_TYPE', 'webrtc')
        
        # Derived configurations
        self.conn_mode = self._determine_connection_mode()
        self.rviz_config = self._get_rviz_config()
        self.urdf_file = self._get_urdf_file()
        
        # Package paths
        self.package_dir = get_package_share_directory('go2_robot_sdk')
        self.config_paths = self._get_config_paths()
        
        print(f"� Go2 Launch Configuration:")
        print(f"   Robot IPs: {self.robot_ip_list}")
        print(f"   Connection: {self.conn_type} ({self.conn_mode})")
        print(f"   URDF: {self.urdf_file}")
    
    def _parse_ip_list(self, robot_ip: str) -> List[str]:
        """Parse robot IP addresses from environment variable"""
        return robot_ip.replace(" ", "").split(",") if robot_ip else []
    
    def _determine_connection_mode(self) -> str:
        """Determine connection mode based on IP list and connection type"""
        return "single" if len(self.robot_ip_list) == 1 and self.conn_type != "cyclonedx" else "multi"
    
    def _get_rviz_config(self) -> str:
        """Get appropriate RViz configuration file"""
        if self.conn_type == 'cyclonedx':
            return "cyclonedx_config.rviz"
        elif self.conn_mode == 'single':
            return "single_robot_conf.rviz"
        else:
            return "multi_robot_conf.rviz"
    
    def _get_urdf_file(self) -> str:
        """Get appropriate URDF file"""
        return 'go2.urdf' if self.conn_mode == 'single' else 'multi_go2.urdf'
    
    def _get_config_paths(self) -> dict:
        """Get all configuration file paths"""
        return {
            'joystick': os.path.join(self.package_dir, 'config', 'joystick.yaml'),
            'twist_mux': os.path.join(self.package_dir, 'config', 'twist_mux.yaml'),
            'slam': os.path.join(self.package_dir, 'config', 'mapper_params_online_async.yaml'),
            'nav2': os.path.join(self.package_dir, 'config', 'nav2_params.yaml'),
            'rviz': os.path.join(self.package_dir, 'config', self.rviz_config),
            'urdf': os.path.join(self.package_dir, 'urdf', self.urdf_file),
        }


class Go2NodeFactory:
    """Factory for creating Go2 robot nodes"""
    
    def __init__(self, config: Go2LaunchConfig):
        self.config = config
    
    def create_launch_arguments(self) -> List[DeclareLaunchArgument]:
        """Create all launch arguments"""
        return [
            DeclareLaunchArgument('rviz2', default_value='true', description='Launch RViz2'),
            DeclareLaunchArgument('nav2', default_value='true', description='Launch Nav2'),
            DeclareLaunchArgument('slam', default_value='true', description='Launch SLAM'),
            DeclareLaunchArgument('foxglove', default_value='true', description='Launch Foxglove Bridge'),
            DeclareLaunchArgument('joystick', default_value='false', description='Launch joystick'),
            DeclareLaunchArgument('teleop', default_value='false', description='Launch teleoperation'),
            DeclareLaunchArgument(
                'nav2_params_file',
                default_value=self.config.config_paths['nav2'],
                description='Nav2 parameter file',
            ),
            DeclareLaunchArgument(
                'nav2_start_delay_sec',
                default_value='10.0',
                description='Delay Nav2 until live SLAM, TF, and sensing topics are publishing',
            ),
            DeclareLaunchArgument(
                'rviz_start_delay_sec',
                default_value='14.0',
                description='Delay RViz so Nav2 action servers are available before sending goals',
            ),
            DeclareLaunchArgument(
                'cmd_vel_axis_mode',
                default_value='standard',
                description='Go2 command adapter axis mode: standard or swap_xy',
            ),
            DeclareLaunchArgument(
                'pointcloud_aggregator',
                default_value='false',
                description='Publish accumulated debug pointcloud history',
            ),
        ]
    
    def create_robot_state_nodes(self) -> List[Node]:
        """Create robot state publisher nodes"""
        nodes = []
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        
        if self.config.conn_mode == 'single':
            # Single robot configuration
            robot_desc = self._load_urdf_content(self.config.config_paths['urdf'])
            
            nodes.extend([
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='go2_robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'robot_description': robot_desc
                    }],
                    arguments=[self.config.config_paths['urdf']]
                ),
                self._create_pointcloud_to_laserscan_node()
            ])
        else:
            # Multi-robot configuration
            base_urdf = self._load_urdf_content(self.config.config_paths['urdf'])
            
            for i, _ in enumerate(self.config.robot_ip_list):
                robot_desc = base_urdf.format(robot_num=f"robot{i}")
                
                nodes.extend([
                    Node(
                        package='robot_state_publisher',
                        executable='robot_state_publisher',
                        name='go2_robot_state_publisher',
                        output='screen',
                        namespace=f"robot{i}",
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'robot_description': robot_desc
                        }],
                        arguments=[self.config.config_paths['urdf']]
                    ),
                    self._create_pointcloud_to_laserscan_node(f"robot{i}")
                ])
        
        return nodes
    
    def _load_urdf_content(self, urdf_path: str) -> str:
        """Load URDF file content"""
        with open(urdf_path, 'r') as file:
            return file.read()
    
    def _create_pointcloud_to_laserscan_node(self, namespace: str = None) -> Node:
        """Create pointcloud to laserscan conversion node"""
        target_frame = f'{namespace}/base_link' if namespace else 'base_link'
        parameters = {
            'target_frame': target_frame,
            'transform_tolerance': 0.5,
            # Keep the scan focused on obstacle-height returns. The raw Go2
            # cloud can include floor, body/leg, and far sparse points that
            # make Nav2 mark the robot/start as occupied.
            'min_height': 0.15,
            'max_height': 1.00,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872665,
            'scan_time': 0.1,
            'range_min': 0.55,
            'range_max': 5.0,
            'use_inf': True,
            'concurrency_level': 1,
        }
        if namespace:
            # Multi-robot setup
            return Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name=f'{namespace}_pointcloud_to_laserscan',
                remappings=[
                    ('cloud_in', f'{namespace}/point_cloud2'),
                    ('scan', f'{namespace}/scan'),
                ],
                parameters=[parameters],
                output='screen',
            )
        else:
            # Single robot setup
            return Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='go2_pointcloud_to_laserscan',
                remappings=[
                    ('cloud_in', 'point_cloud2'),
                    ('scan', 'scan'),
                ],
                parameters=[parameters],
                output='screen',
            )
    
    def create_core_nodes(self) -> List[Node]:
        """Create core Go2 robot nodes"""
        with_pointcloud_aggregator = LaunchConfiguration('pointcloud_aggregator', default='false')
        return [
            # Main robot driver (clean architecture)
            Node(
                package='go2_robot_sdk',
                executable='go2_driver_node',
                name='go2_driver_node',
                output='screen',
                parameters=[{
                    'robot_ip': self.config.robot_ip,
                    'token': self.config.robot_token,
                    'conn_type': self.config.conn_type,
                    'obstacle_avoidance': False,
                    'cmd_vel_linear_gain': 1.0,
                    'cmd_vel_angular_gain': 1.0,
                    'cmd_vel_min_linear_x': 0.08,
                    'cmd_vel_min_angular_z': 0.18,
                    'cmd_vel_max_linear_x': 0.75,
                    'cmd_vel_max_angular_z': 0.90,
                    'cmd_vel_axis_mode': LaunchConfiguration('cmd_vel_axis_mode'),
                    'cmd_vel_invert_linear_x': False,
                    'cmd_vel_invert_linear_y': False,
                    'cmd_vel_invert_angular_z': False,
                }],
            ),
            # LiDAR processing node (new separate package)
            Node(
                package='lidar_processor',
                executable='lidar_to_pointcloud',
                name='lidar_to_pointcloud',
                parameters=[{
                    'robot_ip_lst': self.config.robot_ip_list,
                    'map_name': self.config.map_name,
                    'map_save': self.config.save_map
                }],
            ),
            # Advanced point cloud aggregator
            Node(
                package='lidar_processor',
                executable='pointcloud_aggregator',
                name='pointcloud_aggregator',
                condition=IfCondition(with_pointcloud_aggregator),
                parameters=[{
                    'max_range': 20.0,
                    'min_range': 0.1,
                    'height_filter_min': -2.0,
                    'height_filter_max': 3.0,
                    'downsample_rate': 5,
                    'publish_rate': 10.0
                }],
            ),
            # TTS Node (new separate package)
            Node(
                package='speech_processor',
                executable='tts_node',
                name='tts_node',
                parameters=[{
                    'api_key': os.getenv('ELEVENLABS_API_KEY', ''),
                    'provider': 'elevenlabs',
                    'voice_name': 'XrExE9yKIg1WjnnlVkGX',
                    'local_playback': False,
                    'use_cache': True,
                    'audio_quality': 'standard'
                }],
            ),
        ]
    
    def create_teleop_nodes(self) -> List[Node]:
        """Create teleoperation and joystick nodes"""
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        with_joystick = LaunchConfiguration('joystick', default='false')
        with_teleop = LaunchConfiguration('teleop', default='false')
        with_nav2 = LaunchConfiguration('nav2', default='true')
        teleop_without_nav2 = PythonExpression([
            "'", with_teleop, "' == 'true' and '", with_nav2, "' != 'true'"
        ])
        
        return [
            # Joystick node
            Node(
                package='joy',
                executable='joy_node',
                condition=IfCondition(with_joystick),
                parameters=[self.config.config_paths['joystick']]
            ),
            # Teleop twist joy node
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='go2_teleop_node',
                condition=IfCondition(with_joystick),
                parameters=[self.config.config_paths['twist_mux']],
            ),
            # Twist multiplexer
            Node(
                package='twist_mux',
                executable='twist_mux',
                output='screen',
                condition=IfCondition(teleop_without_nav2),
                parameters=[
                    {'use_sim_time': use_sim_time},
                    self.config.config_paths['twist_mux']
                ],
            ),
        ]
    
    def create_visualization_nodes(self) -> List[Node]:
        """Create visualization nodes (RViz, Foxglove)"""
        with_rviz2 = LaunchConfiguration('rviz2', default='true')
        rviz_start_delay_sec = LaunchConfiguration('rviz_start_delay_sec')
        
        return [
            # RViz2 starts after Nav2 has had time to activate its action servers.
            TimerAction(
                period=rviz_start_delay_sec,
                actions=[
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        condition=IfCondition(with_rviz2),
                        name='go2_rviz2',
                        output='screen',
                        arguments=['-d', self.config.config_paths['rviz']],
                        parameters=[{'use_sim_time': False}]
                    ),
                ],
            ),
        ]
    
    def create_include_launches(self) -> List[IncludeLaunchDescription]:
        """Create included launch descriptions"""
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        with_foxglove = LaunchConfiguration('foxglove', default='true')
        with_slam = LaunchConfiguration('slam', default='true')
        with_nav2 = LaunchConfiguration('nav2', default='true')
        nav2_params_file = LaunchConfiguration('nav2_params_file')
        nav2_start_delay_sec = LaunchConfiguration('nav2_start_delay_sec')
        
        foxglove_launch = os.path.join(
            get_package_share_directory('foxglove_bridge'),
            'launch', 'foxglove_bridge_launch.xml'
        )
        nav2_launch = os.path.join(
            self.config.package_dir,
            'launch',
            'navigation_no_docking.launch.py',
        )
        
        return [
            # Foxglove Bridge
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(foxglove_launch),
                condition=IfCondition(with_foxglove),
            ),
            # SLAM Toolbox
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('slam_toolbox'),
                                'launch', 'online_async_launch.py')
                ]),
                condition=IfCondition(with_slam),
                launch_arguments={
                    'slam_params_file': self.config.config_paths['slam'],
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
            # Nav2 needs the live SLAM map/TF and scan chain before lifecycle autostart.
            TimerAction(
                period=nav2_start_delay_sec,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([nav2_launch]),
                        condition=IfCondition(with_nav2),
                        launch_arguments={
                            'params_file': nav2_params_file,
                            'use_sim_time': use_sim_time,
                            'autostart': 'true',
                            'use_composition': 'False',
                            'use_respawn': 'False',
                            'log_level': 'info',
                        }.items(),
                    ),
                ],
            ),
        ]


def generate_launch_description():
    """Generate the launch description for Go2 robot system"""
    
    # Initialize configuration and factory
    config = Go2LaunchConfig()
    factory = Go2NodeFactory(config)
    
    # Create all components
    launch_args = factory.create_launch_arguments()
    robot_state_nodes = factory.create_robot_state_nodes()
    core_nodes = factory.create_core_nodes()
    teleop_nodes = factory.create_teleop_nodes()
    visualization_nodes = factory.create_visualization_nodes()
    include_launches = factory.create_include_launches()
    
    # Combine all elements
    launch_entities = (
        launch_args +
        robot_state_nodes +
        core_nodes +
        teleop_nodes +
        visualization_nodes +
        include_launches
    )
    
    return LaunchDescription(launch_entities)
