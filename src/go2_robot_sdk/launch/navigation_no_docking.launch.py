# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Direct MPPI override. RotationShim is disabled for now because
    # FollowPath.primary_controller.* params were not reaching controller_server.
    shim_mppi_overrides = {
        'FollowPath.plugin': 'nav2_mppi_controller::MPPIController',

        'FollowPath.motion_model': 'DiffDrive',
        'FollowPath.time_steps': 12,
        'FollowPath.model_dt': 0.5,
        'FollowPath.batch_size': 250,
        'FollowPath.iteration_count': 1,

        'FollowPath.vx_min': 0.0,
        'FollowPath.vx_max': 0.35,
        'FollowPath.vy_max': 0.0,
        'FollowPath.wz_max': 0.35,

        'FollowPath.ax_max': 0.25,
        'FollowPath.ax_min': -0.25,
        'FollowPath.ay_max': 0.0,
        'FollowPath.ay_min': 0.0,
        'FollowPath.az_max': 0.60,

        'FollowPath.vx_std': 0.14,
        'FollowPath.vy_std': 0.0,
        'FollowPath.wz_std': 0.14,

        'FollowPath.temperature': 0.30,
        'FollowPath.gamma': 0.015,
        'FollowPath.visualize': False,
        'FollowPath.regenerate_noises': False,
        'FollowPath.open_loop': False,
        'FollowPath.transform_tolerance': 0.5,
        'FollowPath.prune_distance': 1.0,

        'FollowPath.critics': [
            'ConstraintCritic',
            'CostCritic',
            'GoalCritic',
            'GoalAngleCritic',
            'PathAlignCritic',
            'PathFollowCritic',
            'PathAngleCritic',
            'PreferForwardCritic',
            'VelocityDeadbandCritic',
        ],

        'FollowPath.ConstraintCritic.enabled': True,
        'FollowPath.ConstraintCritic.cost_power': 1,
        'FollowPath.ConstraintCritic.cost_weight': 4.0,

        'FollowPath.CostCritic.enabled': True,
        'FollowPath.CostCritic.cost_power': 1,
        'FollowPath.CostCritic.cost_weight': 4.0,
        'FollowPath.CostCritic.critical_cost': 300.0,
        'FollowPath.CostCritic.collision_cost': 1000000.0,
        'FollowPath.CostCritic.consider_footprint': False,
        'FollowPath.CostCritic.near_goal_distance': 0.5,
        'FollowPath.CostCritic.trajectory_point_step': 3,

        'FollowPath.GoalCritic.enabled': True,
        'FollowPath.GoalCritic.cost_power': 1,
        'FollowPath.GoalCritic.cost_weight': 8.0,
        'FollowPath.GoalCritic.threshold_to_consider': 1.2,

        'FollowPath.GoalAngleCritic.enabled': True,
        'FollowPath.GoalAngleCritic.cost_power': 1,
        'FollowPath.GoalAngleCritic.cost_weight': 1.5,
        'FollowPath.GoalAngleCritic.threshold_to_consider': 0.5,

        'FollowPath.PathAlignCritic.enabled': True,
        'FollowPath.PathAlignCritic.cost_power': 1,
        'FollowPath.PathAlignCritic.cost_weight': 2.0,
        'FollowPath.PathAlignCritic.max_path_occupancy_ratio': 0.30,
        'FollowPath.PathAlignCritic.trajectory_point_step': 6,
        'FollowPath.PathAlignCritic.threshold_to_consider': 0.6,
        'FollowPath.PathAlignCritic.offset_from_furthest': 6,
        'FollowPath.PathAlignCritic.use_path_orientations': False,

        'FollowPath.PathFollowCritic.enabled': True,
        'FollowPath.PathFollowCritic.cost_power': 1,
        'FollowPath.PathFollowCritic.cost_weight': 18.0,
        'FollowPath.PathFollowCritic.offset_from_furthest': 5,
        'FollowPath.PathFollowCritic.threshold_to_consider': 1.2,

        'FollowPath.PathAngleCritic.enabled': True,
        'FollowPath.PathAngleCritic.cost_power': 1,
        'FollowPath.PathAngleCritic.cost_weight': 0.2,
        'FollowPath.PathAngleCritic.offset_from_furthest': 4,
        'FollowPath.PathAngleCritic.threshold_to_consider': 0.5,
        'FollowPath.PathAngleCritic.max_angle_to_furthest': 1.0,
        'FollowPath.PathAngleCritic.mode': 0,

        'FollowPath.PreferForwardCritic.enabled': True,
        'FollowPath.PreferForwardCritic.cost_power': 1,
        'FollowPath.PreferForwardCritic.cost_weight': 6.0,
        'FollowPath.PreferForwardCritic.threshold_to_consider': 0.5,

        'FollowPath.VelocityDeadbandCritic.enabled': True,
        'FollowPath.VelocityDeadbandCritic.cost_power': 1,
        'FollowPath.VelocityDeadbandCritic.cost_weight': 20.0,
        'FollowPath.VelocityDeadbandCritic.deadband_velocities': [0.0, 0.0, 0.0],
    }

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {
                        'current_goal_checker': 'general_goal_checker',
                        'current_progress_checker': 'progress_checker',
                    },
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_out')],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_recovery_unused')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[
                            configured_params,
                            {
                                'current_goal_checker': 'general_goal_checker',
                                'current_progress_checker': 'progress_checker',
                            },
                        ],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_out')],
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='nav2_behaviors::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_recovery_unused')],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                    ),
                ],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
