#!/usr/bin/env python3
"""
Parrot Drone Navigation Launch
Group 16 - 41068 Robotics Studio 1

Launches Nav2 stack for Parrot drone with proper namespacing.
This file launches Nav2 nodes directly to ensure namespace compatibility.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    ld = LaunchDescription()

    # -------------------------------------------------------------------------
    # PATHS
    # -------------------------------------------------------------------------
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])
    
    # -------------------------------------------------------------------------
    # LAUNCH ARGS
    # -------------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup Nav2'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([config_path, 'parrot_nav2_params.yaml']),
        description='Full path to Nav2 params file'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='parrot',
        description='Robot namespace'
    )
    
    ld.add_action(use_sim_time_arg)
    ld.add_action(autostart_arg)
    ld.add_action(params_file_arg)
    ld.add_action(namespace_arg)
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    
    # -------------------------------------------------------------------------
    # PARAMETER REWRITING FOR NAMESPACE
    # -------------------------------------------------------------------------
    # This ensures all parameters get the correct namespace prefix
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True
    )
    
    # -------------------------------------------------------------------------
    # NAV2 NODES
    # -------------------------------------------------------------------------
    
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]}
        ]
    )
    
    # Group all Nav2 nodes with namespace
    nav2_group = GroupAction([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager
    ])
    
    ld.add_action(nav2_group)
    
    return ld