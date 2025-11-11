#!/usr/bin/env python3
"""
WORKING Nav2 Launch for Parrot - Plugin Names Fixed
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True'
    )
    
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='parrot'
    )
    
    common_params = {'use_sim_time': use_sim_time}
    
    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=namespace,
        output='screen',
        parameters=[
            common_params,
            {
                'controller_frequency': 10.0,
                'min_x_velocity_threshold': 0.001,
                'min_y_velocity_threshold': 0.001,
                'min_theta_velocity_threshold': 0.001,
                'failure_tolerance': 0.5,
                'progress_checker_plugin': 'progress_checker',
                'goal_checker_plugins': ['general_goal_checker'],
                'controller_plugins': ['FollowPath'],
                'progress_checker.plugin': 'nav2_controller::SimpleProgressChecker',
                'progress_checker.required_movement_radius': 0.5,
                'progress_checker.movement_time_allowance': 15.0,
                'general_goal_checker.plugin': 'nav2_controller::SimpleGoalChecker',
                'general_goal_checker.stateful': True,
                'general_goal_checker.xy_goal_tolerance': 1.0,
                'general_goal_checker.yaw_goal_tolerance': 0.5,
                'FollowPath.plugin': 'dwb_core::DWBLocalPlanner',
                'FollowPath.min_vel_x': -0.5,
                'FollowPath.min_vel_y': -0.5,
                'FollowPath.max_vel_x': 2.0,
                'FollowPath.max_vel_y': 2.0,
                'FollowPath.max_vel_theta': 1.0,
                'FollowPath.min_speed_xy': 0.0,
                'FollowPath.max_speed_xy': 2.0,
                'FollowPath.acc_lim_x': 2.0,
                'FollowPath.acc_lim_y': 2.0,
                'FollowPath.acc_lim_theta': 2.0,
                'FollowPath.decel_lim_x': -2.5,
                'FollowPath.decel_lim_y': -2.5,
                'FollowPath.decel_lim_theta': -2.5,
                'FollowPath.vx_samples': 20,
                'FollowPath.vy_samples': 10,
                'FollowPath.vtheta_samples': 20,
                'FollowPath.sim_time': 1.5,
                'FollowPath.xy_goal_tolerance': 1.0,
                'FollowPath.trans_stopped_velocity': 0.25,
                'FollowPath.short_circuit_trajectory_evaluation': True,
                'FollowPath.stateful': True,
                'FollowPath.critics': ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'],
                'FollowPath.BaseObstacle.scale': 0.02,
                'FollowPath.PathAlign.scale': 32.0,
                'FollowPath.GoalAlign.scale': 24.0,
                'FollowPath.PathDist.scale': 32.0,
                'FollowPath.GoalDist.scale': 24.0,
                'FollowPath.RotateToGoal.scale': 32.0,
            }
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('odom', 'odometry/filtered')
        ]
    )
    
    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=namespace,
        output='screen',
        parameters=[
            common_params,
            {
                'expected_planner_frequency': 5.0,
                'planner_plugins': ['GridBased'],
                'GridBased.plugin': 'nav2_navfn_planner/NavfnPlanner',
                'GridBased.tolerance': 0.5,
                'GridBased.use_astar': False,
                'GridBased.allow_unknown': True,
            }
        ]
    )
    
    # Behavior Server - FIXED PLUGIN NAMES
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=namespace,
        output='screen',
        parameters=[
            common_params,
            {
                'costmap_topic': 'local_costmap/costmap_raw',
                'footprint_topic': 'local_costmap/published_footprint',
                'cycle_frequency': 10.0,
                'behavior_plugins': ['spin', 'backup', 'wait'],
                'spin.plugin': 'nav2_behaviors/Spin',  # FIXED: / not ::
                'backup.plugin': 'nav2_behaviors/BackUp',  # FIXED: / not ::
                'wait.plugin': 'nav2_behaviors/Wait',  # FIXED: / not ::
                'global_frame': 'odom',
                'robot_base_frame': 'base_link',
                'transform_tolerance': 0.5,
                'simulate_ahead_time': 2.0,
                'max_rotational_vel': 1.0,
                'min_rotational_vel': 0.4,
                'rotational_acc_lim': 2.0,
            }
        ],
        remappings=[('cmd_vel', 'cmd_vel')]
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=namespace,
        output='screen',
        parameters=[
            common_params,
            {
                'global_frame': 'map',
                'robot_base_frame': 'base_link',
                'odom_topic': '/odometry/filtered',
                'bt_loop_duration': 10,
                'default_server_timeout': 20,
                'wait_for_service_timeout': 1000,
                'plugin_lib_names': [
                    'nav2_compute_path_to_pose_action_bt_node',
                    'nav2_follow_path_action_bt_node',
                    'nav2_spin_action_bt_node',
                    'nav2_wait_action_bt_node',
                    'nav2_back_up_action_bt_node',
                    'nav2_clear_costmap_service_bt_node',
                    'nav2_is_stuck_condition_bt_node',
                    'nav2_goal_reached_condition_bt_node',
                    'nav2_goal_updated_condition_bt_node',
                    'nav2_is_path_valid_condition_bt_node',
                    'nav2_rate_controller_bt_node',
                    'nav2_distance_controller_bt_node',
                    'nav2_speed_controller_bt_node',
                    'nav2_navigate_to_pose_action_bt_node',
                    'nav2_controller_cancel_bt_node',
                ]
            }
        ],
        remappings=[('odom', 'odometry/filtered')]
    )
    
    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=namespace,
        output='screen',
        parameters=[
            common_params,
            {
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])