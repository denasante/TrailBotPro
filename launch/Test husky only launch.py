#!/usr/bin/env python3
"""
Test Launch - Husky Only
Quick diagnostic to isolate the crash source
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Flag to enable use_sim_time'
    )
    world_launch_arg = DeclareLaunchArgument(
        'world', default_value='simple_trees',
        description='Which world to load', choices=['simple_trees', 'large_demo']
    )

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(world_launch_arg)

    use_sim_time = LaunchConfiguration('use_sim_time')

    # GAZEBO
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [
                PathJoinSubstitution([pkg_path, 'worlds', [LaunchConfiguration('world'), '.sdf']]),
                ' -r'
            ]
        }.items()
    )
    ld.add_action(gazebo)

    # HUSKY ONLY
    husky_robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str
    )

    husky_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='husky',
        parameters=[{
            'robot_description': husky_robot_description_content,
            'use_sim_time': use_sim_time,
            'frame_prefix': 'husky/'
        }],
    )

    husky_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='husky',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization.yaml']),
            {
                'use_sim_time': use_sim_time,
                'odom_frame': 'husky/odom',
                'base_link_frame': 'husky/base_link',
                'world_frame': 'husky/odom',
            }
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered'),
            ('accel/filtered', 'accel/filtered'),
        ],
    )

    husky_spawn = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_husky',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-name', 'husky',
            '-topic', '/husky/robot_description',
            '-x', '0.0', '-y', '-2.0', '-z', '0.5'
        ],
    )

    husky_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='husky_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/model/husky/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/model/husky/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/husky/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/model/husky/cmd_vel@geometry_msgs/msg/Twist[ignition.msgs.Twist',
            '/model/husky/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/husky/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/husky/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/model/husky/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/model/husky/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/model/husky/joint_states', '/husky/joint_states'),
            ('/model/husky/odometry', '/husky/odometry'),
            ('/model/husky/imu', '/husky/imu'),
            ('/model/husky/cmd_vel', '/husky/cmd_vel'),
            ('/model/husky/scan', '/husky/scan'),
            ('/model/husky/scan/points', '/husky/scan/points'),
            ('/model/husky/camera/image', '/husky/camera/image'),
            ('/model/husky/camera/depth_image', '/husky/camera/depth_image'),
            ('/model/husky/camera/points', '/husky/camera/points'),
        ],
        output='screen',
    )

    husky_group = GroupAction(
        actions=[husky_rsp, husky_ekf, husky_spawn, husky_bridge],
    )

    delayed_husky = TimerAction(period=2.0, actions=[husky_group])
    ld.add_action(delayed_husky)

    return ld