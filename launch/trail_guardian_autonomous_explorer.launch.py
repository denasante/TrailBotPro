#!/usr/bin/env python3
"""
WORKING Trail Guardian with Nav2 - V2 with Fixed Plugin Names
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
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    ld = LaunchDescription()

    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # Launch args
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')
    world_arg = DeclareLaunchArgument('world', default_value='simple_trees')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='True')
    enable_slam_arg = DeclareLaunchArgument('enable_slam', default_value='True')
    enable_nav2_arg = DeclareLaunchArgument('enable_nav2', default_value='True')

    ld.add_action(use_sim_time_arg)
    ld.add_action(world_arg)
    ld.add_action(rviz_arg)
    ld.add_action(enable_slam_arg)
    ld.add_action(enable_nav2_arg)

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Gazebo
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

    # PARROT STAGE 1: Spawn (t=5s)
    parrot_robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf_drone', 'parrot.urdf.xacro'])]),
        value_type=str
    )

    parrot_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='parrot',
        parameters=[{
            'robot_description': parrot_robot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': 'parrot/',
        }]
    )

    parrot_spawn = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_parrot',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-name', 'parrot', '-topic', '/parrot/robot_description',
                   '-x', '2.0', '-y', '0.0', '-z', '2.5']
    )

    parrot_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='parrot_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/model/parrot/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/model/parrot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/parrot/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/model/parrot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/parrot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/parrot/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/model/parrot/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
        remappings=[
            ('/model/parrot/joint_states', '/parrot/joint_states'),
            ('/model/parrot/odometry', '/parrot/odometry'),
            ('/model/parrot/imu', '/parrot/imu'),
            ('/model/parrot/cmd_vel', '/parrot/cmd_vel'),
            ('/model/parrot/scan', '/parrot/scan'),
            ('/model/parrot/camera/image', '/parrot/camera/image'),
            ('/model/parrot/camera/depth_image', '/parrot/camera/depth_image'),
        ],
        output='screen'
    )

    delayed_stage1 = TimerAction(
        period=5.0,
        actions=[GroupAction([parrot_rsp, parrot_spawn, parrot_bridge])]
    )
    ld.add_action(delayed_stage1)

    # PARROT STAGE 2: EKF (t=10s)
    parrot_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='parrot',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'parrot_robot_localization.yaml']),
            {
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
            }
        ],
        remappings=[('odometry/filtered', 'odometry/filtered')]
    )

    delayed_stage2 = TimerAction(period=10.0, actions=[parrot_ekf])
    ld.add_action(delayed_stage2)

    # PARROT STAGE 3: SLAM (t=15s)
    parrot_slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='parrot',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'parrot_slam_params.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(LaunchConfiguration('enable_slam'))
    )

    delayed_stage3 = TimerAction(period=15.0, actions=[parrot_slam])
    ld.add_action(delayed_stage3)

    # PARROT STAGE 4: NAV2 (t=20s) - V2 with fixed plugin names
    parrot_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_path, 'launch', 'parrot_nav2_direct_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': 'parrot',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_nav2'))
    )
    
    delayed_stage4 = TimerAction(period=20.0, actions=[parrot_nav2])
    ld.add_action(delayed_stage4)

    # RVIZ (t=3s)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    delayed_rviz = TimerAction(period=3.0, actions=[rviz_node])
    ld.add_action(delayed_rviz)

    return ld