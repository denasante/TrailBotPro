#!/usr/bin/env python3
"""
Trail Guardian Dual Robot Launch - Simplified Version
Group 16 - 41068 Robotics Studio 1

FIXED VERSION:
- Removed frame_prefix from robot_state_publisher
- Fixed EKF frame names to work with namespace
- Reduced spawn timings (5s, 10s)
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

    # -------------------------------------------------------------------------
    # PATHS
    # -------------------------------------------------------------------------
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # -------------------------------------------------------------------------
    # LAUNCH ARGS
    # -------------------------------------------------------------------------
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Flag to enable use_sim_time'
    )
    world_launch_arg = DeclareLaunchArgument(
        'world', default_value='simple_trees',
        description='Which world to load', choices=['simple_trees', 'large_demo', 'forest_trail']
    )
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='False', description='Flag to launch RViz'
    )
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2', default_value='False', description='Flag to launch Nav2'
    )
    start_husky_arg = DeclareLaunchArgument(
        'start_husky', default_value='True', description='Start Husky ground vehicle'
    )
    start_drone_arg = DeclareLaunchArgument(
        'start_drone', default_value='True', description='Start Parrot drone'
    )

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(world_launch_arg)
    ld.add_action(rviz_launch_arg)
    ld.add_action(nav2_launch_arg)
    ld.add_action(start_husky_arg)
    ld.add_action(start_drone_arg)

    use_sim_time = LaunchConfiguration('use_sim_time')

    # -------------------------------------------------------------------------
    # GAZEBO (Fortress / ign)
    # -------------------------------------------------------------------------
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

    # =========================================================================
    # HUSKY (namespace: /husky)
    # =========================================================================
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
            'frame_prefix': 'husky/',  # Add namespace prefix to all TF frames
        }],
        condition=IfCondition(LaunchConfiguration('start_husky'))
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
        condition=IfCondition(LaunchConfiguration('start_husky'))
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
            '-x', '0.0', '-y', '-1.0', '-z', '0.5'
        ],
        condition=IfCondition(LaunchConfiguration('start_husky'))
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
            ('/model/husky/odometry', '/husky/odometry/raw'),
            ('/model/husky/imu', '/husky/imu/raw'),
            ('/model/husky/cmd_vel', '/husky/cmd_vel'),
            ('/model/husky/scan', '/husky/scan/raw'),
            ('/model/husky/scan/points', '/husky/scan/points'),
            ('/model/husky/camera/image', '/husky/camera/image'),
            ('/model/husky/camera/depth_image', '/husky/camera/depth_image'),
            ('/model/husky/camera/points', '/husky/camera/points'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_husky'))
    )

    husky_odom_relay = Node(
        package='41068_ignition_bringup',
        executable='frame_id_relay.py',
        name='husky_odom_frame_relay',
        parameters=[{
            'input_topic': '/husky/odometry/raw',
            'output_topic': '/husky/odometry',
            'message_type': 'nav_msgs/msg/Odometry',
            'frame_id': 'husky/odom',
            'child_frame_id': 'husky/base_link',
            'broadcast_tf': True,
            'tf_frame_id': 'husky/odom',
            'tf_child_frame_id': 'husky/base_link',
            'tf_use_msg_time': True,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(LaunchConfiguration('start_husky'))
    )

    husky_imu_relay = Node(
        package='41068_ignition_bringup',
        executable='frame_id_relay.py',
        name='husky_imu_frame_relay',
        parameters=[{
            'input_topic': '/husky/imu/raw',
            'output_topic': '/husky/imu',
            'message_type': 'sensor_msgs/msg/Imu',
            'frame_id': 'husky/base_link',
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(LaunchConfiguration('start_husky'))
    )

    husky_scan_relay = Node(
        package='41068_ignition_bringup',
        executable='frame_id_relay.py',
        name='husky_scan_frame_relay',
        parameters=[{
            'input_topic': '/husky/scan/raw',
            'output_topic': '/husky/scan',
            'message_type': 'sensor_msgs/msg/LaserScan',
            'frame_id': 'husky/base_scan',
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(LaunchConfiguration('start_husky'))
    )

    husky_group = GroupAction(
        actions=[
            husky_rsp,
            husky_ekf,
            husky_spawn,
            husky_bridge,
            husky_odom_relay,
            husky_imu_relay,
            husky_scan_relay,
        ],
        condition=IfCondition(LaunchConfiguration('start_husky'))
    )

    # Delay Husky by 5s to ensure Gazebo world fully initializes
    delayed_husky = TimerAction(period=5.0, actions=[husky_group])

    ld.add_action(delayed_husky)

    # =========================================================================
    # PARROT (namespace: /parrot)
    # =========================================================================
    parrot_robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path, 'urdf_drone', 'parrot.urdf.xacro'])]),
        value_type=str
    )

    parrot_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='parrot',
        parameters=[{
            'robot_description': parrot_robot_description_content,
            'use_sim_time': use_sim_time,
            'frame_prefix': 'parrot/',  # Add namespace prefix to all TF frames
        }],
        condition=IfCondition(LaunchConfiguration('start_drone'))
    )

    parrot_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='parrot',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization.yaml']),
            {
                'use_sim_time': use_sim_time,
                'two_d_mode': False,  # 3D for aerial
                'odom_frame': 'parrot/odom',
                'base_link_frame': 'parrot/base_link',
                'world_frame': 'parrot/odom',
            }
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered'),
            ('accel/filtered', 'accel/filtered'),
        ],
        condition=IfCondition(LaunchConfiguration('start_drone'))
    )

    parrot_spawn = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_parrot',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-name', 'parrot',
            '-topic', '/parrot/robot_description',
            '-x', '2.0', '-y', '0.0', '-z', '2.5'
        ],
        condition=IfCondition(LaunchConfiguration('start_drone'))
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
            '/model/parrot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',  # FIXED: ] = ROS->Gazebo
            '/model/parrot/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/model/parrot/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
        remappings=[
            ('/model/parrot/joint_states', '/parrot/joint_states'),
            ('/model/parrot/odometry', '/parrot/odometry/raw'),
            ('/model/parrot/imu', '/parrot/imu/raw'),
            ('/model/parrot/cmd_vel', '/parrot/cmd_vel'),
            ('/model/parrot/camera/image', '/parrot/camera/image'),
            ('/model/parrot/camera/depth_image', '/parrot/camera/depth_image'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_drone'))
    )

    parrot_odom_relay = Node(
        package='41068_ignition_bringup',
        executable='frame_id_relay.py',
        name='parrot_odom_frame_relay',
        parameters=[{
            'input_topic': '/parrot/odometry/raw',
            'output_topic': '/parrot/odometry',
            'message_type': 'nav_msgs/msg/Odometry',
            'frame_id': 'parrot/odom',
            'child_frame_id': 'parrot/base_link',
            'broadcast_tf': True,
            'tf_frame_id': 'parrot/odom',
            'tf_child_frame_id': 'parrot/base_link',
            'tf_use_msg_time': True,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(LaunchConfiguration('start_drone'))
    )

    parrot_imu_relay = Node(
        package='41068_ignition_bringup',
        executable='frame_id_relay.py',
        name='parrot_imu_frame_relay',
        parameters=[{
            'input_topic': '/parrot/imu/raw',
            'output_topic': '/parrot/imu',
            'message_type': 'sensor_msgs/msg/Imu',
            'frame_id': 'parrot/base_link',
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(LaunchConfiguration('start_drone'))
    )

    parrot_group = GroupAction(
        actions=[
            parrot_rsp,
            parrot_ekf,
            parrot_spawn,
            parrot_bridge,
            parrot_odom_relay,
            parrot_imu_relay,
        ],
        condition=IfCondition(LaunchConfiguration('start_drone'))
    )

    # Delay Parrot spawn to t=10s (5 seconds after Husky)
    delayed_parrot = TimerAction(period=10.0, actions=[parrot_group])
    ld.add_action(delayed_parrot)

    # =========================================================================
    # VISUALIZATION & NAVIGATION
    # =========================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    return ld
