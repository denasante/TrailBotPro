from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Paths
    pkg_path    = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # --- Launch args (same style as your working file) ---
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument('rviz', default_value='False'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='True'))

    ld.add_action(DeclareLaunchArgument(
        'world', default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    ))

    # ======================================================
    # Robot descriptions (URDF XACRO -> robot_state_publisher)
    # ======================================================

    # Husky
    husky_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str
    )
    husky_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='husky_state_publisher',
        parameters=[{
            'robot_description': husky_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': 'husky/'
        }]
        # NOTE: publishes on /robot_description (default)
    )
    ld.add_action(husky_state_pub)

    # Parrot (publish on its own description topic to avoid collisions)
    parrot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf_drone', 'parrot.urdf.xacro'])]),
        value_type=str
    )
    parrot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='parrot_state_publisher',
        parameters=[{
            'robot_description': parrot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': 'parrot/'
        }],
        # Remap so Gazebo can subscribe to a distinct topic
        remappings=[('robot_description', '/parrot/robot_description')]
    )
    ld.add_action(parrot_state_pub)

    # ======================================================
    # Gazebo world (same include pattern)
    # ======================================================
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [
                PathJoinSubstitution([pkg_path, 'worlds', [LaunchConfiguration('world'), '.sdf']]),
                ' -r'
            ]
        }.items()
    )
    ld.add_action(gazebo)

    # ======================================================
    # Spawn both robots from their *description topics*
    # ======================================================
    spawn_husky = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-name', 'husky', '-topic', '/robot_description', '-z', '0.2']
    )
    ld.add_action(spawn_husky)

    spawn_parrot = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-name', 'parrot', '-topic', '/parrot/robot_description', '-z', '2.0']
    )
    ld.add_action(spawn_parrot)

    # ======================================================
    # Bridge (uses your existing YAML)
    # ======================================================
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(gazebo_bridge)

    # ======================================================
    # Robot Localization (two EKFs; inputs remapped to bridge topics)
    # (expects your config/robot_localization.yaml to have ekf_husky and ekf_parrot sections)
    # ======================================================
    ekf_husky = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_husky',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', '/husky/odometry/filtered'),
            ('odometry', '/odometry'),      # from bridge
            ('imu', '/imu')
        ]
    )
    ld.add_action(ekf_husky)

    ekf_parrot = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_parrot',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', '/parrot/odometry/filtered'),
            ('odometry', '/parrot/odometry'),
            ('imu', '/parrot/imu')
        ]
    )
    ld.add_action(ekf_parrot)

    # ======================================================
    # RViz (optional) — unchanged
    # ======================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # ======================================================
    # Nav2 / SLAM (optional) — unchanged
    # ======================================================
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    return ld