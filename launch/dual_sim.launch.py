from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = FindPackageShare('41068_ignition_bringup')
    cfg = PathJoinSubstitution([pkg, 'config'])

    # ---- args ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz         = LaunchConfiguration('rviz')
    nav2         = LaunchConfiguration('nav2')
    start_rover  = LaunchConfiguration('start_rover')
    start_drone  = LaunchConfiguration('start_drone')
    world        = LaunchConfiguration('world')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('rviz', default_value='True'),
        DeclareLaunchArgument('nav2', default_value='False'),
        DeclareLaunchArgument('start_rover', default_value='True'),
        DeclareLaunchArgument('start_drone', default_value='True'),
        DeclareLaunchArgument('world', default_value='simple_trees',
                              choices=['simple_trees', 'large_demo'])
    ]

    # ---- Gazebo (Ignition/GZ) ----
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'
        ])),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([pkg, 'worlds', [world, '.sdf']]), ' -r']
        }.items()
    )

    # ---- ros_ign_bridge (configured for BOTH rover + drone) ----
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([cfg, 'gazebo_bridge.yaml']),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # ========= DRONE GROUP (namespace: /parrot) =========
    drone_rdf = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf_drone', 'parrot.urdf.xacro'])]),
        value_type=str
    )

    drone_group = GroupAction([
        PushRosNamespace('parrot'),

        # robot_state_publisher (holds robot_description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': drone_rdf, 'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_parrot',
            output='screen',
            parameters=[PathJoinSubstitution([cfg, 'robot_localization.yaml']),
                        {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry', '/parrot/odometry'),
                ('imu', '/parrot/imu')
            ],
        ),

        # Spawn from parameter
        Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': drone_rdf  # inject the same description here
            }],
            arguments=['-param', 'robot_description', '-name', 'parrot', '-z', '2.0']
        )
    ], condition=IfCondition(start_drone))

    # ========= ROVER GROUP (namespace: /husky) =========
    husky_rdf = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str
    )

    rover_group = GroupAction([
        PushRosNamespace('husky'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': husky_rdf, 'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_husky',
            output='screen',
            parameters=[PathJoinSubstitution([cfg, 'robot_localization.yaml']),
                        {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry', '/odometry'),
                ('imu', '/imu'),
                ('odometry/filtered', '/husky/odometry/filtered')  # LHS no leading slash
            ],
        ),

        # Spawn from parameter
        Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': husky_rdf
            }],
            # offset to avoid collisions on spawn
            arguments=['-param', 'robot_description', '-name', 'husky',
                      '-x', '0.0', '-y', '-2.0', '-z', '0.4']
        )
    ], condition=IfCondition(start_rover))

    # ---- RViz (optional) ----
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([cfg, '41068.rviz'])],
        condition=IfCondition(rviz)
    )

    # ---- Nav2 (optional) ----
    nav2_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', '41068_navigation.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(nav2)
    )

    return LaunchDescription(args + [gz, bridge, drone_group, rover_group, rviz2, nav2_inc])
