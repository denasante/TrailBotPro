# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.conditions import IfCondition
# from launch.substitutions import (Command, LaunchConfiguration,
#                                   PathJoinSubstitution)
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():

#     ld = LaunchDescription()

#     # Get paths to directories
#     pkg_path = FindPackageShare('41068_ignition_bringup')
#     config_path = PathJoinSubstitution([pkg_path,
#                                        'config'])

#     # Additional command line arguments
#     use_sim_time_launch_arg = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='True',
#         description='Flag to enable use_sim_time'
#     )
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     ld.add_action(use_sim_time_launch_arg)
#     rviz_launch_arg = DeclareLaunchArgument(
#         'rviz',
#         default_value='False',
#         description='Flag to launch RViz'
#     )
#     ld.add_action(rviz_launch_arg)
#     nav2_launch_arg = DeclareLaunchArgument(
#         'nav2',
#         default_value='True',
#         description='Flag to launch Nav2'
#     )
#     ld.add_action(nav2_launch_arg)

#     # Load robot_description and start robot_state_publisher
#     robot_description_content = ParameterValue(
#         Command(['xacro ',
#                  PathJoinSubstitution([pkg_path,
#                                        'urdf_drone',
#                                        'parrot.urdf.xacro'])]),
#         value_type=str)
#     robot_state_publisher_node = Node(package='robot_state_publisher',
#                                       executable='robot_state_publisher',
#                                       parameters=[{
#                                           'robot_description': robot_description_content,
#                                           'use_sim_time': use_sim_time
#                                       }])
#     ld.add_action(robot_state_publisher_node)

#     # Publish odom -> base_link transform **using robot_localization**
#     robot_localization_node = Node(
#         package='robot_localization',
#         executable='ekf_node',
#         name='robot_localization',
#         output='screen',
#         parameters=[PathJoinSubstitution([config_path,
#                                           'robot_localization.yaml']),
#                     {'use_sim_time': use_sim_time}]
#     )
#     ld.add_action(robot_localization_node)

#     # Start Gazebo to simulate the robot in the chosen world
#     world_launch_arg = DeclareLaunchArgument(
#         'world',
#         default_value='simple_trees',
#         description='Which world to load',
#         choices=['simple_trees', 'large_demo']
#     )
#     ld.add_action(world_launch_arg)
#     gazebo = IncludeLaunchDescription(
#         PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
#                              'launch', 'ign_gazebo.launch.py']),
#         launch_arguments={
#             'ign_args': [PathJoinSubstitution([pkg_path,
#                                                'worlds',
#                                                [LaunchConfiguration('world'), '.sdf']]),
#                          ' -r']}.items()
#     )
#     ld.add_action(gazebo)

#     # Spawn robot in Gazebo
#     robot_spawner = Node(
#         package='ros_ign_gazebo',
#         executable='create',
#         output='screen',
#         parameters=[{'use_sim_time': use_sim_time}],
#         arguments=['-topic', '/robot_description', '-z', '2.0'] # z is height above ground
#     )
#     ld.add_action(robot_spawner)

#     # Bridge topics between gazebo and ROS2
#     gazebo_bridge = Node(
#         package='ros_ign_bridge',
#         executable='parameter_bridge',
#         parameters=[{'config_file': PathJoinSubstitution([config_path,
#                                                           'gazebo_bridge.yaml']),
#                     'use_sim_time': use_sim_time}]
#     )
#     ld.add_action(gazebo_bridge)

#     # rviz2 visualises data
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         output='screen',
#         parameters=[{'use_sim_time': use_sim_time}],
#         arguments=['-d', PathJoinSubstitution([config_path,
#                                                '41068.rviz'])],
#         condition=IfCondition(LaunchConfiguration('rviz'))
#     )
#     ld.add_action(rviz_node)

#     # Nav2 enables mapping and waypoint following
#     nav2 = IncludeLaunchDescription(
#         PathJoinSubstitution([pkg_path,
#                               'launch',
#                               '41068_navigation.launch.py']),
#         launch_arguments={
#             'use_sim_time': use_sim_time
#         }.items(),
#         condition=IfCondition(LaunchConfiguration('nav2'))
#     )
#     ld.add_action(nav2)

#     return ld


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # --- Launch Arguments ---
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument('rviz', default_value='False'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='True'))

    # --- NEW: Drone-specific topic args ---
    ld.add_action(DeclareLaunchArgument('odom_topic', default_value='/parrot/odometry'))
    ld.add_action(DeclareLaunchArgument('imu_topic',  default_value='/parrot/imu'))
    odom_topic = LaunchConfiguration('odom_topic')
    imu_topic  = LaunchConfiguration('imu_topic')

    # --- Robot Description ---
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf_drone',
                                       'parrot.urdf.xacro'])]),
        value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(robot_state_publisher_node)

    # --- Robot Localization (EKF) ---
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        # ✅ Remap EKF input topics to drone’s bridge topics
        remappings=[
            ('odometry', '/parrot/odometry'),
            ('imu', '/parrot/imu'),
        ],
    )
    ld.add_action(robot_localization_node)

    # --- Gazebo world ---
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    ld.add_action(world_launch_arg)

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'),
            'launch',
            'ign_gazebo.launch.py'
        ]),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([
                pkg_path,
                'worlds',
                [LaunchConfiguration('world'), '.sdf']
            ]), ' -r']
        }.items()
    )
    ld.add_action(gazebo)

    # --- Spawn the drone ---
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '2.0']
    )
    ld.add_action(robot_spawner)

    # --- Bridge Gazebo <-> ROS2 topics ---
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(gazebo_bridge)

    # --- RViz (optional) ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # --- Nav2 / SLAM (optional) ---
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    return ld