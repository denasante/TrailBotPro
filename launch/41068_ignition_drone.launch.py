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
#         arguments=['-topic', '/robot_description', '-name', 'parrot', '-x', '-5.0', '-y', '-5.0','-z', '2.0'] # z is height above ground
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

#     slam = IncludeLaunchDescription(
#         PathJoinSubstitution([FindPackageShare('slam_toolbox'),
#                              'launch', 'online_async_launch.py']),
#         launch_arguments={
#             'use_sim_time': use_sim_time,
#             'slam_params_file': PathJoinSubstitution([config_path, 'slam_params.yaml'])
#         }.items()
#     )
#     ld.add_action(slam)

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

#     parrot_bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         output='screen',
#         arguments=[
#             '/model/parrot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
#             '/model/parrot/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
#             '/model/parrot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
#             '/model/parrot/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
#         ],
#         remappings=[
#             ('/model/parrot/odometry', '/odometry'),
#             ('/model/parrot/imu', '/imu'),
#             ('/model/parrot/cmd_vel', '/drone/cmd_vel'),   # <-- key line
#             ('/model/parrot/camera/image', '/camera/image'),
#         ],
#         parameters=[{'use_sim_time': use_sim_time}],
#     )
#     ld.add_action(parrot_bridge)

#     drone = Node(
#     package='41068_ignition_bringup',
#     executable='dronecontroller',
#     name='dronecontroller',
#     output='screen',
#     parameters=[
#         {'use_sim_time': use_sim_time},
#         {
#             'use_obstacle_flag': True,
#             'stop_on_tags': True,
#             'stop_tag_ids': [7, 42],
#             'clear_required': 10,
#         }
#     ],
#     remappings=[
#         ('/model/parrot/odometry', '/odometry'),
#         # ('/cmd_vel', '/parrot/cmd_vel'),
#         ('/tag_detections', '/tag_detections'),
#         ('/detections/obstacle', '/detections/obstacle'),
#     ]
# )
#     ld.add_action(drone)

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

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)
    
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    # Load robot_description and start robot_state_publisher
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf_drone',
                                       'parrot.urdf.xacro'])]),
        value_type=str)
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                          'use_sim_time': use_sim_time
                                      }])
    ld.add_action(robot_state_publisher_node)

    # Publish odom -> base_link transform **using robot_localization**
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # Start Gazebo to simulate the robot in the chosen world
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    ld.add_action(world_launch_arg)
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path,
                                               'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']}.items()
    )
    ld.add_action(gazebo)

    # Spawn robot in Gazebo
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-name', 'parrot', '-x', '-5.0', '-y', '-5.0','-z', '2.0'] # z is height above ground
    )
    ld.add_action(robot_spawner)

    # Bridge topics between gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,
                                               '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # slam = IncludeLaunchDescription(
    #     PathJoinSubstitution([FindPackageShare('slam_toolbox'),
    #                          'launch', 'online_async_launch.py']),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'slam_params_file': PathJoinSubstitution([config_path, 'slam_params.yaml'])
    #     }.items()
    # )
    # ld.add_action(slam)

    # Nav2 enables mapping and waypoint following
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path,
                              'launch',
                              '41068_navigation.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    parrot_bridge = Node(
        package='ros_gz_bridge',  # or 'ros_ign_bridge' if using Ignition
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # Core telemetry
            '/model/parrot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/parrot/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/model/parrot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
    
            # Camera feeds
            '/model/parrot/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/model/parrot/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    
            # ✅ Add LiDAR bridge (GZ → ROS)
            '/model/parrot/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
        remappings=[
            ('/model/parrot/odometry', '/odometry'),
            ('/model/parrot/imu', '/imu'),
            ('/model/parrot/cmd_vel', '/drone/cmd_vel'),
            ('/model/parrot/camera/image', '/camera/image'),
            ('/model/parrot/camera/camera_info', '/camera/camera_info'),
    
            # ✅ Clean ROS scan topic
            ('/model/parrot/scan', '/scan'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    ld.add_action(parrot_bridge)

    cam_info = Node(
        package='41068_ignition_bringup',
        executable='static_camera_info_pub',
        name='static_camera_info_pub',
        output='screen',
        parameters=[{
            'camera_info_yaml': PathJoinSubstitution([config_path, 'camera_info.yaml'])
        }],
        remappings=[
            ('/camera/image', '/parrot/camera/image_raw'),
        ]
    )

    apriltag = Node(
        package='apriltag_ros', executable='apriltag_node',
        parameters=[{
            'image_transport': 'raw',
            'family': '36h11',
            'size': 0.20,                   # 20 cm tag in the world
            'max_hamming': 1
        }],
        remappings=[
            ('/image_rect', '/parrot/camera/image_raw'),
            ('/camera_info', '/camera/camera_info')
        ],
        output='screen'
    )
    
    ld.add_action(cam_info)
    ld.add_action(apriltag)

    drone = Node(
    package='41068_ignition_bringup',
    executable='dronecontroller',
    name='dronecontroller',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time},
                {'use_obstacle_flag': True, 'stop_on_tags': True,
                 'stop_tag_ids': [7, 42], 'clear_required': 10}],
    remappings=[
        ('/model/parrot/odometry', '/odometry'),
        ('/tag_detections', '/tag_detections'),
        ('/detections/obstacle', '/detections/obstacle'),
        # key line: the node publishes on "cmd_vel" → send it to /drone/cmd_vel
        ('cmd_vel', '/drone/cmd_vel'),
    ]
)
    ld.add_action(drone)

    return ld
