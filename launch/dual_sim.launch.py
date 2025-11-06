from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, GroupAction,
    TimerAction, OpaqueFunction, SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

import os, subprocess, xml.etree.ElementTree as ET

# ---------- Helpers ----------
def _run_xacro(path: str) -> str:
    try:
        return subprocess.check_output(['xacro', path], stderr=subprocess.STDOUT).decode('utf-8')
    except (FileNotFoundError, subprocess.CalledProcessError):
        return subprocess.check_output(['python3', '-m', 'xacro', path], stderr=subprocess.STDOUT).decode('utf-8')

def _tag_name(node):
    return node.tag.split('}', 1)[1] if '}' in node.tag else node.tag

def _to_bool(s) -> bool:
    return str(s).lower() in ('1', 'true', 'yes', 'on')

def _strip_visual_sensors(urdf_xml: str, kill_types=('camera', 'depth')) -> str:
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError:
        return urdf_xml

    kill_words = tuple(kill_types)
    kill_links = set()

    for link in list(root.findall('link')):
        name = link.attrib.get('name', '')
        if any(kw in name.lower() for kw in kill_words):
            kill_links.add(name)
            root.remove(link)

    for joint in list(root.findall('joint')):
        jname = joint.attrib.get('name', '').lower()
        parent = joint.find('parent'); child = joint.find('child')
        parent_link = parent.attrib.get('link', '') if parent is not None else ''
        child_link  = child.attrib.get('link', '')  if child  is not None else ''
        if any(kw in jname for kw in kill_words) or parent_link in kill_links or child_link in kill_links:
            root.remove(joint)

    def _should_remove_gz(elem):
        blob = ET.tostring(elem, encoding='utf-8').decode('utf-8').lower()
        if any(kw in blob for kw in kill_words):
            return True
        ref = elem.attrib.get('reference', '')
        if ref and (ref in kill_links or any(kw in ref.lower() for kw in kill_words)):
            return True
        return False

    for gz in list(root.findall('gazebo')):
        if _should_remove_gz(gz):
            root.remove(gz)
    for elem in list(root):
        if _tag_name(elem) == 'gazebo':
            blob = ET.tostring(elem, encoding='utf-8').decode('utf-8').lower()
            if any(kw in blob for kw in kill_words):
                try: root.remove(elem)
                except: pass

    for joint in list(root.findall('joint')):
        parent = joint.find('parent'); child = joint.find('child')
        parent_link = parent.attrib.get('link', '') if parent is not None else ''
        child_link  = child.attrib.get('link', '')  if child  is not None else ''
        if parent_link in kill_links or child_link in kill_links:
            root.remove(joint)

    return ET.tostring(root, encoding='unicode')

def _strip_gz_system(urdf_xml: str, system_name_substr='gazebo::systems::Sensors') -> str:
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError:
        return urdf_xml

    for gz in list(root.findall('gazebo')):
        blob = ET.tostring(gz, encoding='utf-8').decode('utf-8')
        if system_name_substr in blob:
            root.remove(gz)

    for elem in list(root):
        if _tag_name(elem) == 'gazebo':
            blob = ET.tostring(elem, encoding='utf-8').decode('utf-8')
            if system_name_substr in blob:
                try: root.remove(elem)
                except: pass

    return ET.tostring(root, encoding='unicode')

# ---------- Spawners ----------
def _spawn_husky(context, *args, **kwargs):
    use_sim = _to_bool(LaunchConfiguration('use_sim_time').perform(context))
    strip_on = LaunchConfiguration('strip_cameras_on').perform(context).lower()
    pkg_share = FindPackageShare('41068_ignition_bringup').perform(context)

    urdf = _run_xacro(os.path.join(pkg_share, 'urdf', 'husky.urdf.xacro'))
    for key in ('ignition::gazebo::systems::Sensors', 'gazebo::systems::Sensors', 'gz::sim::systems::Sensors'):
        urdf = _strip_gz_system(urdf, key)
    if strip_on == 'husky':
        urdf = _strip_visual_sensors(urdf, kill_types=('camera', 'depth'))

    cfg = os.path.join(pkg_share, 'config', 'robot_localization.yaml')

    return [GroupAction([
        PushRosNamespace('husky'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf,
                          'use_sim_time': use_sim,
                          'frame_prefix': 'husky/',
                          'use_tf_static': False}],   # <<< publish static frames on /tf
             output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_husky',
             parameters=[cfg, {
                 'use_sim_time': use_sim,
                 'publish_tf': True,
                 'base_link_frame': 'husky/base_link',
                 'odom_frame':      'husky/odom',
                 'world_frame':     'husky/odom'
             }],
             remappings=[('odometry', '/husky/odometry'),
                         ('imu', '/husky/imu'),
                         ('odometry/filtered', '/husky/odometry/filtered')],
             output='screen'),
        Node(package='ros_ign_gazebo', executable='create', output='screen',
             parameters=[{'use_sim_time': use_sim, 'robot_description': urdf}],
             arguments=['-param', 'robot_description', '-name', 'husky', '-x', '0.0', '-y', '-2.0', '-z', '0.4']),
    ])]

def _spawn_parrot(context, *args, **kwargs):
    use_sim = _to_bool(LaunchConfiguration('use_sim_time').perform(context))
    strip_on = LaunchConfiguration('strip_cameras_on').perform(context).lower()
    pkg_share = FindPackageShare('41068_ignition_bringup').perform(context)

    urdf = _run_xacro(os.path.join(pkg_share, 'urdf_drone', 'parrot.urdf.xacro'))
    for key in ('ignition::gazebo::systems::Sensors', 'gazebo::systems::Sensors', 'gz::sim::systems::Sensors'):
        urdf = _strip_gz_system(urdf, key)
    if strip_on == 'parrot':
        urdf = _strip_visual_sensors(urdf, kill_types=('camera', 'depth'))

    cfg = os.path.join(pkg_share, 'config', 'robot_localization.yaml')

    return [GroupAction([
        PushRosNamespace('parrot'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf,
                          'use_sim_time': use_sim,
                          'frame_prefix': 'parrot/',
                          'use_tf_static': False}],   # <<< publish static frames on /tf
             output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_parrot',
             parameters=[cfg, {
                 'use_sim_time': use_sim,
                 'publish_tf': True,
                 'base_link_frame': 'parrot/base_link',
                 'odom_frame':      'parrot/odom',
                 'world_frame':     'parrot/odom'
             }],
             remappings=[('odometry', '/parrot/odometry'),
                         ('imu', '/parrot/imu')],
             output='screen'),
        Node(package='ros_ign_gazebo', executable='create', output='screen',
             parameters=[{'use_sim_time': use_sim, 'robot_description': urdf}],
             arguments=['-param', 'robot_description', '-name', 'parrot', '-z', '2.0']),
    ])]

# ---------- Main LD ----------
def generate_launch_description():
    pkg = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup')])
    cfg = PathJoinSubstitution([pkg, 'config'])

    use_sim_time   = LaunchConfiguration('use_sim_time')
    gui            = LaunchConfiguration('gui')
    paused         = LaunchConfiguration('paused')
    world_file     = LaunchConfiguration('world_file')
    start_rover    = LaunchConfiguration('start_rover')
    start_drone    = LaunchConfiguration('start_drone')
    rviz           = LaunchConfiguration('rviz')
    nav2           = LaunchConfiguration('nav2')
    spawn_delay_s  = LaunchConfiguration('spawn_delay_s')
    headless       = LaunchConfiguration('headless')
    server_only    = LaunchConfiguration('server_only')
    strip_cameras_on = LaunchConfiguration('strip_cameras_on')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('gui',          default_value='True'),
        DeclareLaunchArgument('paused',       default_value='False'),
        DeclareLaunchArgument('world_file',   default_value='simple_trees.sdf'),
        DeclareLaunchArgument('start_rover',  default_value='True'),
        DeclareLaunchArgument('start_drone',  default_value='True'),
        DeclareLaunchArgument('rviz',         default_value='False'),
        DeclareLaunchArgument('nav2',         default_value='False'),
        DeclareLaunchArgument('spawn_delay_s', default_value='7.0'),
        DeclareLaunchArgument('headless',      default_value='False'),
        DeclareLaunchArgument('server_only',   default_value='False'),
        DeclareLaunchArgument('strip_cameras_on', default_value='none'),  # parrot|husky|none
    ]

    env = [
        SetEnvironmentVariable('IGN_HEADLESS_RENDERING', headless),
        SetEnvironmentVariable('GZ_SIM_HEADLESS_RENDERING', headless),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '0'),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '4.5'),
        SetEnvironmentVariable('MESA_GLSL_VERSION_OVERRIDE', '450'),
    ]

    world_path = PathJoinSubstitution([pkg, 'worlds', world_file])
    gz_args = PythonExpression([
        '"', world_path, '"',
        ' + (" -s" if "', server_only, '" == "True" else "")',
        ' + (" -r" if "', paused, '" == "False" else "")'
    ])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'
        ])),
        launch_arguments={'gz_args': gz_args, 'gui': gui}.items()
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': PathJoinSubstitution([cfg, 'gazebo_bridge.yaml']),
                     'use_sim_time': use_sim_time}]
    )

    # Anchors on /tf (NOT /tf_static)
    husky_anchor = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='world_to_husky_odom',
        arguments=['0','0','0','0','0','0','world','husky/odom'],
        parameters=[{'use_tf_static': False}],
        output='screen'
    )
    parrot_anchor = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='world_to_parrot_odom',
        arguments=['0','0','0','0','0','0','world','parrot/odom'],
        parameters=[{'use_tf_static': False}],
        output='screen'
    )

    husky_of  = OpaqueFunction(function=_spawn_husky)
    parrot_of = OpaqueFunction(function=_spawn_parrot)

    actions = env + [
        gz, bridge,
        husky_anchor, parrot_anchor,
        GroupAction([husky_of], condition=IfCondition(start_rover)),
        TimerAction(period=spawn_delay_s, actions=[GroupAction([parrot_of])], condition=IfCondition(start_drone)),
        Node(package='rviz2', executable='rviz2', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['-d', PathJoinSubstitution([cfg, '41068.rviz'])],
             condition=IfCondition(rviz)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', '41068_navigation.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(nav2)
        )
    ]

    return LaunchDescription(args + actions)
