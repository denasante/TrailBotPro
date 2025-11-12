from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="41068_ignition_bringup",
                executable="control_ui",
                name="control_ui",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
