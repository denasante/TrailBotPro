#!/usr/bin/env python3
"""Generic ROS 2 relay that overwrites frame IDs and optionally broadcasts TF."""
from __future__ import annotations

import copy
import importlib

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FrameIdRelay(Node):
    def __init__(self) -> None:
        super().__init__('frame_id_relay')

        self._input_topic = self._declare_and_get_str('input_topic')
        self._output_topic = self._declare_and_get_str('output_topic')
        self._type_string = self._declare_and_get_str('message_type')
        self._frame_id = self._declare_and_get_str('frame_id')
        self._child_frame_id = self._declare_and_get_str('child_frame_id')
        depth = int(self.declare_parameter('qos_depth', 10).value)

        if not self._input_topic or not self._output_topic or not self._type_string:
            raise RuntimeError('input_topic, output_topic, and message_type parameters are required')

        self._msg_type = self._resolve_msg_type(self._type_string)
        self._pub = self.create_publisher(self._msg_type, self._output_topic, depth)
        self._sub = self.create_subscription(self._msg_type, self._input_topic, self._callback, depth)

        self._broadcast_tf = bool(self.declare_parameter('broadcast_tf', False).value)
        self._tf_parent = self._declare_and_get_str('tf_frame_id') or self._frame_id
        self._tf_child = self._declare_and_get_str('tf_child_frame_id') or self._child_frame_id
        self._tf_from_msg_time = bool(self.declare_parameter('tf_use_msg_time', True).value)
        self._tf_broadcaster: TransformBroadcaster | None = TransformBroadcaster(self) if self._broadcast_tf else None

        self.get_logger().info(
            f"Relaying {self._input_topic} -> {self._output_topic} as {self._type_string} "
            f"(frame_id={self._frame_id or 'unchanged'}, child_frame_id={self._child_frame_id or 'unchanged'}, "
            f"broadcast_tf={self._broadcast_tf})"
        )

    def _declare_and_get_str(self, name: str, default: str = '') -> str:
        return str(self.declare_parameter(name, default).value)

    def _resolve_msg_type(self, type_string: str):
        try:
            module_path, class_name = type_string.rsplit('/', 1)
            module = importlib.import_module(module_path.replace('/', '.'))
            return getattr(module, class_name)
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(f'Failed to import message type {type_string}: {exc}') from exc

    def _callback(self, msg) -> None:
        relay_msg = copy.deepcopy(msg)
        if self._frame_id and hasattr(relay_msg, 'header'):
            relay_msg.header.frame_id = self._frame_id
        if self._child_frame_id and hasattr(relay_msg, 'child_frame_id'):
            relay_msg.child_frame_id = self._child_frame_id

        self._pub.publish(relay_msg)

        if self._broadcast_tf and self._tf_broadcaster:
            self._broadcast_tf_from_msg(relay_msg)

    def _broadcast_tf_from_msg(self, msg) -> None:
        # Only nav_msgs/Odometry is supported for TF broadcasting.
        if not isinstance(msg, Odometry):
            return
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp if self._tf_from_msg_time else self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self._tf_parent or msg.header.frame_id
        tf_msg.child_frame_id = self._tf_child or msg.child_frame_id
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrameIdRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
