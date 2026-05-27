#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Odometry から odom -> base_link の TF を publish する."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    """Odometry の pose を TF として配信するノード."""

    def __init__(self) -> None:
        """通信設定を初期化する."""

        super().__init__("odom_tf_broadcaster")
        self.declare_parameter("odom_topic", "/ypspur_ros/odom")
        self.declare_parameter("parent_frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        odom_topic = str(self.get_parameter("odom_topic").value)
        self.parent_frame_id = str(self.get_parameter("parent_frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.get_logger().info(
            f"odom_tf_broadcaster started: {odom_topic} -> "
            f"{self.parent_frame_id}/{self.child_frame_id}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        """Odometry を TransformStamped として publish する."""

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id or self.parent_frame_id
        transform.child_frame_id = msg.child_frame_id or self.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main(args=None) -> None:
    """エントリーポイント."""

    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
