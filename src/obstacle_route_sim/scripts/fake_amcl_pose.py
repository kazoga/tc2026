#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gazebo 真値 pose から /amcl_pose 互換 PoseWithCovarianceStamped を生成する."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class GazeboTrueAmclPose(Node):
    """Gazebo 上の robot モデル真値を AMCL pose として publish するノード."""

    def __init__(self) -> None:
        """通信設定を初期化する."""

        super().__init__("fake_amcl_pose")
        self.declare_parameter("pose_topic", "/gazebo/dynamic_pose_info")
        self.declare_parameter("amcl_topic", "/amcl_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("target_pose_index", 0)

        pose_topic = str(self.get_parameter("pose_topic").value)
        amcl_topic = str(self.get_parameter("amcl_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.target_pose_index = int(self.get_parameter("target_pose_index").value)
        self._missing_pose_count = 0

        self.create_subscription(TFMessage, pose_topic, self._on_pose_info, 10)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, amcl_topic, 10)
        self.get_logger().info(
            f"fake_amcl_pose started: {pose_topic} -> {amcl_topic}, "
            f"frame={self.frame_id}, target_pose_index={self.target_pose_index}"
        )

    def _on_pose_info(self, msg: TFMessage) -> None:
        """Gazebo dynamic pose info から robot モデルの真値 pose を抽出する."""

        if self.target_pose_index < 0 or self.target_pose_index >= len(msg.transforms):
            self._missing_pose_count += 1
            if self._missing_pose_count == 1 or self._missing_pose_count % 200 == 0:
                self.get_logger().warn(
                    "Gazebo dynamic pose info に対象 index がありません: "
                    f"target_pose_index={self.target_pose_index}, "
                    f"transforms={len(msg.transforms)}"
                )
            return

        transform = msg.transforms[self.target_pose_index]
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose.pose.position.x = transform.transform.translation.x
        pose.pose.pose.position.y = transform.transform.translation.y
        pose.pose.pose.position.z = transform.transform.translation.z
        pose.pose.pose.orientation = transform.transform.rotation
        pose.pose.covariance = [0.0] * 36
        self.publisher.publish(pose)
        self._missing_pose_count = 0


def main(args=None) -> None:
    """エントリーポイント."""

    rclpy.init(args=args)
    node = GazeboTrueAmclPose()
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
