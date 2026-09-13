#!/usr/bin/env python3
"""FAST-LIO の登録済み点群を時刻付き分割ファイルへ保存する."""
import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class LioMapRecorder(Node):
    """推定済み camera_init 点群だけを保存し、真値補正を行わない."""

    def __init__(self, output: Path) -> None:
        super().__init__('lio_map_recorder')
        self.output = output
        output.mkdir(parents=True, exist_ok=False)
        self.previous = -float('inf')
        self.count = 0
        self.create_subscription(PointCloud2, '/cloud_registered', self.on_cloud, 10)

    def on_cloud(self, message: PointCloud2) -> None:
        """1 Hz の観測を 0.25 m voxel で間引き、メモリ使用を制限する."""
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if stamp-self.previous < 1.0:
            return
        data = point_cloud2.read_points(message, field_names=['x', 'y', 'z'])
        xyz = np.column_stack([data[k].reshape(-1) for k in ['x', 'y', 'z']])
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if not len(xyz):
            return
        _, indices = np.unique(np.floor(xyz/.25).astype(np.int64), axis=0, return_index=True)
        np.savez_compressed(self.output/f'{self.count:06d}.npz',
                            xyz=xyz[indices].astype(np.float32), stamp=stamp)
        self.previous = stamp
        self.count += 1
        if self.count % 60 == 1:
            self.get_logger().info(f'登録済み点群を保存: {self.count} 観測')
        (self.output/'manifest.json').write_text(json.dumps(dict(
            source_topic='/cloud_registered', frame=message.header.frame_id,
            interval_s=1.0, voxel_m=.25, scans=self.count, last_stamp=stamp)))


def main() -> None:
    """外部からの停止で保存済み分割点群を維持する."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = LioMapRecorder(args.output)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
