"""Record all ordinary topics, replacing the cumulative map with 10 s snapshots."""
import argparse
import math
import os
import signal
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class MapRecordRelay(Node):
    def __init__(self, period_s=10.0):
        if not math.isfinite(period_s) or period_s <= 0:
            raise ValueError('Map recording period must be positive and finite')
        super().__init__('map_record_relay')
        self._latest = None
        self._publisher = self.create_publisher(PointCloud2, '/Laser_map_record', 1)
        self._subscription = self.create_subscription(
            PointCloud2, '/Laser_map', self._receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._timer = self.create_timer(period_s, self._publish_latest)

    def _receive(self, message):
        # Keep one full snapshot; preserve the source frame, timestamp and data.
        self._latest = message

    def _publish_latest(self):
        if self._latest is not None:
            self._publisher.publish(self._latest)
            self._latest = None  # Do not repeat stale maps if the source stops.


def record_command(output, max_bag_size):
    return ['ros2', 'bag', 'record', '--all-topics', '--storage', 'sqlite3',
            '--exclude-regex', '^/Laser_map$', '--output', output,
            '--max-bag-size', str(max_bag_size), '--disable-keyboard-controls']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--max-bag-size', type=int, default=1024**3)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = None
    process = None
    result = 0
    try:
        node = MapRecordRelay()
        # Forward one stop signal to the recorder group; avoid duplicate SIGINT.
        process = subprocess.Popen(record_command(args.output, args.max_bag_size),
                                   stdin=subprocess.DEVNULL, start_new_session=True)
        while rclpy.ok() and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=.2)
        if process.poll() is not None:
            result = process.returncode
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        result = 1
        raise
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=3.)
                    result = 1
            if process.returncode:
                result = 1
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(result)
