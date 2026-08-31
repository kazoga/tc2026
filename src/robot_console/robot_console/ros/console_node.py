"""robot_console 次期UI向けの ROS 2 Node実装。

`RobotConsoleNode` はROS 2 topic購読・publishのみを担当し、受け取った
メッセージをそのまま `ConsoleCore` の `update_*()` へ委譲する薄いラッパーで
ある（`robot_console_gui_architecture_design.md` 3.1節）。業務ロジック・
状態集約は一切持たない。手動操作系publisherは `ConsoleCore.bind_publishers()`
経由で送信関数として登録し、`ConsoleCore` 自体はROSに依存しない。
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rtk_gps_um982_msgs.msg import RtkStatus
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Bool, Int32, String
from tc_geo_msgs.msg import GeoPoseWithQuality
from tc_route_msgs.msg import (
    FollowerState,
    ManagerStatus,
    ObstacleAvoidanceHint,
    Route,
    RouteState,
)

from ..core.console_core import ConsoleCore

DEFAULT_NODE_NAME = 'robot_console_gui'
# rtk_gps_um982ノードのlaunch側namespace（rtk_gps）に合わせた絶対パス。
RTK_STATUS_TOPIC = '/rtk_gps/rtk_gps_um982_node/rtk_status'


class RobotConsoleNode(Node):
    """ROS 2 topicを購読し `ConsoleCore` へ橋渡しするだけのNode。"""

    def __init__(self, core: ConsoleCore, *, node_name: str = DEFAULT_NODE_NAME) -> None:
        super().__init__(node_name)
        self._core = core

        self.create_subscription(RouteState, 'route_state', self._core.update_route_state, 10)
        self.create_subscription(
            ManagerStatus, 'manager_status', self._core.update_manager_status, 10
        )
        self.create_subscription(Route, 'active_route', self._core.update_route, 10)
        self.create_subscription(
            FollowerState, 'follower_state', self._core.update_follower_state, 10
        )
        self.create_subscription(
            ObstacleAvoidanceHint,
            'obstacle_avoidance_hint',
            self._core.update_obstacle_hint,
            10,
        )
        self.create_subscription(
            PoseStamped, 'active_target', self._core.update_active_target, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            'localization/pose_enu',
            self._core.update_pose_enu,
            10,
        )
        self.create_subscription(
            GeoPoseWithQuality,
            'localization/pose_llh',
            self._core.update_pose_llh,
            10,
        )
        self.create_subscription(
            RtkStatus, RTK_STATUS_TOPIC, self._core.update_gps_status, 10
        )
        self.create_subscription(
            ImageMsg,
            'sensor_viewer',
            lambda msg: core.update_sensor_image(
                'sensor_viewer', 'Sensor Viewer', '/sensor_viewer', msg
            ),
            10,
        )
        self.create_subscription(
            ImageMsg,
            'perception/road_blockage/decision_image',
            lambda msg: core.update_sensor_image(
                'road_blockage',
                'Road Blockage',
                '/perception/road_blockage/decision_image',
                msg,
            ),
            10,
        )
        self.create_subscription(
            ImageMsg,
            'perception/traffic_signal/decision_image',
            lambda msg: core.update_sensor_image(
                'traffic_signal',
                'Traffic Signal',
                '/perception/traffic_signal/decision_image',
                msg,
            ),
            10,
        )

        # 先頭にスラッシュを付けないことで launch からの remap を可能にする。
        self._manual_pub = self.create_publisher(Bool, 'manual_start', 10)
        self._sig_pub = self.create_publisher(Int32, 'sig_recog', 10)
        self._road_pub = self.create_publisher(Bool, 'road_blocked', 10)
        self._obstacle_hint_pub = self.create_publisher(
            ObstacleAvoidanceHint, 'obstacle_avoidance_hint', 10
        )
        self._frame_image_path_pub = self.create_publisher(String, '/frame_image_path', 10)

        core.bind_publishers(
            manual_start=lambda value: self._manual_pub.publish(Bool(data=value)),
            sig_recog=lambda value: self._sig_pub.publish(Int32(data=value)),
            road_blocked=lambda value: self._road_pub.publish(Bool(data=value)),
            obstacle_hint=lambda active, clearance, left, right: self._obstacle_hint_pub.publish(
                ObstacleAvoidanceHint(
                    front_blocked=active,
                    front_clearance_m=clearance,
                    left_offset_m=left,
                    right_offset_m=right,
                )
            ),
            frame_image=lambda path: self._frame_image_path_pub.publish(String(data=path)),
        )


@dataclass
class RosHandle:
    """バックグラウンドで動かしているROS 2 Nodeへのハンドル。"""

    node: RobotConsoleNode
    thread: threading.Thread

    def stop(self) -> None:
        """Nodeを破棄しROSスレッドを安全に停止する。"""

        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.thread.join(timeout=5.0)


def start_ros_thread(core: ConsoleCore, *, node_name: str = DEFAULT_NODE_NAME) -> RosHandle:
    """`RobotConsoleNode` を生成し、別スレッドでexecutorを回す。

    Qtイベントループ（`ui_qt_main.py`）やHTTPサーバスレッド（`web_main.py`）と
    rclpy executorを分離するために用いる（architecture_design.md 14.1節）。
    """

    if not rclpy.ok():
        rclpy.init()
    node = RobotConsoleNode(core, node_name=node_name)

    def _spin() -> None:
        try:
            rclpy.spin(node)
        except Exception:  # pragma: no cover - shutdown時の例外は無視する
            pass

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    return RosHandle(node=node, thread=thread)
