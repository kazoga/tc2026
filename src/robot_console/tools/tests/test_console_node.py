"""ros/console_node.py の最小テスト。

`RobotConsoleNode` はROS 2 topic購読のみを持つ薄いラッパーであり、実際の
購読・メッセージ受信を伴う動作確認は `ros2-local-run` スキルに従いノード単体
起動で行う（本テストではimportとNode生成のみを確認する）。ROS 2環境が無い
（`rclpy` をimportできない）環境では自動的にskipする。
"""

from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')

from robot_console.core.console_core import ConsoleCore  # noqa: E402
from robot_console.core.launch_profile import LaunchProfileStore  # noqa: E402
from robot_console.ros.console_node import RobotConsoleNode, start_ros_thread  # noqa: E402

REPO_PROFILE_PATH = Path(__file__).resolve().parents[2] / 'config' / 'node_launch_profiles.yaml'


def _make_core() -> ConsoleCore:
    return ConsoleCore(profile_store=LaunchProfileStore(REPO_PROFILE_PATH))


def test_robot_console_node_is_a_rclpy_node():
    if not rclpy.ok():
        rclpy.init()
    try:
        node = RobotConsoleNode(_make_core(), node_name='test_robot_console_node')
        assert node.get_name() == 'test_robot_console_node'
    finally:
        node.destroy_node()


def test_start_ros_thread_returns_running_handle_and_stops_cleanly():
    handle = start_ros_thread(_make_core(), node_name='test_robot_console_node_thread')

    assert handle.thread.is_alive()

    handle.stop()

    assert not handle.thread.is_alive()
