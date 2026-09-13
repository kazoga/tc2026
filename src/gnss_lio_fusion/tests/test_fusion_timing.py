"""ROS graphを起動せず、時刻内挿・姿勢の折返しと取付位置を確認する."""
from collections import deque
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry
from gnss_lio_fusion.fusion_core import FusionFilter
from gnss_lio_fusion.fusion_node import FusionNode


def test_lio_interpolation_uses_measurement_time_and_wraps_yaw() -> None:
    node = SimpleNamespace(filter=FusionFilter(), lios=deque([
        (1., np.array([0., 0., math.radians(179)]), 0., 0.),
        (2., np.array([1., 2., math.radians(-179)]), 0., 0.),
    ]))
    pose, _, _ = FusionNode.lio_at(node, 1.25)
    assert pose[:2] == pytest.approx([.25, .5])
    assert pose[2] == pytest.approx(math.radians(179.5))
    assert FusionNode.lio_at(node, .9) is None
    assert FusionNode.lio_at(node, 2.1) is None


def test_lio_lever_arm_is_removed_before_planar_prediction() -> None:
    node = SimpleNamespace(filter=FusionFilter(), lios=deque(), events=[],
                           values={'lio_height_m': .6, 'lio_yaw_offset_deg': 0.})
    msg = Odometry()
    msg.header.stamp.sec = 1
    pitch = .2
    msg.pose.pose.orientation.y = math.sin(pitch/2)
    msg.pose.pose.orientation.w = math.cos(pitch/2)
    msg.pose.pose.position.x = 10.+.6*math.sin(pitch)
    msg.pose.pose.position.y = 20.
    FusionNode.on_lio(node, msg)
    assert node.lios[0][1][:2] == pytest.approx([10., 20.])
