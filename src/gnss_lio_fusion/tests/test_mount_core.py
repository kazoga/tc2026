"""25度取付・旋回・車体傾斜で、原点補正が架空の移動を生まないことを検査する。"""
from collections import deque
import math
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from gnss_lio_fusion.mount_core import base_from_sensor, horizontal_lever
from gnss_lio_fusion.fusion_node import FusionNode
from gnss_lio_fusion.fusion_core import FusionFilter
from nav_msgs.msg import Odometry


@pytest.mark.parametrize('angles', [[0, 0, 0], [0, 0, 90], [10, -12, 170]])
def test_mounted_sensor_recovers_vehicle_pose(angles):
    vehicle = Rotation.from_euler('xyz', angles, degrees=True)
    mount = Rotation.from_euler('y', 25, degrees=True)
    position = np.array([4., 5., .2])
    lever = np.array([-.425, .02, 1.005])
    measured = position + vehicle.apply(lever)
    result, rpy = base_from_sensor(measured, (vehicle*mount).as_quat(), lever,
                                   [0., math.radians(25), 0.])
    assert result == pytest.approx(position)
    assert rpy == pytest.approx(np.radians(angles))
    assert horizontal_lever(lever, *rpy) == pytest.approx(vehicle.apply(lever))


def test_lio_rotates_in_place_without_lever_arm_motion():
    lever = np.array([-.425, .02, 1.005])
    node = SimpleNamespace(filter=FusionFilter(), lios=deque(), events=[], values={
        'lio_height_m': lever[2], 'lio_forward_m': lever[0], 'lio_left_m': lever[1],
        'lio_mount_roll_deg': 0., 'lio_mount_pitch_deg': 25., 'lio_mount_yaw_deg': 0.,
        'lio_yaw_offset_deg': 0.})
    for i, angle in enumerate([0., 30., 90.]):
        base = Rotation.from_euler('z', angle, degrees=True)
        sensor = base*Rotation.from_euler('y', 25, degrees=True)
        msg = Odometry()
        msg.header.stamp.sec = i+1
        xyz = np.array([10., 20., 0.])+base.apply(lever)
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = xyz
        q = msg.pose.pose.orientation
        q.x, q.y, q.z, q.w = sensor.as_quat()
        FusionNode.on_lio(node, msg)
    for sample in node.lios:
        assert sample[1][:2] == pytest.approx([10., 20.])
        assert sample[2:] == pytest.approx([0., 0.])
