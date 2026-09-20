"""実際のRecorder変換で、傾けたMID-360の地面がz=0へ戻ることを確認する。"""
from collections import deque
from types import SimpleNamespace
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from route_survey.recorder_node import Recorder
from route_survey.survey_core import Pose


def test_pitched_cloud_and_offset_are_applied_once():
    mount = Rotation.from_euler('y', 25, degrees=True)
    lever = np.array([-.4, .02, 1.04])
    x, y = np.meshgrid(np.arange(-.595, .6, .025), np.arange(-2.99, 3, .025))
    ground = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    sensor = mount.inv().apply(ground-lever)
    node = SimpleNamespace(p={'cloud_frame': 'body', 'lidar_forward_m': lever[0],
        'lidar_left_m': lever[1], 'lidar_height_m': lever[2]},
        poses=deque([Pose(1., 10., 20., .5)]), attitudes=deque([(1., 0., 0.)]),
        scan_window=deque(maxlen=60), voxels={})
    node.mount_rpy = lambda: [0., math.radians(25), 0.]
    header = Header(frame_id='body'); header.stamp.sec = 1
    Recorder.cloud(node, create_cloud_xyz32(header, sensor.astype(np.float32)))
    assert node.width['left'] > 2.5 and node.width['right'] > 2.5
    points = np.array(list(node.voxels.values()))
    assert np.max(abs(points[:, 2])) < 1e-6
    assert points[:, 0].mean() == pytest.approx(10., abs=.1)
    assert points[:, 1].mean() == pytest.approx(20., abs=.1)
