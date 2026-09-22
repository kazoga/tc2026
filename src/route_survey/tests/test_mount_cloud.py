"""実際のRecorder変換で、傾けたMID-360の地面がz=0へ戻ることを確認する。"""
from collections import deque
from types import SimpleNamespace
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from sensor_msgs_py.point_cloud2 import create_cloud
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from route_survey.recorder_node import Recorder
from route_survey.survey_core import Pose
from route_survey.surface_core import SurfaceWindow


def test_pitched_cloud_and_offset_are_applied_once():
    mount = Rotation.from_euler('y', 25, degrees=True)
    lever = np.array([-.4, .02, 1.04])
    x, y = np.meshgrid(np.arange(-.595, .6, .025), np.arange(-2.99, 3, .025))
    ground = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    sensor = mount.inv().apply(ground-lever)
    yaw = Rotation.from_euler('z', .5).as_matrix()
    base = np.array([10., 20., 0.])
    node = SimpleNamespace(p={'cloud_frame': 'body', 'lidar_forward_m': lever[0],
        'lidar_left_m': lever[1], 'lidar_height_m': lever[2],
        'surface_margin_m': .35, 'surface_max_seed_intensity': 30.},
        poses=deque([Pose(1., 10., 20., .5)]), attitudes=deque([(1., 0., 0., base, .5, base+yaw@lever, yaw@mount.as_matrix())]),
        surface_window=SurfaceWindow(), last_surface_compute=-math.inf, voxels={})
    node.mount_rpy = lambda: [0., math.radians(25), 0.]
    header = Header(frame_id='body'); header.stamp.sec = 1
    fields=[PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
            for i,name in enumerate(['x','y','z','intensity'])]
    points=np.column_stack([sensor,np.full(len(sensor),15.)]).astype(np.float32)
    Recorder.cloud(node, create_cloud(header, fields, points))
    assert node.width['left'] > 2.5 and node.width['right'] > 2.5
    points = np.array(list(node.voxels.values()))
    assert np.max(abs(points[:, 2])) < 1e-6
    assert points[:, 0].mean() == pytest.approx(10., abs=.1)
    assert points[:, 1].mean() == pytest.approx(20., abs=.1)
