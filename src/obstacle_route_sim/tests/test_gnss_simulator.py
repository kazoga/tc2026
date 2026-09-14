"""ROS graph を起動せず、GNSS 模擬計測と遅延・途絶を確認する."""

from collections import deque
import importlib.util
import math
from pathlib import Path
import random
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped
from geo_pose_converter.geo_core import LlhPoint, ProjectionConfig, llh_to_enu

SPEC = importlib.util.spec_from_file_location('gnss_simulator_node',
    Path(__file__).resolve().parents[1]/'tools/gnss_simulator_node.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def fixture(monkeypatch):
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: 10.)
    truth = TransformStamped()
    truth.transform.rotation.z = math.sin(math.pi/4)
    truth.transform.rotation.w = math.cos(math.pi/4)
    return SimpleNamespace(values=dict(dropout_sec=0., dropout_duration_sec=0.,
        horizontal_sigma_m=0., vertical_sigma_m=0., heading_sigma_deg=0., latency_sec=.2),
        projection=ProjectionConfig(36., 140., 50.), random=random.Random(42),
        truth=truth, last_truth=10., started=0., queue=deque())


def test_master_axle_height_slave_baseline_and_heading(monkeypatch):
    node = fixture(monkeypatch)
    MODULE.GnssSimulator.sample(node)
    due, fixes, status, imu = node.queue[0]
    assert due == 10.2
    master, slave = [llh_to_enu(LlhPoint(f.latitude,f.longitude,f.altitude),node.projection)
                     for f in fixes]
    assert (master.x,master.y,master.z) == pytest.approx((0,0,.7),abs=1e-7)
    assert (slave.x,slave.y,slave.z) == pytest.approx((0,-.5,.7),abs=1e-7)
    assert status.heading_deg == pytest.approx(0,abs=1e-6)
    assert status.baseline_length_m == .5
    assert imu.angular_velocity_covariance[0] == -1


def test_dropout_clears_pending_delivery_and_recovers(monkeypatch):
    node = fixture(monkeypatch)
    MODULE.GnssSimulator.sample(node)
    node.values.update(dropout_sec=8.,dropout_duration_sec=3.)
    MODULE.GnssSimulator.sample(node)
    assert not node.queue
    monkeypatch.setattr(MODULE.time,'monotonic',lambda: 11.)
    node.last_truth = 11.
    MODULE.GnssSimulator.sample(node)
    assert len(node.queue) == 1


def test_stale_truth_does_not_repeat_fresh_fixes(monkeypatch):
    node = fixture(monkeypatch)
    MODULE.GnssSimulator.sample(node)
    node.last_truth = 9.5
    MODULE.GnssSimulator.sample(node)
    assert not node.queue


def test_float_status_noise_covariance_and_heading(monkeypatch):
    node = fixture(monkeypatch)
    node.previous_float = True
    node.values.update(rate_hz=10.,float_heading_sigma_deg=5.)
    node.environment = SimpleNamespace(sample=lambda x,y,dt:
        dict(floating=True,bias=(1.2,1.2),sigma_m=.6))
    MODULE.GnssSimulator.sample(node)
    _,fixes,status,imu=node.queue[0]
    assert status.rtk_state == status.STATE_RTK_FLOAT
    assert fixes[0].position_covariance[0] == pytest.approx(1.8)
    assert status.heading_stddev_deg == 5.
    assert imu.orientation_covariance[8] == pytest.approx(math.radians(5.)**2)


def test_baseline_shift_changes_physical_geometry_and_measured_status(monkeypatch):
    node = fixture(monkeypatch)
    node.values.update(baseline_nominal_m=.5, baseline_shift_m=.02,
                       baseline_shift_after_s=5., baseline_sigma_m=0.)
    MODULE.GnssSimulator.sample(node)
    _, fixes, status, _ = node.queue[0]
    master, slave = [llh_to_enu(LlhPoint(f.latitude, f.longitude, f.altitude), node.projection)
                     for f in fixes]
    assert math.dist((master.x, master.y, master.z), (slave.x, slave.y, slave.z)) == pytest.approx(.52, abs=1e-7)
    assert status.baseline_length_m == pytest.approx(.52)
    node.values['baseline_sigma_m'] = .01
    MODULE.GnssSimulator.sample(node)
    assert node.queue[-1][2].baseline_length_m != status.baseline_length_m


def test_um982_raw_heading_is_master_to_slave(monkeypatch):
    node = fixture(monkeypatch)
    node.values['heading_reference'] = 'antenna_baseline'
    MODULE.GnssSimulator.sample(node)
    _, _, status, _ = node.queue[0]
    assert status.heading_deg == pytest.approx(180., abs=1e-6)
    vehicle_yaw = math.radians(90-status.heading_deg+180.)
    assert math.sin(vehicle_yaw) == pytest.approx(1.)


def test_start_fix_region_is_fixed_in_space_and_restores_building_effects(monkeypatch):
    node = fixture(monkeypatch)
    node.values.update(rate_hz=10., float_heading_sigma_deg=5., start_fix_radius_m=10.)
    node.environment = SimpleNamespace(sample=lambda x,y,dt:
        dict(floating=True,bias=(1.2,1.2),sigma_m=.6))
    node.get_logger = lambda: SimpleNamespace(info=lambda message: None)
    MODULE.GnssSimulator.sample(node)
    assert node.queue[-1][2].rtk_state == node.queue[-1][2].STATE_RTK_FIX
    assert node.queue[-1][1][0].position_covariance[0] == 0.
    node.truth.transform.translation.x = 10.1
    MODULE.GnssSimulator.sample(node)
    assert node.queue[-1][2].rtk_state == node.queue[-1][2].STATE_RTK_FLOAT
    assert node.queue[-1][1][0].position_covariance[0] == pytest.approx(1.8)
    node.truth.transform.translation.x = 2.
    MODULE.GnssSimulator.sample(node)
    assert node.queue[-1][2].rtk_state == node.queue[-1][2].STATE_RTK_FIX
    assert node.start_xy == (0., 0.)
