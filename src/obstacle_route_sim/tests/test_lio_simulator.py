"""LIO の入力整形と IMU 同位置配置を確認する."""
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

TOOLS=Path(__file__).parents[1]/'tools'
sys.path.insert(0,str(TOOLS))
from prepare_fastlio_trial import add_imu


def test_imu_is_colocated_and_has_specific_force_noise():
    root=ET.fromstring('<sdf><world><model name="icart_mini"><link name="base_link">'
        '<sensor name="mid360"><pose>0.1 0 0.39 0 0 0</pose><lidar><scan><horizontal><samples>360</samples></horizontal><vertical><samples>16</samples></vertical></scan><range><max>30</max></range></lidar></sensor>'
        '</link></model></world></sdf>')
    add_imu(root)
    sensor=root.find('.//sensor[@name="lio_imu"]')
    assert sensor.findtext('pose')=='0.1 0 0.39 0 0 0'
    assert sensor.findtext('update_rate')=='200'
    assert len(sensor.findall('.//noise'))==6
    with pytest.raises(ValueError):add_imu(root)


def test_nonfinite_and_range_returns_are_rejected():
    pytest.importorskip('rclpy')
    from lio_sensor_adapter_node import finite_xyzi
    data=np.array([(1,2,3,0),(np.inf,0,1,0),(np.nan,1,1,0),(.1,0,0,0),(31,0,0,0)],
                  dtype=[(k,'f4') for k in ['x','y','z','intensity']])
    result=finite_xyzi(data,.6,30.)
    assert result.shape==(1,4)
    assert np.isfinite(result).all()


def test_settling_uses_sensor_time_and_resets():
    pytest.importorskip('rclpy')
    from lio_sensor_adapter_node import LioSensorAdapter
    from types import SimpleNamespace
    node=SimpleNamespace(first=None,settle=3.)
    stamp=lambda t: SimpleNamespace(sec=t,nanosec=0)
    assert not LioSensorAdapter.ready(node,stamp(10))
    assert not LioSensorAdapter.ready(node,stamp(12))
    assert LioSensorAdapter.ready(node,stamp(13))
    assert not LioSensorAdapter.ready(node,stamp(1))


def test_initial_alignment_recovers_rotated_frames_without_fitting_path():
    from lio_evaluation_core import evaluate_lio
    times=np.linspace(0,20,201)
    truth=np.column_stack([times,100+times,50+times*0,np.full(201,np.pi/2)])
    lio=np.column_stack([times,times*0,-times,times*0,times*0,times*0,times*0,np.ones(201)])
    result,_=evaluate_lio(truth,lio)
    assert result['pass_lio']
    assert result['max_xy_error_m']<1e-8
    lio[:,2]*=.5
    result,_=evaluate_lio(truth,lio)
    assert not result['pass_lio']
    assert result['final_xy_error_m']==pytest.approx(10.)


def test_missing_lio_fails_instead_of_counting_navigation_success():
    from lio_evaluation_core import evaluate_lio
    result,_=evaluate_lio(np.zeros((2,4)),np.empty((0,8)))
    assert not result['pass_lio']


def test_lio_preparation_preserves_independent_planar_urg():
    """LIO 点群の高密度化で URG の走査面・取付・topic を変えない."""
    from build_terrain_trial import make_robot
    root = ET.Element('sdf')
    world = ET.SubElement(root, 'world')
    make_robot(world)
    urg = root.find('.//sensor[@name="top_urg"]')
    before = ET.tostring(urg)
    add_imu(root)
    assert ET.tostring(urg) == before
    assert urg.findtext('topic') == '/scan'
    assert urg.find('lidar/scan/vertical') is None
    assert urg.findtext('lidar/scan/horizontal/samples') == '1080'
    assert urg.findtext('update_rate') == '20'
