import math
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).parents[1]))
from route_survey.survey_core import Pose, Survey, traversable_width
from route_survey.storage_core import save
from geo_pose_converter.geo_core import ProjectionConfig
from route_planner.route_builder import parse_waypoint_csv


def cloud(kind='flat'):
    x,y=np.meshgrid(np.arange(-.595,.6,.025),np.arange(-2.99,3,.025))
    z=np.zeros_like(x)
    if kind=='curb':z[y>=1.]=.12
    if kind=='drop':z[y>=1.]=-.12
    if kind=='slope':z=np.maximum(0,y)*.5
    p=np.column_stack([x.ravel(),y.ravel(),z.ravel()])
    if kind=='missing':p=p[p[:,1]<1.]
    return p


@pytest.mark.parametrize('kind',['curb','drop','missing','slope'])
def test_no_crossing_edge_or_unknown(kind):
    r=traversable_width(cloud(kind))
    assert r['left']<=.725+1e-6
    assert r['left_reason']!='range_limit'
    assert r['right']>2.5


def test_flat_and_empty():
    assert traversable_width(cloud())['left']>2.5
    assert traversable_width(np.empty((0,3)))['left']==0


def test_sampling_buttons_and_corner(tmp_path):
    s=Survey();w=traversable_width(cloud())
    p=Pose(0,0,0,0);s.update(p,w);s.joy({0},p,w);s.joy(set(),p,w)
    for i in range(1,121):
        p=Pose(i*.1,i*.1,0,0);s.update(p,w)
    assert len(s.rows)==3
    s.joy({1},p,w);s.joy({1},p,w)
    assert sum(r['line_is_stop'] for r in s.rows)==1
    s.joy(set(),p,w)
    for i in range(1,31):
        p=Pose(12+i*.1,12,i*.1,math.pi/2);s.update(p,w)
    assert any(r['reason']=='turn' for r in s.rows)
    s.joy({2},p,w);s.joy(set(),p,w);s.joy({3},p,w)
    assert not s.active and sum(r['signal_is_stop'] for r in s.rows)==1
    save(tmp_path,s.rows,ProjectionConfig(36,140,67),np.empty((0,3)),False)
    restored=parse_waypoint_csv(str(tmp_path/'fixed/waypoints.csv'),ProjectionConfig(36,140,67))
    assert len(restored)==len(s.rows)
    assert restored[-1].signal_stop
    assert all(r.altitude is None for r in restored)


def test_stationary_noise_and_jump():
    s=Survey();w=traversable_width(np.empty((0,3)));p=Pose(0,0,0,0)
    s.update(p,w);s.joy({0},p,w)
    for i in range(1,1000):s.update(Pose(i*.1,.02*math.sin(i),.02*math.cos(i),0),w)
    assert len(s.rows)==1
    assert not s.update(Pose(100,100,0,0),w)
    assert len(s.rows)==1


def test_input_gap_requires_review():
    s=Survey();w=traversable_width(cloud());p=Pose(0,0,0,0)
    s.update(p,w);s.joy({0},p,w)
    assert not s.update(Pose(2,1,0,0),w)
    assert s.update(Pose(2.1,1.1,0,0),w)
    assert s.rows[-1]['reason']=='input_gap'
    assert s.rows[-1]['left_is_open']==0


@pytest.mark.parametrize('kind', ['flat','curb','drop'])
def test_noisy_ground_confidence(kind):
    p=cloud(kind);p[:,2]+=np.random.default_rng(7).normal(0,.02,len(p))
    r=traversable_width(p)
    if kind=='flat':assert r['left']>1. and r['right']>1.
    else:assert r['left']<=.725+1e-6
    assert r['ground_noise_sigma_m']>0
