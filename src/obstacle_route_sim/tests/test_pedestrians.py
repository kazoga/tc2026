"""Birth/death visibility, continuous movement and geometry regression tests."""
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import pytest
sys.path.insert(0, str(Path(__file__).parents[1]/'tools'))
from pedestrian_core import Crowd, Ground, Walker, model_sdf, sensor_reach


def ground():
    return Ground(dict(grid=dict(w=101, h=101, ox=-100, oy=-100, res=2,
                                 height=[0.]*(101*101))))


def test_lifecycle_stays_outside_sensor_and_motion_is_bounded():
    crowd = Crowd(ground(), 10, density=2)
    seen_inside = False
    for tick in range(1200):
        robot = (tick*.005, 0)
        old = {w.name: (w.x, w.y) for w in crowd.walkers.values()}
        births, exits = crowd.step(robot, .05)
        for name in exits:
            w = crowd.walkers.pop(name)
            assert math.dist((w.x, w.y), robot) > 10+.3
        for w in crowd.walkers.values():
            assert math.dist(old[w.name], (w.x, w.y)) <= 1.4*.05+1e-8
            seen_inside |= math.dist((w.x, w.y), robot) < 8
        for w in births:
            assert math.dist((w.x, w.y), robot) > 10+.3
            crowd.walkers[w.name] = w
    assert seen_inside


def test_density_zero_preserves_visible_people_and_seed_repeats():
    c = Crowd(ground(), 10)
    c.walkers['a'] = Walker('a', 2, 2, 0, 0, 1)
    c.set_density(0)
    assert c.step((0, 0), .05) == ([], [])
    for density in [-1, float('nan'), float('inf')]:
        with pytest.raises(ValueError):
            c.set_density(density)
    a, b = Crowd(ground(), 10, 5), Crowd(ground(), 10, 5)
    assert a.step((0, 0), .05) == b.step((0, 0), .05)
    assert a.step((0, 0), 0) == ([], [])
    assert a.step((0, 0), 10) == ([], [])
    assert Crowd(ground(), 70, 1000).target == 200
    assert Crowd(ground(), 10, 5).target > Crowd(ground(), 10, 1).target


def test_ground_blocks_swept_paths_and_matches_mesh_triangles():
    g = Ground(dict(grid=dict(w=2, h=2, res=10, ox=0, oy=0, height=[0, 1, 2, 4]),
                    obstacles=[dict(poly=[[4, 0], [6, 0], [6, 10], [4, 10]])]))
    assert g.height(8, 2) == pytest.approx(1.4)
    assert g.height(2, 8) == pytest.approx(2.)
    assert not g.valid(8, 5, (2, 5))
    assert not g.valid(-1, 0)


def test_model_has_visible_and_collision_geometry_and_range_uses_sensor_offset(tmp_path):
    w = Walker('pedestrian_1', 1, 2, 3, .4, 1)
    root = ET.fromstring(model_sdf(w))
    assert len(root.findall('.//visual')) == len(root.findall('.//collision')) == 4
    path = tmp_path/'world.sdf'
    path.write_text('''<sdf><world name="demo"><model name="icart_mini"><link name="base">
      <sensor name="mid"><pose>0 0 1 0 0 0</pose><lidar><range><max>70</max></range></lidar></sensor>
      <sensor name="urg"><lidar><range><max>30</max></range></lidar></sensor>
      </link></model></world></sdf>''')
    assert sensor_reach(path) == ('demo', 71)


def test_barriers_and_point_obstacles_are_not_walkable():
    g = Ground(dict(grid=dict(w=21, h=21, res=1, ox=-10, oy=-10, height=[0]*441),
                    obstacles=[dict(type='circle', x=2, y=2, r=.2),
                               dict(type='segment', pts=[[-2, -2], [-2, 2]], r=.1)]))
    assert not g.valid(2.4, 2)
    assert not g.valid(-1, 0, (-3, 0))
    assert g.valid(5, 5)
