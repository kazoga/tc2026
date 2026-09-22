"""全線分離隔、移動上限、削除判定を確認する."""
import importlib.util
from pathlib import Path

import pytest
pytest.importorskip('shapely')
from shapely import affinity
from shapely.geometry import LineString, Point, box

spec = importlib.util.spec_from_file_location(
    'corridor', Path(__file__).parents[1]/'tools/clear_route_corridor.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_between_waypoints_requires_shift():
    route = LineString([(0, 0), (100, 0)])
    shape = Point(50, .8).buffer(.3)
    action, dx, dy, after = module.clearance_action(shape, route)
    assert action == 'move'
    assert affinity.translate(shape, dx, dy).distance(route) >= 1.01
    assert dx*dx+dy*dy <= 4.000001


def test_large_building_requires_removal():
    assert module.clearance_action(box(-5,-5,5,5), LineString([(-10,0),(10,0)]))[0] == 'remove'


def test_clear_model_keeps_position():
    assert module.clearance_action(Point(4,2), LineString([(0,0),(10,0)]))[:3] == ('keep',0,0)
