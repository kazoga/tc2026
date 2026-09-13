"""LLH正本の保存で走行座標・方位・属性を変えないことを確認する."""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from geo_pose_converter.geo_core import ProjectionConfig
from route_planner.route_builder import parse_waypoint_csv
from icart_bringup.route_storage_core import FIELDS, canonical_rows, write_route


@pytest.mark.parametrize('rotation', [0., .7])
def test_enu_roundtrip_and_flags(tmp_path, rotation):
    source = tmp_path/'enu.csv'
    source.write_text('label,x,y,z,q1,q2,q3,q4,right_is_open,left_is_open,'
                      'line_is_stop,signal_is_stop,isnot_skipnum\n'
                      'start,2300,-900,0,0,0,0,1,1.2,2.3,1,0,1\n'
                      'end,2310,-890,0,0,0,1,0,0,1,0,1,0\n')
    projection = ProjectionConfig(36., 140., 67., map_yaw_offset_rad=rotation)
    rows = canonical_rows(source, projection)
    dest = tmp_path/'llh.csv'
    write_route(dest, rows)
    assert dest.read_text().splitlines()[0].split(',') == FIELDS
    assert all(row['altitude'] == '' for row in rows)
    before = parse_waypoint_csv(str(source), projection)
    after = parse_waypoint_csv(str(dest), projection)
    for a, b in zip(before, after):
        assert math.hypot(a.pose.position.x-b.pose.position.x,
                          a.pose.position.y-b.pose.position.y) < 1e-5
        assert abs(abs(a.pose.orientation.z*b.pose.orientation.z+
                       a.pose.orientation.w*b.pose.orientation.w)-1) < 1e-10
        assert (a.label, a.right_open, a.left_open, a.line_stop, a.signal_stop, a.not_skip) == (
            b.label, b.right_open, b.left_open, b.line_stop, b.signal_stop, b.not_skip)


def test_complete_llh_is_preserved(tmp_path):
    source = tmp_path/'llh.csv'
    source.write_text('label,latitude,longitude,altitude,heading_deg\nA,36,140,88,359\n')
    rows = canonical_rows(source, ProjectionConfig(36., 140.))
    assert (rows[0]['latitude'], rows[0]['longitude'], rows[0]['heading_deg']) == (36, 140, 359)
    assert rows[0]['altitude'] == ''
