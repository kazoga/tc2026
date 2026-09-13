"""route_plannerの保存仕様に従い、共有経路をLLH正本へ変換する."""
import csv
import math
from pathlib import Path

from geo_pose_converter.geo_core import (
    ProjectionConfig, enu_to_llh_on_ground, yaw_enu_rad_to_heading_deg,
)
from route_planner.route_builder import parse_waypoint_csv


FIELDS = ['label', 'latitude', 'longitude', 'altitude', 'heading_deg',
          'right_is_open', 'left_is_open', 'line_is_stop', 'signal_is_stop', 'isnot_skipnum']


def canonical_rows(source: Path, projection: ProjectionConfig) -> list[dict]:
    """既存readerと同じ正本選択を行い、属性と順序を保つ."""
    records = parse_waypoint_csv(str(source), projection=projection)
    if not records:
        raise ValueError('waypointが空です')
    rows = []
    for wp in records:
        if wp.latitude is not None and wp.longitude is not None and wp.heading_deg is not None:
            lat, lon, heading = wp.latitude, wp.longitude, wp.heading_deg
        else:
            llh = enu_to_llh_on_ground(wp.pose.position.x, wp.pose.position.y, projection,
                                       ground_altitude=projection.origin_altitude)
            q = wp.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            lat, lon = llh.latitude, llh.longitude
            heading = yaw_enu_rad_to_heading_deg(yaw+projection.map_yaw_offset_rad)
        if not all(math.isfinite(v) for v in [lat, lon, heading]):
            raise ValueError('waypoint座標・方位が有限値ではありません')
        rows.append(dict(zip(FIELDS, [wp.label, lat, lon, '', heading % 360,
                                    wp.right_open, wp.left_open, int(wp.line_stop),
                                    int(wp.signal_stop), int(wp.not_skip)])))
    return rows


def write_route(path: Path, rows: list[dict]) -> None:
    """高度空欄のLLH CSVを保存し、ENUを重複保存しない."""
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
