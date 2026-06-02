"""geo_core の座標変換に関する単体テスト."""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geo_pose_converter.geo_core import (  # noqa: E402
    EnuPoint,
    LlhPoint,
    ProjectionConfig,
    bearing_from_map_delta,
    enu_to_llh,
    heading_deg_to_yaw_enu_rad,
    llh_to_enu,
    load_projection_config_from_yaml,
    yaw_enu_rad_to_heading_deg,
)


def test_llh_enu_round_trip_preserves_point() -> None:
    """LLH -> ENU -> LLH の往復で元の座標へ戻ることを確認する."""

    projection = ProjectionConfig(
        origin_latitude=36.082331,
        origin_longitude=140.111681,
        origin_altitude=25.0,
        map_yaw_offset_rad=0.0,
        projection_id="test",
    )
    original = LlhPoint(36.082421, 140.111792, 27.5)

    enu = llh_to_enu(original, projection)
    restored = enu_to_llh(enu, projection)

    assert math.isclose(restored.latitude, original.latitude, abs_tol=1e-8)
    assert math.isclose(restored.longitude, original.longitude, abs_tol=1e-8)
    assert math.isclose(restored.altitude, original.altitude, abs_tol=1e-4)


def test_projection_yaw_offset_rotates_map_axes() -> None:
    """map_yaw_offset_rad により map 軸と ENU 軸が回転して対応することを確認する."""

    projection = ProjectionConfig(
        origin_latitude=36.0,
        origin_longitude=140.0,
        origin_altitude=0.0,
        map_yaw_offset_rad=math.pi / 2.0,
    )
    point = enu_to_llh(EnuPoint(10.0, 0.0, 0.0), projection)
    mapped = llh_to_enu(point, projection)

    assert math.isclose(mapped.x, 10.0, abs_tol=1e-6)
    assert math.isclose(mapped.y, 0.0, abs_tol=1e-6)


def test_heading_yaw_conversions_use_north_clockwise_heading() -> None:
    """heading は真北0度・時計回り、ENU yaw は東0rad・反時計回りとして相互変換する."""

    assert math.isclose(heading_deg_to_yaw_enu_rad(0.0), math.pi / 2.0)
    assert math.isclose(heading_deg_to_yaw_enu_rad(90.0), 0.0)
    assert math.isclose(yaw_enu_rad_to_heading_deg(math.pi / 2.0), 0.0)
    assert math.isclose(yaw_enu_rad_to_heading_deg(0.0), 90.0)


def test_bearing_from_map_delta_respects_projection_offset() -> None:
    """map 座標上の差分から現在の投影設定に基づく方位を算出する."""

    projection = ProjectionConfig(origin_latitude=36.0, origin_longitude=140.0, map_yaw_offset_rad=0.0)
    assert math.isclose(bearing_from_map_delta(0.0, 1.0, projection), 0.0)
    assert math.isclose(bearing_from_map_delta(1.0, 0.0, projection), 90.0)



def test_load_projection_config_from_yaml_reads_common_params(tmp_path: Path) -> None:
    """ROS 2 wildcard parameterからProjectionConfigを読み込む."""

    yaml_path = tmp_path / "default.yaml"
    yaml_path.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    projection_id: tokyo_station\n"
        "    datum: WGS84\n"
        "    map_frame_id: map\n"
        "    earth_frame_id: earth\n"
        "    origin_latitude: 35.681382\n"
        "    origin_longitude: 139.766084\n"
        "    origin_altitude: 3.86\n"
        "    map_yaw_offset_rad: 0.1\n",
        encoding="utf-8",
    )

    projection = load_projection_config_from_yaml(str(yaml_path))

    assert projection.projection_id == "tokyo_station"
    assert projection.origin_latitude == 35.681382
    assert projection.origin_longitude == 139.766084
    assert projection.origin_altitude == 3.86
    assert projection.map_yaw_offset_rad == 0.1
