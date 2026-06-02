"""message_utils の高度有効フラグに関する単体テスト."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_msgs.msg import Pose
from std_msgs.msg import Header

from geo_pose_converter.geo_core import LlhPoint, ProjectionConfig
from geo_pose_converter.message_utils import (
    llh_to_pose_with_covariance,
    make_active_target_llh,
    make_geo_pose,
    pose_to_llh_pose,
)


def _projection() -> ProjectionConfig:
    return ProjectionConfig(
        origin_latitude=36.0,
        origin_longitude=140.0,
        origin_altitude=25.0,
        map_yaw_offset_rad=0.0,
    )


def test_llh_to_pose_with_covariance_keeps_enu_z_zero() -> None:
    """GNSS高度があっても走行用ENU zは0.0へ正規化する."""

    pose = llh_to_pose_with_covariance(
        Header(),
        LlhPoint(36.000001, 140.000002, 30.0),
        90.0,
        True,
        _projection(),
    )

    assert pose.pose.pose.position.z == 0.0


def test_pose_to_llh_pose_marks_projected_altitude_invalid_by_default() -> None:
    """ENU逆投影で作ったLLH poseは既定で高度を有効扱いしない."""

    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 0.0
    pose.orientation.w = 1.0

    geo_pose = pose_to_llh_pose(Header(), pose, _projection(), "active_target")

    assert not geo_pose.point.has_altitude


def test_make_active_target_llh_ignores_invalid_altitude_for_distance() -> None:
    """高度無効のtarget/currentでも2D距離を算出できる."""

    projection = _projection()
    current = make_geo_pose(
        Header(),
        LlhPoint(36.0, 140.0, 0.0),
        0.0,
        True,
        "current",
        has_altitude=False,
    )
    target = make_geo_pose(
        Header(),
        LlhPoint(36.000001, 140.0, 0.0),
        0.0,
        True,
        "target",
        has_altitude=False,
    )

    msg = make_active_target_llh(Header(), 1, -1, "", target, current, projection, True)

    assert msg.is_avoidance_subgoal
    assert msg.distance_m > 0.0
    assert not msg.target_pose.point.has_altitude
