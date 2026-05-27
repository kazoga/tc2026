#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""road_geometry の単体テスト."""

from __future__ import annotations

import math
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from road_geometry import (  # noqa: E402
    get_polyline_points,
    interpolate_pose_on_polyline,
    road_model_name,
    total_length,
    width_key,
    world_template_name,
)


def test_width_key_accepts_supported_widths() -> None:
    """対応道幅をファイル名キーへ変換できる."""

    assert width_key(2.0) == "w2"
    assert width_key(3.0) == "w3"
    assert width_key(5.0) == "w5"


def test_straight_geometry() -> None:
    """直線道路の距離と補間 pose が正しい."""

    points = get_polyline_points("straight")
    assert total_length(points) == 100.0
    pose = interpolate_pose_on_polyline(points, 25.0)
    assert pose.x == 25.0
    assert pose.y == 0.0
    assert pose.yaw == 0.0


def test_crank_geometry_corner_yaw() -> None:
    """クランク道路の曲がり角後は y 軸方向 yaw になる."""

    points = get_polyline_points("crank")
    assert total_length(points) == 150.0
    pose = interpolate_pose_on_polyline(points, 60.0)
    assert pose.x == 50.0
    assert pose.y == 10.0
    assert math.isclose(pose.yaw, math.pi / 2.0)


def test_model_and_world_name_validation() -> None:
    """road_type と width からモデル名・world 名を生成できる."""

    assert road_model_name("straight", 2.0) == "road_straight_100m_w2"
    assert road_model_name("crank", 5.0) == "road_crank_50m_w5"
    assert world_template_name("scurve", 3.0) == "road_scurve_w3.world"
