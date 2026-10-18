"""つくば2026 本走行コース（選択課題A非対応）の登録内容を検査する.

登録した経路が route_survey の成果物と同一であること、route_config.yaml から
全域を固定ブロックとして組み立てられること、起動カードが実在するファイルを
指していることを確認する。経路の値そのもの（回避幅・停止位置）の妥当性は
実機走行で確認する範囲であり、ここでは扱わない。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest
import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]
sys.path.insert(0, str(PACKAGE_DIR))
sys.path.insert(0, str(WORKSPACE_DIR / "src" / "geo_pose_converter"))

from geo_pose_converter.geo_core import load_projection_config_from_yaml  # noqa: E402
from route_planner.route_builder import RouteBuilder  # noqa: E402

ROUTE_DIR = PACKAGE_DIR / "routes" / "tsukuba2026"
ROUTE_CSV = ROUTE_DIR / "fixed" / "waypoints.csv"
SURVEY_CSV = (
    WORKSPACE_DIR / "src" / "route_survey" / "results" / "tsukuba2026_waypoints.csv"
)
PROJECTION_YAML = (
    WORKSPACE_DIR / "src" / "geo_pose_converter" / "params" / "default.yaml"
)

# 最大waypoint間隔。build_route.py が0.75 m以下へ等分補間する。
MAX_SPACING_M = 0.75


@pytest.fixture(scope="module")
def route_waypoints() -> list:
    """route_config.yaml から組み立てたwaypoint列を返す."""

    projection = load_projection_config_from_yaml(str(PROJECTION_YAML))
    builder = RouteBuilder(
        str(ROUTE_DIR / "route_config.yaml"),
        csv_base_dir=str(ROUTE_DIR),
        projection=projection,
    )
    builder.load()
    return builder.build_route("", "", []).waypoints


def test_registered_csv_matches_route_survey_result() -> None:
    """登録した経路CSVが route_survey の成果物と一致する."""

    assert ROUTE_CSV.read_bytes() == SURVEY_CSV.read_bytes()


def test_route_config_covers_whole_course_as_fixed(route_waypoints: list) -> None:
    """全域が1つの固定ブロックとして読み込まれる."""

    assert len(route_waypoints) == 3704
    assert route_waypoints[0].label == "WP0000"
    assert route_waypoints[-1].label == "E17"
    assert all(wp.segment_is_fixed for wp in route_waypoints)


def test_route_keeps_stop_attributes_and_spacing(route_waypoints: list) -> None:
    """停止属性と点間隔が生成時のまま保たれている."""

    assert sum(1 for wp in route_waypoints if wp.line_stop) == 16
    assert not any(wp.signal_stop for wp in route_waypoints)

    spacing = [
        math.dist(
            (previous.pose.position.x, previous.pose.position.y),
            (current.pose.position.x, current.pose.position.y),
        )
        for previous, current in zip(route_waypoints, route_waypoints[1:])
    ]
    assert max(spacing) <= MAX_SPACING_M + 1e-6


def test_launch_cards_point_to_existing_files() -> None:
    """route_planner と route_manager の起動カードが揃っている."""

    planner_card = yaml.safe_load(
        (PACKAGE_DIR / "params" / "tsukuba2026.yaml").read_text(encoding="utf-8")
    )["route_planner"]["ros__parameters"]
    assert (PACKAGE_DIR / planner_card["config_yaml_path"]).is_file()
    assert (PACKAGE_DIR / planner_card["csv_base_dir"]).is_dir()

    manager_card_path = (
        WORKSPACE_DIR / "src" / "route_manager" / "params" / "tsukuba2026.yaml"
    )
    manager_card = yaml.safe_load(manager_card_path.read_text(encoding="utf-8"))
    assert "route_manager" in manager_card
