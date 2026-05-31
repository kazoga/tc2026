#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成スクリプトの単体テスト."""

from __future__ import annotations

import csv
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from generate_pylon_world import generate_pylon_poses, generate_world  # noqa: E402
from generate_waypoints import CSV_HEADER, write_waypoint_csv  # noqa: E402


def test_generate_waypoint_csv_header_and_offsets(tmp_path: Path) -> None:
    """route_planner 互換 CSV header と端点 offset を確認する."""

    output = tmp_path / "waypoints.csv"
    count = write_waypoint_csv("straight", 5.0, str(output), step_m=5.0)
    assert count > 2

    with output.open(newline="", encoding="utf-8") as csv_file:
        rows = list(csv.DictReader(csv_file))

    assert rows
    assert list(rows[0].keys()) == CSV_HEADER
    assert float(rows[0]["x"]) == 1.0
    assert float(rows[-1]["x"]) == 99.0
    assert rows[0]["node"] == "-1"


def test_pylon_generation_is_seed_reproducible() -> None:
    """同一 seed で pylon 配置が再現する."""

    poses_a = generate_pylon_poses("crank", 5.0, seed=123)
    poses_b = generate_pylon_poses("crank", 5.0, seed=123)
    assert poses_a == poses_b
    assert poses_a
    assert all((pose.x**2 + pose.y**2) ** 0.5 >= 5.0 for pose in poses_a)


def test_generate_world_adds_robot_and_pylons(tmp_path: Path) -> None:
    """base world に robot と pylon include を追加できる."""

    base_world = tmp_path / "base.world"
    base_world.write_text(
        """<?xml version="1.0"?><sdf version="1.9"><world name="test"/></sdf>""",
        encoding="utf-8",
    )
    output = tmp_path / "generated.world"
    generate_world(
        base_world=base_world,
        output=output,
        road_type="straight",
        road_width=5.0,
        enable_pylons=True,
        seed=1,
        spawn_robot=True,
        robot_x=1.0,
        robot_y=0.0,
        robot_z=0.16,
    )

    tree = ET.parse(output)
    includes = tree.findall(".//include")
    uris = [include.findtext("uri") for include in includes]
    assert "model://robot" in uris
    assert "model://pylon" in uris


def test_generate_world_adds_route_blocker_without_random_pylons(tmp_path: Path) -> None:
    """経路上の決定的な blocker pylon を単独生成できる."""

    base_world = tmp_path / "base.world"
    base_world.write_text(
        """<?xml version="1.0"?><sdf version="1.9"><world name="test"/></sdf>""",
        encoding="utf-8",
    )
    output = tmp_path / "generated.world"
    poses = generate_world(
        base_world=base_world,
        output=output,
        road_type="straight",
        road_width=5.0,
        enable_pylons=False,
        seed=1,
        spawn_robot=False,
        robot_x=1.0,
        robot_y=0.0,
        robot_z=0.16,
        enable_route_blocker=True,
        route_blocker_distance=8.0,
    )

    assert len(poses) == 1
    assert poses[0].name == "straight_route_blocker"
    assert poses[0].x == 8.0
    assert poses[0].y == 0.0

    tree = ET.parse(output)
    assert tree.findtext(".//include/name") == "straight_route_blocker"
