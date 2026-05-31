#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pylon と robot include を追加した Gazebo world を生成する."""

from __future__ import annotations

import argparse
import math
import random
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence

from road_geometry import get_polyline_points, interpolate_pose_on_polyline, total_length


@dataclass(frozen=True)
class PylonPose:
    """生成 world に挿入する pylon pose."""

    name: str
    x: float
    y: float
    z: float
    yaw: float


def _indent_xml(element: ET.Element) -> None:
    """ElementTree の出力を読みやすく整形する."""

    try:
        ET.indent(element, space="  ")
    except AttributeError:
        pass


def _lateral_offsets(
    rng: random.Random,
    num_pylons: int,
    road_width: float,
    min_lateral_gap: float,
    pylon_block_half: float,
) -> List[float]:
    """通過可能隙間を残す pylon の横方向オフセットを返す."""

    arrangements = ["center"] if num_pylons == 1 else ["spread", "cluster"]
    rng.shuffle(arrangements)
    for arrangement in arrangements:
        margin = road_width * 0.1
        usable_width = road_width - 2.0 * margin
        if num_pylons == 1 or arrangement == "center":
            offsets = [0.0]
        elif arrangement == "cluster":
            start = -0.3 * (num_pylons - 1) * 0.5
            offsets = [start + 0.3 * index for index in range(num_pylons)]
        elif num_pylons == 2:
            delta = usable_width * 0.25
            offsets = [-delta, delta]
        else:
            delta = usable_width / 6.0
            offsets = [-delta, 0.0, delta]

        half_width = road_width * 0.5 - margin
        clipped = [max(min(offset, half_width), -half_width) for offset in offsets]
        intervals = sorted(
            (
                max(-road_width * 0.5, offset - pylon_block_half),
                min(road_width * 0.5, offset + pylon_block_half),
            )
            for offset in clipped
        )
        covered = sum(end - start for start, end in intervals)
        if road_width - covered >= min_lateral_gap:
            return clipped
    return [0.0]


def generate_pylon_poses(
    road_type: str,
    road_width: float,
    seed: int,
    min_longitudinal_spacing: float = 5.0,
    longitudinal_margin: float = 1.0,
    min_lateral_gap: float = 1.0,
    origin_safety_radius: float = 5.0,
) -> List[PylonPose]:
    """道路中心線に沿った pylon 配置を生成する."""

    rng = random.Random(seed)
    points = get_polyline_points(road_type)
    length = total_length(points)
    s_min = max(longitudinal_margin, origin_safety_radius)
    s_max = max(length - longitudinal_margin, s_min)
    count = max(2, int(math.ceil(length / 50.0)) * 3)

    axis_positions: List[float] = []
    attempts = 0
    while len(axis_positions) < count and attempts < 500:
        candidate = rng.uniform(s_min, s_max)
        if all(abs(candidate - existing) >= min_longitudinal_spacing for existing in axis_positions):
            axis_positions.append(candidate)
        attempts += 1
    axis_positions.sort()

    poses: List[PylonPose] = []
    pylon_index = 0
    for center_distance in axis_positions:
        center_pose = interpolate_pose_on_polyline(points, center_distance)
        num_pylons = rng.randint(1, 3)
        offsets = _lateral_offsets(
            rng,
            num_pylons,
            road_width,
            min_lateral_gap=min_lateral_gap,
            pylon_block_half=0.25,
        )
        for offset in offsets:
            nx = -math.sin(center_pose.yaw)
            ny = math.cos(center_pose.yaw)
            x = center_pose.x + offset * nx
            y = center_pose.y + offset * ny
            if math.hypot(x, y) < origin_safety_radius:
                continue
            poses.append(
                PylonPose(
                    name=f"{road_type}_pylon_{pylon_index:03d}",
                    x=x,
                    y=y,
                    z=0.35,
                    yaw=0.0,
                )
            )
            pylon_index += 1
    return poses


def _add_include(world: ET.Element, uri: str, name: str, pose: Optional[str] = None) -> None:
    """world 要素へ include を追加する."""

    include = ET.SubElement(world, "include")
    ET.SubElement(include, "uri").text = uri
    ET.SubElement(include, "name").text = name
    if pose is not None:
        ET.SubElement(include, "pose").text = pose


def generate_world(
    base_world: Path,
    output: Path,
    road_type: str,
    road_width: float,
    enable_pylons: bool,
    seed: int,
    spawn_robot: bool,
    robot_x: float,
    robot_y: float,
    robot_z: float,
    enable_route_blocker: bool = False,
    route_blocker_distance: float = 8.0,
) -> Sequence[PylonPose]:
    """base world から実行用 world を生成する."""

    tree = ET.parse(base_world)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"world element not found: {base_world}")

    if spawn_robot:
        _add_include(
            world,
            "model://robot",
            "obstacle_route_robot",
            f"{robot_x:.6f} {robot_y:.6f} {robot_z:.6f} 0 0 0",
        )

    poses: List[PylonPose] = []
    if enable_pylons:
        poses.extend(generate_pylon_poses(road_type, road_width, seed))

    if enable_route_blocker:
        points = get_polyline_points(road_type)
        center_pose = interpolate_pose_on_polyline(points, route_blocker_distance)
        poses.append(
            PylonPose(
                name=f"{road_type}_route_blocker",
                x=center_pose.x,
                y=center_pose.y,
                z=0.35,
                yaw=0.0,
            )
        )

    for pose in poses:
        _add_include(
            world,
            "model://pylon",
            pose.name,
            f"{pose.x:.6f} {pose.y:.6f} {pose.z:.6f} 0 0 {pose.yaw:.6f}",
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    _indent_xml(root)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return poses


def main() -> None:
    """コマンドラインエントリーポイント."""

    parser = argparse.ArgumentParser()
    parser.add_argument("--road", choices=["straight", "crank", "scurve"], required=True)
    parser.add_argument("--width", type=float, required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--base-world", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--enable-pylons", action="store_true")
    parser.add_argument("--enable-route-blocker", action="store_true")
    parser.add_argument("--route-blocker-distance", type=float, default=8.0)
    parser.add_argument("--spawn-robot", action="store_true")
    parser.add_argument("--robot-x", type=float, default=1.0)
    parser.add_argument("--robot-y", type=float, default=0.0)
    parser.add_argument("--robot-z", type=float, default=0.16)
    args = parser.parse_args()

    poses = generate_world(
        base_world=Path(args.base_world),
        output=Path(args.output),
        road_type=args.road,
        road_width=args.width,
        enable_pylons=args.enable_pylons,
        seed=args.seed,
        spawn_robot=args.spawn_robot,
        robot_x=args.robot_x,
        robot_y=args.robot_y,
        robot_z=args.robot_z,
        enable_route_blocker=args.enable_route_blocker,
        route_blocker_distance=args.route_blocker_distance,
    )
    print(f"Generated {args.output} with {len(poses)} pylons.")


if __name__ == "__main__":
    main()
