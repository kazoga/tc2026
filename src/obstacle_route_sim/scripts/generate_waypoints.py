#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""道路中心線から route_planner 互換 waypoint CSV を生成する."""

from __future__ import annotations

import argparse
import csv
import math
from typing import List

from road_geometry import (
    get_polyline_points,
    interpolate_pose_on_polyline,
    cumulative_distances,
    total_length,
    unique_sorted_distances,
    yaw_to_quaternion,
)


CSV_HEADER = [
    "label",
    "latitude",
    "longitude",
    "x",
    "y",
    "z",
    "q1",
    "q2",
    "q3",
    "q4",
    "right_is_open",
    "left_is_open",
    "line_is_stop",
    "signal_is_stop",
    "isnot_skipnum",
    "node",
]


def waypoint_distances(
    road_type: str,
    step_m: float = 5.0,
    angle_threshold_rad: float = math.radians(25.0),
    start_offset_m: float = 1.0,
    end_offset_m: float = 1.0,
) -> List[float]:
    """waypoint を配置する中心線上距離を生成する."""

    points = get_polyline_points(road_type)
    total = total_length(points)
    valid_start = min(max(start_offset_m, 0.0), total)
    valid_end = max(min(total - max(end_offset_m, 0.0), total), valid_start)

    distances = [valid_start, valid_end]
    current = valid_start + step_m
    while current <= valid_end + 1.0e-6:
        distances.append(current)
        current += step_m

    cumulative = cumulative_distances(points)
    for index in range(1, len(points) - 1):
        prev_pose = interpolate_pose_on_polyline(points, cumulative[index] - 1.0e-6)
        next_pose = interpolate_pose_on_polyline(points, cumulative[index] + 1.0e-6)
        dyaw = (next_pose.yaw - prev_pose.yaw + math.pi) % (2.0 * math.pi) - math.pi
        if abs(dyaw) >= angle_threshold_rad:
            if valid_start - 1.0e-6 <= cumulative[index] <= valid_end + 1.0e-6:
                distances.append(cumulative[index])

    return unique_sorted_distances(distances)


def write_waypoint_csv(road_type: str, width: float, output: str, step_m: float) -> int:
    """waypoint CSV を書き出す.

    Args:
        road_type (str): 道路種別.
        width (float): 道幅 [m].
        output (str): 出力 CSV パス.
        step_m (float): waypoint 間隔 [m].

    Returns:
        int: 出力 waypoint 数.
    """

    points = get_polyline_points(road_type)
    half_width = float(width) * 0.5
    distances = waypoint_distances(road_type, step_m=step_m)

    with open(output, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(CSV_HEADER)
        for index, distance in enumerate(distances):
            pose = interpolate_pose_on_polyline(points, distance)
            q1, q2, q3, q4 = yaw_to_quaternion(pose.yaw)
            writer.writerow(
                [
                    index,
                    "",
                    "",
                    f"{pose.x:.6f}",
                    f"{pose.y:.6f}",
                    "0.000000",
                    f"{q1:.10f}",
                    f"{q2:.10f}",
                    f"{q3:.10f}",
                    f"{q4:.10f}",
                    f"{half_width:.6f}",
                    f"{half_width:.6f}",
                    0,
                    0,
                    1,
                    -1,
                ]
            )
    return len(distances)


def main() -> None:
    """コマンドラインエントリーポイント."""

    parser = argparse.ArgumentParser()
    parser.add_argument("--road", choices=["straight", "crank", "scurve"], required=True)
    parser.add_argument("--width", type=float, required=True)
    parser.add_argument("--output", default="waypoints.csv")
    parser.add_argument("--step", type=float, default=5.0)
    args = parser.parse_args()

    count = write_waypoint_csv(args.road, args.width, args.output, args.step)
    print(f"Generated {args.output} with {count} waypoints.")


if __name__ == "__main__":
    main()
