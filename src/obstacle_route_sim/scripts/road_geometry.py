#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""道路中心線と幅キーを共有するユーティリティ."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


Point2D = Tuple[float, float]

SUPPORTED_ROADS = ("straight", "crank", "scurve")
SUPPORTED_WIDTHS = (2.0, 3.0, 5.0)


@dataclass(frozen=True)
class Pose2D:
    """2D pose を表す軽量データ."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Segment:
    """道路中心線の 1 セグメント."""

    p0: Point2D
    p1: Point2D

    @property
    def dx(self) -> float:
        """X 方向差分."""

        return self.p1[0] - self.p0[0]

    @property
    def dy(self) -> float:
        """Y 方向差分."""

        return self.p1[1] - self.p0[1]

    @property
    def length(self) -> float:
        """セグメント長 [m]."""

        return math.hypot(self.dx, self.dy)

    @property
    def yaw(self) -> float:
        """セグメント yaw [rad]."""

        return math.atan2(self.dy, self.dx)

    @property
    def center(self) -> Point2D:
        """セグメント中心座標."""

        return (0.5 * (self.p0[0] + self.p1[0]), 0.5 * (self.p0[1] + self.p1[1]))


def width_key(width: float) -> str:
    """道幅からファイル名用キーを返す.

    Args:
        width (float): 道幅 [m].

    Returns:
        str: `w2`, `w3`, `w5` のいずれか.

    Raises:
        ValueError: 未対応の道幅が指定された場合.
    """

    for supported in SUPPORTED_WIDTHS:
        if abs(float(width) - supported) < 1.0e-6:
            return f"w{int(supported)}"
    raise ValueError(f"Unsupported road_width: {width}")


def road_model_name(road_type: str, width: float) -> str:
    """道路モデル名を返す."""

    key = width_key(width)
    if road_type == "straight":
        return f"road_straight_100m_{key}"
    if road_type == "crank":
        if key == "w2":
            raise ValueError("crank road does not support width 2.0")
        return f"road_crank_50m_{key}"
    if road_type == "scurve":
        if key == "w2":
            raise ValueError("scurve road does not support width 2.0")
        return f"road_scurve_100m_{key}"
    raise ValueError(f"Unsupported road_type: {road_type}")


def world_template_name(road_type: str, width: float) -> str:
    """world template ファイル名を返す."""

    key = width_key(width)
    if road_type == "straight":
        return f"road_straight_{key}.world"
    if road_type == "crank":
        if key == "w2":
            raise ValueError("crank road does not support width 2.0")
        return f"road_crank_{key}.world"
    if road_type == "scurve":
        if key == "w2":
            raise ValueError("scurve road does not support width 2.0")
        return f"road_scurve_{key}.world"
    raise ValueError(f"Unsupported road_type: {road_type}")


def get_polyline_points(road_type: str) -> List[Point2D]:
    """道路タイプに対応する中心線頂点列を返す."""

    if road_type == "straight":
        return [(0.0, 0.0), (100.0, 0.0)]

    if road_type == "crank":
        return [(0.0, 0.0), (50.0, 0.0), (50.0, 50.0), (100.0, 50.0)]

    if road_type == "scurve":
        length_x = 100.0
        amplitude = 20.0
        num_points = 41
        return [
            (
                length_x * i / (num_points - 1),
                amplitude * math.sin(2.0 * math.pi * i / (num_points - 1)),
            )
            for i in range(num_points)
        ]

    raise ValueError(f"Unsupported road_type: {road_type}")


def build_segments(points: Sequence[Point2D]) -> List[Segment]:
    """頂点列から中心線セグメントを生成する."""

    if len(points) < 2:
        raise ValueError("Polyline must contain at least two points.")
    return [Segment(points[i], points[i + 1]) for i in range(len(points) - 1)]


def cumulative_distances(points: Sequence[Point2D]) -> List[float]:
    """頂点列の累積距離を返す."""

    distances = [0.0]
    for segment in build_segments(points):
        distances.append(distances[-1] + segment.length)
    return distances


def total_length(points: Sequence[Point2D]) -> float:
    """中心線の総延長 [m] を返す."""

    return cumulative_distances(points)[-1]


def interpolate_pose_on_polyline(points: Sequence[Point2D], distance: float) -> Pose2D:
    """中心線上の距離に対応する pose を補間する."""

    segments = build_segments(points)
    remaining = max(float(distance), 0.0)
    for segment in segments:
        if remaining <= segment.length + 1.0e-9:
            ratio = 0.0 if segment.length <= 1.0e-9 else remaining / segment.length
            return Pose2D(
                x=segment.p0[0] + segment.dx * ratio,
                y=segment.p0[1] + segment.dy * ratio,
                yaw=segment.yaw,
            )
        remaining -= segment.length
    last = segments[-1]
    return Pose2D(x=last.p1[0], y=last.p1[1], yaw=last.yaw)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """yaw[rad] を roll/pitch なしの quaternion へ変換する."""

    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def unique_sorted_distances(distances: Iterable[float], eps: float = 1.0e-6) -> List[float]:
    """近接重複を除去した昇順距離配列を返す."""

    merged: List[float] = []
    for distance in sorted(float(value) for value in distances):
        if not merged or abs(distance - merged[-1]) > eps:
            merged.append(distance)
    return merged
