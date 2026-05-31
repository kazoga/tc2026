#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gazebo Sim 用の道路モデル SDF を生成する補助スクリプト."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import List

from road_geometry import Segment, build_segments, get_polyline_points, road_model_name


def _model_config(model_name: str) -> str:
    """model.config の XML を生成する."""

    return f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Auto-generated road model: {model_name}</description>
</model>
"""


def _subdivide(segment: Segment, max_panel_length: float) -> List[Segment]:
    """セグメントを指定長以下のパネルへ分割する."""

    count = max(1, int(math.ceil(segment.length / max_panel_length)))
    return [
        Segment(
            (
                segment.p0[0] + segment.dx * i / count,
                segment.p0[1] + segment.dy * i / count,
            ),
            (
                segment.p0[0] + segment.dx * (i + 1) / count,
                segment.p0[1] + segment.dy * (i + 1) / count,
            ),
        )
        for i in range(count)
    ]


def build_road_model_sdf(
    model_name: str,
    segments: List[Segment],
    width: float,
    panel_max_length: float = 5.0,
    height: float = 0.1,
    overlap_length: float = 0.0,
) -> str:
    """道路モデル SDF を生成する."""

    panels: List[Segment] = []
    for segment in segments:
        panels.extend(_subdivide(segment, panel_max_length))

    links = []
    for index, panel in enumerate(panels):
        center_x, center_y = panel.center
        size = f"{panel.length + overlap_length * 2.0:.3f} {width:.3f} {height:.3f}"
        pose = f"{center_x:.3f} {center_y:.3f} {height / 2.0:.3f} 0 0 {panel.yaw:.6f}"
        links.append(
            f"""
    <link name="panel_{index:03d}">
      <pose>{pose}</pose>
      <collision name="collision">
        <geometry><box><size>{size}</size></box></geometry>
        <surface>
          <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <ambient>0.22 0.22 0.22 1</ambient>
          <diffuse>0.32 0.32 0.32 1</diffuse>
        </material>
      </visual>
    </link>"""
        )

    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>true</static>
{''.join(links)}
  </model>
</sdf>
"""


def write_road_model(road_type: str, width: float, output_dir: Path) -> Path:
    """指定道路モデルを書き出す."""

    model_name = road_model_name(road_type, width)
    segments = build_segments(get_polyline_points(road_type))
    overlap = width * 0.5 if road_type in {"crank", "scurve"} else 0.0
    model_dir = output_dir / model_name
    model_dir.mkdir(parents=True, exist_ok=True)
    (model_dir / "model.config").write_text(_model_config(model_name), encoding="utf-8")
    (model_dir / "model.sdf").write_text(
        build_road_model_sdf(model_name, segments, width, overlap_length=overlap),
        encoding="utf-8",
    )
    return model_dir


def main() -> None:
    """コマンドラインエントリーポイント."""

    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="models")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    for width in (2.0, 3.0, 5.0):
        print(f"Generated {write_road_model('straight', width, output_dir)}")
    for road_type in ("crank", "scurve"):
        for width in (3.0, 5.0):
            print(f"Generated {write_road_model(road_type, width, output_dir)}")


if __name__ == "__main__":
    main()
