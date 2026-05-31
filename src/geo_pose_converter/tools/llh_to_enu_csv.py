#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Waypoint CSVのLLH座標からENU座標を一括生成するツール。

src/geo_pose_converter/tools 配下に配置して使用する想定。

使用例:
  python3 src/geo_pose_converter/tools/llh_to_enu_csv.py \
    --input src/route_planner/routes \
    --config src/geo_pose_converter/params/defaults.yaml \
    --in-place

latitude/longitude列を持つCSVを再帰的に探索し、x/y/z列へENU座標を書き込む。
heading_deg列がある場合は、q1/q2/q3/q4へENU yaw quaternionも書き込む。
altitude列が無い場合は0.0[m]として扱う。
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Any

import yaml

# colcon install前でもリポジトリ直下から直接実行できるようにする。
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))

from geo_pose_converter.geo_core import (  # noqa: E402
    LlhPoint,
    ProjectionConfig,
    heading_deg_to_yaw_enu_rad,
    llh_to_enu,
    yaw_to_quaternion,
)


def _parse_optional_float(value: str | None) -> float | None:
    """空文字をNoneとして扱うfloatパーサ。"""

    text = (value or "").strip()
    if not text:
        return None
    return float(text)


def _quaternion_from_heading_deg(
    heading_deg: float,
) -> tuple[float, float, float, float]:
    """heading_degからmap frame上のquaternionを生成する。"""

    yaw = heading_deg_to_yaw_enu_rad(heading_deg)
    return yaw_to_quaternion(yaw)


def load_projection_config(config_path: Path) -> ProjectionConfig:
    """ROS 2 params YAMLからProjectionConfigを生成する。"""

    with config_path.open("r", encoding="utf-8") as f:
        data: dict[str, Any] = yaml.safe_load(f)

    params: dict[str, Any] = {}

    # ROS 2の全ノード共通パラメータ。
    if "/**" in data:
        params.update(data["/**"].get("ros__parameters", {}))

    required_keys = ["origin_latitude", "origin_longitude"]
    missing = [key for key in required_keys if key not in params]
    if missing:
        raise ValueError(f"projection configに必須キーがありません: {missing}")

    return ProjectionConfig(
        origin_latitude=float(params["origin_latitude"]),
        origin_longitude=float(params["origin_longitude"]),
        origin_altitude=float(params.get("origin_altitude", 0.0)),
        map_yaw_offset_rad=float(params.get("map_yaw_offset_rad", 0.0)),
        projection_id=str(params.get("projection_id", "default")),
        datum=str(params.get("datum", "WGS84")),
        map_frame_id=str(params.get("map_frame_id", "map")),
        earth_frame_id=str(params.get("earth_frame_id", "earth")),
    )


def is_target_csv(path: Path) -> bool:
    """処理対象CSVか判定する。"""

    return path.is_file() and path.suffix.lower() == ".csv"


def convert_csv(
    csv_path: Path,
    projection: ProjectionConfig,
    output_path: Path,
) -> bool:
    """1つのCSVに対してlatitude/longitudeからx/y/zを生成する。"""

    with csv_path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            return False

        fieldnames = list(reader.fieldnames)
        required = {"latitude", "longitude"}
        if not required.issubset(set(fieldnames)):
            return False

        for name in ["x", "y", "z", "q1", "q2", "q3", "q4"]:
            if name not in fieldnames:
                fieldnames.append(name)

        rows = []
        for row in reader:
            latitude_text = (row.get("latitude") or "").strip()
            longitude_text = (row.get("longitude") or "").strip()
            altitude_text = (row.get("altitude") or "").strip()
            heading_deg = _parse_optional_float(row.get("heading_deg"))

            if not latitude_text or not longitude_text:
                rows.append(row)
                continue

            latitude = float(latitude_text)
            longitude = float(longitude_text)
            altitude = float(altitude_text) if altitude_text else 0.0

            enu = llh_to_enu(
                LlhPoint(latitude=latitude, longitude=longitude, altitude=altitude),
                projection,
            )

            row["x"] = f"{enu.x:.6f}"
            row["y"] = f"{enu.y:.6f}"
            row["z"] = f"{enu.z:.6f}"
            if heading_deg is not None:
                qx, qy, qz, qw = _quaternion_from_heading_deg(heading_deg)
                row["q1"] = f"{qx:.9f}"
                row["q2"] = f"{qy:.9f}"
                row["q3"] = f"{qz:.9f}"
                row["q4"] = f"{qw:.9f}"
            rows.append(row)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    return True


def resolve_output_path(
    input_root: Path,
    csv_path: Path,
    output_root: Path | None,
) -> Path:
    """出力先CSVパスを決定する。"""

    if output_root is None:
        return csv_path

    relative_path = (
        csv_path.relative_to(input_root) if input_root.is_dir() else csv_path.name
    )
    if isinstance(relative_path, str):
        return output_root / relative_path
    return output_root / relative_path


def main() -> None:
    """エントリポイント。"""

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="CSVファイルまたはCSVディレクトリ")
    parser.add_argument("--config", required=True, help="geo_pose_converterのparams YAML")
    parser.add_argument("--output-dir", default=None, help="出力先ディレクトリ。未指定時は --in-place 必須")
    parser.add_argument("--in-place", action="store_true", help="元CSVを直接上書きする")
    args = parser.parse_args()

    input_path = Path(args.input).resolve()
    config_path = Path(args.config).resolve()
    output_root = Path(args.output_dir).resolve() if args.output_dir else None

    if not args.in_place and output_root is None:
        raise ValueError("--in-place または --output-dir のどちらかを指定してください。")

    projection = load_projection_config(config_path)

    csv_files = [input_path] if input_path.is_file() else sorted(input_path.rglob("*.csv"))

    converted_count = 0
    skipped_count = 0

    for csv_path in csv_files:
        if not is_target_csv(csv_path):
            continue

        output_path = (
            csv_path
            if args.in_place
            else resolve_output_path(input_path, csv_path, output_root)
        )
        converted = convert_csv(csv_path, projection, output_path)

        if converted:
            converted_count += 1
            print(f"converted: {csv_path} -> {output_path}")
        else:
            skipped_count += 1
            print(f"skipped: {csv_path}")

    print(f"done: converted={converted_count}, skipped={skipped_count}")


if __name__ == "__main__":
    main()
