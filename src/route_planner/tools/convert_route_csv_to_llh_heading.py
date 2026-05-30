#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""route_planner/routes 配下のCSVをLLH+heading形式へ移行するスクリプト.

処理内容:
  * 指定ディレクトリ以下を再帰的に探索し、対象CSVを変換する。
  * latitude / longitude が存在しないCSVはスキップする。
  * latitude / longitude が空欄または不正なCSVもスキップする。
  * x, y, z 列は列として残し、値だけ空にする。
  * q1, q2, q3, q4 列は heading_deg から導出した quaternion を設定する。
  * longitude の直後に altitude, heading_deg 列を配置する。
  * altitude は全行 0 にする。
  * heading_deg は次waypointへの方位角にする。
  * 最終行は一つ前の heading_deg を維持する。
  * CSVは直接上書きし、.bak ファイルは作成しない。

このスクリプトは再実行可能。
すでに altitude / heading_deg があるCSVでも列を重複追加せず、値だけ再計算する。
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Sequence


LATITUDE_COL = "latitude"
LONGITUDE_COL = "longitude"
ALTITUDE_COL = "altitude"
HEADING_COL = "heading_deg"

ENU_POSITION_VALUE_COLS = ("x", "y", "z")
QUATERNION_COLS = ("q1", "q2", "q3", "q4")
ENU_POSE_VALUE_COLS = ENU_POSITION_VALUE_COLS + QUATERNION_COLS


def _normalize_heading_deg(heading_deg: float) -> float:
    """headingを[0, 360)へ正規化する."""
    return float(heading_deg) % 360.0


def _try_parse_float(value: str) -> float | None:
    """文字列をfloatに変換できる場合だけfloatを返す."""
    text = str(value).strip()
    if not text:
        return None

    try:
        return float(text)
    except ValueError:
        return None


def _parse_float(value: str, *, path: Path, row_number: int, column: str) -> float:
    """CSVセルをfloatに変換する.

    ここに到達する時点では事前チェック済みの想定だが、
    異常時に原因が分かるように詳細な例外を出す。
    """
    parsed = _try_parse_float(value)
    if parsed is None:
        raise ValueError(f"{path}: {row_number}行目の {column} が空または不正です。")
    return parsed


def _heading_deg_to_yaw_enu_rad(heading_deg: float) -> float:
    """真北基準CW heading[deg]をENU yaw[rad]へ変換する."""
    return math.pi / 2.0 - math.radians(float(heading_deg))


def _quaternion_from_heading_deg(
    heading_deg: float,
) -> tuple[float, float, float, float]:
    """heading_degからgeometry_msgs互換quaternionを生成する."""
    yaw = _heading_deg_to_yaw_enu_rad(heading_deg)
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _bearing_deg_from_llh(
    lat1_deg: float,
    lon1_deg: float,
    lat2_deg: float,
    lon2_deg: float,
) -> float:
    """2つの緯度経度から真北基準CWの初期方位角[deg]を計算する.

    heading_deg の定義:
      * 真北を 0 deg
      * 東を 90 deg
      * 南を 180 deg
      * 西を 270 deg
      * 時計回り

    geo_pose_converter の bearing_from_map_delta() と同じく、
    atan2(east, north) の考え方に合わせた角度を返す。
    """
    lat1 = math.radians(lat1_deg)
    lat2 = math.radians(lat2_deg)
    dlon = math.radians(lon2_deg - lon1_deg)

    east_component = math.sin(dlon) * math.cos(lat2)
    north_component = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    )

    # 同一点の場合は方位が定義できない。
    # 呼び出し側で直前方位を維持する。
    if abs(east_component) < 1.0e-15 and abs(north_component) < 1.0e-15:
        raise ValueError("same point")

    return _normalize_heading_deg(
        math.degrees(math.atan2(east_component, north_component))
    )


def _has_required_columns(header: Sequence[str]) -> bool:
    """変換に必要な列が存在するか確認する."""
    columns = set(header)
    return LATITUDE_COL in columns and LONGITUDE_COL in columns


def _has_enu_pose_columns(header: Sequence[str]) -> bool:
    """旧ENU pose列を持つroute waypoint CSVか確認する."""
    columns = set(header)
    return any(column in columns for column in ENU_POSE_VALUE_COLS)


def _has_complete_llh(rows: Sequence[dict[str, str]]) -> bool:
    """全行に有効なlatitude/longitudeが入っているか確認する."""
    if not rows:
        return False

    for row in rows:
        latitude = _try_parse_float(row.get(LATITUDE_COL, ""))
        longitude = _try_parse_float(row.get(LONGITUDE_COL, ""))

        if latitude is None or longitude is None:
            return False

    return True


def _is_target_csv(
    header: Sequence[str],
    rows: Sequence[dict[str, str]],
) -> tuple[bool, str]:
    """変換対象CSVか判定する.

    Returns:
        (対象ならTrue, 理由文字列)
    """
    if not _has_required_columns(header):
        return False, "missing latitude/longitude columns"

    if not _has_enu_pose_columns(header):
        return False, "missing ENU pose columns"

    if not _has_complete_llh(rows):
        return False, "missing or invalid latitude/longitude values"

    return True, "target"


def _build_output_header(input_header: Sequence[str]) -> list[str]:
    """altitude, heading_deg を longitude 直後へ配置したヘッダを作る.

    既に altitude / heading_deg が存在する場合は一度除外し、
    longitude の直後へ入れ直す。
    これにより、再実行しても列が重複しない。
    """
    output_header: list[str] = []

    for column in input_header:
        if column in (ALTITUDE_COL, HEADING_COL):
            continue

        output_header.append(column)

        if column == LONGITUDE_COL:
            output_header.append(ALTITUDE_COL)
            output_header.append(HEADING_COL)

    return output_header


def _compute_headings(rows: Sequence[dict[str, str]], path: Path) -> list[float]:
    """各行のheading_degを計算する."""
    if not rows:
        return []

    headings: list[float] = []
    last_valid_heading = 0.0

    for index, row in enumerate(rows):
        if index < len(rows) - 1:
            current_row_number = index + 2  # ヘッダ行を1行目として数える。
            next_row_number = index + 3

            lat1 = _parse_float(
                row.get(LATITUDE_COL, ""),
                path=path,
                row_number=current_row_number,
                column=LATITUDE_COL,
            )
            lon1 = _parse_float(
                row.get(LONGITUDE_COL, ""),
                path=path,
                row_number=current_row_number,
                column=LONGITUDE_COL,
            )
            lat2 = _parse_float(
                rows[index + 1].get(LATITUDE_COL, ""),
                path=path,
                row_number=next_row_number,
                column=LATITUDE_COL,
            )
            lon2 = _parse_float(
                rows[index + 1].get(LONGITUDE_COL, ""),
                path=path,
                row_number=next_row_number,
                column=LONGITUDE_COL,
            )

            try:
                last_valid_heading = _bearing_deg_from_llh(lat1, lon1, lat2, lon2)
            except ValueError:
                # 同一点などで方位が定義できない場合は、一つ前の方位を維持する。
                pass

            headings.append(last_valid_heading)
        else:
            # 最終行は一つ前の方位を維持する。
            headings.append(last_valid_heading)

    return headings


def _read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    """CSVを読み込む."""
    with path.open("r", encoding="utf-8-sig", newline="") as file_obj:
        reader = csv.DictReader(file_obj)
        if reader.fieldnames is None:
            return [], []

        header = list(reader.fieldnames)
        rows = list(reader)

    return header, rows


def convert_csv_file(path: Path, *, make_backup: bool = False) -> tuple[bool, str]:
    """CSVファイルを変換する.

    Returns:
        (変換したならTrue, 結果理由)
    """
    input_header, rows = _read_csv(path)

    if not input_header:
        return False, "empty csv"

    is_target, reason = _is_target_csv(input_header, rows)
    if not is_target:
        return False, reason

    output_header = _build_output_header(input_header)
    headings = _compute_headings(rows, path)

    for row, heading in zip(rows, headings):
        # ENU位置は後段の llh_to_enu_csv.py で再生成するため値だけ消す。
        for column in ENU_POSITION_VALUE_COLS:
            if column in row:
                row[column] = ""

        qx, qy, qz, qw = _quaternion_from_heading_deg(heading)
        row["q1"] = f"{qx:.9f}"
        row["q2"] = f"{qy:.9f}"
        row["q3"] = f"{qz:.9f}"
        row["q4"] = f"{qw:.9f}"
        row[ALTITUDE_COL] = "0"
        row[HEADING_COL] = f"{heading:.6f}"

    with path.open("w", encoding="utf-8", newline="") as file_obj:
        writer = csv.DictWriter(
            file_obj,
            fieldnames=output_header,
            extrasaction="ignore",
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)

    return True, "converted"


def find_csv_files(root_dir: Path) -> list[Path]:
    """指定ディレクトリ以下のCSVファイルを再帰的に列挙する."""
    return sorted(path for path in root_dir.rglob("*.csv") if path.is_file())


def main() -> None:
    parser = argparse.ArgumentParser(
        description="route CSVのENU座標値を消去し、altitude/heading_degを追加します。"
    )
    parser.add_argument(
        "root_dir",
        type=Path,
        help="再帰探索するCSVルートディレクトリ。例: src/route_planner/routes",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="互換用オプション。現在は常に .bak を作成せずCSVを直接上書きする。",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="対象CSVの検出だけ行い、ファイルは更新しない。",
    )
    args = parser.parse_args()

    root_dir = args.root_dir
    if not root_dir.exists():
        raise FileNotFoundError(f"ディレクトリが存在しません: {root_dir}")
    if not root_dir.is_dir():
        raise NotADirectoryError(f"ディレクトリではありません: {root_dir}")

    csv_files = find_csv_files(root_dir)
    if not csv_files:
        print(f"CSVファイルが見つかりませんでした: {root_dir}")
        return

    converted_count = 0
    skipped_count = 0

    for csv_path in csv_files:
        if args.dry_run:
            input_header, rows = _read_csv(csv_path)
            is_target, reason = _is_target_csv(input_header, rows)

            if is_target:
                converted_count += 1
                print(f"DRY-RUN target: {csv_path}")
            else:
                skipped_count += 1
                print(f"SKIP {reason}: {csv_path}")

            continue

        converted, reason = convert_csv_file(csv_path, make_backup=False)

        if converted:
            converted_count += 1
            print(f"CONVERTED: {csv_path}")
        else:
            skipped_count += 1
            print(f"SKIP {reason}: {csv_path}")

    print(
        "done: "
        f"converted={converted_count}, "
        f"skipped={skipped_count}, "
        f"total_csv={len(csv_files)}"
    )


if __name__ == "__main__":
    main()
