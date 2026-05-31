"""route_builder の LLH CSV 入出力に関する単体テスト."""

from __future__ import annotations

import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from route_planner.route_builder import parse_waypoint_csv, write_waypoints_to_csv  # noqa: E402


def test_parse_waypoint_csv_reads_llh_altitude_and_heading(tmp_path: Path) -> None:
    """waypoint CSV の LLH/altitude/heading_deg を WaypointRecord に保持する."""

    csv_path = tmp_path / "waypoints.csv"
    csv_path.write_text(
        "label,x,y,z,q1,q2,q3,q4,latitude,longitude,altitude,heading_deg\n"
        "wp1,1.0,2.0,3.0,0,0,0,1,36.082331,140.111681,25.5,90.0\n",
        encoding="utf-8",
    )

    waypoints = parse_waypoint_csv(str(csv_path))

    assert len(waypoints) == 1
    assert waypoints[0].latitude == 36.082331
    assert waypoints[0].longitude == 140.111681
    assert waypoints[0].altitude == 25.5
    assert waypoints[0].heading_deg == 90.0


def test_write_waypoints_to_csv_keeps_llh_columns(tmp_path: Path) -> None:
    """WaypointRecord を CSV に書き戻す際も LLH 系カラムを出力する."""

    source_path = tmp_path / "source.csv"
    source_path.write_text(
        "label,x,y,z,q1,q2,q3,q4,lat,lon,alt,heading\n"
        "wp1,1.0,2.0,3.0,0,0,0,1,36.082331,140.111681,25.5,90.0\n",
        encoding="utf-8",
    )
    waypoints = parse_waypoint_csv(str(source_path))
    out_path = tmp_path / "out.csv"

    write_waypoints_to_csv(str(out_path), waypoints)

    with out_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert rows[0]["latitude"] == "36.082331"
    assert rows[0]["longitude"] == "140.111681"
    assert rows[0]["altitude"] == "25.5"
    assert rows[0]["heading_deg"] == "90.0"
