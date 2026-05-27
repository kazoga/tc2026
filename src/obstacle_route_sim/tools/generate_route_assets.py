#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""obstacle_route_sim 用の route_planner/route_manager 資産を生成する."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Iterable, List, Sequence

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from generate_waypoints import write_waypoint_csv  # noqa: E402


@dataclass(frozen=True)
class RouteAssetSpec:
    """生成対象のシミュレーション route 資産定義."""

    road_type: str
    road_width: float

    @property
    def route_id(self) -> str:
        """route/config 名に使う識別子を返す."""

        return f"{self.road_type}_w{int(self.road_width)}"


DEFAULT_ROUTE_SPECS = (
    RouteAssetSpec("straight", 2.0),
    RouteAssetSpec("straight", 3.0),
    RouteAssetSpec("straight", 5.0),
    RouteAssetSpec("scurve", 3.0),
    RouteAssetSpec("scurve", 5.0),
    RouteAssetSpec("crank", 3.0),
    RouteAssetSpec("crank", 5.0),
)


def _workspace_root_from_tool() -> Path:
    """source tree 内の配置から colcon workspace root を推定する."""

    return Path(__file__).resolve().parents[3]


def _write_text_if_changed(path: Path, content: str) -> bool:
    """内容が異なる場合のみ UTF-8 text を書き込む."""

    if path.exists() and path.read_text(encoding="utf-8") == content:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    return True


def _route_config_yaml(route_id: str) -> str:
    """route_planner RouteBuilder 用 route_config.yaml を返す."""

    return (
        "blocks:\n"
        f"  - {{ type: fixed, name: {route_id}, segment_id: \"fixed/waypoints.csv\" }}\n"
    )


def _route_planner_param_yaml(route_id: str) -> str:
    """route_planner node 用 param YAML を返す."""

    route_base = f"routes/obstacle_route_sim/{route_id}"
    return (
        "route_planner:\n"
        "  ros__parameters:\n"
        f"    config_yaml_path: \"{route_base}/route_config.yaml\"\n"
        f"    csv_base_dir: \"{route_base}\"\n"
        "    map_image_path: \"\"\n"
        "    map_worldfile_path: \"\"\n"
    )


def _route_manager_param_yaml(goal_label: int) -> str:
    """route_manager node 用 param YAML を返す."""

    return (
        "route_manager:\n"
        "  ros__parameters:\n"
        "    start_label: \"0\"\n"
        f"    goal_label: \"{goal_label}\"\n"
        "    checkpoint_labels: []\n"
        "    planner_timeout_sec: 5.0\n"
        "    planner_retry_count: 2\n"
        "    planner_connect_timeout_sec: 10.0\n"
        "    state_publish_rate_hz: 1.0\n"
        "    image_encoding_check: false\n"
        "    report_stuck_timeout_sec: 5.0\n"
        "    offset_step_max_m: 1.0\n"
    )


def generate_assets(
    workspace_root: Path,
    specs: Sequence[RouteAssetSpec],
    step_m: float,
) -> List[str]:
    """指定 workspace に route 資産一式を生成する.

    Args:
        workspace_root (Path): colcon workspace root.
        specs (Sequence[RouteAssetSpec]): 生成対象 route 一覧.
        step_m (float): waypoint 間隔 [m].

    Returns:
        List[str]: 生成または確認したファイルの説明行.
    """

    route_planner_root = workspace_root / "src" / "route_planner"
    route_manager_root = workspace_root / "src" / "route_manager"
    messages: List[str] = []

    for spec in specs:
        route_id = spec.route_id
        route_dir = route_planner_root / "routes" / "obstacle_route_sim" / route_id
        waypoint_csv = route_dir / "fixed" / "waypoints.csv"
        waypoint_csv.parent.mkdir(parents=True, exist_ok=True)
        waypoint_count = write_waypoint_csv(
            spec.road_type,
            spec.road_width,
            str(waypoint_csv),
            step_m=step_m,
        )

        route_config = route_dir / "route_config.yaml"
        _write_text_if_changed(route_config, _route_config_yaml(route_id))

        param_name = f"obstacle_route_{route_id}.yaml"
        planner_param = route_planner_root / "params" / param_name
        manager_param = route_manager_root / "params" / param_name
        _write_text_if_changed(planner_param, _route_planner_param_yaml(route_id))
        _write_text_if_changed(manager_param, _route_manager_param_yaml(waypoint_count - 1))

        messages.append(
            f"{route_id}: waypoints={waypoint_count}, goal_label={waypoint_count - 1}"
        )

    return messages


def _parse_specs(raw_specs: Iterable[str]) -> List[RouteAssetSpec]:
    """CLI 指定の road:width を RouteAssetSpec へ変換する."""

    specs: List[RouteAssetSpec] = []
    for raw_spec in raw_specs:
        road_type, _, width_text = raw_spec.partition(":")
        if not road_type or not width_text:
            raise ValueError(f"spec は road:width 形式で指定してください: {raw_spec}")
        specs.append(RouteAssetSpec(road_type, float(width_text)))
    return specs


def build_arg_parser() -> argparse.ArgumentParser:
    """CLI 引数パーサを構築する."""

    parser = argparse.ArgumentParser(
        description="obstacle_route_sim の各 world に対応する route/config を生成する"
    )
    parser.add_argument(
        "--workspace-root",
        type=Path,
        default=_workspace_root_from_tool(),
        help="colcon workspace root。未指定時は source tree 内の配置から推定する",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=5.0,
        help="waypoint 間隔 [m]",
    )
    parser.add_argument(
        "--spec",
        action="append",
        default=[],
        help="生成対象を road:width 形式で追加指定する。未指定時は全 world を生成する",
    )
    return parser


def main() -> None:
    """コマンドラインエントリーポイント."""

    parser = build_arg_parser()
    args = parser.parse_args()
    specs = _parse_specs(args.spec) if args.spec else list(DEFAULT_ROUTE_SPECS)
    messages = generate_assets(args.workspace_root.resolve(), specs, args.step)
    for message in messages:
        print(message)


if __name__ == "__main__":
    main()
