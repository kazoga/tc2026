"""route/follower/targetのROSメッセージを表示用Viewへ変換する純粋関数群。

`RouteView` は `route_state` / `manager_status` / `active_route` の3トピックが
それぞれ別のフィールドを埋めるため、本モジュールの `apply_*_msg()` 系関数は
既存の `RouteView` を受け取り、対象フィールドだけを更新した新しい `RouteView`
を返す（`dataclasses.replace` によるイミュータブルな部分更新）。

ROSメッセージ型を直接importしないため、`rclpy` が無い環境でも単体テスト
できる（テストでは `types.SimpleNamespace` 等の同形オブジェクトを渡す）。
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, List

from .metrics import compute_progress_ratio
from .snapshot_model import FollowerView, ObstacleStateView, RouteView, RouteWaypointView, TargetView

_ROUTE_STATUS_LABELS = {
    0: 'unknown',
    1: 'idle',
    2: 'running',
    3: 'updating_route',
    4: 'holding',
    5: 'completed',
    6: 'error',
}


def route_status_label(value: int) -> str:
    """`tc_route_msgs/RouteState.status`（uint8）を表示用文字列へ変換する。"""

    return _ROUTE_STATUS_LABELS.get(int(value), 'unknown')


def apply_route_state_msg(route: RouteView, msg: Any) -> RouteView:
    """`tc_route_msgs/RouteState`（`route_state`）の内容を`RouteView`へ反映する。"""

    return replace(
        route,
        state=route_status_label(msg.status),
        route_version=int(msg.route_version),
        current_index=int(msg.current_index),
        total_waypoints=int(msg.total_waypoints),
        progress_ratio=compute_progress_ratio(msg.current_index, msg.total_waypoints),
    )


def apply_manager_status_msg(route: RouteView, msg: Any) -> RouteView:
    """`tc_route_msgs/ManagerStatus`（`manager_status`）の内容を`RouteView`へ反映する。"""

    return replace(
        route,
        last_decision=str(getattr(msg, 'decision', '')),
        last_replan_reason=str(getattr(msg, 'last_cause', '')),
    )


def waypoints_from_route_msg(msg: Any) -> List[RouteWaypointView]:
    """`tc_route_msgs/Route.waypoints[]` から地図重畳用のwaypoint列を組み立てる。

    `has_geo_pose` が真の要素のみ緯度経度を埋め、偽の要素はNoneのままとする
    （`geo_pose_source`がENUからの逆投影に依存し得るため、地図表示側で
    座標変換は行わない方針: architecture_design.md 6章）。
    """

    waypoints: List[RouteWaypointView] = []
    for waypoint in getattr(msg, 'waypoints', []):
        latitude = None
        longitude = None
        if bool(getattr(waypoint, 'has_geo_pose', False)):
            point = waypoint.geo_pose.point
            latitude = float(point.latitude)
            longitude = float(point.longitude)
        waypoints.append(RouteWaypointView(index=int(waypoint.index), latitude=latitude, longitude=longitude))
    return waypoints


def apply_route_msg(route: RouteView, msg: Any) -> RouteView:
    """`tc_route_msgs/Route`（`active_route`）の内容を`RouteView`へ反映する。"""

    return replace(
        route,
        route_version=int(msg.version),
        waypoints=waypoints_from_route_msg(msg),
    )


def follower_view_from_msg(msg: Any) -> FollowerView:
    """`tc_route_msgs/FollowerState`（`follower_state`）から`FollowerView`を組み立てる。"""

    return FollowerView(
        state=str(msg.state),
        active_waypoint_index=int(msg.active_waypoint_index),
        active_waypoint_label=str(msg.active_waypoint_label),
        stagnation_reason=str(getattr(msg, 'last_stagnation_reason', '')),
        avoidance_attempt_count=int(msg.avoidance_attempt_count),
        front_blocked=bool(msg.front_blocked),
        front_clearance_m=float(msg.front_clearance_m),
        left_offset_m=float(msg.left_offset_m),
        right_offset_m=float(msg.right_offset_m),
    )


def obstacle_view_from_hint_msg(msg: Any) -> ObstacleStateView:
    """`tc_route_msgs/ObstacleAvoidanceHint`（`obstacle_avoidance_hint`）から
    `ObstacleStateView` を組み立てる。
    """

    return ObstacleStateView(
        front_blocked=bool(msg.front_blocked),
        front_clearance_m=float(msg.front_clearance_m),
        left_offset_m=float(msg.left_offset_m),
        right_offset_m=float(msg.right_offset_m),
    )


def target_view_from_pose_msg(msg: Any) -> TargetView:
    """`geometry_msgs/PoseStamped`（`active_target`）から`TargetView`を組み立てる。

    distance_m/within_arrival_thresholdは自己位置との相対計算が必要なため
    ここでは埋めない（`ConsoleCore.update_active_target()`が算出する）。
    """

    position = msg.pose.position
    return TargetView(x_m=float(position.x), y_m=float(position.y), z_m=float(position.z))
