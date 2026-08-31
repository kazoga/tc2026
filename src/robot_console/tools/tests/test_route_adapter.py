"""core/route_adapter.py の単体テスト（ROS非依存）。"""

from types import SimpleNamespace

from robot_console.core.route_adapter import (
    apply_manager_status_msg,
    apply_route_msg,
    apply_route_state_msg,
    follower_view_from_msg,
    obstacle_view_from_hint_msg,
    route_status_label,
    target_view_from_pose_msg,
    waypoints_from_route_msg,
)
from robot_console.core.snapshot_model import RouteView


def _waypoint(index, *, has_geo_pose=False, latitude=None, longitude=None):
    geo_pose = SimpleNamespace(point=SimpleNamespace(latitude=latitude, longitude=longitude))
    return SimpleNamespace(index=index, has_geo_pose=has_geo_pose, geo_pose=geo_pose)


def test_route_status_label_maps_known_values():
    assert route_status_label(2) == 'running'
    assert route_status_label(6) == 'error'


def test_route_status_label_falls_back_to_unknown():
    assert route_status_label(99) == 'unknown'


def test_apply_route_state_msg_updates_progress_fields():
    msg = SimpleNamespace(status=2, route_version=4, current_index=3, total_waypoints=10)

    route = apply_route_state_msg(RouteView(), msg)

    assert route.state == 'running'
    assert route.route_version == 4
    assert route.current_index == 3
    assert route.total_waypoints == 10
    assert route.progress_ratio == 0.3


def test_apply_route_state_msg_preserves_waypoints_from_other_updates():
    route = RouteView(waypoints=[])
    msg = SimpleNamespace(status=2, route_version=4, current_index=1, total_waypoints=2)

    updated = apply_route_state_msg(route, msg)

    assert updated.waypoints == route.waypoints  # 他フィールドは変更しない


def test_apply_manager_status_msg_updates_decision_fields():
    msg = SimpleNamespace(decision='avoid', last_cause='obstacle_detected')

    route = apply_manager_status_msg(RouteView(), msg)

    assert route.last_decision == 'avoid'
    assert route.last_replan_reason == 'obstacle_detected'


def test_waypoints_from_route_msg_extracts_lat_lon_only_when_has_geo_pose():
    msg = SimpleNamespace(
        waypoints=[
            _waypoint(0, has_geo_pose=True, latitude=36.083, longitude=140.113),
            _waypoint(1, has_geo_pose=False),
        ]
    )

    waypoints = waypoints_from_route_msg(msg)

    assert waypoints[0].index == 0
    assert waypoints[0].latitude == 36.083
    assert waypoints[1].latitude is None
    assert waypoints[1].longitude is None


def test_apply_route_msg_updates_version_and_waypoints():
    msg = SimpleNamespace(version=7, waypoints=[_waypoint(0, has_geo_pose=True, latitude=1.0, longitude=2.0)])

    route = apply_route_msg(RouteView(), msg)

    assert route.route_version == 7
    assert len(route.waypoints) == 1


def test_follower_view_from_msg_maps_all_fields():
    msg = SimpleNamespace(
        state='following', active_waypoint_index=5, active_waypoint_label='A-5',
        last_stagnation_reason='', avoidance_attempt_count=1, front_blocked=True,
        front_clearance_m=0.5, left_offset_m=0.1, right_offset_m=0.2,
    )

    view = follower_view_from_msg(msg)

    assert view.state == 'following'
    assert view.active_waypoint_index == 5
    assert view.front_blocked is True


def test_obstacle_view_from_hint_msg_maps_all_fields():
    msg = SimpleNamespace(front_blocked=True, front_clearance_m=0.8, left_offset_m=0.1, right_offset_m=0.2)

    view = obstacle_view_from_hint_msg(msg)

    assert view.front_blocked is True
    assert view.front_clearance_m == 0.8
    assert view.left_offset_m == 0.1
    assert view.right_offset_m == 0.2


def test_target_view_from_pose_msg_extracts_position_only():
    msg = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=3.0, y=4.0, z=0.0)))

    view = target_view_from_pose_msg(msg)

    assert view.x_m == 3.0
    assert view.y_m == 4.0
    assert view.distance_m == 0.0  # 未算出（ConsoleCore側で計算）
