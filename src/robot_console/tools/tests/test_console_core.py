"""ConsoleCore（ROS非依存の状態集約Facade）の単体テスト。"""

from pathlib import Path
from types import SimpleNamespace

from robot_console.core.console_core import ConsoleCore
from robot_console.core.freshness import FreshnessLevel
from robot_console.core.launch_profile import LaunchProfileStore
from robot_console.core.snapshot_model import ConsoleSnapshot
from robot_console.utils import NodeLaunchStatus

REPO_PROFILE_PATH = Path(__file__).resolve().parents[2] / 'config' / 'node_launch_profiles.yaml'


def _make_core() -> ConsoleCore:
    return ConsoleCore(profile_store=LaunchProfileStore(REPO_PROFILE_PATH))


def test_build_snapshot_returns_console_snapshot_with_default_views():
    core = _make_core()

    snapshot = core.build_snapshot()

    assert isinstance(snapshot, ConsoleSnapshot)
    assert snapshot.operation_state.phase == '未起動'
    assert snapshot.route_state.waypoints == []


def test_build_snapshot_includes_health_for_all_profiles():
    core = _make_core()

    snapshot = core.build_snapshot()

    profile_ids = {item.profile_id for item in snapshot.health}
    assert profile_ids == {profile.profile_id for profile in core._profiles}
    assert len(snapshot.health) == 14
    assert all(item.status == 'STOPPED' for item in snapshot.health)


def test_launch_status_callback_updates_health_and_launch_profiles():
    core = _make_core()
    profile_id = core._profiles[0].profile_id

    core._on_launch_status(profile_id, NodeLaunchStatus.RUNNING, 4242, None)
    snapshot = core.build_snapshot()

    assert snapshot.launch_profiles[profile_id].status == NodeLaunchStatus.RUNNING
    assert snapshot.launch_profiles[profile_id].process_id == 4242
    health = next(item for item in snapshot.health if item.profile_id == profile_id)
    assert health.status == 'RUNNING'


def test_launch_status_callback_handles_simulator_suffix_separately():
    core = _make_core()
    profile_id = core._profiles[0].profile_id

    core._on_launch_status(f'{profile_id}:sim', NodeLaunchStatus.RUNNING, 555, None)
    snapshot = core.build_snapshot()

    state = snapshot.launch_profiles[profile_id]
    assert state.simulator_status == NodeLaunchStatus.RUNNING
    assert state.simulator_process_id == 555
    assert state.status == NodeLaunchStatus.STOPPED  # 本体側は未変更


def test_launch_status_callback_ignores_unknown_profile_id():
    core = _make_core()

    core._on_launch_status('unknown_profile', NodeLaunchStatus.RUNNING, 1, None)

    assert 'unknown_profile' not in core.build_snapshot().launch_profiles


def test_launch_log_callback_forwards_to_log_manager():
    core = _make_core()
    profile_id = core._profiles[0].profile_id

    core._on_launch_log(profile_id, '[INFO] hello\n')

    assert core.build_snapshot().logs[profile_id] == ['[INFO] hello\n']


def test_request_launch_ignores_unknown_profile_id():
    core = _make_core()

    core.request_launch('unknown_profile')  # 例外を送出しないことのみ確認


def _waypoint(index, *, has_geo_pose=False, latitude=None, longitude=None):
    geo_pose = SimpleNamespace(point=SimpleNamespace(latitude=latitude, longitude=longitude))
    return SimpleNamespace(index=index, has_geo_pose=has_geo_pose, geo_pose=geo_pose)


def test_update_route_state_and_manager_status_merge_into_same_route_view():
    core = _make_core()

    core.update_route_state(
        SimpleNamespace(status=2, route_version=1, current_index=2, total_waypoints=4)
    )
    core.update_manager_status(SimpleNamespace(decision='avoid', last_cause='obstacle_detected'))
    route = core.build_snapshot().route_state

    assert route.state == 'running'
    assert route.progress_ratio == 0.5
    assert route.last_decision == 'avoid'
    assert route.last_replan_reason == 'obstacle_detected'


def test_update_route_populates_waypoints():
    core = _make_core()

    core.update_route(
        SimpleNamespace(
            version=3,
            waypoints=[_waypoint(0, has_geo_pose=True, latitude=36.083, longitude=140.113)],
        )
    )
    route = core.build_snapshot().route_state

    assert route.route_version == 3
    assert route.waypoints[0].latitude == 36.083


def test_update_follower_state_reflects_in_snapshot():
    core = _make_core()

    core.update_follower_state(
        SimpleNamespace(
            state='following', active_waypoint_index=2, active_waypoint_label='A-2',
            last_stagnation_reason='', avoidance_attempt_count=0, front_blocked=False,
            front_clearance_m=5.0, left_offset_m=0.0, right_offset_m=0.0,
        )
    )

    assert core.build_snapshot().follower_state.active_waypoint_label == 'A-2'


def test_update_obstacle_hint_sets_freshness_ok_immediately_after_update():
    core = _make_core()

    core.update_obstacle_hint(
        SimpleNamespace(front_blocked=True, front_clearance_m=0.3, left_offset_m=0.0, right_offset_m=0.0)
    )
    obstacle = core.build_snapshot().obstacle_state

    assert obstacle.front_blocked is True
    assert obstacle.freshness == FreshnessLevel.OK


def _pose_enu_msg(x, y, z=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
    )


def _pose_llh_msg(latitude, longitude):
    return SimpleNamespace(
        pose=SimpleNamespace(
            point=SimpleNamespace(latitude=latitude, longitude=longitude, altitude=0.0, has_altitude=False),
            has_yaw_enu=False,
            yaw_enu_rad=0.0,
            child_frame_id='base_link',
        )
    )


def test_update_pose_enu_then_pose_llh_merge_into_single_localization_view():
    core = _make_core()

    core.update_pose_enu(_pose_enu_msg(1.0, 2.0))
    core.update_pose_llh(_pose_llh_msg(36.083, 140.113))
    localization = core.build_snapshot().localization_state

    assert localization.x_m == 1.0  # pose_enu由来の値が失われていない
    assert localization.y_m == 2.0
    assert localization.latitude == 36.083
    assert localization.longitude == 140.113
    assert localization.source == 'pose_llh'
    assert localization.freshness == FreshnessLevel.OK


def test_update_gps_status_reflects_in_snapshot_with_ok_freshness():
    core = _make_core()

    core.update_gps_status(
        SimpleNamespace(
            rtk_state=4, rtk_state_raw='RTK_FIX', num_satellites=19, hdop=0.7,
            correction_age_s=0.9, rtcm_bytes_received=1000, heading_deg=87.3,
            heading_stddev_deg=0.6, baseline_length_m=1.2, latitude=36.083,
            longitude=140.113, altitude=25.0,
        )
    )
    gps = core.build_snapshot().gps_state

    assert gps.rtk_state == 'RTK_FIX'
    assert gps.fix_freshness == FreshnessLevel.OK
    assert gps.heading_freshness == FreshnessLevel.OK


def test_update_active_target_computes_distance_from_current_localization():
    core = _make_core()
    core.update_pose_enu(_pose_enu_msg(0.0, 0.0))

    core.update_active_target(
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=3.0, y=4.0, z=0.0)))
    )
    target = core.build_snapshot().target_state

    assert target.distance_m == 5.0  # 3-4-5の直角三角形
    assert target.freshness == FreshnessLevel.OK


def test_update_active_target_without_localization_uses_zero_distance():
    core = _make_core()

    core.update_active_target(
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=3.0, y=4.0, z=0.0)))
    )
    target = core.build_snapshot().target_state

    assert target.distance_m == 0.0


def _image_msg(width=2, height=2, encoding='rgb8'):
    channels = 3
    return SimpleNamespace(
        width=width, height=height, encoding=encoding, data=bytes([255]) * (width * height * channels)
    )


def test_update_sensor_image_stores_image_and_metadata():
    core = _make_core()

    core.update_sensor_image('sensor_viewer', 'Sensor Viewer', '/sensor_viewer', _image_msg())
    snapshot = core.build_snapshot()

    assert core.image_store.get('sensor_viewer') is not None
    panel = next(p for p in snapshot.sensor_panels if p.panel_id == 'sensor_viewer')
    assert panel.title == 'Sensor Viewer'
    assert panel.width == 2
    assert panel.height == 2
    assert panel.freshness == FreshnessLevel.OK


def test_update_sensor_image_with_undecodable_message_keeps_metadata_without_image():
    core = _make_core()

    core.update_sensor_image('sensor_viewer', 'Sensor Viewer', '/sensor_viewer', _image_msg(width=0, height=0))
    snapshot = core.build_snapshot()

    assert core.image_store.get('sensor_viewer') is None
    panel = next(p for p in snapshot.sensor_panels if p.panel_id == 'sensor_viewer')
    assert panel.width == 0


def test_send_manual_start_updates_manual_controls_and_calls_publisher():
    core = _make_core()
    sent = []
    core.bind_publishers(manual_start=lambda value: sent.append(value))

    core.send_manual_start(True)
    manual_controls = core.build_snapshot().manual_controls

    assert sent == [True]
    assert manual_controls.manual_start_value is True
    assert manual_controls.manual_start_last_sent_at is not None
    assert manual_controls.input_source == 'gui'


def test_send_manual_start_without_bound_publisher_only_updates_state():
    core = _make_core()

    core.send_manual_start(True)  # publisher未登録でも例外を送出しないことを確認

    assert core.build_snapshot().manual_controls.manual_start_value is True


def test_send_sig_recog_updates_manual_controls_and_calls_publisher():
    core = _make_core()
    sent = []
    core.bind_publishers(sig_recog=lambda value: sent.append(value))

    core.send_sig_recog(2)

    assert sent == [2]
    assert core.build_snapshot().manual_controls.sig_recog_value == 2


def test_send_road_blocked_updates_manual_controls_and_calls_publisher():
    core = _make_core()
    sent = []
    core.bind_publishers(road_blocked=lambda value: sent.append(value))

    core.send_road_blocked(True)
    manual_controls = core.build_snapshot().manual_controls

    assert sent == [True]
    assert manual_controls.road_blocked_value is True
    assert manual_controls.road_blocked_source == 'gui'


def test_send_obstacle_hint_override_calls_publisher_with_all_fields():
    core = _make_core()
    sent = []
    core.bind_publishers(obstacle_hint=lambda *args: sent.append(args))

    core.send_obstacle_hint_override(True, 1.5, 0.1, 0.2)
    manual_controls = core.build_snapshot().manual_controls

    assert sent == [(True, 1.5, 0.1, 0.2)]
    assert manual_controls.obstacle_hint_override_active is True


def test_send_obstacle_hint_stop_publishes_cleared_values():
    core = _make_core()
    sent = []
    core.bind_publishers(obstacle_hint=lambda *args: sent.append(args))

    core.send_obstacle_hint_stop()

    assert sent == [(False, 0.0, 0.0, 0.0)]
    assert core.build_snapshot().manual_controls.obstacle_hint_override_active is False


def test_send_frame_image_request_calls_publisher():
    core = _make_core()
    sent = []
    core.bind_publishers(frame_image=lambda path: sent.append(path))

    core.send_frame_image_request('/tmp/frame.png')

    assert sent == ['/tmp/frame.png']
