"""web/json_codec.py の単体テスト。"""

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.snapshot_model import (
    ConsoleSnapshot,
    GpsStateView,
    HealthSummaryView,
    ImageReference,
    LocalizationStateView,
    ManualControlsView,
    OperationStateView,
    TargetView,
)
from robot_console.web.json_codec import (
    build_health_payload,
    build_map_state_payload,
    build_sensor_panels_payload,
    build_snapshot_payload,
)


def _sample_snapshot() -> ConsoleSnapshot:
    return ConsoleSnapshot(
        operation_state=OperationStateView(
            environment='実機', drive_mode='自律', phase='走行中', route_progress=0.5,
            current_waypoint='A-10', next_waypoint='A-11',
        ),
        gps_state=GpsStateView(rtk_state='RTK_FIX', num_satellites=18),
        localization_state=LocalizationStateView(
            source='pose_enu', x_m=1.0, y_m=2.0, freshness=FreshnessLevel.OK
        ),
        target_state=TargetView(distance_m=3.5, within_arrival_threshold=False),
        manual_controls=ManualControlsView(manual_start_value=True),
        sensor_panels=[
            ImageReference(
                panel_id='sensor_viewer', title='Sensor Viewer', topic='/sensor_viewer',
                freshness=FreshnessLevel.OK,
            )
        ],
        health=[
            HealthSummaryView(profile_id='route_manager', category='route_stack', status='RUNNING')
        ],
    )


def test_snapshot_payload_contains_documented_sections():
    payload = build_snapshot_payload(_sample_snapshot())

    assert payload['operation']['phase'] == '走行中'
    assert payload['gps']['rtk_state'] == 'RTK_FIX'
    assert payload['localization']['x_m'] == 1.0
    assert payload['target']['distance_m'] == 3.5
    assert payload['sensor_panels'][0]['panel_id'] == 'sensor_viewer'
    assert payload['health'][0]['profile_id'] == 'route_manager'


def test_snapshot_payload_excludes_manual_controls_and_launch_profiles():
    payload = build_snapshot_payload(_sample_snapshot())

    assert 'manual_controls' not in payload
    assert 'launch_profiles' not in payload
    assert 'logs' not in payload
    assert 'log_paths' not in payload


def test_health_payload_excludes_override_inputs():
    payload = build_health_payload(_sample_snapshot())

    for profile in payload['profiles']:
        assert 'override_inputs' not in profile
        assert set(profile.keys()) == {
            'profile_id', 'category', 'status', 'health', 'required_but_not_selected'
        }


def test_freshness_enums_are_serialized_as_plain_strings():
    payload = build_snapshot_payload(_sample_snapshot())

    assert payload['localization']['freshness'] == 'OK'
    assert isinstance(payload['gps']['fix_freshness'], str)


def test_map_state_payload_uses_localization_and_target():
    payload = build_map_state_payload(_sample_snapshot())

    assert payload['current_position']['x_m'] == 1.0
    assert payload['target_position']['x_m'] is None  # サンプルではx_m未設定
    assert payload['route_progress']['progress_ratio'] == 0.0  # RouteViewは既定値


def test_sensor_panels_payload_matches_snapshot_panels():
    payload = build_sensor_panels_payload(_sample_snapshot())

    assert len(payload['panels']) == 1
    assert payload['panels'][0]['panel_id'] == 'sensor_viewer'


def test_timestamp_is_iso_formatted_string():
    payload = build_snapshot_payload(_sample_snapshot())

    assert 'T' in payload['timestamp']
