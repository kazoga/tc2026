"""LocalizationSensorTab の単体テスト（`QT_QPA_PLATFORM=offscreen` 前提）。"""

import pytest
from PIL import Image
from PyQt5 import QtWidgets

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.image_store import ImageStore
from robot_console.core.snapshot_model import (
    ConsoleSnapshot,
    GpsStateView,
    ImageReference,
    LocalizationStateView,
    OperationStateView,
    TargetView,
)
from robot_console.ui_qt.localization_sensor_tab import (
    DEFAULT_SENSOR_PANELS,
    LocalizationSensorTab,
)


@pytest.fixture(scope='module')
def qt_app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def test_default_construction_shows_default_sensor_panels(qt_app):
    tab = LocalizationSensorTab()

    assert tab._route_overlay_group.title() == '地図 / Route Overlay'
    assert tab._route_map_panel.title() == 'Route Map'
    assert tab._grid_layout.count() == len(DEFAULT_SENSOR_PANELS)


def test_update_snapshot_reflects_summary_and_localization_card(qt_app):
    tab = LocalizationSensorTab()
    snapshot = ConsoleSnapshot(
        operation_state=OperationStateView(
            phase='走行中', current_waypoint='A-10', next_waypoint='A-11', route_progress=0.4
        ),
        gps_state=GpsStateView(rtk_state='RTK_FIX', num_satellites=15, hdop=0.9),
        localization_state=LocalizationStateView(
            source='pose_enu', x_m=1.5, y_m=2.5, yaw_deg=90.0, freshness=FreshnessLevel.OK
        ),
        target_state=TargetView(distance_m=3.2, within_arrival_threshold=True),
    )

    tab.update_snapshot(snapshot)

    assert tab._phase_label.text() == '走行中'
    assert tab._waypoint_label.text() == 'A-10 -> A-11'
    assert tab._progress_label.text() == '40.0%'
    assert tab._loc_position_label.text() == 'x=1.50 y=2.50'
    assert tab._loc_yaw_label.text() == '90.0 deg'
    assert tab._gps_rtk_label.text() == 'RTK_FIX'
    assert '到達' in tab._target_distance_label.text()


def test_sensor_panels_from_snapshot_replace_defaults(qt_app):
    tab = LocalizationSensorTab()
    snapshot = ConsoleSnapshot(
        sensor_panels=[
            ImageReference(panel_id='route_map', title='Route Map', topic='/active_route'),
            ImageReference(panel_id='sensor_viewer', title='Sensor Viewer', topic='/sensor_viewer'),
        ]
    )

    tab.update_snapshot(snapshot)

    # route_mapはグリッドから除外され、専用パネルへ表示される
    assert tab._grid_layout.count() == 1


def test_image_store_image_is_rendered_in_route_map_panel(qt_app):
    store = ImageStore()
    store.set('route_map', Image.new('RGB', (8, 8), color='red'))
    tab = LocalizationSensorTab(image_store=store)

    tab.update_snapshot(ConsoleSnapshot())

    assert tab._route_map_panel._image_label.text() == ''


def test_back_to_dashboard_signal_emitted_on_button_click(qt_app):
    tab = LocalizationSensorTab()
    received = []
    tab.back_to_dashboard_requested.connect(lambda: received.append(True))

    back_button = tab.findChildren(QtWidgets.QPushButton)[0]
    assert back_button.text() == 'ダッシュボードへ戻る'
    back_button.click()

    assert received == [True]


def test_sensor_freshness_summary_reports_worst_level(qt_app):
    tab = LocalizationSensorTab()
    snapshot = ConsoleSnapshot(
        sensor_panels=[
            ImageReference(panel_id='sensor_viewer', freshness=FreshnessLevel.OK),
            ImageReference(panel_id='lidar_view', freshness=FreshnessLevel.LOST),
        ]
    )

    tab.update_snapshot(snapshot)

    assert tab._sensor_freshness_label.text() == 'LOST'
