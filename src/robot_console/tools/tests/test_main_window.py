"""MainWindow骨格の自動テスト（`QT_QPA_PLATFORM=offscreen` 前提）。"""

import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5 import QtWidgets

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.snapshot_model import ConsoleSnapshot, HealthSummaryView
from robot_console.ui_qt.main_window import (
    TAB_TITLE_CONSOLE_LOG,
    TAB_TITLE_DASHBOARD,
    TAB_TITLE_LAUNCH_SETTINGS,
    TAB_TITLE_LOCALIZATION_SENSOR,
    MainWindow,
)
from robot_console.ui_qt.widgets.scaled_canvas import LOGICAL_HEIGHT, LOGICAL_WIDTH, ScaledCanvas


@pytest.fixture(scope='module')
def qt_app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def test_main_window_has_four_tabs_in_expected_order(qt_app):
    window = MainWindow()
    tab_widget = window.tab_widget

    assert tab_widget.count() == 4
    assert tab_widget.tabText(0) == TAB_TITLE_DASHBOARD
    assert tab_widget.tabText(1) == TAB_TITLE_LOCALIZATION_SENSOR
    assert tab_widget.tabText(2) == TAB_TITLE_LAUNCH_SETTINGS
    assert tab_widget.tabText(3) == TAB_TITLE_CONSOLE_LOG
    assert tab_widget.currentWidget() is window.dashboard_tab


def test_main_window_has_no_status_bar(qt_app):
    window = MainWindow()
    # QMainWindow は statusBar() を一度も呼ばない限りステータスバーを生成しない。
    assert window.findChild(QtWidgets.QStatusBar) is None


def test_central_widget_is_scaled_canvas_with_logical_size(qt_app):
    window = MainWindow()
    canvas = window.centralWidget()

    assert isinstance(canvas, ScaledCanvas)
    assert canvas.content is window.tab_widget
    assert canvas.content.size().width() == LOGICAL_WIDTH
    assert canvas.content.size().height() == LOGICAL_HEIGHT


def test_scaled_canvas_keeps_aspect_ratio_scale_uniform_after_resize(qt_app):
    canvas = ScaledCanvas(QtWidgets.QWidget())
    canvas.resize(800, 600)
    canvas.show()

    transform = canvas.view_transform
    assert transform.m11() > 0.0
    # KeepAspectRatio のため水平・垂直の拡縮率は一致する。
    assert transform.m11() == pytest.approx(transform.m22())


def test_node_health_chip_click_navigates_to_console_log_tab(qt_app):
    window = MainWindow()
    window.dashboard_tab.update_snapshot(
        ConsoleSnapshot(
            health=[
                HealthSummaryView(
                    profile_id='obstacle_monitor', status='ERROR', health=FreshnessLevel.LOST
                )
            ]
        )
    )
    window.console_log_tab.update_snapshot(ConsoleSnapshot())

    chip = window.dashboard_tab.node_health_card._chip_layout.itemAt(0).widget()
    chip.click()

    assert window.tab_widget.currentWidget() is window.console_log_tab
    assert window.console_log_tab._selected_profile_id == 'obstacle_monitor'


def test_gps_pose_detail_button_navigates_to_localization_sensor_tab(qt_app):
    window = MainWindow()

    window.dashboard_tab.view_localization_sensor_requested.emit()

    assert window.tab_widget.currentWidget() is window.localization_sensor_tab


def test_localization_sensor_tab_back_button_navigates_to_dashboard(qt_app):
    window = MainWindow()
    window.tab_widget.setCurrentWidget(window.localization_sensor_tab)

    window.localization_sensor_tab.back_to_dashboard_requested.emit()

    assert window.tab_widget.currentWidget() is window.dashboard_tab


def test_console_log_tab_back_button_navigates_to_dashboard(qt_app):
    window = MainWindow()
    window.tab_widget.setCurrentWidget(window.console_log_tab)

    window.console_log_tab.back_to_dashboard_requested.emit()

    assert window.tab_widget.currentWidget() is window.dashboard_tab


def test_launch_settings_plan_changes_propagate_to_dashboard_launch_control_card(qt_app):
    window = MainWindow()

    window.launch_settings_tab._environment_combo.setCurrentText('実機')
    window.launch_settings_tab._drive_mode_combo.setCurrentText('自律走行')
    window.launch_settings_tab._on_apply_preset_clicked()

    card = window.dashboard_tab.launch_control_card
    assert card._environment_combo.currentText() == '実機'
    assert card._drive_mode_combo.currentText() == '自律走行'
    assert card._ordered_profile_ids == window.launch_settings_tab.plan.ordered_profile_ids
    assert card._node_combo.count() == len(window.launch_settings_tab.plan.ordered_profile_ids)


def test_dashboard_apply_preset_updates_shared_launch_plan(qt_app):
    """ダッシュボードでの業務モード選択・プリセット適用が起動・設定タブの
    起動予定ノード一覧（唯一の実体）へ反映されることを確認する。"""

    window = MainWindow()
    card = window.dashboard_tab.launch_control_card
    card._environment_combo.setCurrentText('シミュレーション')
    card._drive_mode_combo.setCurrentText('手動走行')

    card.apply_preset_requested.emit(
        card._environment_combo.currentText(), card._drive_mode_combo.currentText()
    )

    assert window.launch_settings_tab.plan.ordered_profile_ids == [
        'obstacle_route_sim',
        'drive_mode_manager',
    ]
    # 適用結果が起動操作カードへも反映される（plan_changed経由）。
    assert card._ordered_profile_ids == ['obstacle_route_sim', 'drive_mode_manager']
