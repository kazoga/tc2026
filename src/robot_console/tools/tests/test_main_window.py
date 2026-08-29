"""MainWindow骨格の自動テスト（`QT_QPA_PLATFORM=offscreen` 前提）。"""

import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5 import QtWidgets

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
