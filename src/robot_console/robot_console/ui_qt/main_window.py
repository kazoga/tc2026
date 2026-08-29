"""PyQt5ローカルUIのメインウィンドウ。"""

from __future__ import annotations

from typing import Optional

from PyQt5 import QtWidgets

from .console_log_tab import ConsoleLogTab
from .dashboard_tab import DashboardTab
from .launch_settings_tab import LaunchSettingsTab
from .localization_sensor_tab import LocalizationSensorTab
from .widgets.scaled_canvas import ScaledCanvas

WINDOW_TITLE = 'robot_console (PyQt5)'
DEFAULT_WINDOW_WIDTH = 1280
DEFAULT_WINDOW_HEIGHT = 720

TAB_TITLE_DASHBOARD = 'ダッシュボード'
TAB_TITLE_LOCALIZATION_SENSOR = '自己位置・センサ情報'
TAB_TITLE_LAUNCH_SETTINGS = '起動・設定'
TAB_TITLE_CONSOLE_LOG = 'コンソールログ'


class MainWindow(QtWidgets.QMainWindow):
    """4タブ構成のPyQt5メインウィンドウ。

    robot_console_gui_screen_function_design.md 2章の方針に従い、全タブ共通の
    上部ステータスバーは設けない。タブ内容はダッシュボードタブを既定表示とし、
    アプリ内コンテンツ領域全体を16:9の論理キャンバスとして拡縮する。
    """

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle(WINDOW_TITLE)
        self.resize(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT)

        self.dashboard_tab = DashboardTab()
        self.localization_sensor_tab = LocalizationSensorTab()
        self.launch_settings_tab = LaunchSettingsTab()
        self.console_log_tab = ConsoleLogTab()

        self.tab_widget = QtWidgets.QTabWidget()
        self.tab_widget.addTab(self.dashboard_tab, TAB_TITLE_DASHBOARD)
        self.tab_widget.addTab(self.localization_sensor_tab, TAB_TITLE_LOCALIZATION_SENSOR)
        self.tab_widget.addTab(self.launch_settings_tab, TAB_TITLE_LAUNCH_SETTINGS)
        self.tab_widget.addTab(self.console_log_tab, TAB_TITLE_CONSOLE_LOG)
        self.tab_widget.setCurrentWidget(self.dashboard_tab)

        self.setCentralWidget(ScaledCanvas(self.tab_widget))

        self.dashboard_tab.node_health_card.profile_selected.connect(
            self._on_node_health_profile_selected
        )

    def _on_node_health_profile_selected(self, profile_id: str) -> None:
        """Node Healthカードのチップ選択を受け、コンソールログタブへ遷移する（9章 画面間導線）。"""

        self.console_log_tab.select_profile(profile_id)
        self.tab_widget.setCurrentWidget(self.console_log_tab)
