"""PyQt5ローカルUIのメインウィンドウ。"""

from __future__ import annotations

from typing import Optional

from PyQt5 import QtWidgets

from .console_log_tab import ConsoleLogTab
from .dashboard_tab import DashboardTab
from .launch_settings_tab import LaunchSettingsTab
from .localization_sensor_tab import LocalizationSensorTab
from .widgets.scaled_canvas import ScaledCanvas
from .widgets.typography import BASE_FONT_POINT_SIZE

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
        self._apply_base_font()
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
        self.dashboard_tab.view_localization_sensor_requested.connect(
            lambda: self.tab_widget.setCurrentWidget(self.localization_sensor_tab)
        )
        self.localization_sensor_tab.back_to_dashboard_requested.connect(
            lambda: self.tab_widget.setCurrentWidget(self.dashboard_tab)
        )
        self.console_log_tab.back_to_dashboard_requested.connect(
            lambda: self.tab_widget.setCurrentWidget(self.dashboard_tab)
        )
        self.launch_settings_tab.plan_changed.connect(self._on_launch_plan_changed)
        self.dashboard_tab.launch_control_card.apply_preset_requested.connect(
            self.launch_settings_tab.apply_preset
        )
        self._on_launch_plan_changed()

    def _on_node_health_profile_selected(self, profile_id: str) -> None:
        """Node Healthカードのチップ選択を受け、コンソールログタブへ遷移する（9章 画面間導線）。"""

        self.console_log_tab.select_profile(profile_id)
        self.tab_widget.setCurrentWidget(self.console_log_tab)

    def _on_launch_plan_changed(self) -> None:
        """起動・設定タブの起動予定ノード一覧を、ダッシュボードの起動操作カードへ反映する。"""

        self.dashboard_tab.launch_control_card.update_plan(
            environment=self.launch_settings_tab.environment,
            drive_mode=self.launch_settings_tab.drive_mode,
            ordered_profile_ids=list(self.launch_settings_tab.plan.ordered_profile_ids),
            profiles_by_id=self.launch_settings_tab.profiles_by_id,
        )

    @staticmethod
    def _apply_base_font() -> None:
        """走行中に数m離れた位置からでも判読しやすいよう、既定フォントを拡大する。"""

        app = QtWidgets.QApplication.instance()
        if app is None:
            return
        font = app.font()
        font.setPointSize(BASE_FONT_POINT_SIZE)
        app.setFont(font)
