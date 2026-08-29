"""ダッシュボードタブ。

走行前確認と走行中監視の主画面。運行状態、GPS/Poseカード、Drive/CmdVelカード、
Eventカード、Manual Opsカード、Node Healthカードは
robot_console_gui_screen_function_design.md 6章に基づき後続フェーズで実装する。
"""

from __future__ import annotations

from typing import Optional

from PyQt5 import QtWidgets

from .widgets.placeholder_panel import PlaceholderPanel


class DashboardTab(QtWidgets.QWidget):
    """走行前確認・走行中監視の主画面。"""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(
            PlaceholderPanel(
                'ダッシュボード',
                '運行状態・GPS/Poseカード・Drive/CmdVelカード・Eventカード・'
                'Manual Opsカード・Node Healthカードは後続フェーズで実装する。',
            )
        )
