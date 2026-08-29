"""自己位置・センサ情報タブ。

運行サマリ、OpenStreetMap上の自己位置・waypoint重畳、センサ・画像パネルは
robot_console_gui_screen_function_design.md 7章に基づき後続フェーズで実装する。
"""

from __future__ import annotations

from typing import Optional

from PyQt5 import QtWidgets

from .widgets.placeholder_panel import PlaceholderPanel


class LocalizationSensorTab(QtWidgets.QWidget):
    """自己位置・センサ情報の詳細確認画面。"""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(
            PlaceholderPanel(
                '自己位置・センサ情報',
                '運行サマリ・OpenStreetMap地図・センサ/画像パネル・鮮度表示は'
                '後続フェーズで実装する。',
            )
        )
