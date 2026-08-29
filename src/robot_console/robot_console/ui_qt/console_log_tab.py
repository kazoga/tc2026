"""コンソールログタブ。

profile別ログ、統合ログ、WARN/ERRORフィルタ、tail追従、検索は
robot_console_gui_screen_function_design.md 5章に基づき後続フェーズで実装する。
"""

from __future__ import annotations

from typing import Optional

from PyQt5 import QtWidgets

from .widgets.placeholder_panel import PlaceholderPanel


class ConsoleLogTab(QtWidgets.QWidget):
    """起動後の詳細確認・異常調査・開発時デバッグ用の画面。"""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(
            PlaceholderPanel(
                'コンソールログ',
                'profile別ログ・統合ログ・WARN/ERRORフィルタ・検索・tail追従は'
                '後続フェーズで実装する。',
            )
        )
