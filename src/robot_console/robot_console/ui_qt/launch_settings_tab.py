"""起動・設定タブ。

業務モード選択、起動候補ツリー、起動予定ノード一覧、ノード設定編集パネル、
起動内容プレビューは robot_console_gui_screen_function_design.md 4章に基づき
後続フェーズで実装する。
"""

from __future__ import annotations

from typing import Optional

from PyQt5 import QtWidgets

from .widgets.placeholder_panel import PlaceholderPanel


class LaunchSettingsTab(QtWidgets.QWidget):
    """業務開始時に使う起動・設定画面。"""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(
            PlaceholderPanel(
                '起動・設定',
                '業務モード選択・起動候補ツリー・起動予定ノード一覧・'
                'ノード設定編集パネル・起動内容プレビューは後続フェーズで実装する。',
            )
        )
