"""PyQt5版 robot_console のスタンドアロン起動エントリポイント。

Phase2時点ではROS 2ノードとの統合は行わず、MainWindowと4タブの骨格のみを
表示する。ROS統合（RobotConsoleNode ⇔ ConsoleCore ⇔ Snapshot）は後続フェーズで
追加し、`ros/console_node.py` からQtイベントループとROS executorスレッドを
分離した構成へ拡張する（robot_console_gui_architecture_design.md 14.1節）。
既存のtkinter版entry point（robot_console）は移行期間中そのまま維持する。
"""

from __future__ import annotations

import sys
from typing import List, Optional

from PyQt5 import QtWidgets

from robot_console.ui_qt.main_window import MainWindow
from robot_console.ui_qt.qt_environment import (
    enable_qtwebengine_shared_opengl_contexts,
    fix_qt_plugin_path_conflict,
)


def main(argv: Optional[List[str]] = None) -> int:
    """PyQt5 UIを起動する。"""

    fix_qt_plugin_path_conflict()
    enable_qtwebengine_shared_opengl_contexts()
    app = QtWidgets.QApplication(argv if argv is not None else sys.argv)
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
