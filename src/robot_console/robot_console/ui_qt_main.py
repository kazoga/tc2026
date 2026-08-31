"""PyQt5版 robot_console のスタンドアロン起動エントリポイント。

`ConsoleCore` を生成し、`ros/console_node.py::start_ros_thread()` でQt
イベントループとは別スレッドのrclpy executorを起動する
（robot_console_gui_architecture_design.md 14.1節）。`QTimer` で
`ConsoleCore.build_snapshot()` を定期ポーリングし、`MainWindow.update_snapshot()`
経由で4タブへ配布する。既存のtkinter版entry point（robot_console）は移行期間中
そのまま維持する。
"""

from __future__ import annotations

import sys
from typing import List, Optional

from PyQt5 import QtCore, QtWidgets

from robot_console.core.console_core import ConsoleCore
from robot_console.ros.console_node import start_ros_thread
from robot_console.ui_qt.main_window import MainWindow
from robot_console.ui_qt.qt_environment import (
    enable_qtwebengine_shared_opengl_contexts,
    fix_qt_plugin_path_conflict,
)

# HTML遠隔観測UI（web/static/app.js の SNAPSHOT_POLL_MS）と同じ更新周期にする。
SNAPSHOT_POLL_MS = 1000


def main(argv: Optional[List[str]] = None) -> int:
    """PyQt5 UIを起動する。"""

    fix_qt_plugin_path_conflict()
    enable_qtwebengine_shared_opengl_contexts()
    app = QtWidgets.QApplication(argv if argv is not None else sys.argv)

    core = ConsoleCore()
    ros_handle = start_ros_thread(core, node_name='robot_console_qt')
    app.aboutToQuit.connect(ros_handle.stop)

    window = MainWindow(core=core)

    timer = QtCore.QTimer(window)
    timer.timeout.connect(lambda: window.update_snapshot(core.build_snapshot()))
    timer.start(SNAPSHOT_POLL_MS)

    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
