"""PyQt5起動前に整えておくべきQt関連の環境変数設定。

`opencv-python` は `import cv2` の時点で `QT_QPA_PLATFORM_PLUGIN_PATH` を
自身が同梱するQtプラグイン（PyQt5とは異なるQtビルド）へ書き換える。
`robot_console.utils`（`core.launch_profile` 等が依存する）は `cv2` を
importするため、同一プロセスでPyQt5の `QApplication` を作成する経路では、
書き換え後の環境変数のままだとQtライブラリのバージョン不整合により
（タイミング依存で）セグメンテーションフォルトを起こし得る。

`QApplication` を作成する前に本モジュールの `fix_qt_plugin_path_conflict()`
を呼び、PyQt5自身のプラグインパスへ明示的に戻す。
"""

from __future__ import annotations

import os


def fix_qt_plugin_path_conflict() -> None:
    """`cv2` によるQtプラグインパスの書き換えをPyQt5自身のパスへ戻す。

    `cv2` が未importの環境やimportに失敗する環境では何もしない。
    """

    try:
        import cv2  # noqa: F401
    except ImportError:
        return

    try:
        from PyQt5 import QtCore
    except ImportError:
        return

    plugin_root = QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    if not plugin_root:
        return
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = os.path.join(plugin_root, 'platforms')
    os.environ['QT_PLUGIN_PATH'] = plugin_root
