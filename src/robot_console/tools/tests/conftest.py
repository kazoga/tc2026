"""tools/tests 配下の pytest 共通セットアップ。

ROS 2 環境が有効化されていない場合でも `robot_console` パッケージを
import できるよう、`sensor_msgs` / `numpy` / `PIL` の最小スタブを用意する。
ROS 2 環境が有効な場合（`sensor_msgs` などが実際に import できる場合）は
本モジュールは何もしない。

また、`PyQt5` を使うテストがディスプレイの無い環境でも実行できるよう、他の
モジュールが `PyQt5` をimportする前に `QT_QPA_PLATFORM=offscreen` を既定値
として設定する。

`opencv-python` は import 時に `QT_QPA_PLATFORM_PLUGIN_PATH` を自身が同梱する
Qtプラグイン（cv2/qt/plugins、PyQt5と異なるQtビルド）へ書き換える。同一プロセス
内でPyQt5のQApplicationがこのパスからプラットフォームプラグインを読み込むと、
Qtライブラリのバージョン不整合により（タイミング依存で）セグメンテーション
フォルトを起こすことを確認したため、`cv2` の読み込み後にPyQt5自身のプラグイン
パスへ明示的に戻す。
"""

from __future__ import annotations

import os
import sys
import types
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    import cv2  # noqa: F401  QT_QPA_PLATFORM_PLUGIN_PATH 書き換えを先に発生させる
except ImportError:
    pass

try:
    from PyQt5 import QtCore

    _qt_plugin_root = QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    if _qt_plugin_root:
        os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = os.path.join(_qt_plugin_root, 'platforms')
        os.environ['QT_PLUGIN_PATH'] = _qt_plugin_root
except ImportError:
    pass


def _install_sensor_stubs() -> None:
    sensor_msgs = types.ModuleType('sensor_msgs')
    msg_module = types.ModuleType('sensor_msgs.msg')

    class Image:  # pragma: no cover - スタブのみ
        height = 0
        width = 0
        encoding = 'rgb8'
        data = b''

    msg_module.Image = Image
    sensor_msgs.msg = msg_module
    sys.modules['sensor_msgs'] = sensor_msgs
    sys.modules['sensor_msgs.msg'] = msg_module


def _install_numpy_stub() -> None:
    numpy_stub = types.ModuleType('numpy')

    def _frombuffer(data, dtype):  # pragma: no cover - スタブのみ
        return []

    numpy_stub.frombuffer = _frombuffer
    sys.modules['numpy'] = numpy_stub


def _install_pil_stub() -> None:
    pil_module = types.ModuleType('PIL')
    image_module = types.ModuleType('PIL.Image')
    imagetk_module = types.ModuleType('PIL.ImageTk')

    class _DummyImage:  # pragma: no cover - スタブのみ
        mode = 'RGB'
        width = 1
        height = 1
        info: dict = {}

        def resize(self, size, _filter=None):
            self.width, self.height = size
            return self

        def copy(self):
            return self

        def paste(self, _other, box=None, mask=None):
            return None

        def convert(self, _mode):
            return self

        def alpha_composite(self, _other):
            return None

    def _fromarray(_array, mode=None):
        return _DummyImage()

    def _new(mode, size, color):
        return _DummyImage()

    image_module.Image = _DummyImage
    image_module.NEAREST = 0
    image_module.fromarray = _fromarray
    image_module.new = _new
    imagetk_module.PhotoImage = _DummyImage
    pil_module.Image = image_module
    pil_module.ImageTk = imagetk_module
    sys.modules['PIL'] = pil_module
    sys.modules['PIL.Image'] = image_module
    sys.modules['PIL.ImageTk'] = imagetk_module


try:
    import sensor_msgs.msg  # noqa: F401
except ImportError:
    _install_sensor_stubs()

try:
    import numpy  # noqa: F401
except ImportError:
    _install_numpy_stub()

try:
    import PIL  # noqa: F401
except ImportError:
    _install_pil_stub()
