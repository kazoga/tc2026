"""robot_console パッケージの公開インタフェース。

正式UIはPyQt5版（`ui_qt_main`）であり、遠隔観測はHTML版（`web_main`）が担当する。
ROS通信は `ros/console_node.py`、状態集約は `core/console_core.py` が持つ。
利用側はこれらのモジュールを直接importする。
"""

from __future__ import annotations

__all__: list = []
