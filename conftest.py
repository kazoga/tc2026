"""ワークスペース全体の pytest 共通セットアップ。

`pytest-forked` は `pytest.mark.forked` の付いたテストを `os.fork()` した
子プロセスで実行する。`fork()` は呼び出したスレッドだけを複製するため、
親プロセスの他スレッドがロックを保持したまま複製されると、そのロックは
子プロセスで永久に解放されない。

ROS context の初期化 (`rclpy.init()` / `rclpy.context.Context.init()`) は
DDS の内部スレッドを生成し、それらは `rclpy.shutdown()` 後も残留する。
そのため pytest の親プロセスで一度でも ROS context を初期化すると、以降の
forked テストは DDS のロックを保持した状態から複製され、子プロセスが
`rclpy.init()` を呼んだ時点でデッドロックする。テストは失敗せず停止し続け、
CI はタイムアウトで打ち切られる。

この規則を守るため、ROS を実際に起動するテストは次のいずれかを使う。

- `pytest.mark.forked` で子プロセスへ隔離する
  (`src/robot_console/tools/tests/` 配下は conftest.py が一律で付与する)
- `subprocess` で別プロセスを起動する
  (`src/gnss_lio_fusion/tests/` 配下で使用)

どちらでも ROS context は pytest の親プロセスではなく別プロセスで初期化され、
親プロセスは DDS スレッドを持たないまま維持される。

`pytest_runtest_setup()` はこの規則が破れた場合にデッドロックの代わりに
テストを失敗させる。検出のため `Context.init()` を包んで、呼び出しが
起きたプロセスを記録する。fork した子プロセスはこの記録を引き継ぐため、
子プロセス側では「親プロセスが初期化済みだった」ことが判別できる。
"""

from __future__ import annotations

import os
from typing import Any

import pytest

# ROS context を初期化したプロセスの PID。未初期化なら None。
_ros_context_initialized_pid: int | None = None


def _install_ros_context_probe() -> None:
    """`Context.init()` を包み、ROS context の初期化を記録する。

    ROS 環境が有効でない場合は何もしない。その場合 ROS を使うテストは
    `pytest.importorskip('rclpy')` などで skip されるため、検査も不要になる。
    """

    try:
        from rclpy.context import Context
    except ImportError:
        return

    original_init = Context.init

    def init(self: Any, *args: Any, **kwargs: Any) -> Any:
        global _ros_context_initialized_pid
        _ros_context_initialized_pid = os.getpid()
        return original_init(self, *args, **kwargs)

    Context.init = init


_install_ros_context_probe()


def pytest_runtest_setup(item: pytest.Item) -> None:
    """forked テストを、親プロセスが ROS を初期化済みなら失敗させる。

    forked テストではこのフックも子プロセス側で実行される。記録された PID が
    自身の PID と異なる場合、親プロセスで ROS context が初期化されており、
    このまま ROS へ触れるとデッドロックする。
    """

    if item.get_closest_marker('forked') is None:
        return
    if _ros_context_initialized_pid in (None, os.getpid()):
        return
    pytest.fail(
        'pytest の親プロセスで ROS context が初期化済みのため、forked テストは '
        'デッドロックする。ROS を起動するテストは pytest.mark.forked または '
        'subprocess で別プロセスへ隔離すること（conftest.py の docstring 参照）。',
        pytrace=False,
    )
