"""robot_console で使用するデータモデル定義。

画面表示用のViewは `core/snapshot_model.py` が持つ。本モジュールは、UI実装に
依存しない基盤的なデータ構造だけを置く。
"""

from __future__ import annotations

from collections import deque
from enum import Enum, auto
from typing import Deque, List
import threading


class NodeLaunchStatus(Enum):
    """ノード起動管理の状態を表す列挙型。"""

    STOPPED = auto()
    STARTING = auto()
    RUNNING = auto()
    STOPPING = auto()
    ERROR = auto()


class ConsoleLogBuffer:
    """リングバッファ方式でログを保持する。"""

    def __init__(self, capacity: int = 2000) -> None:
        self._capacity = capacity
        self._lines: Deque[str] = deque(maxlen=capacity)
        self._lock = threading.Lock()

    @property
    def capacity(self) -> int:
        """最大行数を返す。"""

        return self._capacity

    def append(self, line: str) -> None:
        """ログ行を追加する。"""

        with self._lock:
            self._lines.append(line)

    def snapshot(self) -> List[str]:
        """現在のログをリストで取得する。"""

        with self._lock:
            return list(self._lines)

    def clear(self) -> None:
        """ログをクリアする。"""

        with self._lock:
            self._lines.clear()
