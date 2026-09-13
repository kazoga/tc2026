"""制御入力の受信間隔を ROS 非依存で監視する."""

import math
from typing import Mapping


class InputWatchdog:
    """単調時計による受信時刻を保持し、未受信・期限切れを判定する."""

    def __init__(self, timeouts: Mapping[str, float]) -> None:
        self.timeouts = dict(timeouts)
        if not self.timeouts or any(
            not math.isfinite(value) or value <= 0 for value in self.timeouts.values()
        ):
            raise ValueError('入力タイムアウトは正の有限秒数で指定してください')
        self.last_received: dict[str, float] = {}

    def receive(self, name: str, now: float) -> None:
        """入力の受信時刻を記録する."""
        if name not in self.timeouts or not math.isfinite(now):
            raise ValueError('入力名または受信時刻が不正です')
        self.last_received[name] = now

    def stale_inputs(self, now: float) -> tuple[str, ...]:
        """未受信・期限切れ・時計逆行がある入力名を返す."""
        return tuple(
            name for name, timeout in self.timeouts.items()
            if not math.isfinite(now) or name not in self.last_received
            or not 0 <= now - self.last_received[name] < timeout
        )
