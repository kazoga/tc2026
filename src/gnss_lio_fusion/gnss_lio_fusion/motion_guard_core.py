"""LIOと車輪の相対運動を照合し、異常時だけ車輪で連続した予測を保つ."""
from collections import deque
import math

import numpy as np

from .heading_core import angle_difference


class MotionGuard:
    """絶対位置を比較せず、各odom座標系内の相対変位を共通の連続座標へ積分する."""

    def __init__(self) -> None:
        self.lio = None
        self.wheel = None
        self.pose = None
        self.stamp = None
        self.hold_until = 0.
        self.source = 'initializing'
        self.fallback_count = 0
        self.rejected_count = 0
        self.pairs = deque()
        self.window_rejections = 0

    @staticmethod
    def relative(previous, stamp: float, pose) -> np.ndarray | None:
        """観測間隔と物理上限を超える変位を予測に混入させない."""
        if previous is None or pose is None:
            return None
        dt = stamp-previous[0]
        delta = np.asarray(pose)-previous[1]
        delta[2] = angle_difference(delta[2])
        if (not 0 < dt <= 1.5 or not np.isfinite(delta).all()
                or np.linalg.norm(delta[:2]) > 2.*dt+.15 or abs(delta[2]) > 2.*dt+.15):
            return None
        angle = previous[1][2]
        c, s = math.cos(angle), math.sin(angle)
        return np.array([c*delta[0]+s*delta[1], -s*delta[0]+c*delta[1], delta[2]])

    def select(self, stamp: float, lio, wheel) -> np.ndarray | None:
        """真値を使わず選択する。再採用時にも絶対odom原点へ飛ばさない."""
        if not math.isfinite(stamp) or (self.stamp is not None and stamp <= self.stamp):
            return None
        step_lio = (self.relative(self.lio, stamp, lio) if self.lio is not None
                    and self.lio[0] == self.stamp else None)
        step_wheel = (self.relative(self.wheel, stamp, wheel) if self.wheel is not None
                      and self.wheel[0] == self.stamp else None)
        if lio is not None and np.isfinite(lio).all():
            self.lio = (stamp, np.array(lio, dtype=float))
        if wheel is not None and np.isfinite(wheel).all():
            self.wheel = (stamp, np.array(wheel, dtype=float))
        window_disagreement = False
        if lio is not None and wheel is not None and np.isfinite([lio, wheel]).all():
            self.pairs.append((stamp, np.array(lio), np.array(wheel)))
            while self.pairs and stamp-self.pairs[0][0] > 1.5:
                self.pairs.popleft()
            candidates = [pair for pair in self.pairs if stamp-pair[0] >= 1.]
            if candidates:
                old = candidates[-1]
                duration = stamp-old[0]
                window_lio = self.relative((old[0], old[1]), stamp, lio)
                window_wheel = self.relative((old[0], old[2]), stamp, wheel)
                if window_lio is not None and window_wheel is not None:
                    window_disagreement = (
                        np.linalg.norm(window_lio[:2]-window_wheel[:2]) > .12+.18*duration
                        or abs(angle_difference(window_lio[2]-window_wheel[2])) > .1+.1*duration)
        self.window_rejections += int(window_disagreement)
        if self.pose is None:
            first = lio if lio is not None else wheel
            if first is None or not np.isfinite(first).all():
                return None
            self.pose = np.array(first, dtype=float)
            self.stamp = stamp
            return self.pose.copy()
        dt = stamp-self.stamp
        disagreement = (step_lio is not None and step_wheel is not None and
                        (np.linalg.norm(step_lio[:2]-step_wheel[:2]) > .08+.6*dt or
                         abs(angle_difference(step_lio[2]-step_wheel[2])) > .08+.3*dt))
        if step_lio is None or disagreement or window_disagreement:
            self.hold_until = stamp+1.
        use_wheel = step_wheel is not None and (step_lio is None or stamp < self.hold_until)
        step = step_wheel if use_wheel else step_lio
        if step is None:
            self.rejected_count += 1
            self.source = 'NO_MOTION'
            return None
        self.source = 'WHEEL_FALLBACK' if use_wheel else 'LIO'
        self.fallback_count += int(use_wheel)
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose[:2] += [c*step[0]-s*step[1], s*step[0]+c*step[1]]
        self.pose[2] = angle_difference(self.pose[2]+step[2])
        self.stamp = stamp
        return self.pose.copy()
