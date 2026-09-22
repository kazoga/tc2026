"""手動走行経路の採取と、観測された地面だけに基づく回避幅推定."""
from dataclasses import dataclass
import math

import numpy as np


def angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass
class Pose:
    stamp: float
    x: float
    y: float
    yaw: float


def traversable_width(points: np.ndarray, cell: float = .2, max_width: float = 3.,
                      step: float = .06, margin: float = .35, **kwargs) -> dict:
    """局所平面・粗さ・反射強度で連続した回避可能幅を求める。"""
    from .surface_core import surface_width
    return surface_width(points, cell, max_width, step, margin, **kwargs)


class Survey:
    """距離は移動距離で測り、ボタンは立上りだけを受け付ける."""
    def __init__(self, spacing: float = 5., turn_deg: float = 25.) -> None:
        if spacing <= 0 or not 0 < turn_deg < 180:
            raise ValueError('spacing/turn_degが範囲外')
        self.spacing = spacing
        self.turn = math.radians(turn_deg)
        self.active = False
        self.rows = []
        self.last = None
        self.distance = 0.
        self.distance_anchor = None
        self.buttons = set()
        self.rejections = 0
        self.gap = False

    def add(self, pose: Pose, width: dict, reason: str) -> None:
        row = dict(x=pose.x, y=pose.y, yaw=pose.yaw, stamp=pose.stamp,
                   right_is_open=width['right'], left_is_open=width['left'],
                   line_is_stop=0, signal_is_stop=0, isnot_skipnum=1,
                   reason=reason, observation=dict(width))
        if self.rows and math.hypot(pose.x-self.rows[-1]['x'], pose.y-self.rows[-1]['y']) < .3:
            row = self.rows[-1]
        else:
            self.rows.append(row)
        if reason == 'line_stop':
            row['line_is_stop'] = 1
        if reason == 'signal_stop':
            row['signal_is_stop'] = 1
        self.distance = 0.
        self.distance_anchor = pose

    def update(self, pose: Pose, width: dict) -> bool:
        if not all(math.isfinite(v) for v in [pose.stamp, pose.x, pose.y, pose.yaw]):
            self.rejections += 1
            return False
        if self.last:
            dt = pose.stamp-self.last.stamp
            ds = math.hypot(pose.x-self.last.x, pose.y-self.last.y)
            if dt <= 0 or dt > 1. or ds > 2.*dt+.1:
                self.rejections += 1
                # 欠落後の再開は距離を加算しない。
                if dt > 1.:
                    self.last = pose
                    self.distance_anchor = pose
                    self.gap = True
                return False
            if self.active and self.distance_anchor is not None:
                travel = math.hypot(pose.x-self.distance_anchor.x, pose.y-self.distance_anchor.y)
                if travel >= .2:
                    self.distance += travel
                    self.distance_anchor = pose
        self.last = pose
        if self.active:
            if self.gap:
                self.add(pose, traversable_width(np.empty((0, 3))), 'input_gap')
                self.gap = False
            elif not self.rows:
                self.add(pose, width, 'start')
            elif self.distance >= self.spacing:
                self.add(pose, width, 'distance')
            elif (self.distance >= 1. and
                  abs(angle(pose.yaw-self.rows[-1]['yaw'])) >= self.turn):
                self.add(pose, width, 'turn')
        return True

    def joy(self, pressed: set[int], pose: Pose, width: dict,
            start: int = 0, stop: int = 1, signal: int = 2, finish: int = 3) -> None:
        rising = pressed-self.buttons
        self.buttons = pressed
        for button, command in [(start, 'start'), (stop, 'line_stop'),
                                (signal, 'signal_stop'), (finish, 'finish')]:
            if button in rising:
                self.command(command, pose, width)

    def command(self, command: str, pose: Pose | None, width: dict) -> None:
        if command == 'start' and not self.active and pose is not None:
            self.active = True
            self.add(pose, width, 'start')
        elif self.active and command in ('line_stop', 'signal_stop', 'finish'):
            if pose is not None:
                self.add(pose, width, command)
            if command == 'finish':
                self.active = False
