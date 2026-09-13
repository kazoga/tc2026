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


def traversable_width(points: np.ndarray, cell: float = .1, max_width: float = 3.,
                      step: float = .06, margin: float = .275) -> dict:
    """base座標の地面を前後0.6 mの格子で照合する。未観測セルで拡張を止める.

    中央接地面を基準とし、高低差6 cm、隣接勾配15度、粗さ6 cmを拒否する。
    車体半幅と余裕を引いた横移動量を返す。点群なしは幅0・unknownとする。
    """
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    p = p[np.isfinite(p).all(axis=1)]
    p = p[(abs(p[:, 0]) < .6) & (abs(p[:, 1]) <= max_width)]
    center = p[(abs(p[:, 1]) < .25) & (abs(p[:, 2]) < .2)]
    result = {'left': 0., 'right': 0., 'left_reason': 'unknown',
              'right_reason': 'unknown', 'review_required': True}
    if len(center) < 12:
        return result
    ground = float(np.median(center[:, 2]))
    noise = float(np.median(abs(center[:, 2]-ground))/.6745)
    if noise > .05:
        return result
    result['ground_noise_sigma_m'] = noise
    nx = 6
    for side, sign in [('left', 1), ('right', -1)]:
        previous = ground
        extent = 0.
        reason = 'range_limit'
        for band in range(int(max_width/cell)):
            strip = p[(sign*p[:, 1] >= band*cell) & (sign*p[:, 1] < (band+1)*cell)]
            heights = []
            errors = []
            for i in range(nx):
                column = strip[(strip[:, 0] >= -.6+i*.2) & (strip[:, 0] < -.4+i*.2)]
                if len(column) < 3:
                    reason = 'unknown'
                    break
                low, high = np.quantile(column[:, 2], [.1, .9])
                if high-low > step+6*noise:
                    reason = 'rough_or_obstacle'
                    break
                heights.append(float(np.median(column[:, 2])))
                # 中央値の99%近似区間を段差閾値から差し引く。閾値自体は広げない。
                errors.append(3.23*noise*math.sqrt(1/len(column)+1/len(center)))
            if len(heights) != nx:
                break
            level = float(np.median(heights))
            if max(abs(np.array(heights)-ground)+np.array(errors)) >= step:
                reason = 'curb_or_drop'
                break
            if abs(level-previous) > cell*math.tan(math.radians(15)):
                reason = 'slope'
                break
            previous = level
            extent = (band+1)*cell
        result[side] = max(0., extent-margin)
        result[side+'_reason'] = reason
    return result


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
        if start in rising and not self.active:
            self.active = True
            self.add(pose, width, 'start')
        if self.active:
            if stop in rising:
                self.add(pose, width, 'line_stop')
            if signal in rising:
                self.add(pose, width, 'signal_stop')
            if finish in rising:
                self.add(pose, width, 'finish')
                self.active = False
