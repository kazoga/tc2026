"""静止区間だけで初期worldの鉛直を決め、走行中は固定する。"""
from collections import deque
import numpy as np
from .mount_core import quaternion_rotation


def level_rotation(up):
    """最小回転でworldの上方向を+Zへ合わせる（方位はGNSSで決める）。"""
    up = np.asarray(up, dtype=float)
    if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < .1:
        raise ValueError('重力方向が不正')
    up = up / np.linalg.norm(up)
    v = np.cross(up, [0., 0., 1.])
    c = up[2]
    if c < -.999:
        raise ValueError('逆さまの姿勢では初期化しない')
    k = np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])
    return np.eye(3) + k + k @ k / (1. + c)


class GravityAlignment:
    def __init__(self, duration=2.):
        if not np.isfinite(duration) or duration < 1.:
            raise ValueError('静止確認時間は1秒以上')
        self.duration = duration
        self.reset()

    def reset(self):
        self.matrix = None
        self.samples = deque()
        self.last_stamp = None
        self.frame = None
        self.reason = '静止状態のIMU・LIO・車輪データを待っています'

    def observe(self, stamp, frame, position, quaternion, acceleration, gyro, wheel):
        if self.last_stamp is not None and (stamp <= self.last_stamp or stamp-self.last_stamp > 1.5 or frame != self.frame):
            self.reset()
        self.last_stamp, self.frame = stamp, frame
        if self.matrix is not None:
            return self.matrix
        p, a, g, w = map(lambda x: np.asarray(x, dtype=float), (position, acceleration, gyro, wheel))
        if not np.isfinite(np.concatenate([p, a, g, w])).all():
            self.samples.clear()
            return None
        norm = np.linalg.norm(a)
        # MID360実機はg単位、シミュレータはm/s²。方向だけを使う。
        valid_acc = .85 <= norm <= 1.15 or 8.3 <= norm <= 11.3
        if not valid_acc or np.linalg.norm(g) > .03 or np.max(abs(w)) > .02:
            self.samples.clear()
            self.reason = '動きを検出：停止して水平基準を確定してください'
            return None
        up = quaternion_rotation(quaternion) @ (a / norm)
        if self.samples and np.linalg.norm(p-self.samples[0][1]) > .03:
            self.samples.clear()
        self.samples.append((stamp, p.copy(), up))
        while len(self.samples) > 1 and stamp-self.samples[1][0] >= self.duration:
            self.samples.popleft()
        self.reason = '静止確認中（約2秒）'
        if len(self.samples) >= 10 and stamp-self.samples[0][0] >= self.duration:
            ups = np.array([s[2] for s in self.samples])
            mean = ups.mean(axis=0)
            deviations = np.linalg.norm(ups-mean, axis=1)
            # Judge the window dispersion, not a single normal IMU noise spike.
            if np.sqrt(np.mean(deviations**2)) > .02 or np.max(deviations) > .06:
                self.samples.clear()
                return None
            self.matrix = level_rotation(mean)
            self.reason = '水平基準確定（走行中は固定）'
        return self.matrix
