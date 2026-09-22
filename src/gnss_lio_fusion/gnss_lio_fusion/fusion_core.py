"""相対LIOを予測、GNSS位置・方位を観測とする平面EKF."""
from dataclasses import dataclass
import math

import numpy as np

from .baseline_core import AdaptiveBaseline, BaselineConfig
from .heading_core import HeadingObserver


def wrap(angle: float) -> float:
    """角度差を±πに正規化する."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class FusionConfig:
    """共分散の下限・補正速度は初期の試験用設定である."""
    position_sigma_fix: float = .15
    position_sigma_float: float = 3.
    heading_sigma_fix_deg: float = 1.
    heading_sigma_float_deg: float = 15.
    lio_position_sigma_per_sqrt_m: float = .12
    lio_yaw_sigma_per_sqrt_s: float = .025
    max_lio_speed_mps: float = 2.
    max_lio_yaw_rate: float = 2.
    max_lio_gap_s: float = 1.5
    gps_timeout_s: float = 1.5
    correction_speed_mps: float = .5
    correction_yaw_rate: float = .12
    recovery_s: float = 2.


class FusionFilter:
    """GPSの品質低下では停止せず、LIOの相対変位による予測を継続する."""

    def __init__(self, config: FusionConfig | None = None,
                 baseline: BaselineConfig | None = None) -> None:
        self.config = config or FusionConfig()
        if any(not math.isfinite(v) or v <= 0 for v in vars(self.config).values()):
            raise ValueError('融合パラメータは正の有限値が必要')
        self.baseline = AdaptiveBaseline(baseline)
        self.heading = HeadingObserver()
        self.heading_recovery = False
        self.x = None
        self.p = np.diag([1., 1., .1])
        self.stamp = None
        self.raw = None
        self.last_gps = None
        self.last_good_gps = None
        self.good_since = None
        self.last_observation = None
        self.gps_quality = 0.
        self.lio_ok = True
        self.rejected_lio = 0
        self.rejected_gps = 0
        self.accepted_gps = 0
        self.baseline_status = self.baseline.status(.5, 0., 'learning')

    def advance(self, stamp: float, raw_pose: np.ndarray) -> bool:
        """LIO座標系の絶対位置ではなく、前回姿勢に対する相対変位を伝播する."""
        pose = np.asarray(raw_pose, dtype=float)
        if pose.shape != (3,) or not np.isfinite(pose).all() or not math.isfinite(stamp):
            self.lio_ok = False
            self.rejected_lio += 1
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False
        previous, old_stamp = self.raw, self.stamp
        self.raw, self.stamp = pose.copy(), stamp
        if previous is None or old_stamp is None or self.x is None:
            return True
        dt = stamp-old_stamp
        delta = pose[:2]-previous[:2]
        angle = wrap(pose[2]-previous[2])
        c = self.config
        if (dt > c.max_lio_gap_s or np.linalg.norm(delta) > c.max_lio_speed_mps*dt+.15
                or abs(angle) > c.max_lio_yaw_rate*dt+.15):
            # 異常ジャンプは積分せず、新しいLIO原点からの相対運動を次回に使う。
            self.lio_ok = False
            self.rejected_lio += 1
            self.heading_recovery = True
            self.p += np.diag([.25, .25, max(.05, angle**2)])*min(dt, 5.)
            return False
        self.lio_ok = True
        a = self.x[2]-previous[2]
        rotation = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        step = rotation @ delta
        jacobian = np.eye(3)
        jacobian[0, 2], jacobian[1, 2] = -step[1], step[0]
        self.x[:2] += step
        self.x[2] = wrap(self.x[2]+angle)
        variance = c.lio_position_sigma_per_sqrt_m**2*(np.linalg.norm(step)+.02*dt)
        self.p = jacobian @ self.p @ jacobian.T+np.diag([
            variance, variance, c.lio_yaw_sigma_per_sqrt_s**2*dt])
        return True

    def observe_gps(self, stamp: float, pose: np.ndarray, state: int, satellites: int,
                    baseline_m: float, position_variance: float,
                    heading_stddev_deg: float) -> bool:
        """品質を測定から判定し、GPS回復時も補正速度を制限する."""
        c = self.config
        z = np.asarray(pose, dtype=float)
        if (z.shape != (3,) or not np.isfinite(z).all() or not math.isfinite(stamp)
                or (self.last_gps is not None and stamp <= self.last_gps)):
            self.rejected_gps += 1
            return False
        # 長い途絶期間を一回分の補正許容量に加算しない。
        dt = min(.1, stamp-self.last_gps) if self.last_gps is not None else .1
        if self.last_gps is not None and stamp-self.last_gps > c.gps_timeout_s:
            self.good_since = None
        self.last_gps = stamp
        b = self.baseline.observe(stamp, baseline_m, state == 4, satellites)
        self.baseline_status = b
        satellites_score = float(np.clip((satellites-6)/8., 0., 1.))
        state_score = 1. if state == 4 else .03 if state == 3 else 0.
        quality = state_score*satellites_score*(.25+.75*b['score'])
        self.gps_quality = quality
        good = state == 4 and satellites >= self.baseline.config.min_satellites and b['score'] > .5
        if good:
            self.good_since = stamp if self.good_since is None else self.good_since
        else:
            self.good_since = None
        if self.x is None:
            if (state != 4 or satellites < 8 or b['score'] < .05
                    or not math.isfinite(heading_stddev_deg) or not 0 < heading_stddev_deg <= 15.):
                return False
            self.x = z.copy()
            self.p = np.diag([1., 1., math.radians(10)**2])
            self.accepted_gps += 1
            if good:
                self.last_good_gps = stamp
            self.last_observation = (stamp, z.copy())
            return True
        fix = state == 4
        sigma = c.position_sigma_fix if fix else c.position_sigma_float
        measured = position_variance if math.isfinite(position_variance) and position_variance > 0 else sigma**2
        position_var = max(sigma**2, measured)/max(.05, satellites_score*(.25+.75*b['score']))
        heading_sigma = c.heading_sigma_fix_deg if fix else c.heading_sigma_float_deg
        if math.isfinite(heading_stddev_deg) and heading_stddev_deg > 0:
            heading_sigma = max(heading_sigma, heading_stddev_deg)
        else:
            heading_sigma = 180.
        heading_var = math.radians(heading_sigma)**2/max(.01, satellites_score*b['score'])
        heading_usable = (state in (3, 4) and math.isfinite(heading_stddev_deg)
                          and 0 < heading_stddev_deg <= 30.
                          and satellites >= 8 and b['score'] >= .05)
        self.x[2], self.p[2, 2] = self.heading.update(
            stamp, self.x[2], self.p[2, 2], z[2], heading_var, dt,
            c.correction_yaw_rate, c.max_lio_yaw_rate, heading_usable,
            recover_lio_fault=self.heading_recovery and good)
        if abs(wrap(z[2]-self.x[2])) < math.radians(5) and good:
            self.heading_recovery = False
        if quality < .001:
            self.rejected_gps += 1
            return False
        # 良好FIXの突然の飛びは除外する。安定した新位置は再取得候補として待つ。
        if self.last_observation is not None:
            old_t, old_z = self.last_observation
            if stamp-old_t < .5 and np.linalg.norm(z[:2]-old_z[:2]) > 2.+2.*(stamp-old_t):
                self.good_since = None
                self.last_observation = (stamp, z.copy())
                self.rejected_gps += 1
                return False
        self.last_observation = (stamp, z.copy())
        # GPSの位置biasを方位として説明させない。位置と方位を独立に更新する。
        self.p[:2, 2] = 0.
        self.p[2, :2] = 0.
        measurement = np.eye(2)*position_var
        residual = z[:2]-self.x[:2]
        covariance = self.p[:2, :2]
        innovation = covariance+measurement
        nis = float(residual @ np.linalg.solve(innovation, residual))
        recovered = good and self.good_since is not None and stamp-self.good_since >= c.recovery_s
        if nis > 25. and not recovered:
            if fix:
                self.rejected_gps += 1
                return False
            # FLOATは弱い絶対位置の拘束として残す。大きい残差は重みをさらに下げる。
            measurement *= nis/25.
            innovation = covariance+measurement
        gain = np.linalg.solve(innovation.T, covariance.T).T
        update = gain @ residual
        gain *= min(1., c.correction_speed_mps*dt/max(1e-12, np.linalg.norm(update)))
        self.x[:2] += gain @ residual
        identity = np.eye(2)-gain
        self.p[:2, :2] = identity @ covariance @ identity.T+gain @ measurement @ gain.T
        self.p = (self.p+self.p.T)/2
        self.accepted_gps += 1
        if good:
            self.last_good_gps = stamp
        return True

    def diagnostics(self, stamp: float) -> dict:
        """不確かさが高くても予測を続け、品質に応じた減速値を返す."""
        recent = self.last_good_gps is not None and stamp-self.last_good_gps < self.config.gps_timeout_s
        mode = 'GPS_LIO' if recent and self.lio_ok else 'LIO_PRIORITY' if self.lio_ok else 'LIO_FAULT'
        sigma = float(np.sqrt(max(np.linalg.eigvalsh(self.p[:2, :2]))))
        return dict(mode=mode, gps_quality=self.gps_quality, position_sigma_m=sigma,
                    heading_sigma_deg=math.degrees(math.sqrt(self.p[2, 2])),
                    heading_accepted=self.heading.accepted, heading_rejected=self.heading.rejected,
                    heading_reason=self.heading.reason,
                    speed_limit_mps=(1.1 if recent and sigma < .5 and self.p[2, 2] < math.radians(5)**2
                                     else .6 if sigma < 1. else .25),
                    baseline=self.baseline_status, accepted_gps=self.accepted_gps,
                    rejected_gps=self.rejected_gps, rejected_lio=self.rejected_lio)
