"""位置の外れ値から独立した円周上の方位観測更新."""
import math


def angle_difference(angle: float) -> float:
    """±πをまたぐ差を最短角へ変換する."""
    return math.atan2(math.sin(angle), math.cos(angle))


class HeadingObserver:
    """LIOで予測した方位へ、独立に品質判定したGNSS方位を反映する."""

    def __init__(self) -> None:
        self.previous = None
        self.accepted = 0
        self.rejected = 0
        self.last_accepted = None
        self.reason = 'initializing'
        self.recovery_since = None
        self.recovery_previous = None

    def update(self, stamp: float, yaw: float, variance: float, measured: float,
               observation_variance: float, dt: float, rate_limit: float,
               max_motion_rate: float, usable: bool,
               recover_lio_fault: bool = False) -> tuple[float, float]:
        """急な観測反転を除外し、補正速度を制限したscalar Kalman更新を行う."""
        if not usable:
            self.recovery_since = None
            self.rejected += 1
            self.reason = 'quality_rejected'
            return yaw, variance
        large_residual = abs(angle_difference(measured-yaw)) > math.pi/4
        if large_residual:
            previous = self.recovery_previous
            self.recovery_previous = (stamp, measured)
            consistent = (previous is not None and 0 < stamp-previous[0] <= .5
                          and abs(angle_difference(measured-previous[1])) <
                          max_motion_rate*(stamp-previous[0])+.1)
            if not recover_lio_fault or not consistent or self.recovery_since is None:
                self.recovery_since = stamp
            stable = (recover_lio_fault and consistent and self.recovery_since is not None
                      and stamp-self.recovery_since >= 2.)
            if not stable:
                self.rejected += 1
                self.reason = 'innovation_rejected'
                return yaw, variance
        old = self.previous
        # 棄却値を次の基準にしない。継続する新方位は1秒後から再取得できる。
        if old is not None and 0 < stamp-old[0] < 1.:
            change = abs(angle_difference(measured-old[1]))
            allowance = max_motion_rate*(stamp-old[0])+4*math.sqrt(
                observation_variance+old[2])
            if change > allowance:
                self.rejected += 1
                self.reason = 'angular_jump'
                return yaw, variance
        self.previous = (stamp, measured, observation_variance)
        residual = angle_difference(measured-yaw)
        gain = variance/(variance+observation_variance)
        gain *= min(1., rate_limit*dt/max(abs(gain*residual), 1e-12))
        self.accepted += 1
        self.last_accepted = stamp
        self.reason = 'recovering_lio_fault' if large_residual else 'tracking'
        return (angle_difference(yaw+gain*residual),
                (1-gain)**2*variance+gain**2*observation_variance)
