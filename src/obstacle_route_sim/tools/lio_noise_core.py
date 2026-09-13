"""測距・欠測・IMU 誤差の仮定を再現可能なセンサ入力劣化へ変換する."""
from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class NoiseProfile:
    """標準偏差と定常バイアス標準偏差を SI 単位で保持する."""
    range_sigma: float = 0.
    range_slope: float = 0.
    angle_deg: float = 0.
    dropout: float = 0.
    fade_start: float = 70.
    fade_length: float = 1.
    max_range: float = 70.
    outlier_probability: float = 0.
    accel_sigma: float = 0.
    gyro_sigma: float = 0.
    accel_bias: float = 0.
    gyro_bias: float = 0.
    accel_drift: float = 0.
    gyro_drift: float = 0.
    bias_tau: float = 120.
    scale_error: float = 0.
    imu_offset: float = 0.
    imu_jitter: float = 0.


PROFILES = {
    'reference': NoiseProfile(),
    'field_assumed': NoiseProfile(.02, .0003, .10, .08, 30., 25., 70., .0001,
        .01, .0008, .04, .0004, .005, .0001, 120., .001, .002, .0005),
    'conservative': NoiseProfile(.04, .0006, .15, .20, 25., 15., 55., .001,
        .02, .0016, .08, .001, .015, .0003, 120., .003, .005, .0015),
}


class SensorNoise:
    """LiDAR と IMU の乱数列を分離し、受信数の相互依存を避ける."""

    def __init__(self, profile: str, seed: int) -> None:
        self.profile = PROFILES[profile]
        sequences = np.random.SeedSequence(seed).spawn(2)
        self.lidar_rng, self.imu_rng = [np.random.default_rng(s) for s in sequences]
        p = self.profile
        self.accel_fixed = self.imu_rng.uniform(-p.accel_bias, p.accel_bias, 3)
        self.gyro_fixed = self.imu_rng.uniform(-p.gyro_bias, p.gyro_bias, 3)
        self.scale = self.imu_rng.uniform(-p.scale_error, p.scale_error, (2, 3))
        self.drift = np.zeros((2, 3))
        self.last_imu = None
        self.input_points = 0
        self.output_points = 0
        self.frames = 0

    def metadata(self) -> dict:
        """実現した固定誤差とモデル係数を監査用に返す."""
        return dict(parameters=asdict(self.profile), accel_fixed=self.accel_fixed.tolist(),
                    gyro_fixed=self.gyro_fixed.tolist(), scale=self.scale.tolist(),
                    input_points=self.input_points, output_points=self.output_points,
                    frames=self.frames)

    def cloud(self, values: np.ndarray) -> np.ndarray:
        """距離方向・角度方向の誤差と確率的な未反射を与える."""
        p, rng = self.profile, self.lidar_rng
        self.input_points += len(values)
        self.frames += 1
        if p == PROFILES['reference']:
            self.output_points += len(values)
            return values.copy()
        result = values.copy()
        distance = np.linalg.norm(result[:, :3], axis=1)
        probability = (1-p.dropout)*np.exp(-(np.maximum(distance-p.fade_start, 0)/p.fade_length)**2)
        keep = (distance <= p.max_range) & (rng.random(len(distance)) < probability)
        result, distance = result[keep], distance[keep]
        azimuth = np.arctan2(result[:, 1], result[:, 0])
        elevation = np.arctan2(result[:, 2], np.linalg.norm(result[:, :2], axis=1))
        azimuth += rng.normal(0, np.deg2rad(p.angle_deg), len(distance))
        elevation += rng.normal(0, np.deg2rad(p.angle_deg), len(distance))
        distance += rng.normal(0, p.range_sigma+p.range_slope*np.maximum(distance-10, 0))
        outliers = rng.random(len(distance)) < p.outlier_probability
        distance[outliers] += rng.uniform(-1., 1., outliers.sum())
        result[:, 0] = distance*np.cos(elevation)*np.cos(azimuth)
        result[:, 1] = distance*np.cos(elevation)*np.sin(azimuth)
        result[:, 2] = distance*np.sin(elevation)
        result = result[(distance >= .6) & (distance <= p.max_range)]
        self.output_points += len(result)
        return result

    def imu(self, accel: np.ndarray, gyro: np.ndarray, stamp: float) -> tuple:
        """連続時間 OU バイアスを離散化し、雑音と scale 誤差を重ねる."""
        p, rng = self.profile, self.imu_rng
        dt = 0. if self.last_imu is None else max(0., stamp-self.last_imu)
        self.last_imu = stamp
        alpha = np.exp(-dt/p.bias_tau)
        sigma = np.array([p.accel_drift, p.gyro_drift])[:, None]
        self.drift = alpha*self.drift+sigma*np.sqrt(1-alpha*alpha)*rng.normal(size=(2, 3))
        a = accel*(1+self.scale[0])+self.accel_fixed+self.drift[0]+rng.normal(0,p.accel_sigma,3)
        g = gyro*(1+self.scale[1])+self.gyro_fixed+self.drift[1]+rng.normal(0,p.gyro_sigma,3)
        offset = p.imu_offset+rng.uniform(-p.imu_jitter, p.imu_jitter)
        return a, g, offset
