"""車体とセンサの剛体変換。取付角とLiDAR内部較正を混同しない。"""
import math
import numpy as np


def rotation(roll: float, pitch: float, yaw: float = 0.) -> np.ndarray:
    """Rz(yaw) Ry(pitch) Rx(roll) を返す。正pitchは前方を下げる。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def quaternion_rotation(q) -> np.ndarray:
    """ROS順(x,y,z,w)を正規化して回転行列へ変換する。"""
    a = np.asarray(q, dtype=float)
    norm = np.linalg.norm(a)
    if a.shape != (4,) or not np.isfinite(a).all() or not .5 <= norm <= 1.5:
        raise ValueError('姿勢quaternionが不正')
    x, y, z, w = a/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def rpy(matrix: np.ndarray) -> tuple[float, float, float]:
    """回転行列から車体のroll/pitch/yawを得る。"""
    return (math.atan2(matrix[2, 1], matrix[2, 2]),
            math.asin(float(np.clip(-matrix[2, 0], -1., 1.))),
            math.atan2(matrix[1, 0], matrix[0, 0]))


def base_from_sensor(position, quaternion, translation, mount_rpy):
    """world→IMU姿勢からworld→base姿勢と位置を求める。"""
    world_base = quaternion_rotation(quaternion) @ rotation(*mount_rpy).T
    return np.asarray(position)-world_base @ np.asarray(translation), rpy(world_base)


def horizontal_lever(translation, roll: float, pitch: float, yaw: float) -> np.ndarray:
    """車体固定のレバーアームを世界座標で表す。"""
    return rotation(roll, pitch, yaw) @ np.asarray(translation)
