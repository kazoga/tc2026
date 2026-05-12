"""PositionData → ROS msg 変換 (純関数中心)。"""

import math
from typing import Tuple

from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Header

from rtk_gps_um982_msgs.msg import RtkStatus

_STATE_TO_ENUM = {
    'unknown': RtkStatus.STATE_UNKNOWN,
    'standalone': RtkStatus.STATE_STANDALONE,
    'dgps': RtkStatus.STATE_DGPS,
    'rtk_float': RtkStatus.STATE_RTK_FLOAT,
    'rtk_fix': RtkStatus.STATE_RTK_FIX,
}

_MIN_FIX_TO_ENUM = {
    'none': RtkStatus.STATE_UNKNOWN,
    'standalone': RtkStatus.STATE_STANDALONE,
    'dgps': RtkStatus.STATE_DGPS,
    'float': RtkStatus.STATE_RTK_FLOAT,
    'fix': RtkStatus.STATE_RTK_FIX,
}


def passes_fix_filter(rtk_state: str, min_fix: str) -> bool:
    state_enum = _STATE_TO_ENUM.get(rtk_state, RtkStatus.STATE_UNKNOWN)
    min_enum = _MIN_FIX_TO_ENUM.get(min_fix, RtkStatus.STATE_STANDALONE)
    return state_enum >= min_enum


def heading_pitch_to_enu_quaternion(
    heading_deg: float, pitch_deg: float
) -> Tuple[float, float, float, float]:
    """UM982 の真北 CW heading を REP-103 ENU の quaternion (w,x,y,z) に変換。

    yaw_enu = pi/2 - heading_rad  (heading=0(真北) → yaw_enu=pi/2)
    roll = 0 と仮定。pitch はそのまま渡す。
    """
    heading_rad = math.radians(heading_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_enu = (math.pi / 2.0) - heading_rad

    cy, sy = math.cos(yaw_enu * 0.5), math.sin(yaw_enu * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    cr, sr = 1.0, 0.0

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def make_stamp(stamp_source: str, gnss_unix: float, transport_delay_ms: int, clock) -> Time:
    if stamp_source == 'gnss_utc':
        adjusted = float(gnss_unix) + (float(transport_delay_ms) / 1000.0)
        sec = int(adjusted)
        nsec = int(round((adjusted - sec) * 1e9))
        return Time(seconds=sec, nanoseconds=nsec)
    # ros_time / pps_edge (Phase 2) はどちらも system clock fallback
    return clock.now()


def _make_header(stamp: Time, frame_id: str) -> Header:
    h = Header()
    h.stamp = stamp.to_msg()
    h.frame_id = frame_id
    return h


def _rtk_state_to_navsat_status(state: str) -> int:
    if state in ('rtk_fix', 'rtk_float'):
        return NavSatStatus.STATUS_GBAS_FIX
    if state == 'dgps':
        return NavSatStatus.STATUS_SBAS_FIX
    if state == 'standalone':
        return NavSatStatus.STATUS_FIX
    return NavSatStatus.STATUS_NO_FIX


def position_to_navsatfix(pos, stamp: Time, frame_id: str, hdop_sigma: float = 1.0) -> NavSatFix:
    msg = NavSatFix()
    msg.header = _make_header(stamp, frame_id)
    msg.status.status = _rtk_state_to_navsat_status(pos.rtk_state)
    msg.status.service = (
        NavSatStatus.SERVICE_GPS
        | NavSatStatus.SERVICE_GLONASS
        | NavSatStatus.SERVICE_GALILEO
        | NavSatStatus.SERVICE_COMPASS
    )
    msg.latitude = float(pos.lat) if pos.lat is not None else 0.0
    msg.longitude = float(pos.lon) if pos.lon is not None else 0.0
    msg.altitude = float(pos.alt) if pos.alt is not None else 0.0

    hdop = float(pos.hdop) if pos.hdop is not None else 99.0
    sigma_h = (hdop * hdop_sigma) ** 2
    sigma_v = sigma_h * 4.0  # 垂直は水平の概ね 2σ → 分散 4 倍
    msg.position_covariance = [
        sigma_h, 0.0, 0.0,
        0.0, sigma_h, 0.0,
        0.0, 0.0, sigma_v,
    ]
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


def position_to_imu(pos, stamp: Time, frame_id: str) -> Imu:
    msg = Imu()
    msg.header = _make_header(stamp, frame_id)

    if pos.heading is None:
        msg.orientation_covariance = [-1.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0]
        msg.angular_velocity_covariance = [-1.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0]
        msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0]
        return msg

    pitch_deg = float(pos.pitch) if pos.pitch is not None else 0.0
    w, x, y, z = heading_pitch_to_enu_quaternion(float(pos.heading), pitch_deg)
    msg.orientation.w = w
    msg.orientation.x = x
    msg.orientation.y = y
    msg.orientation.z = z

    heading_var = (math.radians(pos.heading_stddev) ** 2) if pos.heading_stddev else 1e-2
    pitch_var = (math.radians(pos.pitch_stddev) ** 2) if pos.pitch_stddev else 1e-2
    msg.orientation_covariance = [
        1e6, 0.0, 0.0,           # roll: 未計測
        0.0, pitch_var, 0.0,
        0.0, 0.0, heading_var,   # yaw
    ]
    # 角速度・加速度は未計測
    msg.angular_velocity_covariance = [-1.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0]
    msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0]
    return msg


def position_to_rtk_status(pos, stamp: Time, frame_id: str, rtcm_bytes: int) -> RtkStatus:
    msg = RtkStatus()
    msg.header = _make_header(stamp, frame_id)
    msg.rtk_state = _STATE_TO_ENUM.get(pos.rtk_state, RtkStatus.STATE_UNKNOWN)
    msg.rtk_state_raw = pos.rtk_state or ''
    msg.num_satellites = int(pos.num_sats) if pos.num_sats is not None else 0
    msg.hdop = float(pos.hdop) if pos.hdop is not None else 0.0
    msg.heading_deg = float(pos.heading) if pos.heading is not None else 0.0
    msg.heading_stddev_deg = float(pos.heading_stddev) if pos.heading_stddev is not None else 0.0
    msg.pitch_deg = float(pos.pitch) if pos.pitch is not None else 0.0
    msg.pitch_stddev_deg = float(pos.pitch_stddev) if pos.pitch_stddev is not None else 0.0
    msg.baseline_length_m = float(pos.baseline_m) if pos.baseline_m is not None else 0.0
    msg.correction_age_s = float(pos.diff_age) if pos.diff_age is not None else -1.0
    msg.rtcm_bytes_received = int(rtcm_bytes)
    msg.latitude = float(pos.lat) if pos.lat is not None else 0.0
    msg.longitude = float(pos.lon) if pos.lon is not None else 0.0
    msg.altitude = float(pos.alt) if pos.alt is not None else 0.0
    return msg
