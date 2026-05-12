"""converters.py の純関数テスト。"""

import math
from dataclasses import dataclass
from typing import Optional

import pytest

from rtk_gps_um982 import converters
from rtk_gps_um982_msgs.msg import RtkStatus


@dataclass
class FakePos:
    lat: Optional[float] = 35.0
    lon: Optional[float] = 139.0
    alt: Optional[float] = 10.0
    heading: Optional[float] = 0.0
    pitch: Optional[float] = 0.0
    speed_knots: Optional[float] = 0.0
    course: Optional[float] = 0.0
    rtk_state: str = 'rtk_fix'
    num_sats: Optional[int] = 12
    hdop: Optional[float] = 0.8
    baseline_m: Optional[float] = 1.0
    timestamp: float = 1_715_000_000.0
    diff_age: Optional[float] = 1.0
    heading_stddev: Optional[float] = 0.1
    pitch_stddev: Optional[float] = 0.1

    @property
    def is_valid(self) -> bool:
        return self.lat is not None and self.lon is not None


# --- passes_fix_filter ---------------------------------------------------

@pytest.mark.parametrize('state,min_fix,expected', [
    ('rtk_fix', 'fix', True),
    ('rtk_float', 'fix', False),
    ('rtk_float', 'float', True),
    ('dgps', 'float', False),
    ('dgps', 'dgps', True),
    ('standalone', 'standalone', True),
    ('standalone', 'fix', False),
    ('unknown', 'none', True),
    ('unknown', 'standalone', False),
])
def test_passes_fix_filter(state, min_fix, expected):
    assert converters.passes_fix_filter(state, min_fix) is expected


# --- heading_pitch_to_enu_quaternion ------------------------------------

def _quat_yaw_enu(w, x, y, z):
    """quaternion (ENU) から yaw (ZYX-Euler の Z 成分) を抽出。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def test_heading_north_maps_to_yaw_pi_over_2():
    """真北 (heading=0°) は ENU yaw = +π/2 (東基準で 90° 反時計回り = 北)。"""
    w, x, y, z = converters.heading_pitch_to_enu_quaternion(0.0, 0.0)
    assert math.isclose(_quat_yaw_enu(w, x, y, z), math.pi / 2.0, abs_tol=1e-9)


def test_heading_east_maps_to_yaw_zero():
    """heading=90° (東) は ENU yaw = 0。"""
    w, x, y, z = converters.heading_pitch_to_enu_quaternion(90.0, 0.0)
    assert math.isclose(_quat_yaw_enu(w, x, y, z), 0.0, abs_tol=1e-9)


def test_heading_south_maps_to_yaw_minus_pi_over_2():
    """heading=180° (南) は ENU yaw = -π/2。"""
    w, x, y, z = converters.heading_pitch_to_enu_quaternion(180.0, 0.0)
    assert math.isclose(_quat_yaw_enu(w, x, y, z), -math.pi / 2.0, abs_tol=1e-9)


def test_quaternion_is_unit():
    for h in (0.0, 45.0, 123.4, 270.0):
        w, x, y, z = converters.heading_pitch_to_enu_quaternion(h, 5.0)
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        assert math.isclose(norm, 1.0, abs_tol=1e-9)


# --- position_to_rtk_status ---------------------------------------------

class _FakeClock:
    """make_stamp の引数として渡せる最小 stub (gnss_utc 経路では使われない)。"""

    def now(self):
        raise AssertionError('clock.now() should not be called for gnss_utc source')


def test_position_to_rtk_status_maps_fields():
    pos = FakePos(rtk_state='rtk_fix', heading=123.4, baseline_m=1.5)
    stamp = converters.make_stamp('gnss_utc', pos.timestamp, 0, _FakeClock())
    msg = converters.position_to_rtk_status(pos, stamp, 'gps_link', rtcm_bytes=12345)
    assert msg.rtk_state == RtkStatus.STATE_RTK_FIX
    assert msg.rtk_state_raw == 'rtk_fix'
    assert msg.heading_deg == pytest.approx(123.4)
    assert msg.baseline_length_m == pytest.approx(1.5)
    assert msg.rtcm_bytes_received == 12345
    assert msg.header.frame_id == 'gps_link'


def test_position_to_navsatfix_diagonal_covariance():
    pos = FakePos(hdop=1.0)
    stamp = converters.make_stamp('gnss_utc', pos.timestamp, 0, _FakeClock())
    msg = converters.position_to_navsatfix(pos, stamp, 'gps_link', hdop_sigma=2.0)
    # sigma_h = (1.0 * 2.0) ** 2 = 4.0
    assert msg.position_covariance[0] == pytest.approx(4.0)
    assert msg.position_covariance[4] == pytest.approx(4.0)
    # 非対角は 0
    assert msg.position_covariance[1] == 0.0
    assert msg.position_covariance[3] == 0.0


def test_make_stamp_gnss_utc_applies_transport_delay():
    stamp = converters.make_stamp('gnss_utc', 1_715_000_000.0, -10, _FakeClock())
    sec, nsec = stamp.seconds_nanoseconds()
    # -10ms オフセット (float の往復誤差を許容: ±100 ns)
    assert sec == 1_714_999_999
    assert abs(nsec - int(0.99 * 1e9)) < 100
