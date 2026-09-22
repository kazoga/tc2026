"""異常LIO、車輪途絶、復帰時原点差を持つ相対運動の選択を確認する."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from gnss_lio_fusion.motion_guard_core import MotionGuard


def test_lio_jump_uses_wheel_without_absolute_origin_jump() -> None:
    guard = MotionGuard()
    guard.select(0., np.zeros(3), np.array([100., 200., 0.]))
    out = guard.select(.1, np.array([20., 30., 2.]), np.array([100.05, 200., 0.]))
    assert out == pytest.approx([.05, 0., 0.])
    assert guard.source == 'WHEEL_FALLBACK'
    for i in range(2, 30):
        out = guard.select(i*.1, np.array([20.+i*.05, 30., 0.]),
                           np.array([100.+i*.05, 200., 0.]))
    assert abs(out[0]-1.45) < .06
    assert guard.source == 'LIO'


def test_wheel_outage_reanchors_without_double_counting_motion() -> None:
    guard = MotionGuard()
    guard.select(0., np.zeros(3), np.zeros(3))
    guard.select(.1, np.array([.05, 0., 0.]), None)
    out = guard.select(.2, np.array([.1, 0., 0.]), np.array([.1, 0., 0.]))
    assert out[0] == pytest.approx(.1)


def test_complete_lio_outage_uses_wheel_and_both_invalid_do_not_refresh() -> None:
    guard = MotionGuard()
    guard.select(0., np.zeros(3), np.zeros(3))
    for i in range(1, 50):
        out = guard.select(i*.1, None, np.array([i*.05, 0., 0.]))
    assert out[0] == pytest.approx(2.45)
    assert guard.source == 'WHEEL_FALLBACK'
    assert guard.select(5., None, np.array([1000., 0., 0.])) is None
    assert guard.stamp == pytest.approx(4.9)


def test_sustained_scale_error_is_detected_without_a_single_large_jump():
    """各0.1秒差分は小さくても、1秒の移動不足を放置しない."""
    guard = MotionGuard()
    for i in range(101):
        t = i*.1
        selected = guard.select(t, np.array([.25*t, 0., 0.]), np.array([.9*t, 0., 0.]))
    assert guard.source == 'WHEEL_FALLBACK'
    assert guard.window_rejections > 0
    assert abs(selected[0]-9.) < .8
    # 移動量が揃った状態を継続した後にLIOへ復帰する。
    for i in range(101, 140):
        t = i*.1
        guard.select(t, np.array([2.5+.9*(t-10.), 0., 0.]), np.array([.9*t, 0., 0.]))
    assert guard.source == 'LIO'
