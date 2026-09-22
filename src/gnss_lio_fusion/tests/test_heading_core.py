"""方位の境界・観測異常・位置biasからの独立性を検証する."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from gnss_lio_fusion.fusion_core import FusionFilter, wrap
from gnss_lio_fusion.heading_core import HeadingObserver


def test_wrap_crossing_takes_short_arc() -> None:
    observer = HeadingObserver()
    yaw, variance = observer.update(1., math.radians(179), .1, math.radians(-179),
                                    .001, .1, .12, 2., True)
    assert 0 < wrap(yaw-math.radians(179)) <= .01200001
    assert variance > 0


def test_spike_does_not_poison_next_sample() -> None:
    observer = HeadingObserver()
    observer.update(0., 0., .1, 0., .0003, .1, .12, 2., True)
    yaw, _ = observer.update(.1, 0., .1, math.pi, .0003, .1, .12, 2., True)
    assert yaw == 0 and observer.rejected == 1
    observer.update(.2, 0., .1, 0., .0003, .1, .12, 2., True)
    assert observer.accepted == 2


@pytest.mark.parametrize('std', [0., -1., math.nan, math.inf])
def test_unknown_heading_cannot_initialize(std: float) -> None:
    filter_ = FusionFilter()
    filter_.advance(0., np.zeros(3))
    assert not filter_.observe_gps(0., np.zeros(3), 4, 20, .5, .01, std)
    assert filter_.x is None


def test_position_bias_does_not_turn_robot_heading() -> None:
    filters = [FusionFilter(), FusionFilter()]
    for i in range(200):
        for f, offset in zip(filters, [0., 1.2]):
            t = i*.1
            f.advance(t, np.array([t*.5, 0., 0.]))
            f.observe_gps(t, np.array([t*.5, offset if i > 30 else 0., 0.]),
                          4, 20, .5, .0004, .5)
    assert filters[0].x[2] == pytest.approx(filters[1].x[2], abs=1e-10)
    assert filters[1].x[1] > 1.


def test_heading_update_survives_position_rejection() -> None:
    f = FusionFilter()
    for i in range(50):
        f.advance(i*.1, np.zeros(3))
        f.observe_gps(i*.1, np.zeros(3), 4, 20, .5, .0004, .5)
    before = f.heading.accepted
    assert not f.observe_gps(5., np.array([100., 0., .01]), 4, 20, .5, .0004, .5)
    assert f.heading.accepted == before+1
    assert f.x[2] > 0


@pytest.mark.parametrize('state', [0, 1, 2, 5])
def test_invalid_fix_state_cannot_correct_heading(state: int) -> None:
    f = FusionFilter()
    f.advance(0., np.zeros(3))
    assert f.observe_gps(0., np.zeros(3), 4, 20, .5, .0004, .5)
    before = f.x.copy()
    covariance = f.p.copy()
    assert not f.observe_gps(.1, np.array([0., 0., .1]), state, 20, .5, .0004, .5)
    np.testing.assert_array_equal(f.x, before)
    np.testing.assert_array_equal(f.p, covariance)
    assert f.heading.reason == 'quality_rejected'


def test_persistent_half_turn_is_not_learned_as_new_heading() -> None:
    observer = HeadingObserver()
    yaw, variance = 0., .1
    for i in range(60):
        yaw, variance = observer.update(i*.1, yaw, variance, math.pi,
                                         .0003, .1, .12, 2., True)
    assert yaw == 0 and observer.rejected == 60
    yaw, _ = observer.update(6., yaw, variance, .01, .0003, .1, .12, 2., True)
    assert yaw > 0 and observer.accepted == 1


def test_known_lio_fault_allows_stable_fix_heading_recovery() -> None:
    observer = HeadingObserver()
    yaw, variance = 0., 4.
    for i in range(700):
        before = yaw
        yaw, variance = observer.update(i*.1, yaw, variance+.001, math.radians(135),
                                         .0003, .1, .12, 2., True, True)
        assert abs(wrap(yaw-before)) <= .012000001
    assert abs(wrap(yaw-math.radians(135))) < math.radians(1)
    assert observer.rejected >= 19


def test_recovery_restarts_after_quality_dropout() -> None:
    observer = HeadingObserver()
    for i in range(50):
        usable = i != 10
        observer.update(i*.1, 0., 4., 2., .0003, .1, .12, 2., usable, True)
    assert observer.accepted > 0
