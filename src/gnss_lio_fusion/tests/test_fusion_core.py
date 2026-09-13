"""baseline再取得と品質に応じた予測・補正・異常処理を確認する."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from gnss_lio_fusion.baseline_core import AdaptiveBaseline, BaselineConfig
from gnss_lio_fusion.fusion_core import FusionFilter


def test_baseline_learns_transport_change_and_freezes_float() -> None:
    b = AdaptiveBaseline()
    for i in range(40):
        result = b.observe(i*.1, .515+(-1)**i*.001, True, 20)
    assert b.ready and b.reference == pytest.approx(.515, abs=.001)
    before = b.reference
    for i in range(40, 140):
        result = b.observe(i*.1, .58, False, 10)
    assert b.reference == before
    assert result['score'] < .01
    for i in range(140, 250):
        b.observe(i*.1, .535+(-1)**i*.001, True, 20)
    assert b.reference == pytest.approx(.535, abs=.001)
    assert b.reacquisitions == 1


def test_baseline_does_not_learn_spikes_or_low_satellites() -> None:
    b = AdaptiveBaseline()
    for i in range(40):
        b.observe(i*.1, .5, True, 20)
    assert b.observe(4., .6, True, 20)['score'] < .01
    for i in range(41, 180):
        b.observe(i*.1, .55, True, 8)
    assert b.reference == pytest.approx(.5)
    assert b.observe(18., float('nan'), True, 20)['score'] == 0
    assert b.observe(17., .5, True, 20)['mode'] == 'out_of_order'


def initialized() -> FusionFilter:
    f = FusionFilter()
    f.advance(0., np.zeros(3))
    assert f.observe_gps(0., np.zeros(3), 4, 20, .5, .02**2, .5)
    return f


def test_gps_outage_continues_lio_and_uncertainty_grows() -> None:
    f = initialized()
    initial = f.p.copy()
    for i in range(1, 101):
        f.advance(i*.1, np.array([i*.05, 0., 0.]))
    assert f.x[0] == pytest.approx(5.)
    assert f.p[0, 0] > initial[0, 0]
    assert f.diagnostics(10.)['mode'] == 'LIO_PRIORITY'
    assert f.diagnostics(10.)['speed_limit_mps'] > 0


def test_large_lio_jump_rejected_without_poisoning_future_increments() -> None:
    f = initialized()
    assert not f.advance(.1, np.array([100., 0., 0.]))
    assert f.x[0] == 0
    assert f.advance(.2, np.array([100.05, 0., 0.]))
    assert f.x[0] == pytest.approx(.05)
    assert not f.advance(.3, np.array([math.nan, 0., 0.]))


def test_recovery_caps_correction_and_preserves_positive_covariance() -> None:
    f = initialized()
    for i in range(1, 50):
        f.advance(i*.1, np.array([i*.08, 0., 0.]))
    for i in range(50, 100):
        f.advance(i*.1, np.array([i*.08, 0., 0.]))
        before = f.x.copy()
        f.observe_gps(i*.1, np.array([i*.05, 0., 0.]), 4, 20, .5, .001, .5)
        assert np.linalg.norm(f.x[:2]-before[:2]) <= .0500001
        assert np.linalg.eigvalsh(f.p).min() > 0
    assert f.x[0] < 6.


def test_bad_baseline_downweights_heading_and_float_does_not_dominate() -> None:
    f = initialized()
    for i in range(1, 40):
        f.advance(i*.1, np.zeros(3))
        f.observe_gps(i*.1, np.zeros(3), 4, 20, .5, .0004, .5)
    before = f.x.copy()
    for i in range(40, 70):
        f.advance(i*.1, np.zeros(3))
        f.observe_gps(i*.1, np.array([1.2, 1.2, .3]), 3, 10, .6, .36, 5.)
    assert np.linalg.norm(f.x[:2]-before[:2]) < .2
    assert abs(f.x[2]) < .02


def test_nonzero_initial_yaw_rotates_relative_lio_motion() -> None:
    f = FusionFilter()
    f.advance(0., np.zeros(3))
    f.observe_gps(0., np.array([10., 20., math.pi/2]), 4, 20, .5, .001, .5)
    f.advance(1., np.array([.5, 0., 0.]))
    assert f.x[:2] == pytest.approx([10., 20.5])


def test_nonfinite_baseline_configuration_is_rejected_before_learning() -> None:
    with pytest.raises(ValueError):
        AdaptiveBaseline(BaselineConfig(adapt_tau_s=math.nan))
    with pytest.raises(ValueError):
        AdaptiveBaseline(BaselineConfig(min_satellites=0))
