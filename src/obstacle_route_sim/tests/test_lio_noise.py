"""センサ誤差モデルの分布・seed・欠測・時刻の境界を確認する."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]/'tools'))
from lio_noise_core import PROFILES, SensorNoise


def test_reference_is_exact_identity() -> None:
    points = np.array([[1,2,3,4],[69,0,0,0]], dtype=np.float32)
    assert np.array_equal(SensorNoise('reference', 1).cloud(points), points)


def test_seed_reproduces_and_range_noise_has_expected_scale() -> None:
    points = np.tile([10.,0.,0.,1.], (100000,1)).astype(np.float32)
    a = SensorNoise('field_assumed', 7).cloud(points)
    b = SensorNoise('field_assumed', 7).cloud(points)
    assert np.array_equal(a,b)
    assert .91 < len(a)/len(points) < .93
    error = np.linalg.norm(a[:,:3],axis=1)-10
    assert np.std(error[abs(error)<.1]) == pytest.approx(.02, abs=.001)
    angles = np.arctan2(a[:,1],a[:,0])
    assert np.std(angles) == pytest.approx(np.deg2rad(.10), rel=.02)


def test_distant_returns_are_reduced_and_conservative_is_harder() -> None:
    points = np.tile([50.,0.,0.,1.],(50000,1)).astype(np.float32)
    a = len(SensorNoise('field_assumed',1).cloud(points))
    b = len(SensorNoise('conservative',1).cloud(points))
    assert b < a < len(points)*.6
    points[:,0] = 60
    assert len(SensorNoise('conservative',1).cloud(points)) == 0


def test_imu_noise_and_time_order() -> None:
    n = SensorNoise('conservative', 12)
    times, accel = [], []
    for i in range(10000):
        a, g, offset = n.imu(np.zeros(3),np.zeros(3),i*.005)
        assert np.isfinite(g).all()
        times.append(i*.005+offset)
        accel.append(a)
    assert np.all(np.diff(times)>0)
    assert np.std(np.diff(np.array(accel),axis=0))/np.sqrt(2) == pytest.approx(.02,rel=.04)
    assert np.max(abs(n.accel_fixed)) <= PROFILES['conservative'].accel_bias


def test_relative_metrics_ignore_global_rotation_but_detect_scale(tmp_path: Path) -> None:
    """固定した地図座標のずれと相対移動の失敗を区別する."""
    import csv
    from review_lio_noise_trials import trajectory_metrics
    times = np.arange(0., 20., .1)
    with (tmp_path/'trajectory.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['sim_s','x','y','yaw'])
        writer.writerows([[t,t,0.,0.] for t in times])
    def save_lio(scale: float) -> None:
        with (tmp_path/'lio_trajectory.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['sim_s','x','y','qx','qy','qz','qw'])
            writer.writerows([[t,10.,20.+t*scale,0.,0.,np.sqrt(.5),np.sqrt(.5)]
                              for t in times])
    save_lio(1.)
    exact = trajectory_metrics(tmp_path)
    assert exact['rigid_xy_rmse_m'] < 1e-10
    assert exact['rpe_5s_rmse_m'] < 1e-10
    save_lio(.9)
    assert trajectory_metrics(tmp_path)['rpe_5s_rmse_m'] == pytest.approx(.5,abs=.02)
    with (tmp_path/'trajectory.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['sim_s','x','y','yaw'])
        writer.writerows([[t,0.,0.,0.] for t in times])
    assert not trajectory_metrics(tmp_path)['valid']
