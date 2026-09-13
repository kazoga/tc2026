"""事後整合が拡大縮小や鏡映による地図変形を隠さないことを確認する."""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parents[1]/'tools'))
from review_full_lio_trial import rigid_fit


def test_rigid_fit_removes_only_constant_rotation_and_translation() -> None:
    points = np.random.default_rng(1).normal(size=(100, 3))
    target = points @ Rotation.from_euler('xyz', [.2, -.3, .8]).as_matrix()+[3, 8, -2]
    rotation, translation = rigid_fit(points, target)
    assert np.allclose(points @ rotation+translation, target)
    stretched = points*2
    rotation, translation = rigid_fit(stretched, target)
    assert np.sqrt(np.mean((stretched @ rotation+translation-target)**2)) > .5


def test_rigid_fit_does_not_accept_reflected_map() -> None:
    points = np.random.default_rng(2).normal(size=(100, 3))
    target = points*[-1, 1, 1]
    rotation, translation = rigid_fit(points, target)
    assert np.linalg.det(rotation) > .999999
    assert np.sqrt(np.mean((points @ rotation+translation-target)**2)) > .5
