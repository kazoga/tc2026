"""建物近傍の状態遷移と相関誤差を確認する."""
import importlib.util
from pathlib import Path

import pytest

spec=importlib.util.spec_from_file_location('gnss_environment_core',
    Path(__file__).parents[1]/'tools/gnss_environment_core.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_hysteresis_and_recovery():
    model=module.BuildingDegradation([[(0,0),(5,0),(5,5),(0,5)]],2,3)
    assert model.sample(6,2,.1)['floating']
    assert model.sample(7.5,2,.1)['floating']
    assert not model.sample(8.1,2,.1)['floating']
    assert not model.sample(7.5,2,.1)['floating']


def test_bias_accumulates_and_decays():
    model=module.BuildingDegradation([[(0,0),(5,0),(5,5),(0,5)]])
    for _ in range(200):result=model.sample(6,2,.1)
    assert result['bias'][0]>1
    for _ in range(300):result=model.sample(100,2,.1)
    assert result['bias'][0]<.05


def test_invalid_threshold_rejected():
    with pytest.raises(ValueError):module.BuildingDegradation([],3,2)
