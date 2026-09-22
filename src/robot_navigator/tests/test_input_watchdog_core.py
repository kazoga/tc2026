"""受信間隔監視の境界・途絶・復帰を確認する."""

import importlib.util
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    'input_watchdog_core',
    Path(__file__).resolve().parents[1]/'robot_navigator/input_watchdog_core.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
InputWatchdog = MODULE.InputWatchdog


@pytest.mark.parametrize('timeout', [0, -1, float('nan'), float('inf')])
def test_invalid_timeout_fails_at_startup(timeout: float) -> None:
    with pytest.raises(ValueError):
        InputWatchdog({'pose': timeout})


def test_missing_boundary_and_independent_timeouts() -> None:
    guard = InputWatchdog({'pose': 1.0, 'odom': .5})
    assert guard.stale_inputs(10) == ('pose', 'odom')
    guard.receive('pose', 10)
    guard.receive('odom', 10.25)
    assert guard.stale_inputs(10.749) == ()
    assert guard.stale_inputs(10.75) == ('odom',)
    assert guard.stale_inputs(11) == ('pose', 'odom')


def test_recovery_requires_both_inputs_to_be_fresh() -> None:
    guard = InputWatchdog({'pose': 1.0, 'odom': 1.0})
    guard.receive('pose', 0)
    guard.receive('odom', 0)
    guard.receive('pose', 3)
    assert guard.stale_inputs(3) == ('odom',)
    guard.receive('odom', 3.1)
    assert guard.stale_inputs(3.1) == ()


def test_clock_reversal_and_nonfinite_time_stop() -> None:
    guard = InputWatchdog({'pose': 1.0})
    guard.receive('pose', 5)
    assert guard.stale_inputs(4) == ('pose',)
    assert guard.stale_inputs(float('nan')) == ('pose',)
    with pytest.raises(ValueError):
        guard.receive('unknown', 5)
    with pytest.raises(ValueError):
        guard.receive('pose', float('inf'))
