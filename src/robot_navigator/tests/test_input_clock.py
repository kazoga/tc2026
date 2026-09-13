"""低速模擬の計算待ちと、実際の受信途絶を区別する."""
from types import SimpleNamespace
from pathlib import Path
import sys

import pytest

pytest.importorskip('rclpy')
sys.path.insert(0, str(Path(__file__).parents[1]))
from robot_navigator.robot_navigator_node import RobotNavigator
from robot_navigator.input_watchdog_core import InputWatchdog


def test_simulated_clock_does_not_expire_due_to_wall_delay(monkeypatch):
    monkeypatch.setattr('time.monotonic', lambda: 100.)
    clock = SimpleNamespace(nanoseconds=10_000_000_000)
    node = SimpleNamespace(get_parameter=lambda _: SimpleNamespace(value=True),
                           get_clock=lambda: SimpleNamespace(now=lambda: clock))
    watchdog = InputWatchdog({'pose': 1.})
    watchdog.receive('pose', RobotNavigator._input_time_seconds(node))
    clock.nanoseconds += 200_000_000
    assert watchdog.stale_inputs(RobotNavigator._input_time_seconds(node)) == ()
    clock.nanoseconds += 1_000_000_000
    assert watchdog.stale_inputs(RobotNavigator._input_time_seconds(node)) == ('pose',)


def test_real_clock_still_detects_wall_timeout(monkeypatch):
    monkeypatch.setattr('time.monotonic', lambda: 100.)
    node = SimpleNamespace(get_parameter=lambda _: SimpleNamespace(value=False))
    watchdog = InputWatchdog({'pose': 1.})
    watchdog.receive('pose', RobotNavigator._input_time_seconds(node))
    monkeypatch.setattr('time.monotonic', lambda: 101.1)
    assert watchdog.stale_inputs(RobotNavigator._input_time_seconds(node)) == ('pose',)


def test_timer_publishes_stop_and_clears_controller_on_dropout() -> None:
    from geometry_msgs.msg import Twist

    published = []
    guard = InputWatchdog({'pose': 1., 'odom': 1.})
    guard.receive('pose', 0.)
    guard.receive('odom', 0.)
    previous = Twist()
    previous.linear.x = .5
    node = SimpleNamespace(
        input_watchdog=guard, _input_time_seconds=lambda: 1., _stale_inputs=(),
        get_logger=lambda: SimpleNamespace(warn=lambda _: None),
        integral_w=2., prev_yaw_error=.3, prev_cmd_vel=previous,
        cmd_pub=SimpleNamespace(publish=published.append),
    )
    RobotNavigator.on_timer(node)
    assert len(published) == 1
    assert published[0] == Twist()
    assert node.prev_cmd_vel == Twist()
    assert node.integral_w == node.prev_yaw_error == 0.
    assert node._stale_inputs == ('pose', 'odom')
