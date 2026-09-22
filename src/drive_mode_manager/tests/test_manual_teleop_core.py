from drive_mode_manager.manual_teleop_core import ManualTeleopConfig, ManualTeleopCore


def test_manual_deadman_outputs_zero_when_l1_released() -> None:
    core = ManualTeleopCore(ManualTeleopConfig(enable_button=4))
    core.update_joy([0.8, 1.0], [0, 0, 0, 0, 0], 0.0)

    result = core.compute(0.1)

    assert result.linear_x == 0.0
    assert result.angular_z == 0.0
    assert result.reason == 'deadman_released'


def test_axis_deadzone_and_invert() -> None:
    core = ManualTeleopCore(
        ManualTeleopConfig(
            linear_axis=1,
            angular_axis=0,
            linear_scale=2.0,
            angular_scale=3.0,
            deadzone=0.1,
            linear_axis_invert=True,
            enable_button=4,
            turbo_button=-1,
        )
    )
    core.update_joy([0.05, 0.5], [0, 0, 0, 0, 1], 0.0)

    result = core.compute(0.1)

    assert result.linear_x == -1.0
    assert result.angular_z == 0.0
    assert result.enabled


def test_turbo_scales_enabled_command() -> None:
    core = ManualTeleopCore(
        ManualTeleopConfig(
            linear_scale=1.0,
            angular_scale=1.0,
            turbo_button=5,
            turbo_ratio=2.0,
        )
    )
    core.update_joy([0.5, 0.5], [0, 0, 0, 0, 1, 1], 0.0)

    result = core.compute(0.1)

    assert result.linear_x == 1.0
    assert result.angular_z == 1.0


def test_joy_timeout_outputs_zero() -> None:
    core = ManualTeleopCore(ManualTeleopConfig(joy_timeout_s=0.5))
    core.update_joy([0.0, 1.0], [0, 0, 0, 0, 1], 0.0)

    result = core.compute(0.6)

    assert result.linear_x == 0.0
    assert result.reason == 'joy_timeout'


def test_pivot_turn_rejects_small_fore_aft_input_both_directions():
    for turn in (-1., 1.):
        for forward in (-.1, 0., .1):
            core = ManualTeleopCore(ManualTeleopConfig())
            core.update_joy([turn, forward], [0, 0, 0, 0, 1], 0.)
            result = core.compute(.1)
            assert result.linear_x == 0.
            assert result.angular_z == turn * 1.5
            # Differential drive wheel surface velocities have opposite signs.
            left = result.linear_x - result.angular_z * .30737 / 2
            right = result.linear_x + result.angular_z * .30737 / 2
            assert left * right < 0.


def test_deliberate_arc_and_slow_straight_input_are_preserved():
    for axes in ([.8, .3], [0., .1]):
        core = ManualTeleopCore(ManualTeleopConfig())
        core.update_joy(axes, [0, 0, 0, 0, 1], 0.)
        result = core.compute(.1)
        assert result.linear_x == axes[1] * 1.2
        assert result.angular_z == axes[0] * 1.5


def test_nonfinite_linear_input_does_not_turn_into_valid_pivot():
    core = ManualTeleopCore(ManualTeleopConfig())
    core.update_joy([1., float('nan')], [0, 0, 0, 0, 1], 0.)
    assert core.compute(.1).reason == 'invalid_axis_value'
