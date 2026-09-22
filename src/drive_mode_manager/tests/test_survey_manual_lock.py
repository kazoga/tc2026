"""採取中はL1を離してもゼロ自律指令によって手動から離脱しない。"""
from drive_mode_manager.drive_mode_core import (
    DriveModeCore, DriveModeConfig, JoySnapshot, CommandSnapshot, MODE_MANUAL)


def test_survey_stays_manual_after_deadman_release():
    core = DriveModeCore(DriveModeConfig(initial_mode='manual', allow_auto_resume=False))
    for now in [0., 1., 10., 100.]:
        output = core.update(now, JoySnapshot([0]*17, now),
            CommandSnapshot(1., 0., 0., now), CommandSnapshot(.2, 0., 0., now))
        assert output.mode == MODE_MANUAL and output.linear_x == 0.
    buttons = [0]*17; buttons[4] = 1
    output = core.update(100.1, JoySnapshot(buttons, 100.1),
        CommandSnapshot(1., 0., 0., 100.1), CommandSnapshot(.2, 0., 0., 100.1))
    assert output.mode == MODE_MANUAL and output.linear_x == .2
