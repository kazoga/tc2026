from drive_mode_manager.gui_core import DriveStatusGuiCore
from drive_mode_manager.drive_mode_core import MODE_AUTONOMOUS, MODE_MANUAL


class Status:
    mode = MODE_AUTONOMOUS
    output_source = 0
    reason = 'test'
    output_linear_x = 0.0
    output_angular_z = 0.0
    auto_resume_pending = False
    auto_resume_remaining_s = 0.0
    pending_autonomous_linear_x = 0.0
    pending_autonomous_angular_z = 0.0
    joy_available = False
    l1_pressed = False
    autonomous_cmd_alive = False


def test_regulation_state_label_mapping() -> None:
    core = DriveStatusGuiCore()
    status = Status()
    status.mode = MODE_MANUAL

    view = core.update_status(status)
    assert view.state_label == '操縦 / Manual'

    status.mode = MODE_AUTONOMOUS
    status.auto_resume_pending = True
    view = core.update_status(status)
    assert view.state_label == '自律 / Auto'


def test_aspect_ratio_fit_rect() -> None:
    width, height = DriveStatusGuiCore.fit_rect(1000.0, 1000.0)

    assert round(width / height, 6) == round(16.0 / 9.0, 6)
    assert width <= 1000.0
    assert height <= 1000.0


def test_planned_direction_text_warns_for_resume_limits() -> None:
    text = DriveStatusGuiCore.planned_direction_text(
        linear_x=1.0,
        angular_z=1.3,
        turn_preview_seconds=1.0,
        max_linear_x=0.8,
        max_angular_z=1.2,
    )

    assert text.startswith('左旋回')
    assert '速度注意！' in text
    assert '急旋回注意！' in text


def test_planned_direction_text_warns_for_reverse_resume() -> None:
    text = DriveStatusGuiCore.planned_direction_text(
        linear_x=-0.2,
        angular_z=0.0,
        turn_preview_seconds=1.0,
        max_linear_x=0.8,
        max_angular_z=1.2,
    )

    assert text == '直進\n後退注意！'
