"""drive_status_gui_node の表示用 ROS 非依存モデル。"""

from __future__ import annotations

from dataclasses import dataclass
import math

from drive_mode_manager.drive_mode_core import MODE_AUTONOMOUS, MODE_MANUAL


@dataclass(frozen=True)
class DriveStatusView:
    """GUI 描画に使う走行状態の要約。"""

    mode: int = MODE_AUTONOMOUS
    output_source: int = 0
    state_label: str = '自律 / Auto'
    state_color: str = '#198754'
    output_label: str = 'ZERO'
    reason: str = 'waiting'
    output_linear_x: float = 0.0
    output_angular_z: float = 0.0
    auto_resume_pending: bool = False
    auto_resume_remaining_s: float = 0.0
    pending_autonomous_linear_x: float = 0.0
    pending_autonomous_angular_z: float = 0.0
    joy_available: bool = False
    l1_pressed: bool = False
    autonomous_cmd_alive: bool = False
    manual_input_active: bool = False
    manual_cmd_alive: bool = False
    l1_released_elapsed_s: float = 0.0
    rtk_text: str = 'RTK: n/a'
    follower_text: str = 'Waypoint: n/a'


class DriveStatusGuiCore:
    """DriveModeStatus から表示モデルを作る。"""

    def __init__(self) -> None:
        self._view = DriveStatusView()

    def update_status(self, msg: object) -> DriveStatusView:
        """DriveModeStatus 相当の msg から表示モデルを更新する。"""

        mode = int(getattr(msg, 'mode', MODE_AUTONOMOUS))
        output_source = int(getattr(msg, 'output_source', 0))
        if mode == MODE_MANUAL:
            label = '操縦 / Manual'
            color = '#ffc107'
        else:
            label = '自律 / Auto'
            color = '#198754'

        self._view = DriveStatusView(
            mode=mode,
            output_source=output_source,
            state_label=label,
            state_color=color,
            output_label=self._source_label(output_source),
            reason=str(getattr(msg, 'reason', '')),
            output_linear_x=float(getattr(msg, 'output_linear_x', 0.0)),
            output_angular_z=float(getattr(msg, 'output_angular_z', 0.0)),
            auto_resume_pending=bool(getattr(msg, 'auto_resume_pending', False)),
            auto_resume_remaining_s=float(getattr(msg, 'auto_resume_remaining_s', 0.0)),
            pending_autonomous_linear_x=float(getattr(msg, 'pending_autonomous_linear_x', 0.0)),
            pending_autonomous_angular_z=float(getattr(msg, 'pending_autonomous_angular_z', 0.0)),
            joy_available=bool(getattr(msg, 'joy_available', False)),
            l1_pressed=bool(getattr(msg, 'l1_pressed', False)),
            autonomous_cmd_alive=bool(getattr(msg, 'autonomous_cmd_alive', False)),
            manual_input_active=bool(getattr(msg, 'manual_input_active', False)),
            manual_cmd_alive=bool(getattr(msg, 'manual_cmd_alive', False)),
            l1_released_elapsed_s=float(getattr(msg, 'l1_released_elapsed_s', self._view.l1_released_elapsed_s)),
            rtk_text=self._view.rtk_text,
            follower_text=self._view.follower_text,
        )
        return self._view

    def update_rtk_text(self, text: str) -> DriveStatusView:
        """RTK 表示行を更新する。"""

        self._view = DriveStatusView(**{**self._view.__dict__, 'rtk_text': text})
        return self._view

    def update_follower_text(self, text: str) -> DriveStatusView:
        """waypoint 表示行を更新する。"""

        self._view = DriveStatusView(**{**self._view.__dict__, 'follower_text': text})
        return self._view

    def update_l1_released_elapsed(self, elapsed_s: float) -> DriveStatusView:
        """L1 を離してからの経過秒を更新する。"""

        self._view = DriveStatusView(**{**self._view.__dict__, 'l1_released_elapsed_s': elapsed_s})
        return self._view

    def snapshot(self) -> DriveStatusView:
        """現在の表示モデルを返す。"""

        return self._view

    @staticmethod
    def fit_rect(view_w: float, view_h: float, aspect_w: float = 16.0, aspect_h: float = 9.0) -> tuple[float, float]:
        """指定 view に収まる 16:9 描画領域の幅と高さを返す。"""

        target = aspect_w / aspect_h
        if view_w / view_h > target:
            return view_h * target, view_h
        return view_w, view_w / target

    @staticmethod
    def planned_direction_text(
        linear_x: float,
        angular_z: float,
        turn_preview_seconds: float,
        max_linear_x: float,
        max_angular_z: float,
        linear_scale: float = 1.2,
        angular_scale: float = 1.5,
        deadzone: float = 0.05,
        linear_axis_invert: bool = False,
        angular_axis_invert: bool = False,
    ) -> str:
        """復帰予定 cmd から進行予定方向と警告文を作る。"""

        _ = turn_preview_seconds
        _ = max_linear_x
        _ = max_angular_z
        angle_deg = DriveStatusGuiCore.direction_angle_deg_from_cmd_vel(
            linear_x=linear_x,
            angular_z=angular_z,
            linear_scale=linear_scale,
            angular_scale=angular_scale,
            deadzone=deadzone,
            linear_axis_invert=linear_axis_invert,
            angular_axis_invert=angular_axis_invert,
        )
        abs_angle = abs(angle_deg)
        if abs_angle <= 15.0:
            return '前進'
        if abs_angle <= 90.0:
            direction = '右旋回' if angle_deg > 0.0 else '左旋回'
            if abs_angle >= 45.0:
                return direction + '\n急旋回注意！'
            return direction
        return '後進\n後方注意！'

    @staticmethod
    def cmd_vel_to_stick_point(
        linear_x: float,
        angular_z: float,
        linear_scale: float = 1.2,
        angular_scale: float = 1.5,
        deadzone: float = 0.05,
        linear_axis_invert: bool = False,
        angular_axis_invert: bool = False,
    ) -> tuple[float, float]:
        """cmd_vel から ps3_joy_sim 相当の stick 座標を逆算する。"""

        stick_y = DriveStatusGuiCore._inverse_scaled_axis(linear_x, linear_scale)
        stick_x = DriveStatusGuiCore._inverse_scaled_axis(angular_z, angular_scale)
        if linear_axis_invert:
            stick_y *= -1.0
        if angular_axis_invert:
            stick_x *= -1.0
        if abs(stick_y) < max(0.0, deadzone):
            stick_y = 0.0
        if abs(stick_x) < max(0.0, deadzone):
            stick_x = 0.0

        norm = math.hypot(stick_x, stick_y)
        if norm > 1.0:
            stick_x /= norm
            stick_y /= norm
        return stick_x, stick_y

    @staticmethod
    def direction_vector_from_cmd_vel(
        linear_x: float,
        angular_z: float,
        linear_scale: float = 1.2,
        angular_scale: float = 1.5,
        deadzone: float = 0.05,
        linear_axis_invert: bool = False,
        angular_axis_invert: bool = False,
    ) -> tuple[float, float]:
        """cmd_vel から画面上の矢印方向ベクトルを算出する。"""

        stick_x, stick_y = DriveStatusGuiCore.cmd_vel_to_stick_point(
            linear_x=linear_x,
            angular_z=angular_z,
            linear_scale=linear_scale,
            angular_scale=angular_scale,
            deadzone=deadzone,
            linear_axis_invert=linear_axis_invert,
            angular_axis_invert=angular_axis_invert,
        )
        norm = math.hypot(stick_x, stick_y)
        if norm <= 0.0:
            return 0.0, -1.0
        return stick_x / norm, -stick_y / norm

    @staticmethod
    def direction_angle_from_cmd_vel(
        linear_x: float,
        angular_z: float,
        linear_scale: float = 1.2,
        angular_scale: float = 1.5,
        deadzone: float = 0.05,
        linear_axis_invert: bool = False,
        angular_axis_invert: bool = False,
    ) -> float:
        """cmd_vel に対応する画面上の矢印角度を rad で返す。"""

        return math.radians(DriveStatusGuiCore.direction_angle_deg_from_cmd_vel(
            linear_x=linear_x,
            angular_z=angular_z,
            linear_scale=linear_scale,
            angular_scale=angular_scale,
            deadzone=deadzone,
            linear_axis_invert=linear_axis_invert,
            angular_axis_invert=angular_axis_invert,
        ))

    @staticmethod
    def direction_angle_deg_from_cmd_vel(
        linear_x: float,
        angular_z: float,
        linear_scale: float = 1.2,
        angular_scale: float = 1.5,
        deadzone: float = 0.05,
        linear_axis_invert: bool = False,
        angular_axis_invert: bool = False,
    ) -> float:
        """上方向を 0 度、右方向を正として矢印角度を返す。"""

        dx, dy = DriveStatusGuiCore.direction_vector_from_cmd_vel(
            linear_x=linear_x,
            angular_z=angular_z,
            linear_scale=linear_scale,
            angular_scale=angular_scale,
            deadzone=deadzone,
            linear_axis_invert=linear_axis_invert,
            angular_axis_invert=angular_axis_invert,
        )
        return math.degrees(math.atan2(dx, -dy))

    @staticmethod
    def _inverse_scaled_axis(value: float, scale: float) -> float:
        if not math.isfinite(value) or not math.isfinite(scale) or abs(scale) <= 0.0:
            return 0.0
        return max(-1.0, min(1.0, value / scale))

    @staticmethod
    def _source_label(source: int) -> str:
        mapping = {
            0: 'ZERO',
            1: 'AUTONOMOUS_CMD',
            2: 'MANUAL_CMD',
        }
        return mapping.get(source, 'UNKNOWN')
