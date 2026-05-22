"""走行モード状態を常時表示する専用 GUI ノード。"""

from __future__ import annotations

import math
import signal
import sys
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from route_msgs.msg import DriveModeStatus, FollowerState, ManagerStatus

try:
    from rtk_gps_um982_msgs.msg import RtkStatus
except ImportError:  # pragma: no cover - 依存が無い開発環境向け
    RtkStatus = None  # type: ignore[assignment]

from drive_mode_manager.gui_core import DriveStatusGuiCore, DriveStatusView


class DriveStatusRosNode(Node):
    """GUI に渡す表示状態を ROS topic から更新する。"""

    def __init__(self, core: DriveStatusGuiCore) -> None:
        super().__init__('drive_status_gui_node')
        self._core = core
        self._lock = threading.Lock()
        self._l1_released_since_s: float | None = None
        self.create_subscription(DriveModeStatus, 'drive_mode_status', self._on_status, 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 1)
        self.create_subscription(Twist, 'cmd_vel/autonomous', self._on_autonomous_cmd, 1)
        self.create_subscription(FollowerState, 'follower_state', self._on_follower_state, 10)
        self.create_subscription(ManagerStatus, 'manager_status', self._on_manager_status, 10)
        if RtkStatus is not None:
            self.create_subscription(RtkStatus, 'rtk_gps/rtk_status', self._on_rtk_status, 10)
        self.get_logger().info('drive_status_gui_node started')

    def snapshot(self) -> DriveStatusView:
        """GUI thread から読む表示スナップショットを返す。"""

        with self._lock:
            return self._core.snapshot()

    def _on_status(self, msg: DriveModeStatus) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if msg.mode == DriveModeStatus.MODE_MANUAL and not msg.l1_pressed:
            if self._l1_released_since_s is None:
                self._l1_released_since_s = now_s
            l1_released_elapsed_s = max(0.0, now_s - self._l1_released_since_s)
        else:
            self._l1_released_since_s = None
            l1_released_elapsed_s = 0.0
        with self._lock:
            self._core.update_status(msg)
            self._core.update_l1_released_elapsed(l1_released_elapsed_s)

    def _on_cmd_vel(self, _msg: Twist) -> None:
        pass

    def _on_autonomous_cmd(self, _msg: Twist) -> None:
        pass

    def _on_follower_state(self, msg: FollowerState) -> None:
        text = f"Waypoint: {msg.active_waypoint_label} state={msg.state}"
        with self._lock:
            self._core.update_follower_text(text)

    def _on_manager_status(self, msg: ManagerStatus) -> None:
        if not self._core.snapshot().follower_text.startswith('Waypoint: n/a'):
            return
        text = f"Manager: state={msg.state} cause={msg.last_cause}"
        with self._lock:
            self._core.update_follower_text(text)

    def _on_rtk_status(self, msg: object) -> None:
        raw_state = getattr(msg, 'rtk_state_raw', '')
        state_value = getattr(msg, 'rtk_state', getattr(msg, 'state', 'n/a'))
        state = self._rtk_state_text(state_value, raw_state)
        satellites = getattr(msg, 'num_satellites', getattr(msg, 'satellites', 'n/a'))
        heading = getattr(msg, 'heading_deg', None)
        if heading is None or not math.isfinite(float(heading)):
            heading_text = 'heading=n/a'
        else:
            heading_text = f'heading={float(heading):.1f}deg'
        text = f'RTK: state={state} sat={satellites} {heading_text}'
        with self._lock:
            self._core.update_rtk_text(text)


    @staticmethod
    def _rtk_state_text(state_value: object, raw_state: object) -> str:
        raw_text = str(raw_state).strip()
        if raw_text:
            return raw_text
        mapping = {
            0: 'UNKNOWN',
            1: 'STANDALONE',
            2: 'DGPS',
            3: 'RTK FLOAT',
            4: 'RTK FIX',
        }
        try:
            return mapping.get(int(state_value), str(state_value))
        except (TypeError, ValueError):
            return str(state_value)


class DriveStatusWindow:  # pragma: no cover - GUI は手動/統合確認で扱う
    """PyQt5 で 16:9 固定比率の状態表示を描画する。"""

    def __init__(
        self,
        ros_node: DriveStatusRosNode,
        main_display_ratio: float,
        turn_preview_seconds: float,
        manual_to_auto_l1_released_s: float,
        max_autonomous_resume_linear_x: float,
        max_autonomous_resume_angular_z: float,
        direction_linear_scale: float,
        direction_angular_scale: float,
        direction_deadzone: float,
        direction_linear_axis_invert: bool,
        direction_angular_axis_invert: bool,
    ) -> None:
        from PyQt5 import QtCore, QtGui, QtWidgets

        self._qt = QtCore, QtGui, QtWidgets
        self._ros_node = ros_node
        self._main_ratio = min(0.80, max(0.60, main_display_ratio))
        self._turn_preview_seconds = max(0.0, turn_preview_seconds)
        self._manual_to_auto_l1_released_s = max(0.0, manual_to_auto_l1_released_s)
        self._max_autonomous_resume_linear_x = max(0.0, max_autonomous_resume_linear_x)
        self._max_autonomous_resume_angular_z = max(0.0, max_autonomous_resume_angular_z)
        self._direction_linear_scale = max(0.0, direction_linear_scale)
        self._direction_angular_scale = max(0.0, direction_angular_scale)
        self._direction_deadzone = max(0.0, direction_deadzone)
        self._direction_linear_axis_invert = direction_linear_axis_invert
        self._direction_angular_axis_invert = direction_angular_axis_invert
        self._app = QtWidgets.QApplication(sys.argv)
        self._window = QtWidgets.QMainWindow()
        self._window.setWindowTitle('Drive Mode Status')
        self._view = QtWidgets.QGraphicsView()
        self._view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self._view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self._scene = QtWidgets.QGraphicsScene(0, 0, 1600, 900)
        self._view.setScene(self._scene)
        self._window.setCentralWidget(self._view)
        self._window.resize(1280, 720)
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._redraw)
        self._timer.start(100)

    def run(self) -> int:
        """GUI を開始し、終了コードを返す。"""

        self._window.show()
        self._redraw()
        return int(self._app.exec_())

    def quit(self) -> None:
        """外部停止要求を Qt event loop の終了へ変換する。"""

        self._timer.stop()
        self._app.quit()

    def _redraw(self) -> None:
        QtCore, QtGui, QtWidgets = self._qt
        view = self._ros_node.snapshot()
        self._scene.clear()
        self._scene.setBackgroundBrush(QtGui.QBrush(QtGui.QColor('#111111')))
        main_h = 900.0 * self._main_ratio
        main_color = QtGui.QColor(view.state_color)
        self._scene.addRect(0, 0, 1600, main_h, QtGui.QPen(QtCore.Qt.NoPen), QtGui.QBrush(main_color))
        self._add_text(
            view.state_label,
            0,
            120,
            144,
            '#ffffff',
            bold=True,
            max_width=1600,
            alignment='center',
        )
        self._add_mux_status(view, 1185, main_h - 140)

        y0 = main_h
        self._scene.addRect(
            0,
            y0,
            1600,
            900 - y0,
            QtGui.QPen(QtCore.Qt.NoPen),
            QtGui.QBrush(QtGui.QColor('#20242a')),
        )
        left_x = 70
        center_x = 600
        right_x = 1120
        if view.auto_resume_pending:
            self._add_cmd_slot(
                '復帰時Cmd',
                left_x,
                y0,
                view.pending_autonomous_linear_x,
                view.pending_autonomous_angular_z,
                width=430,
            )
            self._add_text('自律走行開始まで', center_x, y0 + 38, 32, '#d8dee9', bold=True, max_width=440)
        else:
            self._add_cmd_slot(
                '出力Cmd',
                left_x,
                y0,
                view.output_linear_x,
                view.output_angular_z,
                width=430,
            )

        if view.auto_resume_pending:
            self._add_text(
                f'{view.auto_resume_remaining_s:.1f} 秒',
                center_x,
                y0 + 92,
                70,
                '#ffd166',
                bold=True,
                max_width=440,
            )
        elif view.mode == 2:
            auto_resume_text = '有効' if view.autonomous_cmd_alive else '無効'
            self._add_text(
                f'自律復帰: {auto_resume_text}',
                center_x,
                y0 + 38,
                32,
                '#d8dee9',
                bold=True,
                max_width=440,
            )
            self._add_text(
                f'L1: {self._yes_no(view.l1_pressed)}',
                center_x,
                y0 + 96,
                40,
                '#ffffff',
                max_width=440,
            )
            self._add_text(
                self._manual_resume_wait_text(view),
                center_x,
                y0 + 154,
                30,
                '#ffd166' if not view.l1_pressed else '#ffffff',
                max_width=440,
            )
        else:
            self._add_text('経路', center_x, y0 + 38, 32, '#d8dee9', bold=True, max_width=440)
            self._add_text(self._route_text(view.follower_text), center_x, y0 + 96, 32, '#ffffff', max_width=440)

        if view.auto_resume_pending:
            self._add_text('復帰方向', right_x, y0 + 38, 32, '#d8dee9', bold=True, max_width=410)
            direction_text = self._planned_direction_text(
                view.pending_autonomous_linear_x,
                view.pending_autonomous_angular_z,
            )
            self._add_text(direction_text, right_x, y0 + 96, 38, '#ffffff', bold=True, max_width=410)
        else:
            self._add_text('GNSS測位', right_x, y0 + 38, 32, '#d8dee9', bold=True, max_width=410)
            self._add_text(self._rtk_text(view.rtk_text), right_x, y0 + 96, 28, '#ffffff', max_width=410)
        self._view.fitInView(self._scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def _manual_resume_wait_text(self, view: DriveStatusView) -> str:
        """手動中の L1 OFF 経過秒と自律復帰待機時間を表示する。"""

        elapsed_s = 0.0 if view.l1_pressed else view.l1_released_elapsed_s
        elapsed_text = str(int(elapsed_s))
        threshold_text = str(int(self._manual_to_auto_l1_released_s))
        if view.autonomous_cmd_alive:
            return f'L1 OFF時間: {elapsed_text} / {threshold_text} 秒'
        return f'L1 OFF時間: {elapsed_text} 秒'

    def _add_mux_status(self, view: DriveStatusView, x: float, y: float) -> None:
        """主表示帯右下に mux の採用元と入力有無を表示する。"""

        lines = [
            f'出力Cmd：{self._mux_output_text(view.output_label)}',
            f'自律Cmd：{self._input_text(view.autonomous_cmd_alive)}',
            f'手動Cmd：{self._input_text(view.manual_cmd_alive)}',
        ]
        self._add_text('\n'.join(lines), x, y, 23, '#ffffff', max_width=390)

    def _add_cmd_slot(
        self,
        title: str,
        x: float,
        y0: float,
        linear_x: float,
        angular_z: float,
        width: float,
    ) -> None:
        """速度値と進行方向矢印を同じレイアウトで描画する。"""

        self._add_text(title, x, y0 + 38, 32, '#d8dee9', bold=True, max_width=width)
        self._add_direction_arrow(x + 72, y0 + 158, 44, linear_x, angular_z, '#ffffff')
        self._add_text(
            f'{linear_x:+.1f} m/s',
            x + 145,
            y0 + 104,
            40,
            '#ffffff',
            bold=True,
            max_width=max(240, width - 145),
        )

    def _planned_direction_text(self, linear_x: float, angular_z: float) -> str:
        """復帰予定 cmd から進行予定方向と注意文を作る。"""

        return DriveStatusGuiCore.planned_direction_text(
            linear_x=linear_x,
            angular_z=angular_z,
            turn_preview_seconds=self._turn_preview_seconds,
            max_linear_x=self._max_autonomous_resume_linear_x,
            max_angular_z=self._max_autonomous_resume_angular_z,
            linear_scale=self._direction_linear_scale,
            angular_scale=self._direction_angular_scale,
            deadzone=self._direction_deadzone,
            linear_axis_invert=self._direction_linear_axis_invert,
            angular_axis_invert=self._direction_angular_axis_invert,
        )

    def _add_direction_arrow(
        self,
        center_x: float,
        center_y: float,
        length: float,
        linear_x: float,
        angular_z: float,
        color: str,
    ) -> None:
        """cmd_vel を stick 座標へ逆変換し、その方向へ矢印を描画する。"""

        QtCore, QtGui, _qtwidgets = self._qt
        dx, dy = DriveStatusGuiCore.direction_vector_from_cmd_vel(
            linear_x=linear_x,
            angular_z=angular_z,
            linear_scale=self._direction_linear_scale,
            angular_scale=self._direction_angular_scale,
            deadzone=self._direction_deadzone,
            linear_axis_invert=self._direction_linear_axis_invert,
            angular_axis_invert=self._direction_angular_axis_invert,
        )
        nx = -dy
        ny = dx
        tip_x = center_x + dx * length
        tip_y = center_y + dy * length
        tail_x = center_x - dx * length * 0.45
        tail_y = center_y - dy * length * 0.45
        head_base_x = center_x + dx * length * 0.48
        head_base_y = center_y + dy * length * 0.48
        shaft_half_width = 7.0
        head_half_width = 24.0
        polygon = QtGui.QPolygonF([
            QtCore.QPointF(tail_x + nx * shaft_half_width, tail_y + ny * shaft_half_width),
            QtCore.QPointF(head_base_x + nx * shaft_half_width, head_base_y + ny * shaft_half_width),
            QtCore.QPointF(head_base_x + nx * head_half_width, head_base_y + ny * head_half_width),
            QtCore.QPointF(tip_x, tip_y),
            QtCore.QPointF(head_base_x - nx * head_half_width, head_base_y - ny * head_half_width),
            QtCore.QPointF(head_base_x - nx * shaft_half_width, head_base_y - ny * shaft_half_width),
            QtCore.QPointF(tail_x - nx * shaft_half_width, tail_y - ny * shaft_half_width),
        ])
        self._scene.addPolygon(
            polygon,
            QtGui.QPen(QtCore.Qt.NoPen),
            QtGui.QBrush(QtGui.QColor(color)),
        )

    def _add_text(
        self,
        text: str,
        x: float,
        y: float,
        size: int,
        color: str,
        bold: bool = False,
        max_width: float | None = None,
        alignment: str = 'left',
    ) -> None:
        QtCore, QtGui, _qtwidgets = self._qt
        item = self._scene.addText(text)
        font = QtGui.QFont('Noto Sans CJK JP', size)
        font.setBold(bold)
        item.setFont(font)
        if max_width is not None:
            item.setTextWidth(max_width)
        if alignment == 'center':
            option = item.document().defaultTextOption()
            option.setAlignment(QtCore.Qt.AlignCenter)
            item.document().setDefaultTextOption(option)
        item.setDefaultTextColor(QtGui.QColor(color))
        item.setPos(x, y)

    @staticmethod
    def _mux_output_text(source_label: str) -> str:
        mapping = {
            'ZERO': 'なし',
            'AUTONOMOUS_CMD': '自律',
            'MANUAL_CMD': '手動',
        }
        return mapping.get(source_label, 'なし')

    @staticmethod
    def _input_text(is_alive: bool) -> str:
        return '入力あり' if is_alive else '入力なし'

    @staticmethod
    def _source_text(source_label: str) -> str:
        mapping = {
            'ZERO': '停止',
            'AUTONOMOUS_CMD': '自律Cmd',
            'MANUAL_CMD': '手動Cmd',
        }
        return mapping.get(source_label, source_label)

    @staticmethod
    def _reason_text(reason: str) -> str:
        mapping = {
            'autonomous_cmd': '自律Cmd入力',
            'autonomous_cmd_timeout': '自律Cmdなし',
            'autonomous_cmd_stale': '復帰Cmd古い',
            'auto_resume_countdown': '復帰待ち',
            'manual_cmd': '手動Cmd入力',
            'manual_cmd_timeout': '手動Cmdなし',
            'manual_l1_released': 'L1離し停止',
            'joy_timeout': 'Joyなし',
        }
        return mapping.get(reason, reason.replace('_', ' '))

    @staticmethod
    def _route_text(text: str) -> str:
        if text.startswith('Waypoint: '):
            body = text.removeprefix('Waypoint: ')
            label, sep, state = body.partition(' state=')
            if sep:
                label_text = label or '-'
                state_text = state or '-'
                return f'WP: {label_text}\n状態: {state_text}'
        if text.startswith('Manager: '):
            return text.replace('Manager: ', '管理: ').replace(' cause=', '\n要因: ')
        return text

    @staticmethod
    def _rtk_text(text: str) -> str:
        if text.startswith('RTK: '):
            return (
                text.removeprefix('RTK: ')
                .replace('state=', '状態: ')
                .replace(' sat=', '\n衛星: ')
                .replace(' heading=', '\n方位: ')
                .replace('heading=', '方位: ')
            )
        return text

    @staticmethod
    def _turn_text(angular_z: float) -> str:
        if abs(angular_z) < 0.03:
            return '直進'
        if angular_z > 0.0:
            return '左旋回'
        return '右旋回'

    @staticmethod
    def _yes_no(value: bool) -> str:
        return 'ON' if value else 'OFF'


def main() -> None:
    rclpy.init()
    core = DriveStatusGuiCore()
    node = DriveStatusRosNode(core)
    node.declare_parameter('main_display_ratio', 0.70)
    node.declare_parameter('turn_preview_seconds', 1.0)
    node.declare_parameter('manual_to_auto_l1_released_s', 1.0)
    node.declare_parameter('max_autonomous_resume_linear_x', 0.8)
    node.declare_parameter('max_autonomous_resume_angular_z', 1.2)
    node.declare_parameter('direction_linear_scale', 1.2)
    node.declare_parameter('direction_angular_scale', 1.5)
    node.declare_parameter('direction_deadzone', 0.05)
    node.declare_parameter('direction_linear_axis_invert', False)
    node.declare_parameter('direction_angular_axis_invert', False)
    main_display_ratio = float(node.get_parameter('main_display_ratio').value)
    turn_preview_seconds = float(node.get_parameter('turn_preview_seconds').value)
    manual_to_auto_l1_released_s = float(node.get_parameter('manual_to_auto_l1_released_s').value)
    max_autonomous_resume_linear_x = float(
        node.get_parameter('max_autonomous_resume_linear_x').value
    )
    max_autonomous_resume_angular_z = float(
        node.get_parameter('max_autonomous_resume_angular_z').value
    )
    direction_linear_scale = float(node.get_parameter('direction_linear_scale').value)
    direction_angular_scale = float(node.get_parameter('direction_angular_scale').value)
    direction_deadzone = float(node.get_parameter('direction_deadzone').value)
    direction_linear_axis_invert = bool(node.get_parameter('direction_linear_axis_invert').value)
    direction_angular_axis_invert = bool(node.get_parameter('direction_angular_axis_invert').value)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    window = DriveStatusWindow(
        node,
        main_display_ratio,
        turn_preview_seconds,
        manual_to_auto_l1_released_s,
        max_autonomous_resume_linear_x,
        max_autonomous_resume_angular_z,
        direction_linear_scale,
        direction_angular_scale,
        direction_deadzone,
        direction_linear_axis_invert,
        direction_angular_axis_invert,
    )
    previous_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, lambda _sig, _frame: window.quit())
    try:
        exit_code = window.run()
    except KeyboardInterrupt:
        exit_code = 0
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
