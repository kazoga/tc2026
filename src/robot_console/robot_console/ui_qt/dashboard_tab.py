"""ダッシュボードタブ。

走行前確認と走行中監視の主画面（robot_console_gui_screen_function_design.md 6章）。
運行状態の詳細確認と手動介入操作をこのタブへ集約し、共通ステータスバーは設けない。
`update_snapshot(ConsoleSnapshot)` を通じて `robot_console.core` が生成するSnapshotの
みを参照し、ROSメッセージ型を直接解釈しない。
"""

from __future__ import annotations

from ..core.gnss_display import ntrip_summary, FRESH
from ..core.freshness import FreshnessLevel

from typing import Optional

from PyQt5 import QtWidgets

from robot_console.core.route_adapter import traveled_waypoint_count
from robot_console.core.snapshot_model import ConsoleSnapshot

from .widgets.color_rules import freshness_color, phase_color, rtk_state_color, ntrip_color
from .widgets.event_banner_card import EventBannerCard, sort_by_priority
from .widgets.launch_control_card import LaunchControlCard
from .widgets.manual_ops_card import ManualOpsCard
from .widgets.node_health_card import NodeHealthCard
from .widgets.status_card import StatusCard, set_label_color
from .widgets.bag_card import BagCard
from .widgets.survey_card import SurveyCard
from .widgets.typography import PHASE_DETAIL_FONT_POINT_SIZE, PHASE_STATUS_FONT_POINT_SIZE


class DashboardTab(QtWidgets.QWidget):
    """走行前確認・走行中監視の主画面（screen_function_design.md 6.2節レイアウト）。"""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)

        self._build_phase_header()
        self.launch_control_card = LaunchControlCard()
        self.route_follower_card = self._build_route_follower_card()
        self.drive_cmd_vel_card = self._build_drive_cmd_vel_card()
        self.gps_pose_card = self._build_gps_pose_card()
        self.event_banner_card = EventBannerCard()
        self.manual_ops_card = ManualOpsCard()
        self.node_health_card = NodeHealthCard()
        self.survey_card = SurveyCard()
        self.bag_card = BagCard()
        self.drive_cmd_vel_card.form_layout.addRow(self.bag_card)

        # 右列（起動操作→Node Health）と「操作が上・状態表示が下」の並びを
        # 揃えるため、左列もManual Ops（操作）を上、Event（状態表示）を下に
        # 縦積みする。EventはHISTORY表示までで内容量が決まり大きな領域を
        # 必要としないため、余白はManual Opsの操作領域に充てる。
        event_and_manual_ops_column = QtWidgets.QWidget()
        event_and_manual_ops_layout = QtWidgets.QVBoxLayout(event_and_manual_ops_column)
        event_and_manual_ops_layout.setContentsMargins(0, 0, 0, 0)
        event_and_manual_ops_layout.addWidget(self.manual_ops_card, 1)
        event_and_manual_ops_layout.addWidget(self.event_banner_card)

        # 起動操作カードはノード起動・停止の操作、Node Healthカードはその結果
        # としての稼働状況表示であり、機能的に対になる。そのため両者を同じ列に
        # 縦積みし、起動操作カードを常に見えるコンパクトな高さに保ちつつ、
        # 空いた高さはNode Healthカードのチップ表示領域に充てる。
        launch_and_health_column = QtWidgets.QWidget()
        launch_and_health_layout = QtWidgets.QVBoxLayout(launch_and_health_column)
        launch_and_health_layout.setContentsMargins(0, 0, 0, 0)
        launch_and_health_layout.addWidget(self.launch_control_card)
        launch_and_health_layout.addWidget(self.node_health_card, 1)

        grid = QtWidgets.QGridLayout(self)
        self.gnss_dropout_alert = QtWidgets.QLabel()
        self.gnss_dropout_alert.setWordWrap(True)
        self.gnss_dropout_alert.setMinimumHeight(40)
        grid.addWidget(self.gnss_dropout_alert, 0, 0, 1, 3)
        grid.addWidget(self._phase_header_group, 1, 0, 1, 3)
        grid.addWidget(self.route_follower_card, 2, 0)
        grid.addWidget(self.drive_cmd_vel_card, 2, 1)
        grid.addWidget(self.gps_pose_card, 2, 2)
        grid.addWidget(event_and_manual_ops_column, 3, 0)
        grid.addWidget(launch_and_health_column, 3, 1, 1, 2)
        for column in range(3):
            grid.setColumnStretch(column, 1)
        grid.setRowStretch(3, 1)
        grid.setRowStretch(2, 1)

        self.update_snapshot(ConsoleSnapshot())

    # ---------- 運行フェーズ領域（6.3節） ----------
    def _build_phase_header(self) -> None:
        """運行フェーズを最重要の一目情報として大きく表示するヘッダーを構築する。

        走行中は数m離れた位置から状態を確認する運用を想定し、フェーズ本体を
        大きな色付き文字で表示する。業務モードや進捗などの詳細は、その下に
        小さめの1行にまとめる（近づいて確認する前提）。
        """

        self._phase_header_group = QtWidgets.QGroupBox('運行フェーズ')

        self._phase_label = QtWidgets.QLabel('-')
        phase_font = self._phase_label.font()
        phase_font.setPointSize(PHASE_STATUS_FONT_POINT_SIZE)
        phase_font.setBold(True)
        self._phase_label.setFont(phase_font)

        detail_font = self._phase_label.font()
        detail_font.setPointSize(PHASE_DETAIL_FONT_POINT_SIZE)
        detail_font.setBold(False)

        self._environment_label = QtWidgets.QLabel('-')
        self._progress_label = QtWidgets.QLabel('-')
        self._waypoint_label = QtWidgets.QLabel('-')
        self._manual_start_label = QtWidgets.QLabel('-')
        self._pause_reason_label = QtWidgets.QLabel('-')
        self._top_event_label = QtWidgets.QLabel('-')

        detail_row = QtWidgets.QHBoxLayout()
        for title, value_label in (
            ('業務モード', self._environment_label),
            ('進捗', self._progress_label),
            ('WP', self._waypoint_label),
            ('manual_start', self._manual_start_label),
            ('停止理由', self._pause_reason_label),
            ('重要イベント', self._top_event_label),
        ):
            title_label = QtWidgets.QLabel(f'{title}:')
            title_label.setFont(detail_font)
            value_label.setFont(detail_font)
            detail_row.addWidget(title_label)
            detail_row.addWidget(value_label)
        detail_row.addStretch(1)

        layout = QtWidgets.QVBoxLayout(self._phase_header_group)
        layout.addWidget(self._phase_label)
        layout.addLayout(detail_row)

    # ---------- Route / Followerカード（6.4節） ----------
    def _build_route_follower_card(self) -> StatusCard:
        card = StatusCard('Route / Follower')
        self._route_state_label = card.add_value_row('route_manager 状態')
        self._route_version_label = card.add_value_row('route_version')
        self._route_decision_label = card.add_value_row('最終decision')
        self._route_replan_reason_label = card.add_value_row('再計画理由')
        self._follower_state_label = card.add_value_row('route_follower 状態')
        self._follower_waypoint_label = card.add_value_row('現在waypoint')
        self._follower_stagnation_label = card.add_value_row('滞留理由')
        self._route_progress_label = card.add_value_row('進捗')
        self._target_distance_label = card.add_value_row('目標距離')
        return card

    # ---------- Drive / CmdVelカード（6.5節） ----------
    def _build_drive_cmd_vel_card(self) -> StatusCard:
        card = StatusCard('Drive / CmdVel')
        self._drive_mode_label = card.add_value_row('drive mode')
        self._drive_output_source_label = card.add_value_row('output source')
        self._drive_auto_resume_label = card.add_value_row('auto resume pending')
        self._cmd_vel_label = card.add_value_row('cmd_vel')
        self._cmd_vel_autonomous_label = card.add_value_row('cmd_vel/autonomous')
        self._odom_label = card.add_value_row('odom')
        return card

    # ---------- GPS / Poseカード（6.6節） ----------
    def _build_gps_pose_card(self) -> StatusCard:
        card = StatusCard('GPS / Pose')
        self._ntrip_label = card.add_value_row('基地局 / RTCM')
        self._ntrip_mount_label = card.add_value_row('接続マウント')
        self._gps_rtk_label = card.add_value_row('RTK')
        self._gps_satellites_label = card.add_value_row('Satellites')
        self._gps_hdop_label = card.add_value_row('HDOP')
        self._gps_correction_label = card.add_value_row('Correction')
        self._gps_rtcm_label = card.add_value_row('RTCM')
        self._gps_heading_label = card.add_value_row('GNSS heading（北CW）')
        self._fusion_heading_label = card.add_value_row('融合yaw（map CCW）')
        self._fusion_quality_label = card.add_value_row('融合品質 / 基準距離')
        self._localization_source_label = card.add_value_row('Localization source')
        self._pose_freshness_label = card.add_value_row('Pose freshness')
        return card

    # ---------- Snapshot反映 ----------
    def update_snapshot(self, snapshot: ConsoleSnapshot) -> None:
        """`ConsoleSnapshot` の内容を各カードへ反映する。"""

        self.bag_card.update_state(snapshot.bag_state)
        self.survey_card.update_state(snapshot.survey_state)
        self._update_phase_header(snapshot)
        self._update_route_follower_card(snapshot)
        self._update_drive_cmd_vel_card(snapshot)
        self._update_gps_pose_card(snapshot)
        self.event_banner_card.update_snapshot(snapshot.event_banners)
        self.manual_ops_card.update_snapshot(snapshot.manual_controls)
        self.node_health_card.update_snapshot(snapshot.health)

    def _update_phase_header(self, snapshot: ConsoleSnapshot) -> None:
        operation = snapshot.operation_state
        self._environment_label.setText(f'{operation.environment} / {operation.drive_mode}')
        self._phase_label.setText(operation.phase)
        set_label_color(self._phase_label, phase_color(operation.phase))
        self._progress_label.setText(f'{operation.route_progress * 100.0:.1f}%')
        self._waypoint_label.setText(
            f'{operation.current_waypoint or "-"} -> {operation.next_waypoint or "-"}'
        )
        self._manual_start_label.setText(str(operation.manual_start))
        self._pause_reason_label.setText(operation.pause_reason or '-')

        ordered_events = sort_by_priority(snapshot.event_banners)
        self._top_event_label.setText(ordered_events[0].message if ordered_events else 'なし')

    def _update_route_follower_card(self, snapshot: ConsoleSnapshot) -> None:
        route = snapshot.route_state
        follower = snapshot.follower_state
        target = snapshot.target_state

        self._route_state_label.setText(route.state)
        self._route_version_label.setText(str(route.route_version))
        self._route_decision_label.setText(route.last_decision or '-')
        self._route_replan_reason_label.setText(route.last_replan_reason or '-')
        self._follower_state_label.setText(follower.state)
        self._follower_waypoint_label.setText(
            f'{follower.active_waypoint_label or "-"} (#{follower.active_waypoint_index})'
        )
        self._follower_stagnation_label.setText(follower.stagnation_reason or '-')
        self._route_progress_label.setText(
            f'{traveled_waypoint_count(route)}/{route.total_waypoints} '
            f'({route.progress_ratio * 100.0:.1f}%)'
        )
        arrival_text = '到達' if target.within_arrival_threshold else '未到達'
        self._target_distance_label.setText(f'{target.distance_m:.2f} m ({arrival_text})')

    def _update_drive_cmd_vel_card(self, snapshot: ConsoleSnapshot) -> None:
        drive = snapshot.drive_mode_state
        self._drive_mode_label.setText(drive.mode)
        self._drive_output_source_label.setText(drive.output_source or '-')
        self._drive_auto_resume_label.setText(str(drive.auto_resume_pending))
        self._cmd_vel_label.setText(
            f'{drive.cmd_vel_linear_mps:.2f} m/s / {drive.cmd_vel_angular_dps:.1f} deg/s'
        )
        set_label_color(self._cmd_vel_label, freshness_color(drive.cmd_vel_freshness))
        if drive.cmd_vel_autonomous_linear_mps is None:
            self._cmd_vel_autonomous_label.setText('-')
        else:
            self._cmd_vel_autonomous_label.setText(
                f'{drive.cmd_vel_autonomous_linear_mps:.2f} m/s / '
                f'{drive.cmd_vel_autonomous_angular_dps:.1f} deg/s'
            )
        self._odom_label.setText(drive.odom_topic or '-')
        set_label_color(self._odom_label, freshness_color(drive.odom_freshness))

    def _update_gps_pose_card(self, snapshot: ConsoleSnapshot) -> None:
        gps = snapshot.gps_state
        localization = snapshot.localization_state

        ntrip = snapshot.ntrip_state
        self._ntrip_label.setText(ntrip_summary(ntrip))
        set_label_color(self._ntrip_label, ntrip_color(ntrip))
        self._ntrip_mount_label.setText(ntrip.mountpoint or '—')
        self._gps_rtk_label.setText(gps.rtk_state if gps.status_freshness == FreshnessLevel.OK
                                    else FRESH[gps.status_freshness])
        set_label_color(self._gps_rtk_label, rtk_state_color(gps.rtk_state, gps.fix_freshness))
        self._gps_satellites_label.setText(f'{gps.num_satellites} sat')
        self._gps_hdop_label.setText(f'{gps.hdop:.2f}')
        self._gps_correction_label.setText(f'{gps.correction_age_s:.2f} s')
        self._gps_rtcm_label.setText(f'{gps.rtcm_bytes_received} B')
        self._gps_heading_label.setText(
            f'{gps.heading_deg:.1f} deg +/- {gps.heading_stddev_deg:.2f}'
        )
        if gps.status_freshness == FreshnessLevel.UNKNOWN:
            for label in (self._gps_satellites_label, self._gps_hdop_label,
                          self._gps_correction_label, self._gps_rtcm_label, self._gps_heading_label):
                label.setText('—')
        dropout = snapshot.gnss_dropout_state
        stale = dropout.freshness.value in ('STALE', 'LOST')
        self.gnss_dropout_alert.setVisible(dropout.active or stale)
        self.gnss_dropout_alert.setText(
            'GNSS途絶模擬：状態未確認（通知が途絶えています）' if stale else
            'GNSS途絶模擬中 — R1を離すと解除 / 融合へのGNSS位置・方位入力を遮断')
        self.gnss_dropout_alert.setStyleSheet(
            'background:#fff3cd;color:#664d03;padding:8px;font-size:16px;font-weight:bold;' if stale else
            'background:#b91c1c;color:white;padding:8px;font-size:16px;font-weight:bold;')
        fusion = snapshot.fusion_state
        fusion_mode = {'WAIT_INITIAL_FIX': '初期FIX待ち',
                       'WAIT_GRAVITY_ALIGNMENT': '水平基準待ち（約2秒静止）'}.get(fusion.mode, fusion.mode)
        self._fusion_heading_label.setText(
            '-' if fusion.yaw_deg is None else
            f'{fusion.yaw_deg:.1f}° / 推定σ {fusion.heading_sigma_deg:.2f}°')
        self._fusion_quality_label.setText(
            f'{fusion_mode} / {fusion.freshness.value}' if fusion.baseline_m is None else
            f'{fusion_mode} / {fusion.baseline_m*1000:.1f} mm / {fusion.freshness.value}')
        set_label_color(self._fusion_quality_label, freshness_color(fusion.freshness))
        self._localization_source_label.setText(localization.source)
        self._pose_freshness_label.setText(localization.freshness.value)
        set_label_color(self._pose_freshness_label, freshness_color(localization.freshness))
