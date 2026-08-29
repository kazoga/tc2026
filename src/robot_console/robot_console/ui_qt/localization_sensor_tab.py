"""自己位置・センサ情報タブ。

ローカルGUIで詳細確認が必要な場合に使う補助画面（robot_console_gui_screen_
function_design.md 7章）。走行中の通常監視はダッシュボード、遠隔または並行
確認はHTML UIを主とするため、本タブには手動介入・起動停止操作は置かない。

地図表示は `robot_console_ui_renewal_input.md` 8.4節の候補C（初期実装では
既存route map画像を表示し、後続フェーズでOSM重畳へ発展させる）を採用する。
OpenStreetMap上へのwaypoint重畳は本フェーズの対象外である。
"""

from __future__ import annotations

from typing import Dict, List, Optional

from PyQt5 import QtCore, QtWidgets

from robot_console.core.freshness import FreshnessLevel
from robot_console.core.image_store import ImageStore
from robot_console.core.snapshot_model import ConsoleSnapshot, ImageReference

from .widgets.color_rules import freshness_color, rtk_state_color
from .widgets.image_panel import ImagePanel
from .widgets.status_card import StatusCard, set_label_color

ROUTE_MAP_PANEL_ID = 'route_map'
GRID_COLUMNS = 3

# screen_function_design.md 7.5節の初期候補パネル（route_mapを除く）。
# 実データ未接続の間は、パネル構成を示すための既定値として使う。
DEFAULT_SENSOR_PANELS: List[ImageReference] = [
    ImageReference(panel_id='sensor_viewer', title='Sensor Viewer', topic='/sensor_viewer'),
    ImageReference(
        panel_id='road_blockage',
        title='Road Blockage',
        topic='/perception/road_blockage/decision_image',
    ),
    ImageReference(
        panel_id='traffic_signal',
        title='Traffic Signal',
        topic='/perception/traffic_signal/decision_image',
    ),
    ImageReference(panel_id='front_camera', title='Front Camera', topic=''),
    ImageReference(panel_id='lidar_view', title='LiDAR View', topic=''),
]


class LocalizationSensorTab(QtWidgets.QWidget):
    """自己位置・センサ情報の詳細確認画面。"""

    back_to_dashboard_requested = QtCore.pyqtSignal()

    def __init__(
        self,
        image_store: Optional[ImageStore] = None,
        parent: Optional[QtWidgets.QWidget] = None,
    ) -> None:
        super().__init__(parent)

        self._image_store = image_store or ImageStore()

        outer = QtWidgets.QVBoxLayout(self)
        outer.addLayout(self._build_header())
        outer.addWidget(self._build_summary_panel())

        row = QtWidgets.QHBoxLayout()
        row.addWidget(self._build_route_overlay_panel(), 2)
        row.addWidget(self._build_localization_gps_card(), 1)
        outer.addLayout(row, 1)

        self._grid_group = QtWidgets.QGroupBox('センサ・画像パネル')
        self._grid_layout = QtWidgets.QGridLayout()
        self._grid_group.setLayout(self._grid_layout)
        outer.addWidget(self._grid_group, 1)

        self.update_snapshot(ConsoleSnapshot())

    def _build_header(self) -> QtWidgets.QHBoxLayout:
        row = QtWidgets.QHBoxLayout()
        back_button = QtWidgets.QPushButton('ダッシュボードへ戻る')
        back_button.clicked.connect(self.back_to_dashboard_requested.emit)
        row.addWidget(back_button)
        row.addStretch(1)
        return row

    # ---------- 運行サマリ（7.3節） ----------
    def _build_summary_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox('運行サマリ')
        self._phase_label = QtWidgets.QLabel('-')
        self._follower_state_label = QtWidgets.QLabel('-')
        self._waypoint_label = QtWidgets.QLabel('-')
        self._progress_label = QtWidgets.QLabel('-')
        self._gps_state_label = QtWidgets.QLabel('-')
        self._localization_freshness_label = QtWidgets.QLabel('-')
        self._sensor_freshness_label = QtWidgets.QLabel('-')

        layout = QtWidgets.QHBoxLayout(group)
        for title, label in (
            ('運行フェーズ', self._phase_label),
            ('follower', self._follower_state_label),
            ('WP', self._waypoint_label),
            ('進捗', self._progress_label),
            ('GPS', self._gps_state_label),
            ('Localization', self._localization_freshness_label),
            ('Sensor', self._sensor_freshness_label),
        ):
            layout.addWidget(QtWidgets.QLabel(f'{title}:'))
            layout.addWidget(label)
        layout.addStretch(1)
        return group

    # ---------- 地図 / route overlay（7.2節左カラム、7.4節） ----------
    def _build_route_overlay_panel(self) -> QtWidgets.QGroupBox:
        self._route_overlay_group = QtWidgets.QGroupBox('地図 / Route Overlay')
        self._route_map_panel = ImagePanel()
        layout = QtWidgets.QVBoxLayout(self._route_overlay_group)
        layout.addWidget(self._route_map_panel)
        return self._route_overlay_group

    # ---------- 自己位置・GPS詳細（7.2節右カラム） ----------
    def _build_localization_gps_card(self) -> StatusCard:
        card = StatusCard('自己位置・GPS詳細')
        self._loc_source_label = card.add_value_row('source')
        self._loc_position_label = card.add_value_row('position')
        self._loc_yaw_label = card.add_value_row('yaw')
        self._loc_freshness_label = card.add_value_row('pose freshness')
        self._gps_rtk_label = card.add_value_row('RTK')
        self._gps_satellites_label = card.add_value_row('Satellites')
        self._gps_hdop_label = card.add_value_row('HDOP')
        self._gps_correction_label = card.add_value_row('Correction')
        self._gps_heading_label = card.add_value_row('Heading')
        self._target_distance_label = card.add_value_row('目標距離')
        return card

    # ---------- Snapshot反映 ----------
    def update_snapshot(self, snapshot: ConsoleSnapshot) -> None:
        """`ConsoleSnapshot` の内容を反映する。"""

        self._update_summary(snapshot)
        self._update_localization_gps_card(snapshot)
        self._update_sensor_panels(snapshot)

    def _update_summary(self, snapshot: ConsoleSnapshot) -> None:
        operation = snapshot.operation_state
        self._phase_label.setText(operation.phase)
        self._follower_state_label.setText(snapshot.follower_state.state)
        self._waypoint_label.setText(
            f'{operation.current_waypoint or "-"} -> {operation.next_waypoint or "-"}'
        )
        self._progress_label.setText(f'{operation.route_progress * 100.0:.1f}%')
        self._gps_state_label.setText(snapshot.gps_state.rtk_state)
        self._localization_freshness_label.setText(snapshot.localization_state.freshness.value)
        set_label_color(
            self._localization_freshness_label,
            freshness_color(snapshot.localization_state.freshness),
        )

        sensor_summary = self._summarize_freshness(
            [panel.freshness for panel in snapshot.sensor_panels]
        )
        self._sensor_freshness_label.setText(sensor_summary)

    @staticmethod
    def _summarize_freshness(levels: List[FreshnessLevel]) -> str:
        if not levels:
            return '-'
        if any(level == FreshnessLevel.LOST for level in levels):
            return 'LOST'
        if any(level == FreshnessLevel.STALE for level in levels):
            return 'STALE'
        if all(level == FreshnessLevel.OK for level in levels):
            return 'OK'
        return 'UNKNOWN'

    def _update_localization_gps_card(self, snapshot: ConsoleSnapshot) -> None:
        localization = snapshot.localization_state
        gps = snapshot.gps_state
        target = snapshot.target_state

        self._loc_source_label.setText(localization.source)
        self._loc_position_label.setText(self._format_position(localization))
        self._loc_yaw_label.setText(
            f'{localization.yaw_deg:.1f} deg' if localization.yaw_deg is not None else '-'
        )
        self._loc_freshness_label.setText(localization.freshness.value)
        set_label_color(self._loc_freshness_label, freshness_color(localization.freshness))

        self._gps_rtk_label.setText(gps.rtk_state)
        set_label_color(self._gps_rtk_label, rtk_state_color(gps.rtk_state, gps.fix_freshness))
        self._gps_satellites_label.setText(f'{gps.num_satellites} sat')
        self._gps_hdop_label.setText(f'{gps.hdop:.2f}')
        self._gps_correction_label.setText(f'{gps.correction_age_s:.2f} s')
        self._gps_heading_label.setText(
            f'{gps.heading_deg:.1f} deg +/- {gps.heading_stddev_deg:.2f}'
        )

        arrival_text = '到達' if target.within_arrival_threshold else '未到達'
        self._target_distance_label.setText(f'{target.distance_m:.2f} m ({arrival_text})')

    @staticmethod
    def _format_position(localization) -> str:
        if localization.x_m is not None and localization.y_m is not None:
            return f'x={localization.x_m:.2f} y={localization.y_m:.2f}'
        if localization.latitude is not None and localization.longitude is not None:
            return f'lat={localization.latitude:.6f} lon={localization.longitude:.6f}'
        return '-'

    def _update_sensor_panels(self, snapshot: ConsoleSnapshot) -> None:
        panels = list(snapshot.sensor_panels) or list(DEFAULT_SENSOR_PANELS)
        panels_by_id: Dict[str, ImageReference] = {panel.panel_id: panel for panel in panels}

        route_map_reference = panels_by_id.pop(
            ROUTE_MAP_PANEL_ID,
            ImageReference(panel_id=ROUTE_MAP_PANEL_ID, title='Route Map', topic='/active_route'),
        )
        self._route_map_panel.update_panel(
            route_map_reference,
            self._image_store.get(route_map_reference.image_id or ROUTE_MAP_PANEL_ID),
        )

        self._rebuild_grid(list(panels_by_id.values()))

    def _rebuild_grid(self, panel_references: List[ImageReference]) -> None:
        while self._grid_layout.count():
            item = self._grid_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        for index, reference in enumerate(panel_references):
            panel = ImagePanel()
            image = self._image_store.get(reference.image_id or reference.panel_id)
            panel.update_panel(reference, image)
            row, column = divmod(index, GRID_COLUMNS)
            self._grid_layout.addWidget(panel, row, column)
