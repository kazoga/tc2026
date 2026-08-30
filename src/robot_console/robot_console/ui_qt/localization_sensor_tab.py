"""自己位置・センサ情報タブ。

ローカルGUIで詳細確認が必要な場合に使う補助画面（robot_console_gui_screen_
function_design.md 7章）。走行中の通常監視はダッシュボード、遠隔または並行
確認はHTML UIを主とするため、本タブには手動介入・起動停止操作は置かない。

運行サマリでダッシュボードと同等の状況把握ができるため、GPS/自己位置の
詳細数値カード（ダッシュボードのGPS/Poseカードと重複する内容）は置かず、
空いた領域を地図表示の拡大に充てる。

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

from .widgets.color_rules import freshness_color
from .widgets.image_panel import ImagePanel
from .widgets.status_card import set_label_color

ROUTE_MAP_PANEL_ID = 'route_map'
GRID_COLUMNS = 2

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

        # GPS/自己位置の詳細カードは置かず（ダッシュボードのGPS/Poseカードと
        # 重複するため）、地図とセンサ・画像パネルへ領域を大きく割り当てる。
        content_row = QtWidgets.QHBoxLayout()
        content_row.addWidget(self._build_route_overlay_panel(), 3)
        content_row.addWidget(self._build_sensor_grid_panel(), 2)
        outer.addLayout(content_row, 1)

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
        # 地図はカメラ画像と異なり、パネル形状に合わせて領域いっぱいに表示する
        # （アスペクト比維持によるレターボックスは行わない）。
        self._route_map_panel = ImagePanel(preserve_aspect_ratio=False)
        layout = QtWidgets.QVBoxLayout(self._route_overlay_group)
        layout.addWidget(self._route_map_panel)
        return self._route_overlay_group

    # ---------- センサ・画像パネル（7.5節） ----------
    def _build_sensor_grid_panel(self) -> QtWidgets.QGroupBox:
        self._grid_group = QtWidgets.QGroupBox('センサ・画像パネル')
        self._grid_layout = QtWidgets.QGridLayout()
        self._grid_group.setLayout(self._grid_layout)
        return self._grid_group

    # ---------- Snapshot反映 ----------
    def update_snapshot(self, snapshot: ConsoleSnapshot) -> None:
        """`ConsoleSnapshot` の内容を反映する。"""

        self._update_summary(snapshot)
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
                # setParent(None) で即座に親子関係を解除し描画から外す。
                # deleteLater() だけでは次のイベントループまで旧Widgetが
                # 画面に残り、新Widgetと重なって表示される。
                widget.setParent(None)
                widget.deleteLater()

        for index, reference in enumerate(panel_references):
            panel = ImagePanel()
            image = self._image_store.get(reference.image_id or reference.panel_id)
            panel.update_panel(reference, image)
            row, column = divmod(index, GRID_COLUMNS)
            self._grid_layout.addWidget(panel, row, column)
