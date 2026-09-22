"""GNSSと基地局の観測画面。接続変更は停止後のセッション設定で行う。"""
from PyQt5 import QtCore, QtWidgets
from ..core.snapshot_model import ConsoleSnapshot
from ..core.gnss_display import gnss_rows
from .widgets.status_card import StatusCard, set_label_color
from .widgets.color_rules import ntrip_color, freshness_color, rtk_state_color


class GnssTab(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        note = QtWidgets.QLabel('GNSS・基地局 ｜ 接続状態と測位状態を個別に確認')
        layout.addWidget(note)
        columns = QtWidgets.QHBoxLayout()
        self.cards, self.fields = [], []
        for title, rows in zip(('基地局 / NTRIP', 'UM982 / GNSS受信'), gnss_rows(ConsoleSnapshot())):
            card = StatusCard(title)
            fields = []
            for key, _ in rows:
                label = card.add_value_row(key)
                label.setTextFormat(QtCore.Qt.PlainText)
                label.setWordWrap(True)
                label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                fields.append(label)
            columns.addWidget(card, 1)
            self.cards.append(card)
            self.fields.append(fields)
        layout.addLayout(columns)
        footer = QtWidgets.QLabel(
            'RTCMはCRC確認済みの受信量。受信中でもRTK FIXとは限りません。\n'
            'GNSS方位はアンテナ間の北基準・時計回り。車体の融合yawはダッシュボードに表示します。\n'
            '基地局の選択変更は停止後、実機セッションを再生成して再起動してください。')
        footer.setWordWrap(True)
        layout.addWidget(footer)
        layout.addStretch(1)
        self.update_snapshot(ConsoleSnapshot())

    def update_snapshot(self, snapshot):
        for labels, rows in zip(self.fields, gnss_rows(snapshot)):
            for label, (_, value) in zip(labels, rows):
                label.setText(value)
        set_label_color(self.fields[0][0], ntrip_color(snapshot.ntrip_state))
        set_label_color(self.fields[1][0], freshness_color(snapshot.gps_state.status_freshness))
        set_label_color(self.fields[1][1], rtk_state_color(
            snapshot.gps_state.rtk_state, snapshot.gps_state.status_freshness))
