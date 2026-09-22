"""保存済みルートの選択と形状確認。"""
import math
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets
from icart_bringup.recorded_route import inspect_recorded_route


class RouteSketch(QtWidgets.QWidget):
    def __init__(self, points, parent=None):
        super().__init__(parent)
        self.points = points
        self.setMinimumHeight(240)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor('#f3f7fb'))
        if not self.points:
            return
        lat0, lon0 = self.points[0]
        points = [((lon-lon0)*math.cos(math.radians(lat0)), lat-lat0) for lat, lon in self.points]
        xs, ys = zip(*points)
        scale = min((self.width()-80)/max(max(xs)-min(xs), 1e-8),
                    (self.height()-80)/max(max(ys)-min(ys), 1e-8))
        mapped = [QtCore.QPointF(self.width()/2+(x-(max(xs)+min(xs))/2)*scale,
                                self.height()/2-(y-(max(ys)+min(ys))/2)*scale) for x, y in points]
        painter.setPen(QtGui.QPen(QtGui.QColor('#2563eb'), 3))
        painter.drawPolyline(QtGui.QPolygonF(mapped))
        for index, point in enumerate(mapped):
            painter.setBrush(QtGui.QColor('#16a34a' if index == 0 else '#dc2626' if index == len(mapped)-1 else '#2563eb'))
            painter.drawEllipse(point, 5, 5)
            painter.drawText(point+QtCore.QPointF(8, -8), str(index))
        painter.setPen(QtGui.QColor('#263342'))
        painter.drawText(12, 24, '↑ 北　緑：始点／赤：終点')


class RecordedRoutePicker(QtWidgets.QWidget):
    selected = QtCore.pyqtSignal(str)

    def __init__(self, current='', root=None):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.combo = QtWidgets.QComboBox()
        self.combo.addItem('記録ルートを選択', '')
        root = Path(root) if root else Path.home()/'route_surveys'
        paths = sorted(root.glob('*/survey'), reverse=True)
        if current and Path(current) not in paths:
            paths.insert(0, Path(current))
        details = None
        for path in paths:
            try:
                info = inspect_recorded_route(path)
            except (ValueError, OSError, KeyError):
                continue
            self.combo.addItem(f"{path.parent.name}（{info['count']}点）", str(path))
            if str(path) == current:
                details = info
        self.combo.setCurrentIndex(max(0, self.combo.findData(current)))
        self.combo.currentIndexChanged.connect(lambda _: self.selected.emit(self.combo.currentData()))
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.combo, 1)
        browse = QtWidgets.QPushButton('フォルダを選ぶ…')
        browse.clicked.connect(self.browse)
        row.addWidget(browse)
        layout.addLayout(row)
        label = QtWidgets.QLabel()
        label.setWordWrap(True)
        label.setTextFormat(QtCore.Qt.PlainText)
        if details:
            place = {'inagi': '稲城', 'tsukuba': 'つくば'}[details['site']]
            label.setText(f"場所：{place}　{details['count']}点　始点 {details['start']} → 終点 {details['goal']}\n"
                          f"採取時の場所・原点・実機設定を使用します。\n"
                          f"左右幅が未確認の点：{details['unknown_width']}\n{current}")
        else:
            label.setText('「終了・保存」済みのルートを選択してください。')
        layout.addWidget(label)
        layout.addWidget(RouteSketch(details['points'] if details else []))

    def browse(self):
        path = QtWidgets.QFileDialog.getExistingDirectory(self, 'survey.jsonがある記録フォルダを選択', str(Path.home()/'route_surveys'))
        if not path:
            return
        try:
            inspect_recorded_route(path)
        except (ValueError, OSError, KeyError) as exc:
            QtWidgets.QMessageBox.warning(self, 'ルートを選択できません', str(exc))
            return
        self.selected.emit(path)
