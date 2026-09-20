"""PC版ダッシュボードのrosbag記録操作。"""
from pathlib import Path
from PyQt5 import QtCore, QtWidgets
from ...core.bag_recorder import BagState

LABELS = dict(IDLE='停止中', STARTING='記録開始中', RECORDING='● 記録中',
              STOPPING='保存処理中', SAVED='保存完了', ERROR='エラー')


class BagCard(QtWidgets.QGroupBox):
    start_requested = QtCore.pyqtSignal(str)
    stop_requested = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__('ROSBAG保存')
        self.setToolTip('全通常トピックを記録します')
        self._initialized = False
        layout = QtWidgets.QVBoxLayout(self)
        row = QtWidgets.QHBoxLayout()
        self.status = QtWidgets.QLabel('停止中')
        self.status.setWordWrap(True)
        self.start_button = QtWidgets.QPushButton('記録開始')
        self.stop_button = QtWidgets.QPushButton('停止・保存')
        self.start_button.clicked.connect(lambda: self.start_requested.emit(self.directory.text().strip()))
        self.stop_button.clicked.connect(self.stop_requested.emit)
        row.addWidget(self.status, 1)
        row.addWidget(self.start_button)
        row.addWidget(self.stop_button)
        layout.addLayout(row)
        path_row = QtWidgets.QHBoxLayout()
        path_row.addWidget(QtWidgets.QLabel('保存先'))
        self.directory = QtWidgets.QLineEdit()
        self.directory.setPlaceholderText('~/rosbags')
        self.choose = QtWidgets.QPushButton('変更…')
        self.choose.clicked.connect(self._choose_directory)
        path_row.addWidget(self.directory, 1)
        path_row.addWidget(self.choose)
        layout.addLayout(path_row)
        self.output = QtWidgets.QLabel('')
        self.output.setTextFormat(QtCore.Qt.PlainText)
        self.output.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.output.setWordWrap(True)
        layout.addWidget(self.output)
        self.update_state(BagState())

    def _choose_directory(self):
        path = QtWidgets.QFileDialog.getExistingDirectory(self, 'ROSBAG保存先', self.directory.text())
        if path:
            self.directory.setText(path)

    def update_state(self, state):
        active = state.state in ('STARTING', 'RECORDING', 'STOPPING')
        self.start_button.setEnabled(not active)
        self.stop_button.setEnabled(state.state in ('STARTING', 'RECORDING'))
        self.directory.setEnabled(not active)
        self.choose.setEnabled(not active)
        if not self._initialized and state.directory:
            self.directory.setText(state.directory)
            self._initialized = True
        seconds = int(state.elapsed_s)
        self.status.setText(f'{LABELS.get(state.state, state.state)}  |  '
                            f'{seconds//60:02d}:{seconds%60:02d}  |  '
                            f'{state.size_bytes/1024**2:.1f} MiB')
        self.status.setToolTip(f'空き容量: {state.free_bytes/1024**3:.1f} GiB' if state.free_bytes else '')
        self.status.setStyleSheet('color: '+{'RECORDING':'#c62828', 'ERROR':'#c62828',
            'SAVED':'#2e7d32'}.get(state.state, '#555555'))
        name = Path(state.output).name if state.output else ''
        self.output.setText('  |  '.join(x for x in (name, state.message) if x))
        self.output.setToolTip('\n'.join(x for x in (state.output, state.message) if x))
