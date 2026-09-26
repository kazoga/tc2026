"""Non-modal clock warning, visible on every tab in real-robot mode."""
from PyQt5 import QtCore, QtWidgets
from ...core.clock_sync_status import clock_sync_status


class ClockSyncWarning(QtWidgets.QLabel):
    def __init__(self, parent=None, *, reader=clock_sync_status):
        super().__init__(parent)
        self._reader = reader
        self._real = False
        self._had_sync = False
        self.setWordWrap(True)
        self.setContentsMargins(16, 8, 16, 8)
        self.setAccessibleName('時刻同期の警告')
        self.setStyleSheet('background:#fff0c2;color:#653700;font-size:18px;font-weight:bold;')
        self.hide()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self.refresh)
        self._timer.start()

    def set_environment(self, environment):
        real = environment in ('実機', '実機（融合）')
        if real != self._real:
            self._had_sync = False
        self._real = real
        self.refresh()

    def refresh(self):
        if not self._real:
            self.hide()
            return
        ready, reason = self._reader()
        if ready:
            self._had_sync = True
            self.hide()
            return
        title = '時刻同期が失われました' if self._had_sync else '時刻同期が未成立です'
        action = ('共通走行システムの停止状態を確認し、同期復旧後に再起動してください。'
                  if self._had_sync else '共通走行システムは同期成立まで起動を待ちます。')
        self.setText(f'⚠ {title}\n{reason} {action}')
        self.show()
