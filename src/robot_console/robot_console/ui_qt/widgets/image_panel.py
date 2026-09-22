"""センサ・画像パネル1枚分の表示Widget（screen_function_design.md 7.5〜7.7節）。

`id/title/type/topic/image/status/updated_at` を持つパネルとして、画像本体が
無い場合はplaceholderを、鮮度が古い場合はSTALE/LOST表示を行う。
"""

from __future__ import annotations

from typing import Optional

from PIL import Image
from PyQt5 import QtCore, QtGui, QtWidgets

from robot_console.core.snapshot_model import ImageReference
from robot_console.utils import format_local_time

from .color_rules import freshness_color

PLACEHOLDER_TEXT = 'No Image'
IMAGE_BACKGROUND = '#202020'
IMAGE_PLACEHOLDER_TEXT_COLOR = '#9e9e9e'


def pil_to_qpixmap(image: Image.Image) -> QtGui.QPixmap:
    """PIL.Image を QPixmap へ変換する。

    `PIL.ImageQt.ImageQt` は環境依存でQtバインディングの自動検出に失敗することが
    あるため、生バイト列からの手動変換を用いる。
    """

    rgba = image.convert('RGBA')
    data = rgba.tobytes('raw', 'RGBA')
    qimage = QtGui.QImage(data, rgba.width, rgba.height, QtGui.QImage.Format_RGBA8888).copy()
    return QtGui.QPixmap.fromImage(qimage)


class ImagePanel(QtWidgets.QFrame):
    """1件のセンサ・画像パネル（`ImageReference` + 画像本体）を表示する。

    カメラ画像やセンサビューアなど、元画像に意味のある固有アスペクト比が
    ある場合は `preserve_aspect_ratio=True`（既定）で歪みなく表示する。
    地図のようにパネル領域いっぱいに表示したい場合は `False` を指定する。

    パネル名とtopic・鮮度・最終更新時刻は同じ見出し行に左右振り分けで置く。
    `QGroupBox` の見出しと状態行を別々に持つと2行分の縦を消費し、その分だけ
    画像領域が狭くなるため、1行へまとめる。
    """

    def __init__(
        self,
        parent: Optional[QtWidgets.QWidget] = None,
        *,
        preserve_aspect_ratio: bool = True,
    ) -> None:
        super().__init__(parent)
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self._preserve_aspect_ratio = preserve_aspect_ratio

        self._title_label = QtWidgets.QLabel('-')
        title_font = self._title_label.font()
        title_font.setBold(True)
        self._title_label.setFont(title_font)
        self._status_label = QtWidgets.QLabel('-')
        self._status_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        header = QtWidgets.QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.addWidget(self._title_label)
        header.addStretch(1)
        header.addWidget(self._status_label)

        self._image_label = QtWidgets.QLabel(PLACEHOLDER_TEXT)
        self._image_label.setAlignment(QtCore.Qt.AlignCenter)
        self._image_label.setMinimumHeight(120)
        self._image_label.setStyleSheet(
            f'background-color: {IMAGE_BACKGROUND}; color: {IMAGE_PLACEHOLDER_TEXT_COLOR};'
        )

        self._layout = QtWidgets.QVBoxLayout(self)
        self._layout.addLayout(header)
        self._layout.addWidget(self._image_label, 1)

        self._pixmap: Optional[QtGui.QPixmap] = None

    def title(self) -> str:
        """パネル名を返す（`QGroupBox.title()` と同じ用途）。"""

        return self._title_label.text()

    def add_footer_widget(self, widget: QtWidgets.QWidget) -> None:
        """画像の下へ補助Widgetを追加する。

        カメラパネルの判定チップのように、そのパネルの画像に対してのみ意味を持つ
        表示を別カードへ分けず、同じ枠内へ収めるために使う。別カードにすると
        見出しと枠の分だけ縦を消費し、画像領域が狭くなる。

        Args:
            widget (QtWidgets.QWidget): 画像の下へ並べるWidget.
        """

        self._layout.addWidget(widget)

    def update_panel(self, reference: ImageReference, image: Optional[Image.Image]) -> None:
        """`ImageReference` メタデータと画像本体（あれば）を反映する。"""

        self._title_label.setText(reference.title or reference.panel_id)

        if image is not None:
            self._pixmap = pil_to_qpixmap(image)
            self._render_pixmap()
        else:
            self._pixmap = None
            self._image_label.setPixmap(QtGui.QPixmap())
            self._image_label.setText(PLACEHOLDER_TEXT)

        # `updated_at` はUTCで保持されるため、表示時のみ日本時間へ変換する。
        updated_text = (
            format_local_time(reference.updated_at) if reference.updated_at else '未受信'
        )
        self._status_label.setText(
            f'{reference.topic or "-"} / {reference.freshness.value} / {updated_text}'
        )
        self._status_label.setStyleSheet(f'color: {freshness_color(reference.freshness)};')

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        self._render_pixmap()

    def _render_pixmap(self) -> None:
        if self._pixmap is None:
            return
        aspect_mode = (
            QtCore.Qt.KeepAspectRatio
            if self._preserve_aspect_ratio
            else QtCore.Qt.IgnoreAspectRatio
        )
        scaled = self._pixmap.scaled(
            self._image_label.size(),
            aspect_mode,
            QtCore.Qt.SmoothTransformation,
        )
        self._image_label.setPixmap(scaled)
