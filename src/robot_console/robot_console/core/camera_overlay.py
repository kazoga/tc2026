"""カメラ生画像へ認識結果を重畳するモジュール。

認識ノード（`traffic_signal_recognizer` / `road_blockage_detector`）は重畳画像を
配信せず、`tc_perception_msgs/PerceptionOverlay` として検出データと判定結果だけを
配信する。本モジュールはそれを生画像へ描画し、Qt UI と HTML UI が同一の描画結果を
共有できるようにする（`ConsoleCore` 配下に置くことで実装を 1 つに保つ）。

## 認識レートが生画像より低いことへの対応

認識処理は生画像より低いレートで動作し得るため、生画像の全フレームに対して
`PerceptionOverlay` が対応するとは限らない。対応するフレームにだけ描画すると
検出枠がほとんどの時間表示されず、ちらついて見える。そのため本モジュールは
直近の認識結果を保持し、以降の生画像フレームにも描き続ける。

保持には次の 3 つの時間しきい値を使う。

* `hold_sec`: この時間更新が無ければ結果を破棄し、描画しない。
* `clear_grace_sec`: 検出 0 件の結果を受け取っても、直前の検出をこの時間は残す。
  認識が 1 回検出を取りこぼしただけで枠が消えて再表示される点滅を防ぐ。
* `sync_tolerance_sec`: 認識結果の元フレームと描画対象フレームの時刻差がこれを
  超える場合、枠を破線で描き「認識が追随できていない」ことを示す。古い結果を
  実線で描くと、現在の状況を表していると誤解させるため。推論周期と推論遅延の
  ぶんは正常動作でも必ずずれるので、しきい値はその数倍に取る。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from PIL import Image, ImageDraw

from ..utils.time_utils import now as utc_now

# 認識種別ごとの描画色。判定に採用された検出と採用外で明度を変え、
# 採用外の検出が判定根拠だと誤解されないようにする。
SOURCE_COLORS: Dict[str, Tuple[int, int, int]] = {
    'traffic_signal': (255, 214, 0),
    'road_blockage': (255, 82, 82),
}
DEFAULT_COLOR: Tuple[int, int, int] = (128, 222, 234)
UNADOPTED_COLOR: Tuple[int, int, int] = (150, 150, 150)

DEFAULT_HOLD_SEC = 1.5
DEFAULT_CLEAR_GRACE_SEC = 0.6
# YOLO の推論間隔 (yolo_detector の detection_interval 既定 0.5 秒、信号認識用の
# launch では 0.2 秒) と推論遅延があるため、正常動作でも認識結果は描画フレームより
# 数百 ms 古い。しきい値をその範囲に置くと常時破線になり警告として機能しないので、
# 推論周期の数倍を取り、検出器が実際に停滞したときだけ破線になるようにする。
DEFAULT_SYNC_TOLERANCE_SEC = 1.0

# 破線時に付ける印。PIL の既定フォントは日本語グリフを持たず、日本語を描くと
# 豆腐表示になるため、画像へ焼き込む文字は ASCII に限定する。
STALE_LABEL_SUFFIX = ' [stale]'

ADOPTED_LINE_WIDTH = 3
UNADOPTED_LINE_WIDTH = 1
DASH_LENGTH = 8


@dataclass
class OverlayDetectionView:
    """画像へ重畳する検出 1 件（`tc_perception_msgs/OverlayDetection` 相当）。"""

    center_x: float = 0.0
    center_y: float = 0.0
    size_x: float = 0.0
    size_y: float = 0.0
    label: str = ''
    score: float = 0.0
    adopted: bool = False

    def to_xyxy(self) -> Tuple[int, int, int, int]:
        """中心座標・大きさを描画用の左上／右下座標へ変換する.

        Returns:
            Tuple[int, int, int, int]: (x1, y1, x2, y2) のピクセル座標.
        """

        half_w = self.size_x / 2.0
        half_h = self.size_y / 2.0
        return (
            int(self.center_x - half_w),
            int(self.center_y - half_h),
            int(self.center_x + half_w),
            int(self.center_y + half_h),
        )


@dataclass
class PerceptionOverlayView:
    """1 認識種別分の重畳データ（`tc_perception_msgs/PerceptionOverlay` 相当）。"""

    source: str = ''
    detections: List[OverlayDetectionView] = field(default_factory=list)
    decision: int = 0
    decision_text: str = ''
    status_note: str = ''
    # 判定根拠となった元画像フレームの時刻。生画像フレームとの対応付けに使う。
    frame_stamp: Optional[float] = None
    # 本ConsoleCoreが受信した時刻。保持期間の判定に使う。
    received_at: Optional[datetime] = None


@dataclass
class _HeldOverlay:
    """保持中の重畳データと、直前に描画対象とした検出。"""

    overlay: PerceptionOverlayView
    # 検出 0 件の結果を受けたあとも clear_grace_sec の間は描き続ける検出。
    retained_detections: List[OverlayDetectionView] = field(default_factory=list)
    retained_until: Optional[datetime] = None


class CameraOverlayRenderer:
    """認識結果を保持し、カメラ生画像へ重畳する。"""

    def __init__(
        self,
        *,
        hold_sec: float = DEFAULT_HOLD_SEC,
        clear_grace_sec: float = DEFAULT_CLEAR_GRACE_SEC,
        sync_tolerance_sec: float = DEFAULT_SYNC_TOLERANCE_SEC,
    ) -> None:
        self._hold_sec = hold_sec
        self._clear_grace_sec = clear_grace_sec
        self._sync_tolerance_sec = sync_tolerance_sec
        self._held: Dict[str, _HeldOverlay] = {}

    def update_overlay(self, overlay: PerceptionOverlayView) -> None:
        """認識結果を保持する（受信のたびに呼ぶ）。

        Args:
            overlay (PerceptionOverlayView): 受信した重畳データ.
        """

        received_at = overlay.received_at or utc_now()
        overlay.received_at = received_at

        previous = self._held.get(overlay.source)
        if overlay.detections:
            # 検出があるので、そのまま描画対象にする。
            self._held[overlay.source] = _HeldOverlay(
                overlay=overlay,
                retained_detections=list(overlay.detections),
                retained_until=None,
            )
            return

        # 検出 0 件。直前に検出があれば clear_grace_sec の間だけ残す。
        retained: List[OverlayDetectionView] = []
        retained_until: Optional[datetime] = None
        if previous is not None and previous.retained_detections:
            still_valid = (
                previous.retained_until is None
                or received_at <= previous.retained_until
            )
            if still_valid:
                retained = list(previous.retained_detections)
                retained_until = previous.retained_until or _add_seconds(
                    received_at, self._clear_grace_sec
                )

        self._held[overlay.source] = _HeldOverlay(
            overlay=overlay,
            retained_detections=retained,
            retained_until=retained_until,
        )

    def active_overlays(self, *, now: Optional[datetime] = None) -> List[PerceptionOverlayView]:
        """保持期間内の認識結果を返す（判定チップの表示に使う）。

        Args:
            now (Optional[datetime]): 判定基準時刻. 省略時は現在時刻.

        Returns:
            List[PerceptionOverlayView]: 保持期間内の重畳データ.
        """

        reference = now or utc_now()
        return [
            held.overlay
            for held in self._held.values()
            if self._is_within_hold(held, reference)
        ]

    def render(
        self,
        image: Image.Image,
        *,
        frame_stamp: Optional[float] = None,
        now: Optional[datetime] = None,
    ) -> Image.Image:
        """生画像へ保持中の認識結果を重畳した画像を返す。

        Args:
            image (Image.Image): カメラ生画像.
            frame_stamp (Optional[float]): 描画対象フレームの時刻 [s].
            now (Optional[datetime]): 判定基準時刻. 省略時は現在時刻.

        Returns:
            Image.Image: 重畳済みの新しい画像（入力画像は変更しない）.
        """

        reference = now or utc_now()
        rendered = image.convert('RGB')
        draw = ImageDraw.Draw(rendered)

        for source in sorted(self._held):
            held = self._held[source]
            if not self._is_within_hold(held, reference):
                continue
            detections = self._drawable_detections(held, reference)
            if not detections:
                continue
            synced = self._is_synced(held.overlay, frame_stamp)
            color = SOURCE_COLORS.get(source, DEFAULT_COLOR)
            for detection in detections:
                self._draw_detection(draw, detection, color, synced)

        return rendered

    def _drawable_detections(
        self, held: _HeldOverlay, reference: datetime
    ) -> List[OverlayDetectionView]:
        """保持中の検出のうち、いま描画すべきものを返す。"""

        if held.overlay.detections:
            return held.overlay.detections
        if held.retained_until is not None and reference <= held.retained_until:
            return held.retained_detections
        return []

    def _is_within_hold(self, held: _HeldOverlay, reference: datetime) -> bool:
        """保持期間内か判定する。"""

        received_at = held.overlay.received_at
        if received_at is None:
            return False
        return (reference - received_at).total_seconds() <= self._hold_sec

    def _is_synced(self, overlay: PerceptionOverlayView, frame_stamp: Optional[float]) -> bool:
        """認識結果が描画対象フレームに対応するものか判定する。

        どちらかの時刻が不明な場合は同期していないものとして扱い、
        対応が取れていない枠を実線で描いて誤解させないようにする。
        """

        if frame_stamp is None or overlay.frame_stamp is None:
            return False
        return abs(frame_stamp - overlay.frame_stamp) <= self._sync_tolerance_sec

    def _draw_detection(
        self,
        draw: ImageDraw.ImageDraw,
        detection: OverlayDetectionView,
        color: Tuple[int, int, int],
        synced: bool,
    ) -> None:
        """検出 1 件を描画する。"""

        x1, y1, x2, y2 = detection.to_xyxy()
        box_color = color if detection.adopted else UNADOPTED_COLOR
        width = ADOPTED_LINE_WIDTH if detection.adopted else UNADOPTED_LINE_WIDTH

        if synced:
            draw.rectangle((x1, y1, x2, y2), outline=box_color, width=width)
        else:
            _draw_dashed_rectangle(draw, (x1, y1, x2, y2), box_color, width)

        label = detection.label
        if detection.score > 0.0:
            label = f'{label}:{detection.score:.2f}'
        if not synced:
            label = f'{label}{STALE_LABEL_SUFFIX}'
        draw.text((x1, max(y1 - 12, 0)), label, fill=box_color)


def _add_seconds(value: datetime, seconds: float) -> datetime:
    """`datetime` へ秒を加算する。"""

    from datetime import timedelta

    return value + timedelta(seconds=seconds)


def _draw_dashed_rectangle(
    draw: ImageDraw.ImageDraw,
    box: Tuple[int, int, int, int],
    color: Tuple[int, int, int],
    width: int,
) -> None:
    """破線の矩形を描画する（PIL には破線指定が無いため線分で構成する）。"""

    x1, y1, x2, y2 = box
    edges = (
        ((x1, y1), (x2, y1)),
        ((x2, y1), (x2, y2)),
        ((x2, y2), (x1, y2)),
        ((x1, y2), (x1, y1)),
    )
    for (start_x, start_y), (end_x, end_y) in edges:
        length = max(abs(end_x - start_x), abs(end_y - start_y))
        if length == 0:
            continue
        step_x = (end_x - start_x) / length
        step_y = (end_y - start_y) / length
        position = 0
        while position < length:
            segment_end = min(position + DASH_LENGTH, length)
            draw.line(
                (
                    start_x + step_x * position,
                    start_y + step_y * position,
                    start_x + step_x * segment_end,
                    start_y + step_y * segment_end,
                ),
                fill=color,
                width=width,
            )
            position += DASH_LENGTH * 2
