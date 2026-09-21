"""CameraOverlayRenderer の単体テスト。

認識レートが生画像より低い前提での保持・消去・同期判定を確認する。
"""

from datetime import timedelta

from PIL import Image

from robot_console.core.camera_overlay import (
    CameraOverlayRenderer,
    OverlayDetectionView,
    PerceptionOverlayView,
)
from robot_console.utils.time_utils import now as utc_now


def _detection(adopted: bool = True) -> OverlayDetectionView:
    return OverlayDetectionView(
        center_x=50.0,
        center_y=40.0,
        size_x=20.0,
        size_y=10.0,
        label='green',
        score=0.9,
        adopted=adopted,
    )


def _overlay(detections, received_at, *, frame_stamp=100.0, source='traffic_signal'):
    return PerceptionOverlayView(
        source=source,
        detections=list(detections),
        decision=1,
        decision_text='GO',
        frame_stamp=frame_stamp,
        received_at=received_at,
    )


def _blank() -> Image.Image:
    return Image.new('RGB', (120, 80), color='black')


def _drawn_pixel_count(image: Image.Image) -> int:
    """黒以外のピクセル数を数える（描画されたかの判定に使う）。"""

    return sum(1 for pixel in image.getdata() if pixel != (0, 0, 0))


def test_detection_is_drawn_on_frames_after_the_matching_one():
    """認識レートが低くても、後続フレームに枠を描き続ける。

    対応するフレームにだけ描くと、ほとんどの時間枠が表示されずちらついて見える。
    """

    renderer = CameraOverlayRenderer()
    base = utc_now()
    renderer.update_overlay(_overlay([_detection()], base, frame_stamp=100.0))

    # 認識結果が対応するフレームより後のフレームでも描画される。
    rendered = renderer.render(_blank(), frame_stamp=100.2, now=base + timedelta(seconds=0.2))

    assert _drawn_pixel_count(rendered) > 0


def test_detection_is_dropped_after_hold_sec():
    """更新が途絶したら保持をやめる。"""

    renderer = CameraOverlayRenderer(hold_sec=1.0)
    base = utc_now()
    renderer.update_overlay(_overlay([_detection()], base))

    rendered = renderer.render(_blank(), frame_stamp=100.0, now=base + timedelta(seconds=1.5))

    assert _drawn_pixel_count(rendered) == 0
    assert renderer.active_overlays(now=base + timedelta(seconds=1.5)) == []


def test_empty_result_keeps_previous_detection_during_grace():
    """検出0件を受けても猶予時間は直前の枠を残す（点滅防止）。"""

    renderer = CameraOverlayRenderer(clear_grace_sec=0.5)
    base = utc_now()
    renderer.update_overlay(_overlay([_detection()], base))
    renderer.update_overlay(_overlay([], base + timedelta(seconds=0.1)))

    during_grace = renderer.render(
        _blank(), frame_stamp=100.0, now=base + timedelta(seconds=0.3)
    )
    after_grace = renderer.render(
        _blank(), frame_stamp=100.0, now=base + timedelta(seconds=0.9)
    )

    assert _drawn_pixel_count(during_grace) > 0
    assert _drawn_pixel_count(after_grace) == 0


def test_stale_frame_is_drawn_with_dashed_box():
    """元フレームと描画フレームの時刻差が大きい枠は破線で描く。

    古い認識結果を実線で描くと、現在のフレームに対する結果だと誤解させるため。
    """

    renderer = CameraOverlayRenderer(sync_tolerance_sec=0.3)
    base = utc_now()
    renderer.update_overlay(_overlay([_detection()], base, frame_stamp=100.0))

    synced = renderer.render(_blank(), frame_stamp=100.1, now=base)
    stale = renderer.render(_blank(), frame_stamp=101.0, now=base)

    # ラベル文字の描画分を拾わないよう、矩形の上辺だけを数えて比較する。
    # 検出は中心(50, 40)・大きさ20x10なので上辺はy=35、x=40..60。
    def _top_edge_pixels(image: Image.Image) -> int:
        return sum(
            1 for x in range(40, 61) if image.getpixel((x, 35)) != (0, 0, 0)
        )

    assert _top_edge_pixels(synced) == 21, '同期時は実線で全ピクセルが埋まる'
    assert _top_edge_pixels(stale) < 21, '非同期時は破線で間引かれる'


def test_unadopted_detection_is_drawn_thinner_than_adopted():
    """判定に採用されなかった検出は細線で描き分ける。"""

    base = utc_now()

    adopted_renderer = CameraOverlayRenderer()
    adopted_renderer.update_overlay(_overlay([_detection(adopted=True)], base))
    adopted = adopted_renderer.render(_blank(), frame_stamp=100.0, now=base)

    unadopted_renderer = CameraOverlayRenderer()
    unadopted_renderer.update_overlay(_overlay([_detection(adopted=False)], base))
    unadopted = unadopted_renderer.render(_blank(), frame_stamp=100.0, now=base)

    assert _drawn_pixel_count(unadopted) < _drawn_pixel_count(adopted)


def test_render_does_not_modify_source_image():
    """入力画像を破壊しない（ImageStoreの保持画像と分離する）。"""

    renderer = CameraOverlayRenderer()
    base = utc_now()
    renderer.update_overlay(_overlay([_detection()], base))

    source = _blank()
    renderer.render(source, frame_stamp=100.0, now=base)

    assert _drawn_pixel_count(source) == 0


def test_overlays_from_multiple_sources_are_drawn_together():
    """信号と経路封鎖の検出を同じ画像へ同時に描ける。

    従来は別画像だったため同時に確認できなかった。
    """

    renderer = CameraOverlayRenderer()
    base = utc_now()
    renderer.update_overlay(_overlay([_detection()], base, source='traffic_signal'))
    renderer.update_overlay(
        _overlay([_detection()], base, source='road_blockage')
    )

    assert len(renderer.active_overlays(now=base)) == 2
