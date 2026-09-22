"""Qt/HTML共通のGNSS表示と公開診断JSONの厳格な取り込み。"""
import json
import math
from .freshness import FreshnessLevel as F
from .snapshot_model import NtripStateView

STATES = dict(UNKNOWN='未受信', DISABLED='無効', CONNECTING='接続待ち',
              RECONNECTING='再接続中', WAITING='接続済・RTCM待ち',
              RECEIVING='補正受信中', STALE='補正途絶', ERROR='設定エラー')
FRESH = {F.UNKNOWN: '未受信', F.OK: '受信中', F.STALE: '更新遅延（前回値）', F.LOST: '情報途絶（前回値）'}


def parse_ntrip_status(raw):
    try:
        data = json.loads(raw)
        if not isinstance(data, dict) or data.get('state') not in STATES:
            return None
        view = NtripStateView()
        for key in ('state', 'host', 'mountpoint', 'station_id', 'station_label', 'site', 'last_error'):
            value = data.get(key, '')
            if not isinstance(value, str) or len(value) > 256 or any(ord(c)<32 for c in value):
                return None
            setattr(view, key, value)
        # 固定のエラー種別だけ公開し、サーバ本文や認証情報を伝播させない。
        if view.last_error not in ('', 'ConnectionError', 'TimeoutError', 'OSError',
                                    'ValueError', 'gaierror', 'ConnectionResetError',
                                    'BrokenPipeError', 'ConnectionRefusedError'):
            view.last_error = '通信エラー'
        for key in ('port', 'rtcm_bytes_total', 'reconnect_count'):
            value = data.get(key, 0)
            if type(value) is not int or value < 0 or (key == 'port' and value > 65535):
                return None
            setattr(view, key, value)
        for key in ('rtcm_bytes_per_s', 'last_rtcm_age_s'):
            value = data.get(key, None if key == 'last_rtcm_age_s' else 0.)
            if value is None and key == 'last_rtcm_age_s':
                continue
            if type(value) not in (float, int) or not math.isfinite(value) or value < 0:
                return None
            setattr(view, key, value)
        if type(data.get('transport_connected', False)) is not bool:
            return None
        view.transport_connected = data.get('transport_connected', False)
        return view
    except (ValueError, TypeError):
        return None


def ntrip_summary(n):
    if n.freshness != F.OK:
        return FRESH[n.freshness]
    return STATES.get(n.state, '不明')


def gnss_rows(snapshot):
    n, g = snapshot.ntrip_state, snapshot.gps_state
    known = g.status_freshness != F.UNKNOWN
    def value(v, fmt):
        return format(v, fmt) if known and math.isfinite(v) else '—'
    base = [
        ('状態', ntrip_summary(n)),
        ('地域', {'inagi': '稲城市', 'tsukuba': 'つくば市'}.get(n.site, n.site) or '—'),
        ('基地局', n.station_label or n.station_id or '—'),
        ('接続先', f'{n.host}:{n.port}' if n.host else '—'),
        ('マウントポイント', n.mountpoint or '—'),
        ('RTCM受信速度', f'{n.rtcm_bytes_per_s:.0f} B/s' if n.freshness == F.OK else '—'),
        ('最終RTCMから', f'{n.last_rtcm_age_s:.1f} s' if n.last_rtcm_age_s is not None and n.freshness == F.OK else '—'),
        ('RTCM累積', f'{n.rtcm_bytes_total:,} B' if n.freshness != F.UNKNOWN else '—'),
        ('再接続回数', str(n.reconnect_count) if n.freshness != F.UNKNOWN else '—'),
        ('直近の通信エラー', n.last_error or '—'),
        ('接続情報の更新', FRESH[n.freshness]),
    ]
    gps = [
        ('GNSS情報の更新', FRESH[g.status_freshness]),
        ('測位状態', g.rtk_state if known else '未受信'),
        ('受信衛星数', value(g.num_satellites, 'd')),
        ('HDOP', value(g.hdop, '.2f')),
        ('補正データ年齢 [s]', value(g.correction_age_s, '.2f')),
        ('アンテナ方位 [° 北CW]', value(g.heading_deg, '.2f')),
        ('方位の標準偏差 [°]', value(g.heading_stddev_deg, '.3f')),
        ('アンテナ間距離 [m]', value(g.baseline_length_m, '.3f')),
        ('主アンテナ緯度 [°]', value(g.latitude, '.8f')),
        ('主アンテナ経度 [°]', value(g.longitude, '.8f')),
        ('主アンテナ高度 [m]', value(g.altitude, '.3f')),
    ]
    return base, gps
