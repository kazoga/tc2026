import json
from dataclasses import replace
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace
import pytest
from PyQt5 import QtWidgets
from robot_console.core.console_core import ConsoleCore
from robot_console.core.launch_profile import LaunchProfileStore
from robot_console.core.freshness import FreshnessLevel as F
from robot_console.core.snapshot_model import ConsoleSnapshot
from robot_console.core.gnss_display import parse_ntrip_status, gnss_rows, ntrip_summary
from robot_console.ui_qt.gnss_tab import GnssTab
from robot_console.web.json_codec import build_snapshot_payload


def core():
    return ConsoleCore(profile_store=LaunchProfileStore(
        Path(__file__).resolve().parents[2]/'config/node_launch_profiles.yaml'))


def test_ros_status_through_core_qt_and_web_then_timeout():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    c = core()
    c.update_ntrip_status(SimpleNamespace(data=json.dumps(dict(
        state='RECEIVING', host='caster', port=2101, mountpoint='TEST',
        rtcm_bytes_total=1000, rtcm_bytes_per_s=80., last_rtcm_age_s=.4,
        password='SECRET', user='PRIVATE', transport_connected=True))))
    snap = c.build_snapshot()
    assert ntrip_summary(snap.ntrip_state) == '補正受信中'
    tab = GnssTab(); tab.update_snapshot(snap)
    assert tab.fields[0][0].text() == '補正受信中'
    assert tab.fields[0][3].text() == 'caster:2101'
    payload = json.dumps(build_snapshot_payload(snap), ensure_ascii=False)
    assert 'SECRET' not in payload and 'PRIVATE' not in payload
    c.freshness.mark_received('ntrip', snap.timestamp-timedelta(seconds=6))
    old = c.build_snapshot()
    assert ntrip_summary(old.ntrip_state) == '情報途絶（前回値）'
    assert dict(gnss_rows(old)[0])['RTCM受信速度'] == '—'
    tab.update_snapshot(old)
    assert '情報途絶' in tab.fields[0][0].text()
    assert dict(gnss_rows(ConsoleSnapshot())[1])['HDOP'] == '—'


def test_simulation_disabled_status_is_distinguished_from_unreceived():
    """シムのGNSSノードが配信するDISABLED診断が「無効」として表示されることを確認する.

    未配信のままだと「未受信」となり、実機で診断が途絶した異常と区別できない.
    """

    c = core()
    c.update_ntrip_status(SimpleNamespace(data=json.dumps(dict(
        state='DISABLED', host='', port=0, mountpoint='', station_id='',
        station_label='', site='', transport_connected=False,
        rtcm_bytes_total=0, rtcm_bytes_per_s=0., last_rtcm_age_s=None,
        reconnect_count=0, last_error=''))))
    snap = c.build_snapshot()
    assert snap.ntrip_state.freshness == F.OK
    assert ntrip_summary(snap.ntrip_state) == '無効'


@pytest.mark.parametrize('bad', ['[]', '{', '{"state":"bogus"}',
    '{"state":"RECEIVING","rtcm_bytes_per_s":NaN}',
    '{"state":"RECEIVING","port":true}',
    '{"state":"RECEIVING","host":{}}',
    '{"state":"RECEIVING","last_rtcm_age_s":-1}'])
def test_malformed_status_not_accepted_or_marked_fresh(bad):
    assert parse_ntrip_status(bad) is None
    c=core();c.update_ntrip_status(SimpleNamespace(data=bad))
    assert c.build_snapshot().ntrip_state.freshness == F.UNKNOWN
