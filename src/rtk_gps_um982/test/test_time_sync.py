"""GNSS時刻を誤認して時計へ配信しないことと、SOCKの再接続を検証する。"""

from datetime import datetime, timezone
from pathlib import Path
import socket
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from rtk_gps_um982.time_sync_core import RmcClockRelay, SOCK_SAMPLE, pack_sample, rmc_utc


def sentence(clock='120000.00', date='140926', status='A', talker='GNRMC'):
    body = f'{talker},{clock},{status},3500,N,13900,E,0,0,{date},,,A'
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f'${body}*{checksum:02X}'


def test_rmc_uses_utc_date_not_host_date():
    expected = datetime(2026, 9, 14, 12, tzinfo=timezone.utc).timestamp()
    assert rmc_utc(sentence()) == expected
    assert rmc_utc(sentence(talker='GPRMC')) == expected


@pytest.mark.parametrize('line', [sentence(status='V'), sentence(date='310226'),
    sentence(clock='246001'), sentence(clock='235960'), sentence()[:-1]+'Z',
    sentence(talker='GPGGA'), '$GPRMC,*00', sentence().split('*')[0]])
def test_invalid_rmc_is_rejected(line):
    assert rmc_utc(line) is None


def test_sample_abi_sign_and_fraction_carry():
    assert SOCK_SAMPLE.unpack(pack_sample(101., 100.9999999)) == (
        101, 0, 101.-100.9999999, 0, 0, 0, 0x534F434B)
    with pytest.raises(ValueError):
        pack_sample(float('nan'), 1.)


def test_socket_missing_then_created_and_restarted(tmp_path):
    path = str(tmp_path/'chrony.sock')
    relay = RmcClockRelay(path)
    try:
        utc = relay.observe(sentence(), 100., 10.)
        assert relay.error and relay.sent == 0
        for clock, received, mono in [('120001', 101., 11.), ('120002', 102., 12.)]:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as server:
                server.bind(path)
                server.settimeout(1.)
                relay.observe(sentence(clock=clock), received, mono)
                sample = SOCK_SAMPLE.unpack(server.recv(100))
                assert sample[2] == utc+(mono-10)-received
                assert sample[3:] == (0, 0, 0, 0x534F434B)
            Path(path).unlink()
        assert relay.sent == 2 and not relay.error
    finally:
        relay.close()


def test_replay_and_invalid_rmc_invalidate_date(tmp_path):
    relay = RmcClockRelay(str(tmp_path/'missing'))
    try:
        utc = relay.observe(sentence(), 1., 10.)
        relay.observe(sentence(), 2., 11.)
        assert relay.anchor is None
        relay.observe(sentence(clock='120001'), 2., 11.)
        assert relay.date_gga(utc+1, 11.1) == utc+1
        relay.observe(sentence(clock='120002', status='V'), 3., 12.)
        assert relay.date_gga(utc+2, 12.1) is None
    finally:
        relay.close()


def test_midnight_and_bad_pc_date_and_staleness(tmp_path):
    relay = RmcClockRelay(str(tmp_path/'missing'))
    try:
        utc = relay.observe(sentence(clock='235959.9'), 1., 10.)
        # GGAがPC由来の誤った日付でも時分秒だけを使用する。
        assert relay.date_gga(86400*3+.1, 10.2) == pytest.approx(utc+.2)
        assert relay.date_gga(86400*3+.1, 12.1) is None
        assert relay.date_gga(86400*3+1000., 10.2) is None
    finally:
        relay.close()


def test_client_relays_same_serial_read_without_opening_another_port(monkeypatch):
    from rtk_gps_um982.time_sync_client import ClockRelayClient
    class Relay:
        observed = []
        def observe(self, *values):
            self.observed.append(values)
    class Serial:
        calls = 0
        def readline(self):
            self.calls += 1
            return (sentence()+'\r\n').encode('ascii')
    relay, serial = Relay(), Serial()
    client = ClockRelayClient(port='unused', clock_relay=relay)
    client._ser = serial
    assert client._readline() == sentence()
    assert serial.calls == 1 and len(relay.observed) == 1
    assert relay.observed[0][0] == sentence()


def test_client_configures_rmc_at_current_port(monkeypatch):
    from um982 import UM982Client
    from rtk_gps_um982.time_sync_client import ClockRelayClient
    configured, commands = [], []
    monkeypatch.setattr(UM982Client, 'set_output_rate', lambda *a, **kw: configured.append(a[1:]))
    client = ClockRelayClient(port='unused', clock_relay=None)
    monkeypatch.setattr(client, '_write_line', commands.append)
    client.set_output_rate(10)
    assert configured == [(10,)] and commands == ['GPRMC 1']
