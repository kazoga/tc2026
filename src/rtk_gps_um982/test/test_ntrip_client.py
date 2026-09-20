"""TCP分割、誤った成功判定、先頭データ欠落、CRC、終了を回帰検査する。"""
from collections import deque
import pytest
from rtk_gps_um982.ntrip_client import crc24q, RtcmFrames, open_stream, NtripClient


def frame():
    body = b'\xd3\x00\x04\x3e\xd0\x00\x00'
    return body+crc24q(body).to_bytes(3, 'big')


class FakeSocket:
    def __init__(self, parts):
        self.parts = deque(parts)
        self.closed = False
    def sendall(self, data):
        self.request = data
    def recv(self, count):
        return self.parts.popleft() if self.parts else b''
    def close(self):
        self.closed = True


@pytest.mark.parametrize('header', [b'ICY 200 OK\r\n\r\n',
    b'HTTP/1.0 200 OK\r\nContent-Type: gnss/data\r\n\r\n'])
def test_header_fragmentation_preserves_initial_rtcm(monkeypatch, header):
    stream = FakeSocket([header[:4], header[4:8], header[8:]+frame()])
    monkeypatch.setattr('socket.create_connection', lambda *a, **k: stream)
    sock, initial = open_stream('example.com', 2101, 'TEST')
    assert RtcmFrames().feed(initial) == [frame()]
    assert b'Authorization:' not in sock.request


@pytest.mark.parametrize('status', [b'SOURCETABLE 200 OK', b'HTTP/1.0 401 Unauthorized', b'ICY 400 Error'])
def test_reject_non_stream_success(monkeypatch, status):
    stream = FakeSocket([status+b'\r\n\r\n'])
    monkeypatch.setattr('socket.create_connection', lambda *a, **k: stream)
    with pytest.raises(ConnectionError):
        open_stream('example.com', 2101, 'TEST')
    assert stream.closed


def test_crc_and_split_frames():
    assert crc24q(b'123456789') == 0xcde703
    parser = RtcmFrames()
    bad = bytearray(frame()); bad[-1] ^= 1
    assert parser.feed(b'\r\n'+bytes(bad)+frame()[:2]) == []
    assert parser.feed(frame()[2:]+frame()) == [frame(), frame()]


def test_stop_remains_joinable():
    client = NtripClient('example.com', 2101, 'TEST', '', '', lambda _: None)
    client.stop(); client.start(); client.join(timeout=1)
    assert not client.is_alive()
