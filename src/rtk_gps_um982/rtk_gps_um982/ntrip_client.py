"""NTRIP v1受信。TCP分割・先頭RTCM・接続切れを扱い、CRC一致フレームだけ渡す。"""
import base64
import socket
import threading
import time

from um982 import UM982Client


def crc24q(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xffffff


class RtcmFrames:
    """分割・結合されたTCP入力からCRC一致のRTCM3フレームを復元する。"""
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            start = self.buffer.find(b'\xd3')
            if start < 0:
                self.buffer.clear()
                break
            del self.buffer[:start]
            if len(self.buffer) < 3:
                break
            if self.buffer[1] & 0xfc:
                del self.buffer[0]
                continue
            size = ((self.buffer[1] & 3) << 8 | self.buffer[2]) + 6
            if len(self.buffer) < size:
                break
            frame = bytes(self.buffer[:size])
            if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], 'big'):
                frames.append(frame)
                del self.buffer[:size]
            else:
                del self.buffer[0]
        return frames


def open_stream(host, port, mountpoint, user='', password='', timeout=5.):
    """成功応答を厳密に判定し、同じrecvに入った先頭RTCMも返す。"""
    if (not host or not mountpoint or any(c in str(v) for v in [host, mountpoint, user, password]
                                        for c in ['\r', '\n'])
            or any(c in mountpoint for c in [' ', '/', '?', '#'])):
        raise ValueError('NTRIP接続先が不正')
    sock = socket.create_connection((host, port), timeout=timeout)
    try:
        auth = ('Authorization: Basic '+base64.b64encode(f'{user}:{password}'.encode()).decode()+'\r\n'
                if user or password else '')
        sock.sendall((f'GET /{mountpoint} HTTP/1.0\r\nHost: {host}\r\n'
                      f'User-Agent: NTRIP tc2026/1.0\r\n{auth}\r\n').encode())
        data = b''
        while b'\r\n' not in data:
            chunk = sock.recv(4096)
            if not chunk or len(data)+len(chunk) > 16384:
                raise ConnectionError('NTRIP応答ヘッダが不正')
            data += chunk
        first, remainder = data.split(b'\r\n', 1)
        parts = first.split()
        if len(parts) < 2 or parts[0] not in [b'ICY', b'HTTP/1.0', b'HTTP/1.1'] or parts[1] != b'200':
            # 認証情報やサーバ本文をログに出さない。
            raise ConnectionError('NTRIP成功応答ではありません（認証・mountpointを確認）')
        if parts[0] == b'ICY':
            # ICYはstatus行だけの場合もある。余分なCRLFはRTCM decoderが除去する。
            return sock, remainder
        while b'\r\n\r\n' not in data:
            chunk = sock.recv(4096)
            if not chunk or len(data)+len(chunk) > 16384:
                raise ConnectionError('NTRIP HTTPヘッダが不正')
            data += chunk
        header, remainder = data.split(b'\r\n\r\n', 1)
        if b'transfer-encoding:' in header.lower():
            raise ConnectionError('NTRIP v1ではTransfer-Encoding非対応')
        return sock, remainder
    except Exception:
        sock.close()
        raise


class NtripClient(threading.Thread):
    """既存UM982Clientのcallback接口を保つ受信スレッド。"""
    def __init__(self, host, port, mountpoint, user, password, on_rtcm_data,
                 get_gga=None, gga_interval=5., on_rtcm_count=None):
        super().__init__(daemon=True)
        self.connection = (host, port, mountpoint, user, password)
        self.on_rtcm_data, self.on_rtcm_count = on_rtcm_data, on_rtcm_count
        self.get_gga, self.gga_interval = get_gga, gga_interval
        self._stop_event = threading.Event()
        self._connected = False
        self._total_bytes = 0
        self.last_error = ''
        self.transport_connected = False
        self.last_rtcm_monotonic = None
        self.connection_attempts = 0
        self._socket = None

    @property
    def is_connected(self):
        return self._connected

    @property
    def total_bytes(self):
        return self._total_bytes

    def stop(self):
        self._stop_event.set()
        if self._socket:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def run(self):
        backoff = 1.
        while not self._stop_event.is_set():
            try:
                self._run_once()
                backoff = 1.
            except (OSError, ValueError, ConnectionError) as exc:
                self.last_error = type(exc).__name__
            finally:
                self._connected = False
            if self._stop_event.wait(backoff):
                break
            backoff = min(30., backoff*2.)

    def _run_once(self):
        self.connection_attempts += 1
        sock, initial = open_stream(*self.connection)
        self._socket = sock
        self.transport_connected = True
        decoder = RtcmFrames()
        last_valid, last_gga = time.monotonic(), -float('inf')
        pending = initial
        try:
            sock.settimeout(1.)
            while not self._stop_event.is_set():
                for frame in decoder.feed(pending):
                    self.on_rtcm_data(frame)
                    self._total_bytes += len(frame)
                    if self.on_rtcm_count:
                        self.on_rtcm_count(len(frame))
                    last_valid = time.monotonic()
                    self.last_rtcm_monotonic = last_valid
                    self._connected = True
                    self.last_error = ''
                now = time.monotonic()
                if now-last_valid > 15.:
                    raise ConnectionError('RTCM受信途絶')
                if self.get_gga and self.gga_interval > 0 and now-last_gga >= self.gga_interval:
                    gga = self.get_gga()
                    if gga:
                        sock.sendall((gga.strip()+'\r\n').encode('ascii'))
                        last_gga = now
                try:
                    pending = sock.recv(4096)
                    if not pending:
                        raise ConnectionError('NTRIP接続終了')
                except socket.timeout:
                    pending = b''
        finally:
            self.transport_connected = False
            sock.close()
            self._socket = None


class CorrectedUM982Client(UM982Client):
    """固定submoduleを変更せず、NTRIPの受信実装のみ差し替える。"""
    def start_ntrip(self, host, port, mountpoint, user, password, gga_interval=5.):
        self.stop_ntrip()
        self._ntrip_client = NtripClient(host, port, mountpoint, user, password,
            self._write_bytes, self._get_latest_gga_raw, gga_interval, self._update_rtcm_count)
        self._ntrip_client.start()

    def stop_ntrip(self):
        if self._ntrip_client:
            self._ntrip_client.stop()
            self._ntrip_client.join(timeout=6.)
            self._ntrip_client = None

    def stop(self):
        self.stop_ntrip()
        super().stop()
