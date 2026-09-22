"""RMCの検証とchrony SOCK時刻配信。ROS・シリアルには依存しない。"""

from datetime import datetime, timezone
import math
import re
import socket
import struct


SOCK_SAMPLE = struct.Struct('@lldiiii')


def rmc_utc(line: str) -> float | None:
    """チェックサム・有効フラグ・暦日を確認し、RMCのUTC秒を返す。"""
    try:
        body, checksum = line.strip().split('*')
        if not body.startswith('$') or len(checksum) != 2:
            return None
        value = 0
        for char in body[1:]:
            value ^= ord(char)
        if value != int(checksum, 16):
            return None
        fields = body[1:].split(',')
        if fields[0] not in ('GPRMC', 'GNRMC') or fields[2] != 'A':
            return None
        if len(fields) > 12 and fields[12] in ('N', 'S'):
            return None
        clock, date = fields[1], fields[9]
        if not re.fullmatch(r'\d{6}(\.\d+)?', clock) or not re.fullmatch(r'\d{6}', date):
            return None
        second = float(clock[4:])
        if not 0 <= second < 60:
            return None  # 閏秒境界はNMEA単独で正常時として配信しない。
        dt = datetime(2000+int(date[4:]), int(date[2:4]), int(date[:2]),
                      int(clock[:2]), int(clock[2:4]), int(second), tzinfo=timezone.utc)
        return dt.timestamp() + second-int(second)
    except (ValueError, IndexError, OverflowError):
        return None


def pack_sample(utc: float, received: float) -> bytes:
    """同一ホスト上のchronyのnative ABIで非PPSサンプルを組み立てる。"""
    if not math.isfinite(utc) or not math.isfinite(received) or min(utc, received) < 0:
        raise ValueError('時刻は有限のUnix秒で指定する')
    micros = round(received*1_000_000)
    sec, usec = divmod(micros, 1_000_000)
    return SOCK_SAMPLE.pack(sec, usec, utc-received, 0, 0, 0, 0x534F434B)


class RmcClockRelay:
    """単一シリアル所有者から呼ぶ時刻リレー。欠測時はサンプルを捏造しない。"""

    def __init__(self, path: str) -> None:
        self.path = path
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.anchor = None
        self.last_sent = None
        self.sent = 0
        self.rejected = 0
        self.error = ''

    def observe(self, line: str, received: float, monotonic: float) -> float | None:
        """新規の有効RMCのみ転送し、UTCと単調時計による日付アンカーを保持する。"""
        utc = rmc_utc(line)
        if utc is None:
            if line.startswith(('$GPRMC,', '$GNRMC,')):
                self.anchor = None
                self.rejected += 1
            return None
        if self.anchor is not None:
            previous, previous_mono = self.anchor
            elapsed = monotonic-previous_mono
            # リプレイ・逆行・大きな飛びを除外し、次の連続サンプルから復帰する。
            if utc <= previous or elapsed < 0 or abs((utc-previous)-elapsed) > 1.0:
                self.anchor = None
                self.rejected += 1
                return None
        if self.last_sent is not None and utc <= self.last_sent:
            self.rejected += 1
            return None
        self.anchor = (utc, monotonic)
        try:
            self.sock.sendto(pack_sample(utc, received), self.path)
            self.sent += 1
            self.last_sent = utc
            self.error = ''
        except OSError as exc:
            self.error = str(exc)
        return utc

    def date_gga(self, gga_utc: float, monotonic: float) -> float | None:
        """GGAの時分秒を直近RMCの日付に合わせる。日付未確定・古い観測は拒否する。"""
        if self.anchor is None or not math.isfinite(gga_utc):
            return None
        utc, received = self.anchor
        if not 0 <= monotonic-received <= 2.0:
            return None
        day = int(utc//86400)*86400
        candidates = [day+shift+gga_utc % 86400 for shift in (-86400, 0, 86400)]
        value = min(candidates, key=lambda candidate: abs(candidate-utc))
        return value if abs(value-utc) <= 2.0 else None

    def close(self) -> None:
        self.sock.close()
