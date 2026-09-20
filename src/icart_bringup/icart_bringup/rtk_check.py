"""シリアル・ロボットに触れず、選択局のRTCM3受信とCRCを短時間確認する。"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import time

from icart_bringup.hardware_core import read_yaml, station_config
from rtk_gps_um982.ntrip_client import open_stream, RtcmFrames


def check(connection: dict, seconds: float = 8.) -> dict:
    result = dict(checked_at=datetime.now(timezone.utc).isoformat(),
                  host=connection['host'], port=connection['port'],
                  mountpoint=connection['mountpoint'], valid_frames=0, rtcm_types=[],
                  receiver_fix_verified=False)
    decoder = RtcmFrames()
    types = set()
    try:
        sock, pending = open_stream(*[connection.get(k, '') for k in
                                     ['host', 'port', 'mountpoint', 'user', 'password']])
        with sock:
            sock.settimeout(1.)
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                for frame in decoder.feed(pending):
                    result['valid_frames'] += 1
                    if len(frame) >= 8:
                        types.add((frame[3] << 4) | (frame[4] >> 4))
                try:
                    pending = sock.recv(8192)
                except socket.timeout:
                    pending = b''
                    continue
                if not pending:
                    break
        result['rtcm_types'] = sorted(types)
    except (OSError, ValueError, ConnectionError) as exc:
        result['error'] = type(exc).__name__
    result['stream_verified'] = result['valid_frames'] > 0
    return result


def main() -> None:
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--site', required=True, choices=['inagi', 'tsukuba'])
    parser.add_argument('--station')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--ntrip-config', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    catalog = read_yaml(Path(get_package_share_directory('icart_bringup'))/'params/rtk_stations.yaml')
    if args.list:
        for key in catalog['sites'][args.site]['stations']:
            item = catalog['stations'][key]
            print(key, item['label'], item.get('coordinate_reference', ''))
        return
    _, connection = station_config(catalog, args.site, args.station,
        read_yaml(args.ntrip_config) if args.ntrip_config else None)
    if not connection['enabled']:
        parser.error('NTRIPなしの設定は接続検査できません')
    result = check(connection)
    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text)
    if not result['stream_verified']:
        raise SystemExit(1)
