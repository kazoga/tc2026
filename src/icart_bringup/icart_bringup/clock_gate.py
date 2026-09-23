"""Read the privileged PTP monitor's status; never change host clocks."""
import argparse
import json
import math
from pathlib import Path
import sys
import time


def ready(path, mono=None, boot_id=None):
    mono=time.monotonic() if mono is None else mono
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip() if boot_id is None else boot_id
    try:
        state=json.loads(Path(path).read_text())
        age=mono-float(state['checked_monotonic'])
        return (state.get('ready') is True and state.get('clock_source') == 'ntp'
                and state.get('boot_id')==boot_id
                and math.isfinite(age) and 0 <= age <= 3.)
    except (OSError,ValueError,KeyError,TypeError):
        return False


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['wait','watch'])
    p.add_argument('--status',default='/run/icart-clock/status.json')
    p.add_argument('--timeout',type=float,default=180.)
    args=p.parse_args()
    deadline=time.monotonic()+args.timeout
    if args.mode=='wait':
        print('NTP/PC/PTP/LiDARの同期成立を待っています',flush=True)
        while time.monotonic()<deadline:
            if ready(args.status):
                print('時刻同期ゲート成立。共通実機スタックを起動します',flush=True)
                return
            time.sleep(.2)
        sys.exit('時刻同期が未成立です。chrony/icart-clockサービスを確認してください')
    while ready(args.status):time.sleep(.2)
    sys.exit('時刻同期条件を喪失しました。走行・位置推定スタックを停止します')

if __name__=='__main__':main()
