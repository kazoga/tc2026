"""既存配線のGNSS→chrony→PTP試験用設定を生成する。OS変更は明示適用のみ。"""
import argparse
from pathlib import Path
import re
from .ptp_trial import prepare


def prepare_host(output: Path, user: str) -> None:
    if not re.fullmatch(r'[a-z_][a-z0-9_-]{0,30}', user):
        raise ValueError('OSユーザー名が不正')
    prepare(output, '/run/chrony/um982.sock')
    fragment = output/'chrony.conf'
    fragment.write_text(fragment.read_text().replace('offset 0.0', 'offset 0.0 noselect'))
    (output/'chrony-acl.conf').write_text(
        '[Service]\nRuntimeDirectoryMode=0750\n'
        f'ExecStartPost=!/usr/bin/setfacl -m u:{user}:x /run/chrony\n'
        f'ExecStartPost=!/usr/bin/setfacl -m u:{user}:rw /run/chrony/um982.sock\n')
    script = output/'apply-host.sh'
    script.write_text('''#!/bin/bash
set -euo pipefail
if [[ $EUID -ne 0 ]]; then
  echo "sudoで実行してください。ROSノードは一般ユーザーで起動します。" >&2
  exit 1
fi
if pgrep -f '(^|/)(fastlio_mapping|fusion_node|ypspur-coordinator|robot_navigator)( |$)' >/dev/null; then
  echo "走行・自己位置推定を停止してから適用してください。" >&2
  exit 1
fi
setup_dir=$(cd -- "$(dirname -- "$0")" && pwd)
backup_dir="/var/backups/robot-clock-$(date +%Y%m%d_%H%M%S)"
mkdir -p "$backup_dir"
if [[ -d /etc/chrony ]]; then cp -a /etc/chrony "$backup_dir/"; fi
if [[ -d /etc/systemd/system/chrony.service.d ]]; then cp -a /etc/systemd/system/chrony.service.d "$backup_dir/"; fi
systemctl is-enabled systemd-timesyncd > "$backup_dir/timesyncd-enabled.txt" 2>&1 || true
systemctl is-active systemd-timesyncd > "$backup_dir/timesyncd-active.txt" 2>&1 || true
apt-get install -y chrony linuxptp acl
# Aptの初回起動後も、以後は時計をstepさせずslewで収束させる。
cp -a /etc/chrony "$backup_dir/chrony-installed"
python3 - <<'CONFIG'
from pathlib import Path
p=Path('/etc/chrony/chrony.conf')
s=p.read_text()
if 'confdir /etc/chrony/conf.d' not in s:
    s+='\\nconfdir /etc/chrony/conf.d\\n'
p.write_text(s)
# 別断片内も含めてmakestepを無効化する。原本は上で退避済み。
for p in [p, *Path('/etc/chrony/conf.d').glob('*.conf')]:
    lines=p.read_text().splitlines()
    p.write_text('\\n'.join('# robot-clock: '+line if line.strip().startswith('makestep ') else line for line in lines)+'\\n')
CONFIG
install -d /etc/chrony/conf.d /etc/systemd/system/chrony.service.d
install -m 644 "$setup_dir/chrony.conf" /etc/chrony/conf.d/um982.conf
install -m 644 "$setup_dir/chrony-acl.conf" /etc/systemd/system/chrony.service.d/um982-acl.conf
systemctl stop systemd-timesyncd 2>/dev/null || true
systemctl daemon-reload
systemctl restart chrony
chronyc tracking
printf '設定完了。退避先: %s\\nGNSSセンサを起動し、外部NTP選択と収束を確認してからPTP試験を開始してください。\\n' "$backup_dir"
''')
    script.chmod(0o755)
    (output/'README.txt').write_text(
        '既存USB + Ethernet配線。外部NTP → chrony → ptp4l -S → MID360。GNSS RMCは比較専用。\n'
        'apply-host.shは管理者で一度適用。chrony/linuxptp/aclをインストールし、\n'
        '既存時刻設定を退避、UM98を追加、makestepを無効化、chronyを再起動する。\n'
        'PPS無しのためUSB/NMEA遅延は残る。offset 0.0は未較正値。\n'
        'PTPは自動起動しない。外部NTP選択・収束を確認してtools/install_clock_service.shで通常サービスを登録する。\n'
        'FAST-LIO/融合はPTPが収束して点群・IMUのtime_type=1を確認した後に起動。\n'
        '既存の同名chrony ACL drop-inがある場合は内容を確認してから適用。\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--user', required=True)
    a=p.parse_args()
    prepare_host(a.output, a.user)


if __name__ == '__main__':
    main()
