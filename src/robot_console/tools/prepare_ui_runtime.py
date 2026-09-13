#!/usr/bin/env python3
"""sudo認証できない環境で、aptの不足Qt WebEngine依存をローカル展開する."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import urllib.request


def prepare(output: Path, headless_tests: bool = False) -> None:
    """既存apt索引で解決し、サイズ・checksumを検証してから展開する."""
    archives = output/'archives'
    runtime = output/'runtime'
    archives.mkdir(parents=True, exist_ok=True)
    runtime.mkdir(exist_ok=True)
    packages = ['python3-pyqt5.qtwebengine', 'qtbase5-dev-tools']
    if headless_tests:
        packages += ['python3-pytest-forked', 'xvfb']
    listing = subprocess.check_output([
        'apt-get', '--print-uris', '--yes', '--download-only', 'install',
        *packages], text=True)
    (output/'apt-resolution.txt').write_text(listing)
    manifest = []
    for line in listing.splitlines():
        if not line.startswith("'"):
            continue
        url, name, size, checksum = shlex.split(line)
        path = archives/name
        if not path.exists():
            with urllib.request.urlopen(url.replace('http:', 'https:', 1), timeout=60) as response:
                path.write_bytes(response.read())
        data = path.read_bytes()
        algorithm, digest = checksum.split(':', 1)
        algorithm = {'MD5Sum': 'md5', 'SHA256': 'sha256'}[algorithm]
        if len(data) != int(size) or hashlib.new(algorithm, data).hexdigest() != digest:
            raise ValueError('apt索引と一致しないarchive: '+name)
        subprocess.run(['dpkg-deb', '-x', str(path), str(runtime)], check=True)
        manifest.append(dict(archive=name, sha256=hashlib.sha256(data).hexdigest()))
    # Qt 5の資源探索は環境変数だけでは移設できないためqt.confを同梱する。
    from PyQt5.QtCore import QLibraryInfo

    if not (runtime/'usr/lib/x86_64-linux-gnu/qt5/libexec/QtWebEngineProcess').exists():
        print('Qt WebEngineはシステム導入済み。ローカル展開は不要です')
        return
    config = ('[Paths]\nPrefix='+str((runtime/'usr').resolve())+'\n'
              'Data=share/qt5\nTranslations=share/qt5/translations\n'
              'LibraryExecutables=lib/x86_64-linux-gnu/qt5/libexec\n'
              'Plugins='+QLibraryInfo.location(QLibraryInfo.PluginsPath)+'\n')
    (runtime/'qt.conf').write_text(config)
    process_dir = runtime/'usr/lib/x86_64-linux-gnu/qt5/libexec'
    if process_dir.exists():
        (process_dir/'qt.conf').write_text(config)
    resource = runtime/'qt_config.qrc'
    resource.write_text('<RCC><qresource prefix="/qt/etc">'
                        '<file alias="qt.conf">qt.conf</file></qresource></RCC>')
    subprocess.run([shutil.which('rcc') or str(runtime/'usr/lib/qt5/bin/rcc'), '-binary', '-o', str(runtime/'qt_config.rcc'), str(resource)],
                   check=True)
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    print('展開先:', runtime)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--headless-tests', action='store_true')
    args = parser.parse_args()
    prepare(args.output, args.headless_tests)
