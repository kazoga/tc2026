#!/usr/bin/env python3
"""Ubuntu 24.04 / ROS Jazzy の Gazebo を sudo なしでローカル展開する.

apt の既存インデックスと導入済み ROS に依存する。システムの package database は変更しない。
"""

import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import urllib.request


def prepare(output: Path) -> None:
    """apt が解決した不足依存のみを検証・展開し、配置先を記録する."""
    output = output.resolve()
    archives = output/'archives'
    runtime = output/'runtime'
    archives.mkdir(parents=True,exist_ok=True)
    runtime.mkdir(exist_ok=True)
    command = ['apt-get','--print-uris','--yes','--download-only','install',
               'ros-jazzy-ros-gz-sim','ros-jazzy-ros-gz-bridge','ruby',
               'python3-shapely','python3-protobuf']
    listing = subprocess.check_output(command,text=True)
    entries = [shlex.split(line) for line in listing.splitlines() if line.startswith("'")]
    (output/'apt-resolution.txt').write_text(listing)

    def download(entry: list[str]) -> tuple[Path, str]:
        url,name,size,checksum = entry
        path = archives/name
        if not path.exists():
            with urllib.request.urlopen(url,timeout=90) as response, path.open('wb') as stream:
                while block := response.read(1024*1024):
                    stream.write(block)
        data = path.read_bytes()
        algorithm,digest = checksum.split(':',1)
        algorithm = {'MD5Sum':'md5','SHA256':'sha256'}[algorithm]
        if len(data) != int(size) or hashlib.new(algorithm,data).hexdigest() != digest:
            raise ValueError(f'apt metadata と一致しない archive: {name}')
        return path,hashlib.sha256(data).hexdigest()

    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        verified = list(pool.map(download,entries))
    manifest = []
    for path, digest in verified:
        subprocess.run(['dpkg-deb','-x',str(path),str(runtime)],check=True)
        manifest.append(dict(archive=path.name,sha256=digest))
    prefix = runtime/'opt/ros/jazzy'
    for config in prefix.glob('opt/*/share/gz/*.yaml'):
        # 展開した Gazebo CLI の設定だけを移設先に対応させる。
        config.write_text(re.sub(r'(?m)^library_path: (.+)$',
            lambda match: 'library_path: '+str(runtime/match.group(1).lstrip('/')),
            config.read_text()))
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(f'実行環境: {runtime}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    prepare(parser.parse_args().output)
