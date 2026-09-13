#!/usr/bin/env python3
"""確認済みworldを、cloneだけで取得できる固定データとして梱包する.

地図更新時のメンテナ用。利用者の起動時には実行せず、同梱archiveを展開する。
"""

import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import tarfile
import xml.etree.ElementTree as ET

import yaml


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def offline_preview(source: Path) -> str:
    """写真やネットワーク参照を除き、形状とThree.jsを単体HTMLに埋め込む."""
    template = (Path(__file__).parent/'terrain3d/full_course_viewer.html').read_text()
    data = json.loads((source/'viewer_data.json').read_text())
    data.pop('imagery', None)
    template = template.replace("const D=await(await fetch('viewer_data.json')).json(),",
                                'const D='+json.dumps(data, separators=(',', ':'))
                                .replace('<', '\\u003c')+',')
    start = template.index('const photo=await new Promise')
    end = template.index('\nconst colors=', start)
    template = (template[:start]+
                "scene.add(new T.Mesh(ground,new T.MeshLambertMaterial({color:'#a5b996',"
                "side:T.DoubleSide})));"+template[end:])
    start = template.index('<div id="photo">')
    end = template.index('</div>', start)+len('</div>')
    template = template[:start]+template[end:]
    template = template.replace('<button id="aerial">写真比較</button>', '')
    template = '\n'.join(line for line in template.splitlines()
                         if "getElementById('aerial').onclick" not in line)
    start = template.index('<footer>')
    end = template.index('</footer>', start)+len('</footer>')
    template = (template[:start]+'<footer>地形：国土地理院DEMを加工 ／ '
                '<a href="https://www.openstreetmap.org/copyright">© OpenStreetMap contributors</a>'
                '（ODbL 1.0）／ 概略経路：つくばチャレンジ2026公式図を参照。'
                '離隔調整済みの試験モデル。航空写真は同梱していません。'
                '実地測量・完走保証ではありません。</footer>'+template[end:])
    # ダウンロード先はarchive内なので、単体閲覧版からの相対リンクは設けない。
    start = template.index('<a href=') if '<a href=' in template[:template.index('<footer>')] else -1
    if start >= 0:
        end = template.index('</a>', start)+len('</a>')
        template = template[:start]+template[end:]
    template = template.replace('地理参照付き航空写真・標高・建物・物体候補を統合。',
                                'オフライン固定地図：標高・建物・物体候補を表示。')
    template = template.replace('全ルート・デジタルツイン', '全ルート・同梱デジタルツイン')
    template = template.replace('全ルートのデジタルツイン</h1>', '全ルート・離隔調整版</h1>')
    template = template.replace("document.getElementById('status').textContent='高さ倍率 1× ／ 物理モデルと同じ地形・物体';",
                                "document.getElementById('status').textContent='オフライン固定地図 ／ 離隔調整版';")
    for name in ['three.min.js', 'OrbitControls.js']:
        script = (source/'viewer_vendor'/name).read_text().replace('</script', '<\\/script')
        template = template.replace(f'<script src="viewer_vendor/{name}"></script>',
                                    '<script>'+script+'</script>')
    license_text = (source/'viewer_vendor/THREE-LICENSE').read_text()
    return '<!-- Three.js license:\n'+license_text+'\n-->\n'+template


def bundle(trial: Path, world: Path, preview_source: Path, route: Path, output: Path) -> None:
    """参照メッシュを丸ごと固定し、写真を含めずSHA-256 manifestを保存する."""
    output.mkdir(parents=True, exist_ok=False)
    tree = ET.parse(world)
    if len(tree.findall(".//sensor[@type='imu']")) != 1:
        raise ValueError('IMU追加済みworldが必要です')
    files = {'world/trial.sdf': world.read_bytes()}
    uris = {element.text for element in tree.findall('.//uri')}
    for uri in sorted(uris):
        if Path(uri).name != uri or not uri.endswith('.obj'):
            raise ValueError('同梱できないメッシュ参照: '+str(uri))
        mesh = (trial/uri).read_bytes()
        files['world/'+uri] = mesh
        for line in mesh.decode().splitlines():
            if line.startswith('mtllib '):
                name = line.split(maxsplit=1)[1]
                if Path(name).name != name:
                    raise ValueError('material参照は同一階層にしてください')
                # 写真テクスチャの代わりに無地の地面色を指定する。形状は保持する。
                material = (trial/name).read_text()
                material = '\n'.join(row for row in material.splitlines()
                                     if not row.startswith('map_'))+'\n'
                material = material.replace('Kd 1 1 1', 'Kd 0.65 0.73 0.59')
                files['world/'+name] = material.encode()
    for name in ['world.json', 'trial.json']:
        files['world/'+name] = (trial/name).read_bytes()
    for name in ['fixed/waypoints.csv', 'route_config.yaml']:
        files['routes/'+name] = (route/name).read_bytes()
    projection = yaml.safe_load((route/'projection.yaml').read_text())
    projection['/**']['ros__parameters']['projection_id'] = 'tsukuba2026_digital_twin'
    files['projection.yaml'] = yaml.safe_dump(projection).encode()
    session = dict(trial_directory='world', trial_sdf='world/trial.sdf',
                   projection_params='projection.yaml', csv_base_dir='routes',
                   route_config='routes/route_config.yaml', start_label='0', goal_label='1141',
                   simulation_domain_id=86, real_domain_id=0,
                   noise_profile='conservative', noise_seed=1, building_gnss=True,
                   hardware_launch='', fastlio_config='')
    files['session.yaml'] = json.dumps(session, indent=2).encode()
    if (preview_source/'corridor_changes.json').exists():
        files['world/corridor_changes.json'] = (preview_source/'corridor_changes.json').read_bytes()
    archive = output/'world.tar.gz'
    with archive.open('wb') as raw, gzip.GzipFile(fileobj=raw, mode='wb', mtime=0,
                                                filename='', compresslevel=9) as compressed:
        with tarfile.open(fileobj=compressed, mode='w') as tar:
            for name, data in sorted(files.items()):
                info = tarfile.TarInfo(name)
                info.size, info.mode = len(data), 0o644
                tar.addfile(info, io.BytesIO(data))
    manifest = dict(version=1, model='tsukuba2026-corridor-1m',
                    archive_sha256=sha256(archive.read_bytes()),
                    source_world_sha256=sha256(world.read_bytes()),
                    files={name: dict(bytes=len(data), sha256=sha256(data))
                           for name, data in sorted(files.items())})
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (output/'preview.html').write_text(offline_preview(preview_source))
    (output/'THREE-LICENSE').write_bytes((preview_source/'viewer_vendor/THREE-LICENSE').read_bytes())
    print(f'{len(files)} files / archive {archive.stat().st_size/1e6:.1f} MB')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--preview-source', type=Path, required=True)
    parser.add_argument('--route', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    bundle(args.trial, args.world, args.preview_source, args.route, args.output)
