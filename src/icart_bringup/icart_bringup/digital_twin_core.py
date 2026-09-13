"""Git同梱の固定地図をネットワークなしで展開し、模擬セッションを起動する."""

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import tempfile


def bundle_directory() -> Path:
    """ソース直接実行とcolcon installの両方で同梱地図を探す."""
    local = Path(__file__).resolve().parents[2]/'obstacle_route_sim/maps/tsukuba2026'
    if (local/'manifest.json').is_file():
        return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('obstacle_route_sim'))/'maps/tsukuba2026'


def digest(path: Path) -> str:
    """大きな地形も一定メモリで照合する."""
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def prepare_bundle(bundle: Path, output: Path) -> Path:
    """ハッシュと相対参照を検査して新規ディレクトリへ原子的に展開する.

    既存セッションは再利用も上書きもしない。変更した経路・設定を保持する。
    """
    output = output.absolute()
    if output.exists():
        raise ValueError('出力先は新規ディレクトリを指定してください: '+str(output))
    manifest = json.loads((bundle/'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('version') != 1:
        raise ValueError('未対応の地図manifestです')
    archive = bundle/'world.tar.gz'
    if digest(archive) != manifest['archive_sha256']:
        raise ValueError('地図archiveのSHA-256が一致しません')
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix='.digital-twin-', dir=output.parent))
    try:
        expected = manifest['files']
        seen = set()
        with tarfile.open(archive, 'r:gz') as source:
            for member in source:
                name = PurePosixPath(member.name)
                if (not member.isfile() or name.is_absolute() or '..' in name.parts
                        or '\\' in member.name or str(name) != member.name
                        or member.name not in expected or member.name in seen
                        or member.size != expected[member.name]['bytes']):
                    raise ValueError('地図archiveのメンバーが不正です: '+member.name)
                target = temporary.joinpath(*name.parts)
                target.parent.mkdir(parents=True, exist_ok=True)
                with source.extractfile(member) as incoming, target.open('wb') as outgoing:
                    shutil.copyfileobj(incoming, outgoing)
                if digest(target) != expected[member.name]['sha256']:
                    raise ValueError('地図ファイルのSHA-256が一致しません: '+member.name)
                seen.add(member.name)
        if seen != set(expected):
            raise ValueError('地図archiveに不足ファイルがあります')
        for required in ['session.yaml', 'world/trial.sdf', 'world/world.json',
                         'world/trial.json', 'routes/fixed/waypoints.csv', 'projection.yaml']:
            if required not in seen:
                raise ValueError('セッション必須ファイルがありません: '+required)
        shutil.copyfile(bundle/'manifest.json', temporary/'bundle_manifest.json')
        temporary.rename(output)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    return output/'session.yaml'


def main() -> None:
    """同梱地図を展開してsimulation専用の共通起動へ渡す."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='新規セッションの保存先')
    parser.add_argument('--prepare-only', action='store_true', help='展開・検証だけ行う')
    parser.add_argument('--start-ui', action='store_true', help='運行UIも起動する')
    parser.add_argument('--domain-id', type=int, choices=range(1, 233), default=86,
                        metavar='1..232', help='模擬環境のDDS domain（既定86）')
    args = parser.parse_args()
    try:
        session = prepare_bundle(bundle_directory(), args.output)
    except (ValueError, OSError, KeyError, tarfile.TarError) as error:
        parser.exit(1, str(error)+'\n')
    data = json.loads(session.read_text())
    data['simulation_domain_id'] = args.domain_id
    session.write_text(json.dumps(data, indent=2)+'\n')
    print('固定地図を展開しました: '+str(session), flush=True)
    if args.prepare_only:
        return
    from icart_bringup.session_core import launch_session
    launch_session(session, 'simulation', start_ui=args.start_ui)


if __name__ == '__main__':
    main()
