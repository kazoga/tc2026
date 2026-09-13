"""固定地図が外部ファイルなしで展開でき、不正archiveを拒否することを確認する."""

import hashlib
import io
import json
from pathlib import Path
import sys
import tarfile
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from icart_bringup.digital_twin_core import bundle_directory, prepare_bundle


def test_committed_bundle_is_self_contained(tmp_path: Path) -> None:
    output = tmp_path/'relocated session'
    session = prepare_bundle(bundle_directory(), output)
    data = json.loads(session.read_text())
    assert data['hardware_launch'] == ''
    assert data['simulation_domain_id'] != data['real_domain_id']
    for key in ['trial_directory', 'trial_sdf', 'projection_params', 'csv_base_dir', 'route_config']:
        assert not Path(data[key]).is_absolute()
        assert (output/data[key]).exists()
    world = output/'world'
    tree = ET.parse(world/'trial.sdf')
    assert len(tree.findall(".//sensor[@type='imu']")) == 1
    for element in tree.findall('.//uri'):
        assert (world/element.text).is_file()
    for material in world.glob('*.mtl'):
        assert 'map_Kd' not in material.read_text()
    assert len((output/'routes/fixed/waypoints.csv').read_text().splitlines()) == 1143
    with pytest.raises(ValueError, match='新規'):
        prepare_bundle(bundle_directory(), output)


def make_archive(tmp_path: Path, member: tarfile.TarInfo, data: bytes = b'x') -> Path:
    bundle = tmp_path/'bundle'
    bundle.mkdir()
    member.size = len(data)
    archive = bundle/'world.tar.gz'
    with tarfile.open(archive, 'w:gz') as stream:
        stream.addfile(member, io.BytesIO(data))
    manifest = dict(version=1, archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                    files={member.name: dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())})
    (bundle/'manifest.json').write_text(json.dumps(manifest))
    return bundle


@pytest.mark.parametrize('name', ['../escape', '/absolute', 'a/../../escape'])
def test_archive_cannot_escape_output(tmp_path: Path, name: str) -> None:
    bundle = make_archive(tmp_path, tarfile.TarInfo(name))
    with pytest.raises(ValueError, match='メンバー'):
        prepare_bundle(bundle, tmp_path/'session')
    assert not (tmp_path/'session').exists()
    assert not (tmp_path/'escape').exists()


def test_archive_symlink_is_rejected(tmp_path: Path) -> None:
    member = tarfile.TarInfo('link')
    member.type, member.linkname = tarfile.SYMTYPE, '../escape'
    bundle = make_archive(tmp_path, member)
    with pytest.raises(ValueError, match='メンバー'):
        prepare_bundle(bundle, tmp_path/'session')


def test_corrupt_archive_is_rejected_before_extraction(tmp_path: Path) -> None:
    bundle = make_archive(tmp_path, tarfile.TarInfo('file'))
    with (bundle/'world.tar.gz').open('ab') as stream:
        stream.write(b'corrupt')
    with pytest.raises(ValueError, match='SHA-256'):
        prepare_bundle(bundle, tmp_path/'session')
    assert not (tmp_path/'session').exists()


def test_missing_files_do_not_publish_partial_session(tmp_path: Path) -> None:
    bundle = make_archive(tmp_path, tarfile.TarInfo('file'))
    with pytest.raises(ValueError, match='必須'):
        prepare_bundle(bundle, tmp_path/'session')
    assert not (tmp_path/'session').exists()


def test_preview_contains_no_external_asset_requests() -> None:
    html = (bundle_directory()/'preview.html').read_text()
    assert 'fetch(\'viewer_data.json\')' not in html
    assert '<script src=' not in html
    assert '<img src=' not in html
    assert 'aerial_texture.jpg' not in html
    assert 'new T.TextureLoader()' not in html
    assert '© OpenStreetMap contributors' in html
