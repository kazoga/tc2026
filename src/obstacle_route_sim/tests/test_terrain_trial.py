"""物理エンジン入力とユーザー指定の取付条件を確認する."""

import importlib.util
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import pytest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
SPEC = importlib.util.spec_from_file_location('build_terrain_trial', TOOLS/'build_terrain_trial.py')
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)


def test_sensor_positions_relative_to_axle() -> None:
    """リンク相対座標と車軸原点の取り違えを検出する."""
    world = ET.Element('world')
    BUILDER.make_robot(world)
    model = world.find('model')
    for name, expected in [('gnss_master', [0,0,.7]), ('gnss_slave', [-.5,0,.7])]:
        pose = list(map(float, model.find(f"link[@name='{name}']/pose").text.split()))
        assert pose[:3] == pytest.approx(expected)
    base = model.find("link[@name='base_link']")
    height = float(base.find('pose').text.split()[2])
    for name, expected in [('top_urg', .3), ('mid360', .6)]:
        relative = float(base.find(f"sensor[@name='{name}']/pose").text.split()[2])
        assert height+relative == pytest.approx(expected)


@pytest.mark.skipif(shutil.which('node') is None, reason='Node.js が必要')
def test_mesh_normals_and_model_references(tmp_path: Path) -> None:
    """実行時異常終了した法線欠落とメッシュ参照切れを検出する."""
    BUILDER.build(tmp_path, 'blocker')
    tree = ET.parse(tmp_path/'trial.sdf')
    for link in tree.findall('.//link'):
        names = [element.get('name') for element in link if element.get('name')]
        assert len(names) == len(set(names))
    for uri in tree.findall('.//mesh/uri'):
        mesh = tmp_path/uri.text
        lines = mesh.read_text().splitlines()
        vertices = [line for line in lines if line.startswith('v ')]
        normals = [line for line in lines if line.startswith('vn ')]
        assert len(vertices) == len(normals) > 0
        for line in normals:
            vector = list(map(float, line.split()[1:]))
            assert sum(v*v for v in vector) == pytest.approx(1)
        for line in lines:
            if line.startswith('f '):
                assert all('//' in index for index in line.split()[1:])


def test_drive_axis_matches_wheel_geometry() -> None:
    """関節の局所軸が車体の横軸に一致し、有限の駆動制約を持つことを確認する."""
    import math

    world = ET.Element('world')
    BUILDER.make_robot(world)
    model = world.find('model')
    for name in ['left_wheel', 'right_wheel']:
        roll = float(model.find(f"link[@name='{name}']/pose").text.split()[3])
        axis = model.find(f"joint[@name='{name}_joint']/axis")
        x, y, z = map(float, axis.find('xyz').text.split())
        assert [x, math.cos(roll)*y-math.sin(roll)*z,
                math.sin(roll)*y+math.cos(roll)*z] == pytest.approx([0., 1., 0.], abs=1e-9)
        assert 0 < float(axis.find('limit/effort').text) < 10
        assert float(axis.find('limit/velocity').text)*.07455 >= .9
