"""取付角によるビーム・地面観測・SDF/IMU姿勢の変化を確認する。"""
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

TOOLS = Path(__file__).parents[1]/'tools'
sys.path.insert(0, str(TOOLS))
from build_terrain_trial import make_robot
from prepare_fastlio_trial import add_imu, prepare


@pytest.mark.parametrize('angle', [0., 25., -25.])
def test_lidar_and_imu_mount_rotate_together(angle):
    root = ET.Element('sdf')
    world = ET.SubElement(root, 'world')
    make_robot(world, angle)
    urg = ET.tostring(root.find('.//sensor[@name="top_urg"]'))
    add_imu(root)
    lidar_pose = root.findtext('.//sensor[@name="mid360"]/pose')
    assert root.findtext('.//sensor[@name="lio_imu"]/pose') == lidar_pose
    assert float(lidar_pose.split()[4]) == pytest.approx(math.radians(angle))
    assert ET.tostring(root.find('.//sensor[@name="top_urg"]')) == urg


@pytest.mark.parametrize('angle', [float('nan'), float('inf'), 90., -90.])
def test_invalid_mount_fails_without_creating_robot(angle):
    world = ET.Element('world')
    with pytest.raises(ValueError):
        make_robot(world, angle)
    assert world.find('model') is None


def test_preparation_updates_scene_and_sdf_without_changing_source(tmp_path):
    source = tmp_path/'source'
    source.mkdir()
    root = ET.Element('sdf')
    make_robot(ET.SubElement(root, 'world'))
    ET.ElementTree(root).write(source/'trial.sdf')
    (source/'scene.json').write_text(json.dumps({'robots': [{'id': 'icart_mini'}]}))
    (source/'trial.json').write_text('{}')
    original = (source/'trial.sdf').read_bytes()
    out = tmp_path/'pitched'
    prepare(source, out, 25.)
    result = ET.parse(out/'trial.sdf')
    assert float(result.findtext('.//sensor[@name="mid360"]/pose').split()[4]) == pytest.approx(math.radians(25))
    assert result.findtext('.//sensor[@name="mid360"]/pose') == result.findtext('.//sensor[@name="lio_imu"]/pose')
    assert json.loads((out/'scene.json').read_text())['robots'][0]['sensors']['lidarPitchDownDeg'] == pytest.approx(25)
    assert (source/'trial.sdf').read_bytes() == original


@pytest.mark.skipif(shutil.which('node') is None, reason='Node.jsが必要')
def test_reference_lidar_and_observed_traversability():
    result = subprocess.run(['node', str(Path(__file__).with_name('lidar_mount_checks.cjs')),
                             str(TOOLS/'terrain3d/terrain3d_v0_4_1.html')],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
