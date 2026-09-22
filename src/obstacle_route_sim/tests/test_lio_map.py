"""地図評価が面外の点を作らず、座標系を維持することを確認する."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]/'tools'))
from evaluate_lio_map import sample_mesh, sample_primitive, transform, voxel_points, write_pcd


def test_triangle_surface_and_interior_sampling(tmp_path: Path) -> None:
    mesh = tmp_path/'triangle.obj'
    mesh.write_text('v 0 0 2\nv 2 0 2\nv 0 2 2\nf 1 2 3\n')
    points = sample_mesh(mesh, .25)
    assert len(points) > 30
    assert np.allclose(points[:, 2], 2)
    assert np.all(points[:, :2] >= 0)
    assert np.all(points[:, :2].sum(axis=1) <= 2.00001)


def test_primitive_surfaces_respect_dimensions() -> None:
    box = sample_primitive(ET.fromstring('<geometry><box><size>2 4 6</size></box></geometry>'), .5)
    assert np.allclose(np.max(abs(box), axis=0), [1, 2, 3])
    assert np.all(np.any(np.isclose(abs(box), [1, 2, 3]), axis=1))
    ellipsoid = sample_primitive(ET.fromstring(
        '<geometry><ellipsoid><radii>2 3 4</radii></ellipsoid></geometry>'), .5)
    assert np.allclose(np.sum((ellipsoid/[2, 3, 4])**2, axis=1), 1)


def test_pose_rotation_and_unsupported_reference() -> None:
    pose = ET.fromstring('<model><pose>1 2 3 0 0 1.5707963267948966</pose></model>')
    assert transform(pose)@np.array([1, 0, 0, 1]) == pytest.approx([1, 3, 3, 1])
    with pytest.raises(ValueError):
        transform(ET.fromstring('<model><pose relative_to="other">0 0 0 0 0 0</pose></model>'))


def test_voxel_keeps_observations_and_binary_pcd(tmp_path: Path) -> None:
    points = np.array([[.01, 0, 0], [.02, 0, 0], [.3, 0, 0], [np.nan, 0, 0]])
    result = voxel_points(points, .25)
    assert result.shape == (2, 3)
    assert np.allclose(result[:, 0], [.01, .3])
    path = tmp_path/'map.pcd'
    write_pcd(path, result)
    header, binary = path.read_bytes().split(b'DATA binary\n')
    assert b'POINTS 2' in header
    assert np.allclose(np.frombuffer(binary, dtype='<f4').reshape(-1, 3), result)


def test_map_pipeline_aligns_initial_pose_without_trajectory_fit(tmp_path: Path) -> None:
    """90 度異なる推定座標から既知の床面へ整合し、全出力を確認する."""
    import json
    from PIL import Image
    from evaluate_lio_map import evaluate

    scans = tmp_path/'lio_map_scans'
    scans.mkdir()
    np.savez(scans/'000000.npz', xyz=np.array([[1, 0, -.6], [0, 1, -.6]]), stamp=0.)
    (tmp_path/'trajectory.csv').write_text(
        'sim_s,x,y,z,yaw\n0,10,20,0,1.5707963267948966\n10,10,21,0,1.5707963267948966\n')
    (tmp_path/'lio_trajectory.csv').write_text(
        'sim_s,x,y,z,qx,qy,qz,qw\n0,0,0,0,0,0,0,1\n10,1,0,0,0,0,0,1\n')
    (tmp_path/'ground.obj').write_text(
        'v 8 19 0\nv 12 19 0\nv 12 23 0\nv 8 23 0\nf 1 2 3\nf 1 3 4\n')
    (tmp_path/'trial.sdf').write_text('<sdf><world><model name="ground"><link><visual>'
        '<geometry><mesh><uri>ground.obj</uri></mesh></geometry></visual></link></model></world></sdf>')
    (tmp_path/'world.json').write_text(json.dumps(dict(grid=dict(
        ox=8, oy=19, w=3, h=3, res=2, height=[0]*9))))
    Image.new('RGB', (4, 4), 'white').save(tmp_path/'aerial_texture.jpg')
    result = evaluate(tmp_path)
    xyz = np.load(tmp_path/'fastlio_map_enu.npy')
    assert xyz[:, :2] == pytest.approx(np.array([[9, 20], [10, 21]]))
    assert np.allclose(xyz[:, 2], 0, atol=np.finfo(np.float32).eps)
    assert result['within_05m_fraction'] == 1.
    assert (tmp_path/'map_comparison.png').is_file()
