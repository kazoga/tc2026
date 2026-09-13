#!/usr/bin/env python3
"""FAST-LIO の登録済み観測を地図化し、SDF 表面と航空写真上で比較する."""
import argparse
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.ndimage import map_coordinates


def voxel_points(xyz: np.ndarray, resolution: float) -> np.ndarray:
    """各 voxel の最初の実観測を残し、存在しない面を補間しない."""
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    _, index = np.unique(np.floor(xyz/resolution).astype(np.int64),
                         axis=0, return_index=True)
    return xyz[index].astype(np.float32)


def write_pcd(path: Path, xyz: np.ndarray) -> None:
    """標準 binary PCD の XYZ float32 として保存する."""
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
              'TYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(xyz)}\nDATA binary\n')
    with path.open('wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(np.asarray(xyz, dtype='<f4').tobytes())


def transform(element: ET.Element) -> np.ndarray:
    """SDF の固定 pose を同次変換にする。relative_to は対象外とする."""
    pose = element.find('pose')
    if pose is not None and pose.get('relative_to'):
        raise ValueError('relative_to は未対応')
    values = np.array(list(map(float, element.findtext('pose', '0 0 0 0 0 0').split())))
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


def sample_mesh(path: Path, spacing: float) -> np.ndarray:
    """三角面の重心座標格子を生成する。各小辺を spacing 以下にする."""
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'v':
            vertices.append(list(map(float, fields[1:4])))
        elif fields[0] == 'f':
            face = [int(field.split('/')[0])-1 for field in fields[1:]]
            faces.extend([[face[0], face[i], face[i+1]] for i in range(1, len(face)-1)])
    vertices = np.array(vertices)
    chunks = []
    for face in faces:
        a, b, c = vertices[face]
        length = max(np.linalg.norm(b-a), np.linalg.norm(c-a), np.linalg.norm(c-b))
        n = max(1, math.ceil(length/spacing))
        for i in range(n+1):
            j = np.arange(n-i+1)
            chunks.append(a+(b-a)*(i/n)+j[:, None]*(c-a)/n)
    return np.concatenate(chunks).astype(np.float32)


def sample_primitive(geometry: ET.Element, spacing: float) -> np.ndarray:
    """描画に使われた円柱・楕円体等の表面を格子化する."""
    shape = list(geometry)[0]
    if shape.tag in ['ellipsoid', 'sphere']:
        radii = (np.array(list(map(float, shape.findtext('radii').split())))
                 if shape.tag == 'ellipsoid' else np.repeat(float(shape.findtext('radius')), 3))
        n = max(12, math.ceil(2*np.pi*max(radii)/spacing))
        u, v = np.meshgrid(np.linspace(0, 2*np.pi, n), np.linspace(0, np.pi, n//2+1))
        return np.column_stack([np.cos(u).ravel()*np.sin(v).ravel(),
                                np.sin(u).ravel()*np.sin(v).ravel(), np.cos(v).ravel()])*radii
    if shape.tag == 'cylinder':
        radius, length = float(shape.findtext('radius')), float(shape.findtext('length'))
        angles = np.linspace(0, 2*np.pi, max(12, math.ceil(2*np.pi*radius/spacing)))
        a, z = np.meshgrid(angles, np.linspace(-length/2, length/2,
                                             max(2, math.ceil(length/spacing)+1)))
        side = np.column_stack([radius*np.cos(a).ravel(), radius*np.sin(a).ravel(), z.ravel()])
        a, r = np.meshgrid(angles, np.linspace(0, radius, max(2, math.ceil(radius/spacing)+1)))
        cap = np.column_stack([r.ravel()*np.cos(a).ravel(), r.ravel()*np.sin(a).ravel(),
                               np.full(a.size, length/2)])
        return np.concatenate([side, cap, cap*np.array([1, 1, -1])])
    if shape.tag == 'box':
        size = np.array(list(map(float, shape.findtext('size').split())))
        faces = []
        for axis in range(3):
            other = [i for i in range(3) if i != axis]
            u, v = np.meshgrid(*[np.linspace(-size[i]/2, size[i]/2,
                                max(2, math.ceil(size[i]/spacing)+1)) for i in other])
            for sign in [-1, 1]:
                face = np.empty((u.size, 3))
                face[:, axis] = sign*size[axis]/2
                face[:, other[0]], face[:, other[1]] = u.ravel(), v.ravel()
                faces.append(face)
        return np.concatenate(faces)
    raise ValueError('未対応の形状: '+shape.tag)


def reference_surface(directory: Path, spacing: float = .5) -> np.ndarray:
    """LiDAR が観測する静的 visual 表面を取り出す。車体を除く."""
    cache = directory/'reference_surface.npy'
    if cache.exists():
        return np.load(cache)
    chunks = []
    world = ET.parse(directory/'trial.sdf').find('world')
    for model in world.findall('model'):
        if model.get('name') == 'icart_mini':
            continue
        for link in model.findall('link'):
            for visual in link.findall('visual'):
                geo = visual.find('geometry')
                if geo.find('mesh') is not None:
                    mesh = geo.find('mesh')
                    points = sample_mesh(directory/mesh.findtext('uri'), spacing)
                    points *= np.array(list(map(float, mesh.findtext('scale', '1 1 1').split())))
                else:
                    points = sample_primitive(geo, spacing)
                matrix = transform(model)@transform(link)@transform(visual)
                chunks.append((points@matrix[:3, :3].T+matrix[:3, 3]).astype(np.float32))
    result = voxel_points(np.concatenate(chunks), .1)
    np.save(cache, result)
    return result


def evaluate(directory: Path) -> dict:
    """初回位置・yaw のみで整合し、全体の ICP 補正を行わず評価する."""
    chunks = []
    for index, file in enumerate(sorted((directory/'lio_map_scans').glob('*.npz'))):
        chunks.append(np.load(file)['xyz'])
        if index % 100 == 99:
            chunks = [voxel_points(np.concatenate(chunks), .25)]
    raw = voxel_points(np.concatenate(chunks), .25)
    write_pcd(directory/'fastlio_map.pcd', raw)
    rows = list(csv.DictReader((directory/'trajectory.csv').open()))
    truth = np.array([[float(r[k]) for k in ['sim_s', 'x', 'y', 'z', 'yaw']] for r in rows])
    rows = list(csv.DictReader((directory/'lio_trajectory.csv').open()))
    lio = np.array([[float(r[k]) for k in ['sim_s', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']]
                    for r in rows])
    first = lio[(lio[:, 0] >= truth[0, 0]) & (lio[:, 0] <= truth[-1, 0])][0]
    position = np.array([np.interp(first[0], truth[:, 0], truth[:, i]) for i in [1, 2, 3]])
    position[2] += .6
    yaw = np.interp(first[0], truth[:, 0], np.unwrap(truth[:, 4]))
    yaw -= Rotation.from_quat(first[4:8]).as_euler('xyz')[2]
    rotation = Rotation.from_euler('z', yaw).as_matrix()
    aligned = (raw-first[1:4])@rotation.T+position
    write_pcd(directory/'fastlio_map_enu.pcd', aligned)
    np.save(directory/'fastlio_map_enu.npy', aligned.astype(np.float32))
    reference = reference_surface(directory)
    # 各点は面上にあり、これは点対サンプル距離。厳密な面距離ではない。
    tree = cKDTree(reference)
    distance, _ = tree.query(aligned, workers=2)
    grid = json.loads((directory/'world.json').read_text())['grid']
    heights = np.array(grid['height']).reshape(grid['h'], grid['w'])
    ground = map_coordinates(heights,
        [(aligned[:, 1]-grid['oy'])/grid['res'], (aligned[:, 0]-grid['ox'])/grid['res']],
        order=1, mode='nearest')
    above_ground = aligned[:, 2]-ground > 1.0
    result = dict(map_points=len(raw), reference_points=len(reference),
                  surface_sample_spacing_m=.5,
                  sample_distance_median_m=float(np.median(distance)),
                  sample_distance_p95_m=float(np.percentile(distance, 95)),
                  sample_distance_rmse_m=float(np.sqrt(np.mean(distance**2))),
                  within_05m_fraction=float(np.mean(distance <= .5)),
                  within_1m_fraction=float(np.mean(distance <= 1.0)),
                  above_ground_points=int(above_ground.sum()),
                  above_ground_p95_m=(float(np.percentile(distance[above_ground], 95))
                                      if above_ground.any() else None),
                  above_ground_within_1m_fraction=(float(np.mean(distance[above_ground] <= 1.0))
                                                  if above_ground.any() else None),
                  alignment=dict(initial_lio_xyz=first[1:4].tolist(),
                                 initial_reference_imu_xyz=position.tolist(), yaw_rad=float(yaw)),
                  limitations=['初回 XY/Z/yaw のみ。姿勢による 0.6 m lever arm 補正は未実施',
                      '参照は生成元 SDF の visual 面。現地測量や独立実測点群との比較ではない',
                      '表面標本との距離にはサンプリング誤差がある。厳密な point-to-mesh 距離ではない',
                      '1 Hz と 0.25 m voxel の観測集積。FAST-LIO 内部 ikd-tree の直接出力ではない',
                      '未観測面を補完せず、遮蔽面を含む地図全体の完全性は保証しない'])
    (directory/'map_evaluation.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    np.save(directory/'map_surface_distances.npy', distance.astype(np.float32))
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False})
    extent = [grid['ox'], grid['ox']+(grid['w']-1)*grid['res'],
              grid['oy'], grid['oy']+(grid['h']-1)*grid['res']]
    stride = max(1, len(aligned)//250000)
    fig, axes = plt.subplots(2, 1, figsize=(12, 13), facecolor='#f5f8fb')
    axes[0].imshow(plt.imread(directory/'aerial_texture.jpg'), extent=extent, alpha=.75)
    axes[0].scatter(*aligned[::stride, :2].T, c=aligned[::stride, 2], s=.3, cmap='viridis',
                    vmin=0, vmax=20)
    axes[0].plot(*truth[:, 1:3].T, color='red', lw=.7, label='真値走行軌跡')
    axes[0].set_title('航空写真＋FAST-LIO 点群地図（高さで着色）')
    axes[0].legend()
    scatter = axes[1].scatter(*aligned[::stride, :2].T, c=distance[::stride],
                              s=.4, cmap='turbo', vmin=0, vmax=2)
    fig.colorbar(scatter, ax=axes[1], label='SDF 表面サンプルへの距離 [m]（2 m 以上は飽和）')
    axes[1].set_title('モデル形状との距離。全体位置合わせで drift を消していない')
    for axis in axes:
        axis.set_aspect('equal')
        axis.set(xlabel='東方向 [m]', ylabel='北方向 [m]', xlim=extent[:2], ylim=extent[2:])
    fig.suptitle('全域 FAST-LIO 点群地図の検証', fontsize=22)
    fig.tight_layout()
    fig.savefig(directory/'map_comparison.png', dpi=160)
    plt.close(fig)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    print(json.dumps(evaluate(parser.parse_args().directory), ensure_ascii=False, indent=2))
