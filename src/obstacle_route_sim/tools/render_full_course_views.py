#!/usr/bin/env python3
"""全域モデルの実メッシュから俯瞰・局所確認用の PNG を生成する."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from PIL import Image


def render(source: Path, output: Path) -> None:
    """地形、物体メッシュ、樹木、概略経路を同じ座標で描画する."""
    data = json.loads((source / 'viewer_data.json').read_text())
    output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False})
    grid = data['grid']
    heights = np.array(grid['height']).reshape(grid['h'], grid['w'])
    xs = grid['ox'] + np.arange(grid['w']) * grid['res']
    ys = grid['oy'] + np.arange(grid['h']) * grid['res']
    photo = np.asarray(Image.open(source / 'aerial_texture.jpg').convert('RGB')) / 255
    width, depth = data['dimensions']
    palette = {'buildings': '#c5ced7', 'vehicles': '#7598b8',
               'hedges': '#598348', 'street': '#73808a'}
    views = [
        ('01_full', '全域俯瞰', (-410, 410, -220, 220), 48, -65),
        ('02_cityhall', '市役所・スタート／ゴール周辺', (-355, -155, -145, 90), 48, -65),
        ('03_connection', '市役所東側・接続道路', (-165, 75, -65, 100), 55, -75),
        ('04_hotel', '北側沿道・折り返し区間', (5, 285, -45, 120), 48, -65),
        ('05_station', '駅前・ホテル南側', (120, 335, -205, -30), 50, -70),
    ]
    route = np.asarray(data['route'])
    for name, title, bounds, elev, azim in views:
        x0, x1, y0, y1 = bounds
        fig = plt.figure(figsize=(14, 10), facecolor='#f3f7fa')
        fig.text(.035, .945, 'つくばチャレンジ2026 ｜ ' + title,
                 fontsize=22, weight='bold', color='#18334a')
        fig.text(.035, .903, 'デジタルツインの実メッシュ表示　｜　高さ倍率 1×',
                 fontsize=13, color='#496074')
        ax = fig.add_axes([.01, .15, .98, .73], projection='3d', computed_zorder=False)
        ax.set_facecolor('#f3f7fa')
        ix = np.where((xs >= x0) & (xs <= x1))[0]
        iy = np.where((ys >= y0) & (ys <= y1))[0]
        step = max(1, len(ix) // 250)
        ix, iy = ix[::step], iy[::step]
        xx, yy = np.meshgrid(xs[ix], ys[iy])
        zz = heights[np.ix_(iy, ix)]
        px = np.clip(((xx / width + .5) * (photo.shape[1]-1)).astype(int), 0, photo.shape[1]-1)
        py = np.clip(((.5 - yy / depth) * (photo.shape[0]-1)).astype(int), 0, photo.shape[0]-1)
        ax.plot_surface(xx, yy, zz, facecolors=photo[py, px], rstride=1, cstride=1,
                        shade=False, linewidth=0, antialiased=False, zorder=1)
        for mesh in data['meshes']:
            vertices = np.asarray(mesh['positions']).reshape(-1, 3)
            if (vertices[:, 0].max() < x0 or vertices[:, 0].min() > x1 or
                    vertices[:, 1].max() < y0 or vertices[:, 1].min() > y1):
                continue
            triangles = vertices.reshape(-1, 3, 3)
            # 面の傾きで明暗を付け、建物の立体形状を判別しやすくする。
            normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
            normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-10)
            light = .68 + .32 * np.abs(normals @ np.array([.3, -.4, .866]))
            rgb = np.array(matplotlib.colors.to_rgb(palette[mesh['layer']]))
            ax.add_collection3d(Poly3DCollection(triangles, facecolors=light[:, None]*rgb,
                                                edgecolors='none', zorder=2))
        for tree in data['trees']:
            if not (x0 <= tree['x'] <= x1 and y0 <= tree['y'] <= y1):
                continue
            angle = np.linspace(0, 2*np.pi, 9)
            ring = np.column_stack([tree['x']+tree['r']*np.cos(angle),
                                    tree['y']+tree['r']*np.sin(angle),
                                    np.full(9, tree['z']+tree['h']*.65)])
            top = [tree['x'], tree['y'], tree['z']+tree['h']]
            bottom = [tree['x'], tree['y'], tree['z']+tree['h']*.35]
            faces = [[ring[i], ring[i+1], tip] for i in range(8) for tip in [top, bottom]]
            ax.add_collection3d(Poly3DCollection(faces, facecolors='#4c7b43',
                                                edgecolors='none', zorder=3))
        ri = np.clip(np.rint((route[:, 0]-grid['ox'])/grid['res']).astype(int), 0, grid['w']-1)
        rj = np.clip(np.rint((route[:, 1]-grid['oy'])/grid['res']).astype(int), 0, grid['h']-1)
        rz = heights[rj, ri]+.7
        visible = ((route[:, 0]>=x0)&(route[:, 0]<=x1)&(route[:, 1]>=y0)&(route[:, 1]<=y1))
        rr = np.where(visible[:, None], route[:, :2], np.nan)
        ax.plot(rr[:, 0], rr[:, 1], rz, color='#ffcd13', linewidth=2.2, zorder=5)
        ax.set(xlim=(x0,x1), ylim=(y0,y1), zlim=(-3,55))
        ax.set_box_aspect((x1-x0,y1-y0,58))
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
        fig.text(.035,.115,'黄色：公式図からの概略ルート（走行済み軌跡ではありません）',
                 fontsize=12,color='#4c6072')
        fig.text(.035,.080,'建物・樹木・車両・植栽を近似再構成。高さ・位置は未校正。全コース完走は未確認。',
                 fontsize=11,color='#4c6072')
        fig.text(.035,.047,'写真：2024-01-10 ／ 国土地理院・© OpenStreetMap contributors・Esri / Vantor / Earthstar Geographics / GIS User Community',
                 fontsize=8,color='#607487')
        fig.savefig(output / (name+'.png'), dpi=160, facecolor=fig.get_facecolor())
        plt.close(fig)
        print(output / (name+'.png'), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    render(args.source, args.output)
