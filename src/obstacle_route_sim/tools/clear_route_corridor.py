#!/usr/bin/env python3
"""元モデルを保持し、概略経路の全線分から 1 m 離隔する試験用モデルを作る."""

import argparse
import copy
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, MultiPoint, Point


def clearance_action(shape, route, margin: float = 1.01) -> tuple:
    """2 m 以内の平行移動を探索し、解消できなければ削除を返す."""
    distance = shape.distance(route)
    if distance >= margin:
        return 'keep', 0., 0., distance
    for radius in np.arange(.1, 2.001, .1):
        for angle in np.arange(0, 360, 5):
            dx, dy = radius*math.cos(math.radians(angle)), radius*math.sin(math.radians(angle))
            moved = affinity.translate(shape, dx, dy)
            if moved.distance(route) >= margin:
                return 'move', float(dx), float(dy), moved.distance(route)
    return 'remove', 0., 0., None


def generate(source: Path, output: Path) -> None:
    """衝突・表示形状を同期して更新し、離隔監査と変更記録を保存する."""
    if source.resolve() == output.resolve():
        raise ValueError('元モデルと出力先は別にする')
    output.mkdir(parents=True, exist_ok=True)
    world = json.loads((source/'world.json').read_text())
    data = json.loads((source/'viewer_data.json').read_text())
    trial = json.loads((source/'trial.json').read_text())
    route = LineString(trial['points'])
    changes, actions, shapes, remaining_meshes = [], {}, [], []
    for record in world['meshes']:
        name = record['name']
        if name == 'terrain':
            shutil.copyfile(source/'terrain.obj', output/'terrain.obj')
            remaining_meshes.append(record)
            continue
        lines = (source/(name+'.obj')).read_text().splitlines()
        vertices = np.array([list(map(float, line.split()[1:4]))
                             for line in lines if line.startswith('v ')])
        # 凸包は凹部も占有扱いにする。三角形間を見落とさない保守的な判定である。
        footprint = MultiPoint(vertices[:, :2]).convex_hull
        action, dx, dy, after = clearance_action(footprint, route)
        actions[name] = (action, dx, dy)
        if action != 'keep':
            changes.append(dict(name=name, id=record.get('id'), action=action,
                                dx=dx, dy=dy, before_m=footprint.distance(route), after_m=after))
        if action == 'remove':
            continue
        shapes.append(affinity.translate(footprint, dx, dy))
        translated = []
        for line in lines:
            if line.startswith('v '):
                x, y, z = map(float, line.split()[1:4])
                line = f'v {x+dx:.9f} {y+dy:.9f} {z:.9f}'
            translated.append(line)
        (output/(name+'.obj')).write_text('\n'.join(translated)+'\n')
        remaining_meshes.append(record)
    trees = []
    for index, tree in enumerate(world['trees']):
        # 球面多角形の近似で過小評価しないよう円判定半径を増やす。
        footprint = Point(tree['x'], tree['y']).buffer(tree['r']+.01, quad_segs=64)
        action, dx, dy, after = clearance_action(footprint, route)
        name = f'tree_{index}'
        actions[name] = (action, dx, dy)
        if action != 'keep':
            changes.append(dict(name=name,id=tree['id'],action=action,dx=dx,dy=dy,
                                before_m=footprint.distance(route),after_m=after))
        if action != 'remove':
            tree['x'] += dx
            tree['y'] += dy
            trees.append(tree)
            shapes.append(affinity.translate(footprint,dx,dy))
    by_id = {record.get('id'): actions[record['name']] for record in world['meshes']
             if record['name'] != 'terrain'}
    for index, tree in enumerate(data['trees']):
        by_id[tree['id']] = actions[f'tree_{index}']
    obstacles = []
    for obstacle in world['obstacles']:
        identifier = obstacle['id']
        action,dx,dy = by_id.get(identifier, by_id.get(identifier.removeprefix('tree:'), ('keep',0,0)))
        # 樹冠 proxy は表示樹木 ID と対応付ける。
        if obstacle['source'] == 'canopy-proxy':
            action,dx,dy = by_id.get(identifier.removeprefix('canopy:'), ('keep',0,0))
        if action == 'remove':
            continue
        for key in ['poly','pts']:
            if key in obstacle:obstacle[key]=[[x+dx,y+dy] for x,y in obstacle[key]]
        if 'holes' in obstacle:obstacle['holes']=[[[x+dx,y+dy] for x,y in h] for h in obstacle['holes']]
        if 'x' in obstacle:obstacle['x']+=dx;obstacle['y']+=dy
        if 'bb' in obstacle:obstacle['bb']=[v+(dx if i%2==0 else dy) for i,v in enumerate(obstacle['bb'])]
        obstacles.append(obstacle)
    world.update(meshes=remaining_meshes,trees=trees,obstacles=obstacles)
    world['worldId'] = str(world.get('worldId',''))+'-corridor1m'
    (output/'world.json').write_text(json.dumps(world,ensure_ascii=False))
    for filename in ['environment.sdf','trial.sdf']:
        xml = ET.parse(source/filename)
        parent = xml.getroot().find('world')
        for model in list(parent.findall('model')):
            action,dx,dy = actions.get(model.get('name'),('keep',0,0))
            if action == 'remove':parent.remove(model)
            elif action == 'move' and model.get('name').startswith('tree_'):
                pose=model.find('pose');values=list(map(float,pose.text.split()))
                values[0]+=dx;values[1]+=dy;pose.text=' '.join(map(str,values))
        ET.indent(xml)
        xml.write(output/filename,encoding='utf-8',xml_declaration=True)
    meshes=[]
    for mesh in data['meshes']:
        action,dx,dy=by_id[mesh['id']]
        if action=='remove':continue
        vertices=np.array(mesh['positions']).reshape(-1,3);vertices[:,:2]+=[dx,dy]
        mesh['positions']=vertices.reshape(-1).tolist();meshes.append(mesh)
    data.update(meshes=meshes,trees=trees,audit=None)
    counts=data['counts']
    counts.update(buildings=sum(m['layer']=='buildings' for m in meshes),trees=len(trees),
                  vehicle_candidates=sum(m['layer']=='vehicles' for m in meshes),
                  hedge_candidates=sum(m['layer']=='hedges' for m in meshes))
    data['counts']=counts
    trial.update(counts=counts,scenario='tc2026_corridor_1m',
                 route_status='公式図概略経路。モデルを離隔のため編集した試験用。実地再現性は未保証')
    (output/'trial.json').write_text(json.dumps(trial,ensure_ascii=False))
    (output/'viewer_data.json').write_text(json.dumps(data,ensure_ascii=False))
    for name in ['terrain.mtl','aerial_texture.jpg','route.csv','analysis_overlay.jpg']:
        shutil.copyfile(source/name,output/name)
    shutil.copytree(source/'viewer_vendor',output/'viewer_vendor',dirs_exist_ok=True)
    html=(source/'index.html').read_text().replace('全ルートのデジタルツイン','全ルート・離隔 1 m 調整版')
    html=html.replace('全域表示・2024年撮影画像／高さ倍率 1×','試験用調整版：モデルを移動・削除／左右 1 m 離隔')
    html=html.replace('<a href="editor.html" class="button" target="_blank">terrain3d で編集</a>',
                      '<a href="corridor_changes.json">モデル変更記録</a>')
    html=html.replace('href="scene.json"','href="world.json"').replace('シーンJSON','調整済み world JSON')
    html=html.replace('全域を再構成していますが、','通路確保のためモデルを変更しています。写真は変更前です。')
    (output/'index.html').write_text(html)
    minimum=min(shape.distance(route) for shape in shapes)
    report=dict(clearance_m=1.,target_m=1.01,max_translation_m=2.,
                method='全線分と物体凸包／樹冠円の平面離隔。地面は除外。上下の高さに関係なく空ける',
                minimum_clearance_m=minimum,remaining_models=len(shapes),
                remaining_conflicts=sum(shape.distance(route)<1. for shape in shapes),
                moved=sum(c['action']=='move' for c in changes),
                removed=sum(c['action']=='remove' for c in changes),changes=changes,
                limitations='試験用の人為的編集。現実の通行可能性、移動先の他物体との接触、全域走行は未検証')
    assert report['remaining_conflicts']==0
    (output/'corridor_changes.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='changes'},ensure_ascii=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();generate(args.source,args.output)
