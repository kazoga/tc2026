#!/usr/bin/env python3
"""概略ルートとモデルの二次元形状の重なりを監査する."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def audit(output: Path) -> dict:
    """車体半径 0.20 m の近似で要確認箇所を列挙する."""
    trial = json.loads((output/'trial.json').read_text())
    world = json.loads((output/'world.json').read_text())
    issues = []
    for index, (x, y) in enumerate(trial['points']):
        for obstacle in world['obstacles']:
            if obstacle['source'] == 'canopy-proxy':
                continue
            if obstacle['z1']-obstacle['z0'] < .05:
                continue
            if obstacle['type'] == 'circle':
                distance = math.hypot(x-obstacle['x'], y-obstacle['y'])-obstacle['r']
            elif obstacle['type'] == 'polygon':
                polygon = np.asarray(obstacle['poly'], np.float32)
                distance = -cv2.pointPolygonTest(polygon, (x, y), True)
                if any(cv2.pointPolygonTest(np.asarray(h, np.float32), (x, y), False) >= 0
                       for h in obstacle.get('holes', [])):
                    continue
            elif obstacle['type'] == 'segment':
                a, b = np.asarray(obstacle['pts'])
                v = b-a
                u = np.clip(np.dot(np.array([x, y])-a, v)/max(np.dot(v, v), 1e-9), 0, 1)
                distance = np.linalg.norm(np.array([x, y])-(a+u*v))-obstacle['r']
            else:
                continue
            if distance < .2:
                issues.append(dict(route_index=index, x=x, y=y, obstacle=obstacle['id'],
                                   source=obstacle['source'], clearance_m=float(distance)))
                break
    result = dict(
        method='2D footprint + 0.20 m radius; canopy excluded; no height clearance or dynamics',
        total_route_samples=len(trial['points']), conflicting_samples=len(issues),
        unique_obstacles=len({item['obstacle'] for item in issues}), conflicts=issues,
        interpretation='概略経路と推定障害物の不整合候補。実地の通行不能を意味しない')
    (output/'geometry_audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    result = audit(parser.parse_args().output)
    print(json.dumps({key: value for key, value in result.items() if key != 'conflicts'},
                     ensure_ascii=False))
