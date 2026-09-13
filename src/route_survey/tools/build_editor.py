#!/usr/bin/env python3
"""採取JSONを、通信不要で編集できる単体HTMLへ変換する."""
import argparse
import base64
import json
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory


def build(source: Path, output: Path, trial: Path | None = None,
          map_points: Path | None = None) -> None:
    data = json.loads(source.read_text())
    data['background'] = None
    if trial:
        meta = json.loads((trial/'trial.json').read_text())
        if meta['projection'] != {k: data['projection'][k] for k in meta['projection']}:
            raise ValueError('航空写真と経路の投影設定が異なります')
        texture = trial/'aerial_texture.jpg'
        if texture.exists():
            data['background'] = dict(image='data:image/jpeg;base64,'+
                base64.b64encode(texture.read_bytes()).decode(), dimensions=meta['dimensions_m'])
        world = json.loads((trial/'world.json').read_text())
        data['obstacles'] = world.get('obstacles', [])
        data['background_notice'] = '航空写真: Esri World Imagery（既存取得画像）。地図: © OpenStreetMap contributors。デジタルツイン由来で現地測量ではない。'
    if map_points:
        points = np.load(map_points)
        points = points[np.isfinite(points[:, :3]).all(axis=1), :3]
        data['cloud'] = points[::max(1, int(np.ceil(len(points)/50000)))].tolist()
        data['cloud_source'] = '指定されたmap座標点群。原点・位置合わせの妥当性は利用者確認。'
    local = Path(__file__).parents[1]/'web/editor.html'
    template = local if local.exists() else Path(get_package_share_directory('route_survey'))/'web/editor.html'
    payload = json.dumps(data, ensure_ascii=False).replace('<', '\\u003c')
    output.write_text(template.read_text().replace('__SURVEY_DATA__', payload), encoding='utf-8')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--survey', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--trial', type=Path)
    p.add_argument('--map-points', type=Path)
    a = p.parse_args()
    build(a.survey, a.output, a.trial, a.map_points)
