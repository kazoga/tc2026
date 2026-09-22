"""記録済みの経路を検証し、自律走行設定へ渡す。"""
import csv
import json
import math
from pathlib import Path


def inspect_recorded_route(directory):
    directory = Path(directory).expanduser().resolve()
    for name in ('survey.json', 'route_config.yaml', 'projection.yaml', 'fixed/waypoints.csv'):
        if not (directory / name).is_file():
            raise ValueError(f'記録ルートに必要なファイルがありません: {name}')
    data = json.loads((directory / 'survey.json').read_text())
    if data.get('active') is not False:
        raise ValueError('ルート記録を「終了・保存」してから選択してください')
    with (directory / 'fixed/waypoints.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError('自律走行には2点以上の経路が必要です')
    points = []
    for row in rows:
        lat, lon = float(row['latitude']), float(row['longitude'])
        if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError('経路の緯度・経度が不正です')
        points.append((lat, lon))
    source_session = directory.parent / 'session/session.yaml'
    if not source_session.is_file():
        raise ValueError('採取時の実機設定（隣接するsession/session.yaml）がありません')
    import yaml
    session = yaml.safe_load(source_session.read_text())
    if session.get('site') not in ('inagi', 'tsukuba'):
        raise ValueError('採取時の場所を確認できません')
    if not (source_session.parent / 'hardware.yaml').is_file():
        raise ValueError('採取時のhardware.yamlがありません')
    return dict(directory=str(directory), count=len(rows), points=points,
                start=rows[0]['label'], goal=rows[-1]['label'], site=session['site'],
                station=session['rtk_station'], session=str(source_session),
                unknown_width=sum(float(r['left_is_open']) <= 0 or float(r['right_is_open']) <= 0 for r in rows))
