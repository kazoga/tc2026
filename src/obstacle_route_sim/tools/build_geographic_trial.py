#!/usr/bin/env python3
"""提供版 terrain3d の地理データ処理を使って現地小区間を再構成する."""

import argparse
import base64
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

from build_terrain_trial import child, make_robot


def download(url: str, target: Path, data: bytes | None = None) -> bytes:
    """取得原本を保存し、再実行時は同じ資料で再現する."""
    if target.exists():
        return target.read_bytes()
    request = urllib.request.Request(url, data=data, headers={'User-Agent': 'terrain3d-course-trial/1.0'})
    with urllib.request.urlopen(request, timeout=60) as response:
        content = response.read()
    target.write_bytes(content)
    return content


def build(output: Path, route: Path, count: int) -> None:
    output.mkdir(parents=True, exist_ok=True)
    sources = output/'source'
    sources.mkdir(exist_ok=True)
    rows = list(csv.DictReader(route.open()))[:count]
    if len(rows) < 2:
        raise ValueError('経路は 2 点以上必要')
    lat0, lon0 = float(rows[0]['latitude']), float(rows[0]['longitude'])
    scale = 6371000*math.pi/180
    points = [((float(r['longitude'])-lon0)*scale*math.cos(math.radians(lat0)),
               (float(r['latitude'])-lat0)*scale) for r in rows]
    side = math.ceil((max(max(abs(x), abs(y)) for x, y in points)+35)*2/10)*10
    bbox = [lat0-side/2/scale, lon0-side/2/scale/math.cos(math.radians(lat0)),
            lat0+side/2/scale, lon0+side/2/scale/math.cos(math.radians(lat0))]
    n = int(side)+1
    latitudes = np.linspace(bbox[2], bbox[0], n)
    longitudes = np.linspace(bbox[1], bbox[3], n)
    x = (longitudes+180)/360*2**15*256
    y = (1-np.arcsinh(np.tan(np.radians(latitudes)))/math.pi)/2*2**15*256
    heights = np.full((n,n), np.nan)
    used = []
    for layer in ['dem5a_png', 'dem5b_png', 'dem5c_png']:
        for tx in range(int(x.min())//256, int(x.max())//256+1):
            for ty in range(int(y.min())//256, int(y.max())//256+1):
                ix = np.flatnonzero((x//256) == tx)
                iy = np.flatnonzero((y//256) == ty)
                if not np.isnan(heights[np.ix_(iy,ix)]).any():
                    continue
                url = f'https://cyberjapandata.gsi.go.jp/xyz/{layer}/15/{tx}/{ty}.png'
                target = sources/f'{layer}_15_{tx}_{ty}.png'
                try:
                    rgb = np.asarray(Image.open(io.BytesIO(download(url,target))).convert('RGB'), dtype=np.int32)
                except Exception as error:
                    used.append(dict(url=url, error=str(error)))
                    continue
                value = rgb[:,:,0]*65536+rgb[:,:,1]*256+rgb[:,:,2]
                dem = np.where(value < 8388608, value, value-16777216)*.01
                dem[value == 8388608] = np.nan
                sample = dem[y[iy].astype(int)%256][:,x[ix].astype(int)%256]
                old = heights[np.ix_(iy,ix)]
                heights[np.ix_(iy,ix)] = np.where(np.isnan(old), sample, old)
                used.append(dict(url=url, sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
        if np.isfinite(heights).all():
            break
    if not np.isfinite(heights).all():
        raise ValueError('標高欠測を合成地面で埋めずに停止する')
    ground = float(heights[n//2,n//2])
    # DEM の標高基準を保持しつつ、Gazebo は出発点を z=0 とする。
    heights -= ground
    query = '[out:json][timeout:45];(way["building"]('+','.join(map(str,bbox))+');>;);out body;'
    osm_url = 'https://overpass-api.de/api/interpreter'
    osm = download(osm_url, sources/'osm.json', urllib.parse.urlencode({'data':query}).encode())
    html = Path(__file__).parent/'terrain3d/terrain3d_v0_4_1.html'
    script = """const fs=require('fs'),vm=require('vm');const h=fs.readFileSync(process.argv[1],'utf8');
for(const n of ['core','geo','recon'])vm.runInThisContext(h.split('/*@'+n+'*/')[1].split('/*@end'+n+'*/')[0]);
console.log(JSON.stringify(TerrainGeo.osmToFeatures(JSON.parse(fs.readFileSync(process.argv[2])),
 {lat:+process.argv[3],lon:+process.argv[4]})));"""
    features = json.loads(subprocess.check_output(['node','-e',script,str(html),
        str(sources/'osm.json'),str(lat0),str(lon0)],text=True))
    scene = dict(app='terrain3d', version=1, unit='m', up='Z', project='つくば市役所・公開地図再構成',
        region=dict(w=side,d=side,res=1,origin=dict(lat=lat0,lon=lon0)),
        source=dict(kind='geo',geo=dict(bbox=bbox,features=features,
            dem=dict(w=n,h=n,res=1,src='国土地理院 DEM5A/5B/5C',type='f32',scale=1,offset=0,nodata=-3.402823466e38,
                     data=base64.b64encode(heights.astype('<f4').tobytes()).decode()),
            reconstruction=dict(version=2,forestFill=False,warnings=[],sources=[],
                status=dict(dem=dict(coverage=1),osm=dict(complete=True))))), sea=dict(level=-100), objects=[],
        robots=[dict(id='icart_mini',type='ugv',x=0,y=0,yaw=0,path=points)])
    (output/'scene.json').write_text(json.dumps(scene,ensure_ascii=False))
    subprocess.run(['node',str(html.parent/'export_world.cjs'),str(output/'scene.json'),str(output)],check=True)
    tree = ET.parse(output/'environment.sdf')
    world = tree.getroot().find('world')
    child(world,'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
    make_robot(world)
    yaw0 = math.atan2(points[1][1],points[1][0])
    world.find("model[@name='icart_mini']/pose").text = f'0 0 0.04 0 0 {yaw0}'
    ET.indent(tree)
    tree.write(output/'trial.sdf',encoding='utf-8',xml_declaration=True)
    with (output/'route.csv').open('w',newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['label','latitude','longitude','x','y','z','q1','q2','q3','q4',
            'right_is_open','left_is_open','line_is_stop','signal_is_stop','isnot_skipnum','node'])
        for i,(px,py) in enumerate(points):
            dx,dy = points[min(i+1,len(points)-1)]
            yaw = math.atan2(dy-py,dx-px)
            writer.writerow([i,rows[i]['latitude'],rows[i]['longitude'],px,py,0,0,0,
                math.sin(yaw/2),math.cos(yaw/2),2,2,0,0,1,-1])
    manifest = dict(scenario='geographic_cityhall', points=points,goal=points[-1],
        projection=dict(origin_latitude=lat0,origin_longitude=lon0,origin_altitude=67.078),
        ground_orthometric_height_m=ground, osm_buildings=len(features['buildings']),
        route_sha256=hashlib.sha256(route.read_bytes()).hexdigest(),
        sources=used+[dict(url=osm_url,sha256=hashlib.sha256(osm).hexdigest())],
        limitations=['国土地理院 DEM・OSM 由来。2026 年の現地測量ではない',
            '既存 2025 年経路の小区間。停止点フラグはこの幾何追従試験では適用しない',
            'DEM5 は段差・縁石を解像しない。建物高さ欠測は提供版の推定値',
            '楕円体原点高 67.078m は近傍公開点群原点から仮置き。絶対高度は未校正',
            'terrain3d の球近似座標。GNSS は同じローカル値を WGS84 ENU として変換'])
    (output/'trial.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    print(json.dumps(dict(side_m=side,points=len(points),buildings=len(features['buildings']),
                         ground_m=ground),ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--route',type=Path,required=True)
    parser.add_argument('--count',type=int,default=12)
    args = parser.parse_args()
    build(args.output,args.route,args.count)
