#!/usr/bin/env python3
"""全ルート領域の航空写真・DEM・OSM を terrain3d と Gazebo に統合する."""

import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess
import urllib.request
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image

from build_terrain_trial import child, make_robot
from geographic_reconstruction_core import (
    filter_tree_candidates, pixel_llh, route_from_pixels, vegetation_strips, vehicle_candidates, world_pixels)


def fetch(url: str, path: Path) -> bytes:
    """原本をキャッシュする。HTTP エラーを空画像で隠さない."""
    if not path.exists():
        with urllib.request.urlopen(url,timeout=45) as response:
            path.write_bytes(response.read())
    return path.read_bytes()


def build(output: Path, config_path: Path) -> dict:
    """取得済み画像と位置合わせ資料から再現可能な全域モデルを生成する."""
    source=output/'source'
    config=json.loads(config_path.read_text())
    bbox=config['bbox']
    origin=dict(lat=(bbox[0]+bbox[2])/2,lon=(bbox[1]+bbox[3])/2)
    scale=6371000*math.pi/180
    width=(bbox[3]-bbox[1])*scale*math.cos(math.radians(origin['lat']))
    depth=(bbox[2]-bbox[0])*scale
    resolution=config['resolution_m']
    nw,nh=math.ceil(width/resolution)+1,math.ceil(depth/resolution)+1
    # 正規コンパイラが端を越えないよう bbox 対応を保って要求寸法を指定する。
    lat=np.linspace(bbox[2],bbox[0],nh)
    lon=np.linspace(bbox[1],bbox[3],nw)
    px,py=world_pixels(lat[:,None],lon[None,:],15)
    xx=np.broadcast_to(px,(nh,nw));yy=np.broadcast_to(py,(nh,nw))
    height=np.full((nh,nw),np.nan)
    dem_sources=[]
    for layer in ['dem5a_png','dem5b_png','dem5c_png','dem_png']:
        zoom=14 if layer=='dem_png' else 15
        xp,yp=world_pixels(lat[:,None],lon[None,:],zoom)
        xp=np.broadcast_to(xp,(nh,nw));yp=np.broadcast_to(yp,(nh,nw))
        tiles={(int(x)//256,int(y)//256) for x,y in zip(xp.ravel(),yp.ravel())}
        def load(tile):
            tx,ty=tile;name=f'{layer}_{zoom}_{tx}_{ty}.png'
            url=f'https://cyberjapandata.gsi.go.jp/xyz/{layer}/{zoom}/{tx}/{ty}.png'
            try:
                raw=fetch(url,source/'tiles'/name)
                rgb=np.asarray(Image.open(io.BytesIO(raw)).convert('RGB'),dtype=np.int32)
                v=rgb[:,:,0]*65536+rgb[:,:,1]*256+rgb[:,:,2]
                z=np.where(v<8388608,v,v-16777216)*.01;z[v==8388608]=np.nan
                return tile,z,dict(url=url,sha256=hashlib.sha256(raw).hexdigest())
            except Exception as error:
                return tile,None,dict(url=url,error=str(error))
        with ThreadPoolExecutor(max_workers=6) as pool:
            for (tx,ty),tile,metadata in pool.map(load,sorted(tiles)):
                dem_sources.append(metadata)
                if tile is None:continue
                selected=(xp//256==tx)&(yp//256==ty)&~np.isfinite(height)
                height[selected]=tile[yp[selected].astype(int)%256,xp[selected].astype(int)%256]
        if np.isfinite(height).all():break
    if not np.isfinite(height).all():raise ValueError('DEM 欠測。全域モデルを生成しない')
    altitude=float(height[nh//2,nw//2]);height-=altitude
    # 写真は Mercator の行間隔を緯度等間隔に変換して、提供版と一致させる。
    photo_meta=json.loads((source/'esri_manifest.json').read_text())
    original=np.asarray(Image.open(source/'esri_mercator.jpg').convert('RGB'))
    pw,ph=original.shape[1],original.shape[0]
    lats=np.linspace(bbox[2],bbox[0],ph);lons=np.linspace(bbox[1],bbox[3],pw)
    ux,uy=world_pixels(lats[:,None],lons[None,:],photo_meta['z'])
    mx=np.broadcast_to(ux-photo_meta['x0'],(ph,pw)).astype(np.float32)
    my=np.broadcast_to(uy-photo_meta['y0'],(ph,pw)).astype(np.float32)
    rgb=cv2.remap(original,mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE)
    Image.fromarray(rgb).save(output/'aerial_texture.jpg',quality=94)
    rgba=np.dstack([rgb,np.full((ph,pw),255,dtype=np.uint8)])
    rgba.tofile(output/'photo.rgba')
    request=dict(osm=str((source/'osm.json').resolve()),origin=origin,width=width,depth=depth,
                 photoWidth=pw,photoHeight=ph,rgba=str((output/'photo.rgba').resolve()),
                 sensitivity=config['tree_sensitivity'])
    (output/'feature_request.json').write_text(json.dumps(request))
    tools=Path(__file__).parent
    subprocess.run(['node',str(tools/'terrain3d/reconstruct_features.cjs'),'features',
                    str(output/'feature_request.json'),str(output/'features.json')],check=True)
    result=json.loads((output/'features.json').read_text())
    features=result['features']
    # OSM 駐車場 polygon を車両候補の検索領域として使う。
    osm=json.loads((source/'osm.json').read_text());nodes={n['id']:n for n in osm['elements'] if n['type']=='node'}
    parking=np.zeros((ph,pw),np.uint8)
    for way in osm['elements']:
        if way['type']!='way' or way.get('tags',{}).get('amenity')!='parking':continue
        if any(i not in nodes for i in way['nodes']):continue
        pts=np.array([[(nodes[i]['lon']-bbox[1])/(bbox[3]-bbox[1])*(pw-1),
                       (bbox[2]-nodes[i]['lat'])/(bbox[2]-bbox[0])*(ph-1)] for i in way['nodes']])
        cv2.fillPoly(parking,[pts.astype(np.int32)],255)
    exclusion=np.fromfile(output/'exclusion_mask.raw',np.uint8).reshape(ph,pw)
    # 親 building 外周も樹冠の除外に使う。屋根の倒れ込みは 3 m 緩衝で抑制する。
    building_mask=np.zeros_like(parking)
    for building in features['buildings']:
        poly=np.array([[(x/width+.5)*(pw-1),(.5-y/depth)*(ph-1)] for x,y in building['poly']])
        cv2.fillPoly(building_mask,[poly.astype(np.int32)],255)
    radius=2*math.ceil(3/(width/pw))+1
    building_mask=cv2.dilate(building_mask,np.ones((radius,radius),np.uint8))
    tree_exclusion=((exclusion==1)|(parking>0)|(building_mask>0)).astype(np.uint8)
    raw_tree_count=len(result['trees'])
    result['trees']=filter_tree_candidates(rgb,result['trees'],tree_exclusion,width,depth)
    result['treeStats'].update(raw_candidates=raw_tree_count,filtered_candidates=len(result['trees']),
        filter='HSV vegetation + parking/building exclusion; winter trees may be missed')
    (output/'filtered_trees.json').write_text(json.dumps(result['trees'],ensure_ascii=False))
    parking[exclusion==1]=0
    cars=vehicle_candidates(rgb,parking,width/pw)
    hedges=vegetation_strips(rgb,tree_exclusion,width/pw)
    objects=[]
    for kind,items in [('vehicle',cars),('hedge',hedges)]:
        for i,item in enumerate(items):
            px0,py0=item['pixel'];x=px0/(pw-1)*width-width/2;y=depth/2-py0/(ph-1)*depth
            objects.append(dict(id=f'photo_{kind}_{i}',type='box',x=x,y=y,yaw=item['yaw_rad'],
                w=item['width_m'],d=item['depth_m'],h=item['height_m'],
                provenance='photo-candidate',confidence='unverified',semantic=kind,
                color='#748da6' if kind=='vehicle' else '#537a3b'))
            item.update(x=x,y=y)
    gsi=json.loads((source/'aerial_manifest.json').read_text())
    registration=json.loads((source/'refined_registration.json').read_text())
    llh=route_from_pixels(config,registration,gsi)
    route=np.c_[(llh[:,1]-origin['lon'])*scale*math.cos(math.radians(origin['lat'])),
                 (llh[:,0]-origin['lat'])*scale]
    if not ((route[:,0]>=-width/2)&(route[:,0]<=width/2)&(route[:,1]>=-depth/2)&(route[:,1]<=depth/2)).all():
        raise ValueError('公式トレースがモデル範囲外')
    # 単純線分を 2 m 以下へ細分化する。これは走行可否を確認済みの経路ではない。
    points=[]
    for a,b in zip(route,route[1:]):
        steps=max(1,math.ceil(np.linalg.norm(b-a)/2))
        points.extend((a+(b-a)*u/steps).tolist() for u in range(steps))
    points.append(route[-1].tolist())
    scene=dict(app='terrain3d',version=1,unit='m',up='Z',project='つくばチャレンジ2026・全ルート再構成',
        region=dict(w=width,d=depth,res=resolution,origin=origin),worldConfig=dict(maxGrid=1025),
        sea=dict(level=-100),objects=objects,
        source=dict(kind='geo',geo=dict(bbox=bbox,features=features,trees=result['trees'],
            dem=dict(w=nw,h=nh,res=resolution,type='f32',scale=1,offset=0,nodata=-3.402823466e38,
                data=base64.b64encode(height.astype('<f4').tobytes()).decode(),src='国土地理院 DEM'),
            photo=dict(w=pw,h=ph,mime='image/jpeg',src='Esri World Imagery',coverage=1,
                data=base64.b64encode((output/'aerial_texture.jpg').read_bytes()).decode()),
            reconstruction=dict(version=2,forestFill=False,sensitivity=config['tree_sensitivity'],
                sources=[dict(src='国土地理院 DEM'),dict(src='OpenStreetMap contributors'),dict(src='Esri World Imagery')],
                warnings=['樹高・車高・低木高は推定。写真撮影時刻と大会時点は異なる。',config['course_status']],
                status=dict(dem=dict(coverage=1),photo=dict(coverage=1),osm=dict(complete=True))))),
        robots=[dict(id='icart_mini',type='ugv',x=points[0][0],y=points[0][1],yaw=0,path=points)])
    (output/'scene.json').write_text(json.dumps(scene,ensure_ascii=False))
    subprocess.run(['node',str(tools/'terrain3d/export_world.cjs'),str(output/'scene.json'),str(output)],check=True)
    # 同じ写真を Gazebo の地形材質にも付ける。UV は local x,y と対応する。
    obj=output/'terrain.obj';lines=obj.read_text().splitlines();uv=[]
    for line in lines:
        if line.startswith('v '):
            a,b,_=map(float,line.split()[1:]);uv.append(f'vt {(a+width/2)/width} {(b+depth/2)/depth}')
    index=next(i for i,l in enumerate(lines) if l.startswith('f '))
    lines[index:index]=uv
    for i,line in enumerate(lines):
        if line.startswith('f '):lines[i]='f '+' '.join(t.replace('//','/'+t.split('//')[0]+'/') for t in line.split()[1:])
    obj.write_text('mtllib terrain.mtl\nusemtl aerial\n'+'\n'.join(lines)+'\n')
    (output/'terrain.mtl').write_text('newmtl aerial\nKd 1 1 1\nmap_Kd aerial_texture.jpg\n')
    tree=ET.parse(output/'environment.sdf');world=tree.getroot().find('world')
    child(world,'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
    make_robot(world)
    first=np.asarray(points[0]);second=np.asarray(points[1]);yaw=math.atan2(*(second-first)[::-1])
    h0=float(height[int((depth/2-first[1])/depth*(nh-1)),int((first[0]+width/2)/width*(nw-1))])
    world.find("model[@name='icart_mini']/pose").text=f'{first[0]} {first[1]} {h0+.08} 0 0 {yaw}'
    ET.indent(tree);tree.write(output/'trial.sdf',encoding='utf-8',xml_declaration=True)
    with (output/'route.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['label','latitude','longitude','x','y','z','q1','q2','q3','q4',
            'right_is_open','left_is_open','line_is_stop','signal_is_stop','isnot_skipnum','node'])
        for i,(x,y) in enumerate(points):
            j=min(i+1,len(points)-1);angle=math.atan2(points[j][1]-y,points[j][0]-x)
            writer.writerow([i,'','',x,y,0,0,0,math.sin(angle/2),math.cos(angle/2),2,2,0,0,1,-1])
    length=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum())
    manifest=dict(scenario='tc2026_full',bbox=bbox,dimensions_m=[width,depth],route_length_m=length,
        route_status=config['course_status'],points=points,goal=points[-1],
        projection=dict(origin_latitude=origin['lat'],origin_longitude=origin['lon'],origin_altitude=67.078),
        height_datum=dict(ground_reference_m=altitude,gnss_ellipsoid_origin='assumed'),
        counts=dict(buildings=len(features['buildings']),roads=len(features['roads']),barriers=len(features['barriers']),
                    street_objects=len(features['points']),trees=len(result['trees']),vehicle_candidates=len(cars),hedge_candidates=len(hedges)),
        photo_analysis=result['treeStats'],registration=registration,dem_sources=dem_sources,
        attribution='国土地理院 / © OpenStreetMap contributors / Esri, Vantor, Earthstar Geographics, GIS User Community',
        limitations=['実地測量・全ルート走行未検証','球近似平面座標。原点楕円体高は未校正',
                     '航空写真の影・冬季落葉・屋根の倒れ込みによる誤検出あり','車両は撮影時点の候補で高さは仮定',
                     '停止線・コーンの大会当日配置、歩行者、信号の動的制御は含まない'])
    (output/'trial.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    (output/'photo_candidates.json').write_text(json.dumps(dict(vehicles=cars,hedges=hedges),ensure_ascii=False,indent=2))
    print(json.dumps(dict(dimensions_m=manifest['dimensions_m'],route_length_m=length,counts=manifest['counts']),ensure_ascii=False))
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    args=parser.parse_args();build(args.output,args.config)
