#!/usr/bin/env python3
"""公開原本・航空写真を取得し、全域モデル生成の前提を揃える."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
import json
import math
import urllib.parse
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image

from geographic_reconstruction_core import register_course_image, world_pixels
from reconstruct_full_course import fetch


def prepare(output: Path, config: dict) -> None:
    source=output/'source';source.mkdir(parents=True,exist_ok=True)
    cache=source/'tiles';cache.mkdir(exist_ok=True)
    bbox=config['bbox']
    fetch(config['course_source'],source/'course_2026.jpg')
    for provider,zoom in [('gsi',18),('esri',19)]:
        a,b=world_pixels(np.array([bbox[2],bbox[0]]),np.array([bbox[1],bbox[3]]),zoom)
        x0,x1=math.floor(a[0]),math.ceil(a[1]);y0,y1=math.floor(b[0]),math.ceil(b[1])
        tasks=[(x,y) for x in range(x0//256,x1//256+1) for y in range(y0//256,y1//256+1)]
        def tile(job):
            x,y=job
            url=(f'https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{zoom}/{x}/{y}.jpg'
                if provider=='gsi' else
                f'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{zoom}/{y}/{x}')
            prefix='photo' if provider=='gsi' else 'esri'
            raw=fetch(url,cache/f'{prefix}_{zoom}_{x}_{y}.jpg')
            return x,y,raw,url
        mosaic=Image.new('RGB',((x1//256-x0//256+1)*256,(y1//256-y0//256+1)*256))
        records=[]
        with ThreadPoolExecutor(max_workers=6) as pool:
            for x,y,raw,url in pool.map(tile,tasks):
                mosaic.paste(Image.open(io.BytesIO(raw)),((x-x0//256)*256,(y-y0//256)*256))
                records.append(dict(url=url,sha256=hashlib.sha256(raw).hexdigest()))
        mosaic=mosaic.crop((x0%256,y0%256,x0%256+x1-x0,y0%256+y1-y0))
        name='aerial' if provider=='gsi' else 'esri'
        mosaic.save(source/f'{name}_mercator.jpg',quality=95)
        (source/f'{name}_manifest.json').write_text(json.dumps(dict(bbox=bbox,z=zoom,
            x0=x0,x1=x1,y0=y0,y1=y1,tiles=records),indent=2))
    osm_url='https://api.openstreetmap.org/api/0.6/map?bbox='+','.join(map(str,[bbox[1],bbox[0],bbox[3],bbox[2]]))
    raw=fetch(osm_url,source/'osm.xml');elements=[]
    for element in ET.fromstring(raw):
        if element.tag not in ['node','way','relation']:continue
        item=dict(type=element.tag,id=int(element.attrib['id']),
            tags={t.attrib['k']:t.attrib['v'] for t in element.findall('tag')})
        if element.tag=='node':item.update(lat=float(element.attrib['lat']),lon=float(element.attrib['lon']))
        if element.tag=='way':item['nodes']=[int(t.attrib['ref']) for t in element.findall('nd')]
        if element.tag=='relation':item['members']=[dict(type=t.attrib['type'],ref=int(t.attrib['ref']),
            role=t.attrib['role']) for t in element.findall('member')]
        elements.append(item)
    (source/'osm.json').write_text(json.dumps(dict(elements=elements)))
    service='https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer?f=pjson'
    fetch(service,source/'esri_service.json')
    query=dict(f='json',geometry=','.join(map(str,[bbox[1],bbox[0],bbox[3],bbox[2]])),
        geometryType='esriGeometryEnvelope',inSR='4326',spatialRel='esriSpatialRelIntersects',
        outFields='*',returnGeometry='false')
    metadata_url='https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/9/query?'+urllib.parse.urlencode(query)
    fetch(metadata_url,source/'esri_extent_metadata.json')
    official=cv2.imread(str(source/'course_2026.jpg'),cv2.IMREAD_GRAYSCALE)
    aerial=cv2.imread(str(source/'aerial_mercator.jpg'),cv2.IMREAD_GRAYSCALE)
    registration=register_course_image(official,aerial,config['control_points'])
    (source/'refined_registration.json').write_text(json.dumps(registration,indent=2))
    (source/'provenance.json').write_text(json.dumps(dict(osm_url=osm_url,
        osm_sha256=hashlib.sha256(raw).hexdigest(),course_url=config['course_source'],
        course_sha256=hashlib.sha256((source/'course_2026.jpg').read_bytes()).hexdigest(),
        note='画像位置合わせ RMSE は画像上の整合であり現地測量誤差ではない'),ensure_ascii=False,indent=2))
    print(json.dumps(dict(registration_inliers=registration['inliers'],rmse_px=registration['rmse_px'],
                         osm_elements=len(elements)),ensure_ascii=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    args=parser.parse_args();prepare(args.output,json.loads(args.config.read_text()))
