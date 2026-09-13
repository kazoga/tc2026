#!/usr/bin/env python3
"""全域再構成を軽量なレイヤー付き 3D viewer と提供版 editor で表示する."""

import argparse
import json
from pathlib import Path
import shutil

import cv2
import numpy as np


def generate(output: Path, three_directory: Path) -> None:
    """衝突メッシュを同じ幾何形状の表示用データへ変換する."""
    scene=json.loads((output/'scene.json').read_text())
    world=json.loads((output/'world.json').read_text())
    trial=json.loads((output/'trial.json').read_text())
    meshes=[]
    for record in world['meshes']:
        if record['name']=='terrain':continue
        positions=[]
        for line in (output/(record['name']+'.obj')).read_text().splitlines():
            if line.startswith('v '):positions.extend(map(float,line.split()[1:4]))
        identifier=record.get('id','')
        layer=('vehicles' if 'photo_vehicle' in identifier else 'hedges' if 'photo_hedge' in identifier
               else 'buildings' if identifier.startswith('building:') else 'street')
        meshes.append(dict(positions=positions,layer=layer,id=identifier))
    data=dict(grid=world['grid'],meshes=meshes,trees=world['trees'],route=trial['points'],
        counts=trial['counts'],routeLength=trial['route_length_m'],dimensions=trial['dimensions_m'],
        audit=json.loads((output/'geometry_audit.json').read_text()) if (output/'geometry_audit.json').exists() else None,
        imagery=json.loads((output/'source/esri_extent_metadata.json').read_text()).get('features',[])
            if (output/'source/esri_extent_metadata.json').exists() else [])
    photo=cv2.imread(str(output/'aerial_texture.jpg'))
    height,width=photo.shape[:2]
    rw,rd=trial['dimensions_m']
    def pixel(x: float,y: float) -> tuple[int,int]:
        return (round((x/rw+.5)*(width-1)),round((.5-y/rd)*(height-1)))
    for tree in world['trees']:
        cv2.circle(photo,pixel(tree['x'],tree['y']),max(2,round(tree['r']/rw*width)),(70,215,70),2)
    for obj in scene['objects']:
        if obj.get('semantic') not in ['vehicle','hedge']:continue
        c,s=np.cos(obj.get('yaw',0)),np.sin(obj.get('yaw',0))
        corners=[]
        for x,y in [(-obj['w']/2,-obj['d']/2),(obj['w']/2,-obj['d']/2),
                    (obj['w']/2,obj['d']/2),(-obj['w']/2,obj['d']/2)]:
            corners.append(pixel(obj['x']+x*c-y*s,obj['y']+x*s+y*c))
        color=(235,180,40) if obj['semantic']=='vehicle' else (30,150,245)
        cv2.polylines(photo,[np.array(corners,np.int32)],True,color,2)
    line=np.array([pixel(*point) for point in trial['points']],np.int32)
    cv2.polylines(photo,[line],False,(0,240,255),4)
    cv2.circle(photo,tuple(line[0]),9,(70,230,70),-1)
    cv2.circle(photo,tuple(line[-1]),9,(30,130,250),-1)
    cv2.imwrite(str(output/'analysis_overlay.jpg'),photo)
    geo_features=[]
    origin=scene['region']['origin'];scale=6371000*np.pi/180
    def ll(x: float,y: float) -> list[float]:
        return [origin['lon']+x/(scale*np.cos(np.radians(origin['lat']))),origin['lat']+y/scale]
    geo_features.append(dict(type='Feature',properties=dict(kind='approximate_course',status=trial['route_status']),
        geometry=dict(type='LineString',coordinates=[ll(*p) for p in trial['points']])))
    for tree in world['trees']:
        geo_features.append(dict(type='Feature',properties=dict(kind='tree',height_m=tree['h'],
            radius_m=tree['r'],source=tree.get('src'),height_source=tree.get('heightSource')),
            geometry=dict(type='Point',coordinates=ll(tree['x'],tree['y']))))
    for obj in scene['objects']:
        geo_features.append(dict(type='Feature',properties=dict(kind=obj.get('semantic'),
            height_m=obj['h'],width_m=obj['w'],depth_m=obj['d'],status='photo-candidate'),
            geometry=dict(type='Point',coordinates=ll(obj['x'],obj['y']))))
    (output/'reconstruction.geojson').write_text(json.dumps(dict(type='FeatureCollection',features=geo_features),ensure_ascii=False))
    (output/'viewer_data.json').write_text(json.dumps(data,separators=(',',':')))
    vendor=output/'viewer_vendor';vendor.mkdir(exist_ok=True)
    for src,dst in [('build/three.min.js','three.min.js'),('examples/js/controls/OrbitControls.js','OrbitControls.js'),('LICENSE','THREE-LICENSE')]:
        shutil.copyfile(three_directory/src,vendor/dst)
    template=(Path(__file__).parent/'terrain3d/full_course_viewer.html').read_text()
    (output/'index.html').write_text(template)
    original=(Path(__file__).parent/'terrain3d/terrain3d_v0_4_1.html').read_text()
    embedded=json.dumps(scene,ensure_ascii=False).replace('<','\\u003c')
    original=original.replace('<script type="application/json" id="embedded-scene"></script>',
        '<script type="application/json" id="embedded-scene">'+embedded+'</script>')
    (output/'editor.html').write_text(original)
    print(output/'index.html')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--three-directory',type=Path,required=True)
    args=parser.parse_args();generate(args.output,args.three_directory)
