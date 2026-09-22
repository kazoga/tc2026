#!/usr/bin/env python3
"""調整済み地図の同一ウェイポイントで FIX/FLOAT/障害物の比較環境を作る."""
import argparse
import csv
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from geo_pose_converter.geo_core import EnuPoint, ProjectionConfig, enu_to_llh


def prepare(source: Path, output: Path, start: int, count: int) -> None:
    """実際の route_planner CSV と同じ field を保持し、ENU/緯度経度を設定する."""
    trial=json.loads((source/'trial.json').read_text())
    if start<0 or count<12 or start+count>len(trial['points']):
        raise ValueError('経路内で少なくとも12点を指定する')
    points=trial['points'][start:start+count]
    grid=json.loads((source/'world.json').read_text())['grid']
    def height(x: float,y: float) -> float:
        i=round((x-grid['ox'])/grid['res']);j=round((y-grid['oy'])/grid['res'])
        return grid['height'][j*grid['w']+i]
    for name in ['fixed','float','obstacle']:
        dest=output/name
        if dest.exists():raise ValueError('既存試験結果を上書きしない: '+str(dest))
        shutil.copytree(source,dest,ignore=shutil.ignore_patterns('views','viewer_vendor','*.html'))
        selected=dict(trial,points=points,goal=points[-1],scenario=name,
                      source_route_start=start,waypoint_count=count)
        (dest/'trial.json').write_text(json.dumps(selected,ensure_ascii=False))
        rows=list(csv.reader((source/'route.csv').open()))
        selected_rows=rows[start+1:start+count+1]
        rows[0].append('heading_deg')
        projection=ProjectionConfig(**trial['projection'])
        for i,row in enumerate(selected_rows):
            row[0]=str(i)
            llh=enu_to_llh(EnuPoint(float(row[3]),float(row[4]),0.),projection)
            row[1],row[2]=str(llh.latitude),str(llh.longitude)
            yaw=2*math.atan2(float(row[8]),float(row[9]))
            row.append(str((90-math.degrees(yaw))%360))
        with (dest/'route.csv').open('w') as stream:
            writer=csv.writer(stream);writer.writerow(rows[0]);writer.writerows(selected_rows)
        xml=ET.parse(source/'trial.sdf');world=xml.getroot().find('world')
        x,y=points[0];nx,ny=points[1]
        yaw=math.atan2(ny-y,nx-x)
        world.find("model[@name='icart_mini']/pose").text=f'{x} {y} {height(x,y)+.08} 0 0 {yaw}'
        if name=='obstacle':
            x,y=points[10]
            model=ET.SubElement(world,'model',name='test_obstacle')
            ET.SubElement(model,'static').text='true'
            ET.SubElement(model,'pose').text=f'{x} {y} {height(x,y)+.4} 0 0 0'
            link=ET.SubElement(model,'link',name='link')
            for kind in ['visual','collision']:
                element=ET.SubElement(link,kind,name=kind)
                box=ET.SubElement(ET.SubElement(element,'geometry'),'box')
                ET.SubElement(box,'size').text='.6 .6 .8'
        xml.write(dest/'trial.sdf',encoding='utf-8',xml_declaration=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--start',type=int,default=100)
    parser.add_argument('--count',type=int,default=22)
    args=parser.parse_args();prepare(args.source,args.output,args.start,args.count)
