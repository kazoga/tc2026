#!/usr/bin/env python3
"""既存試験 world に LiDAR と同位置の Gazebo IMU を追加する."""
import argparse
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def add_imu(root: ET.Element, mid360_pitch_deg: float | None = None) -> None:
    """重力を含む実センサ模擬を追加し、既存ロボットや地形を維持する."""
    if mid360_pitch_deg is not None and (
            not math.isfinite(mid360_pitch_deg) or abs(mid360_pitch_deg) > 89):
        raise ValueError('MID-360前下がり角は有限値で±89度以内にする')
    world=root.find('world')
    if world is None:raise ValueError('world がない')
    model=world.find("model[@name='icart_mini']")
    if model is None:raise ValueError('icart_mini がない')
    base=model.find("link[@name='base_link']")
    if base.find("sensor[@name='lio_imu']") is not None:raise ValueError('IMU は追加済み')
    # 規則格子の疎密で LIO が退化しないよう、専用 world だけ密度を上げる。
    mid=base.find("sensor[@name='mid360']")
    if mid360_pitch_deg is not None:
        pose = mid.find('pose').text.split()
        pose[4] = str(math.radians(mid360_pitch_deg))
        mid.find('pose').text = ' '.join(pose)
    mid.find('lidar/scan/horizontal/samples').text='1800'
    mid.find('lidar/scan/vertical/samples').text='32'
    mid.find('lidar/range/max').text='70'
    ET.SubElement(world,'plugin',filename='gz-sim-imu-system',name='gz::sim::systems::Imu')
    sensor=ET.SubElement(base,'sensor',name='lio_imu',type='imu')
    ET.SubElement(sensor,'pose').text=base.find("sensor[@name='mid360']/pose").text
    ET.SubElement(sensor,'topic').text='/sim/mid360/imu'
    ET.SubElement(sensor,'always_on').text='true'
    ET.SubElement(sensor,'update_rate').text='200'
    imu=ET.SubElement(sensor,'imu')
    for kind,sigma in [('angular_velocity',.0002),('linear_acceleration',.002)]:
        block=ET.SubElement(imu,kind)
        for axis in ['x','y','z']:
            noise=ET.SubElement(ET.SubElement(block,axis),'noise',type='gaussian')
            ET.SubElement(noise,'mean').text='0'
            ET.SubElement(noise,'stddev').text=str(sigma)


def prepare(source: Path,output: Path, mid360_pitch_deg: float | None = None) -> None:
    """元の試験を保持して別の試験環境を作る."""
    if output.exists():raise ValueError('出力先は新規ディレクトリにする')
    tree=ET.parse(source/'trial.sdf')
    add_imu(tree.getroot(), mid360_pitch_deg)
    shutil.copytree(source,output,ignore=shutil.ignore_patterns('ros','views','viewer_vendor','*.log',
                    'trajectory.csv','result.json','evaluation.txt','control.csv','*.html'))
    ET.indent(tree)
    tree.write(output/'trial.sdf',encoding='utf-8',xml_declaration=True)
    pose = tree.find('.//sensor[@name="mid360"]/pose').text
    angle = math.degrees(float(pose.split()[4]))
    for filename in ('scene.json', 'trial.json'):
        path = output/filename
        if not path.exists():
            continue
        data = json.loads(path.read_text())
        if filename == 'scene.json':
            for robot in data.get('robots', []):
                if robot.get('id') == 'icart_mini':
                    robot.setdefault('sensors', {})['lidarPitchDownDeg'] = angle
        else:
            data['mid360_pitch_deg'] = angle
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--mid360-pitch-deg',type=float,
                        help='MID-360前下がり角[度]。正が下向き。省略時は元SDFを維持')
    args=parser.parse_args();prepare(args.source,args.output,args.mid360_pitch_deg)
