#!/usr/bin/env python3
"""terrain3d の合成通路と i-Cart mini の物理モデルを生成する."""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def child(parent: ET.Element, tag: str, text: object = None, **attrs: str) -> ET.Element:
    """XML 要素を追加する."""
    element = ET.SubElement(parent, tag, attrs)
    if text is not None:
        element.text = str(text)
    return element


def make_robot(world: ET.Element, mid360_pitch_deg: float = 0.) -> None:
    """公開駆動諸元と仮定した搭載物による差動二輪モデルを追加する."""
    if not math.isfinite(mid360_pitch_deg) or abs(mid360_pitch_deg) > 89:
        raise ValueError('MID-360前下がり角は有限値で±89度以内にする')
    model = child(world, 'model', name='icart_mini')
    child(model, 'pose', '0 0 0.01 0 0 0')
    # 輪径・輪間隔は公式 param。車体・搭載物の質量慣性は未校正である。
    parts = [
        ('base_link', '-0.10 0 0.21 0 0 0', 10, 'box', '0.4 0.3 0.20'),
        ('left_wheel', '0 0.153685 0.07455 1.57079632679 0 0', .3, 'wheel', ''),
        ('right_wheel', '0 -0.153685 0.07455 1.57079632679 0 0', .3, 'wheel', ''),
        ('caster', '-0.25 0 0.035 0 0 0', .2, 'sphere', '.035'),
        ('mast', '-0.1 0 0.50 0 0 0', 1, 'box', '0.04 0.04 0.4'),
        ('gnss_master', '0 0 0.70 0 0 0', .1, 'box', '0.08 0.08 0.02'),
        ('gnss_slave', '-0.5 0 0.70 0 0 0', .1, 'box', '0.08 0.08 0.02'),
    ]
    for name, pose, mass, shape, size in parts:
        link = child(model, 'link', name=name)
        child(link, 'pose', pose)
        child(child(link, 'inertial'), 'mass', mass)
        inertial = link.find('inertial')
        tensor = child(inertial, 'inertia')
        if shape == 'box':
            x, y, z = map(float, size.split())
            diagonal = [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]
        elif shape == 'wheel':
            transverse = mass*(3*.07455**2+.035**2)/12
            diagonal = [transverse, transverse, mass*.07455**2/2]
        else:
            diagonal = [2*mass*float(size)**2/5]*3
        for axis, value in zip(['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'], diagonal+[0,0,0]):
            child(tensor, axis, value)
        for kind in ['collision', 'visual']:
            item = child(link, kind, name=kind)
            geometry = child(item, 'geometry')
            if shape == 'wheel':
                cylinder = child(geometry, 'cylinder')
                child(cylinder, 'radius', .07455)
                child(cylinder, 'length', .035)
            else:
                child(child(geometry, shape), 'size' if shape == 'box' else 'radius', size)
            if kind == 'visual':
                child(child(item, 'material'), 'diffuse', '0.15 0.35 0.7 1')
            if kind == 'collision' and shape == 'sphere':
                ode = child(child(child(item, 'surface'), 'friction'), 'ode')
                child(ode, 'mu', .001)
                child(ode, 'mu2', .001)
        if shape == 'box':
            contact = child(link, 'sensor', name=name+'_contact', type='contact')
            child(contact, 'topic', '/body_contacts')
            child(contact, 'always_on', 'true')
            child(contact, 'update_rate', 40)
            contact_config = child(contact, 'contact')
            child(contact_config, 'collision', 'collision')
            # Harmonic の Contact system は sensor/topic ではなくこちらを読む。
            child(contact_config, 'topic', '/body_contacts')
        if name != 'base_link':
            joint = child(model, 'joint', name=name+'_joint',
                          type='revolute' if shape == 'wheel' else 'fixed')
            child(joint, 'parent', 'base_link')
            child(joint, 'child', name)
            if shape == 'wheel':
                # 車輪はX軸回り90度のため、関節の局所-Zが車体の+Yに一致する。
                axis = child(joint, 'axis')
                child(axis, 'xyz', '0 0 -1')
                limit = child(axis, 'limit')
                child(limit, 'effort', 2.0)
                child(limit, 'velocity', 20.0)
    base = model.find("link[@name='base_link']")
    sensor = child(base, 'sensor', name='top_urg', type='gpu_lidar')
    # 地上高はユーザー指定。前後オフセットは仮定である。
    child(sensor, 'pose', '0.175 0 0.09 0 0 0')
    child(sensor, 'topic', '/scan')
    child(sensor, 'update_rate', 20)
    child(sensor, 'always_on', 'true')
    lidar = child(sensor, 'lidar')
    horizontal = child(child(lidar, 'scan'), 'horizontal')
    for tag, value in [('samples', 1080), ('resolution', 1),
                       ('min_angle', -2.35619), ('max_angle', 2.35619)]:
        child(horizontal, tag, value)
    ranges = child(lidar, 'range')
    for tag, value in [('min', .2), ('max', 30), ('resolution', .01)]:
        child(ranges, tag, value)
    mid = copy.deepcopy(sensor)
    mid.set('name', 'mid360')
    # SDFは前方+X・上+Z。+Y軸回りの正のpitchが前下がり。
    mid.find('pose').text = f'0.1 0 0.39 0 {math.radians(mid360_pitch_deg):.12g} 0'
    mid.find('topic').text = '/mid360/livox/lidar'
    mid.find('update_rate').text = '10'
    mid.find('lidar/scan/horizontal/samples').text = '360'
    mid.find('lidar/scan/horizontal/min_angle').text = str(-math.pi)
    mid.find('lidar/scan/horizontal/max_angle').text = str(math.pi)
    vertical = child(mid.find('lidar/scan'), 'vertical')
    for tag, value in [('samples', 16), ('resolution', 1),
                       ('min_angle', -.122), ('max_angle', .907)]:
        child(vertical, tag, value)
    base.append(mid)
    plugin = child(model, 'plugin', filename='gz-sim-diff-drive-system',
                   name='gz::sim::systems::DiffDrive')
    for tag, value in [('left_joint', 'left_wheel_joint'), ('right_joint', 'right_wheel_joint'),
                       ('wheel_separation', .30737), ('wheel_radius', .07455),
                       ('topic', '/cmd_vel'), ('odom_topic', '/ypspur_ros/odom'),
                       ('odom_publish_frequency', 40), ('max_linear_velocity', .9),
                       ('min_linear_velocity', -.9), ('max_linear_acceleration', 1.5),
                       ('max_angular_velocity', 1.2), ('min_angular_velocity', -1.2),
                       ('max_angular_acceleration', 1.0), ('min_angular_acceleration', -1.0)]:
        child(plugin, tag, value)
    pose = child(model, 'plugin', filename='gz-sim-pose-publisher-system',
                 name='gz::sim::systems::PosePublisher')
    child(pose, 'publish_model_pose', 'true')
    child(pose, 'publish_link_pose', 'false')
    child(pose, 'use_pose_vector_msg', 'true')
    child(pose, 'update_frequency', 40)


def build(output: Path, scenario: str, mid360_pitch_deg: float = 0.) -> None:
    """シーン、衝突メッシュ、world、経路を同時生成する."""
    if not math.isfinite(mid360_pitch_deg) or abs(mid360_pitch_deg) > 89:
        raise ValueError('MID-360前下がり角は有限値で±89度以内にする')
    output.mkdir(parents=True, exist_ok=True)
    points = [(float(x), 0.0) for x in range(0, 31, 3)]
    if scenario == 'crank':
        points = [(float(x), 0.0) for x in range(0, 16, 3)]
        points += [(15.0, float(y)) for y in range(3, 13, 3)]
        points += [(float(x), 12.0) for x in range(18, 34, 3)]
    objects = [dict(id='wall_n', type='box', x=15, y=4, w=40, d=.2, h=1),
               dict(id='wall_s', type='box', x=15, y=-4, w=40, d=.2, h=1)]
    if scenario == 'crank':
        objects = [dict(id='building', type='box', x=5, y=12, w=8, d=8, h=8)]
    if scenario in ['blocker', 'contact_probe']:
        objects.append(dict(id='blocker', type='box', x=1 if scenario=='contact_probe' else 12,
                            y=0, w=.6, d=.6, h=1))
    objects.append(dict(id='road', type='road', x=0, y=0, pts=points, w=8))
    scene = dict(app='terrain3d', version=1, unit='m', up='Z', project='合成公園通路',
                 region=dict(w=100, d=100, res=1),
                 source=dict(kind='contour', contour=dict(base=0, noise=0, lines=[])),
                 sea=dict(level=-10), objects=objects,
                 vegetation=dict(trees=[dict(id='tree', x=8, y=-6, h=6, r=1.5)]),
                 robots=[dict(id='icart_mini', type='ugv', x=0, y=0, yaw=-math.pi/2,
                              path=points, mobility=dict(width=.35, length=.45),
                              sensors=dict(lidarPitchDownDeg=mid360_pitch_deg,
                                           lidarMinElevationDeg=-7, lidarMaxElevationDeg=52,
                                           height=.6))])
    (output/'scene.json').write_text(json.dumps(scene, ensure_ascii=False), encoding='utf-8')
    subprocess.run(['node', str(Path(__file__).parent/'terrain3d/export_world.cjs'),
                    str(output/'scene.json'), str(output)], check=True)
    tree = ET.parse(output/'environment.sdf')
    world = tree.getroot().find('world')
    child(world, 'plugin', filename='gz-sim-contact-system', name='gz::sim::systems::Contact')
    make_robot(world, mid360_pitch_deg)
    ET.indent(tree)
    tree.write(output/'trial.sdf', encoding='utf-8', xml_declaration=True)
    with (output/'route.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['label','latitude','longitude','x','y','z','q1','q2','q3','q4',
                         'right_is_open','left_is_open','line_is_stop','signal_is_stop',
                         'isnot_skipnum','node'])
        for i, (x, y) in enumerate(points):
            next_x, next_y = points[min(i+1, len(points)-1)]
            yaw = math.atan2(next_y-y, next_x-x)
            writer.writerow([i,'','',x,y,0,0,0,math.sin(yaw/2),math.cos(yaw/2),
                             3,3,0,0,1,-1])
    (output/'trial.json').write_text(json.dumps(dict(scenario=scenario, points=points,
        goal=points[-1], robot=dict(wheel_radius=.07455, tread=.30737),
        mid360_pitch_deg=mid360_pitch_deg,
        limitations=['合成平坦地形', '搭載物・接地摩擦・慣性は未校正',
                     'Top-URG/MID-360は理想GPU ray。SLAM/GNSSは別途追加'])), encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', choices=['straight', 'crank', 'blocker', 'contact_probe'],
                        default='straight')
    parser.add_argument('--mid360-pitch-deg', type=float, default=0.,
                        help='MID-360の前下がり角。度、正が下向き（例: 25）')
    arguments = parser.parse_args()
    build(arguments.output, arguments.scenario, arguments.mid360_pitch_deg)
