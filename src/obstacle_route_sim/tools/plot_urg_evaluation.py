#!/usr/bin/env python3
"""障害物判定時の実受信 URG スキャンと真値軌跡を PNG に表示する."""

import argparse
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np


def plot(directory: Path) -> None:
    """直近スキャンと経路を分け、真値とセンサ座標を混同せず描画する."""
    result = json.loads((directory/'result.json').read_text())
    saved = json.loads((directory/'urg_blocked_scan.json').read_text())
    scan = saved['scan']
    ranges = np.array([r if r is not None else np.nan for r in scan['ranges']])
    angles = scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
    valid = np.isfinite(ranges) & (ranges < 5) & (ranges >= scan['range_min'])
    rows = list(csv.DictReader((directory/'trajectory.csv').open()))
    xy = np.array([[float(r['x']), float(r['y'])] for r in rows])
    route = np.array(json.loads((directory/'trial.json').read_text())['points'])
    obstacle = ET.parse(directory/'trial.sdf').find('.//model[@name="test_obstacle"]')
    center = np.array(list(map(float, obstacle.findtext('pose').split()))[:2])
    size = list(map(float, obstacle.findtext('.//collision/geometry/box/size').split()))
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False})
    fig, axes = plt.subplots(2, 1, figsize=(8, 10), facecolor='#f5f8fb')
    axes[0].scatter(ranges[valid]*np.cos(angles[valid]),
                    ranges[valid]*np.sin(angles[valid]), s=5, label='URG 実受信点')
    axes[0].plot(0, 0, 'r^', label='センサ原点（前方は右）')
    axes[0].set(xlim=(-2, 5), ylim=(-3.5, 3.5), xlabel='前方 [m]', ylabel='左方向 [m]',
                title=f"障害物判定時の直近スキャン／前方余裕 {saved['front_clearance_m']:.2f} m")
    axes[1].plot(*(route-center).T, '--', label='ウェイポイント', color='gray')
    axes[1].plot(*(xy-center).T, label='実走行の真値軌跡')
    axes[1].add_patch(Rectangle((-size[0]/2, -size[1]/2), size[0], size[1],
                                color='#df664c', label='試験障害物'))
    axes[1].set(xlim=(-4, 4), ylim=(-3, 3), xlabel='東方向 [m]', ylabel='北方向 [m]',
                title='障害物付近の回避軌跡（障害物中心を原点に表示）')
    for axis in axes:
        axis.set_aspect('equal')
        axis.grid(alpha=.25)
        axis.legend(fontsize=9)
    fig.suptitle('URG 2D スキャン＋FAST-LIO 同時試験', fontsize=19)
    fig.text(.08, .025,
             f"スキャン {result['counts']['scan']} 件／前方障害判定 "
             f"{result['front_blocked_hint_messages']} 件\n"
             f"ゴール誤差 {result['goal_error_m']:.3f} m／車体等の接触 "
             f"{result['body_contact_events']} 件。センサ仕様・取付前後位置は仮定を含む。",
             fontsize=10)
    fig.tight_layout(rect=(0, .075, 1, .96))
    fig.savefig(directory/'urg_evaluation.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    plot(parser.parse_args().directory)
