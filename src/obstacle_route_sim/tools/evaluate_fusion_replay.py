#!/usr/bin/env python3
"""保存済み全周LIOに模擬GNSSを再生成し、同一入力の融合を評価する."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from gnss_lio_fusion.fusion_core import FusionFilter, wrap
from gnss_environment_core import BuildingDegradation
from review_fusion_trial import statistics


def evaluate(source: Path, output: Path) -> dict:
    """真値はGPS生成と採点にのみ使用し、融合器には観測値だけを渡す."""
    output.mkdir(parents=True, exist_ok=False)
    with (source/'trajectory.csv').open() as stream:
        truth = np.array([[float(r[k]) for k in ['sim_s', 'x', 'y', 'yaw']]
                          for r in csv.DictReader(stream)])
    with (source/'lio_trajectory.csv').open() as stream:
        lio = np.array([[float(r[k]) for k in ['sim_s', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']]
                        for r in csv.DictReader(stream)])
    rotations = Rotation.from_quat(lio[:, 4:8])
    body = lio[:, 1:4]-rotations.apply(np.tile([0., 0., .6], (len(lio), 1)))
    yaw = np.unwrap(rotations.as_euler('xyz')[:, 2])
    world = json.loads((source/'world.json').read_text())
    degradation = BuildingDegradation([o['poly'] for o in world['obstacles'] if o['source'] == 'building'])
    random = np.random.default_rng(42)
    start, end = max(truth[0, 0], lio[0, 0]), min(truth[-1, 0], lio[-1, 0])
    times = np.arange(start, end, .1)
    f = FusionFilter()
    rows = []
    raw_start, aligned_start = None, None
    angle = 0.
    for t in times:
        true = np.array([np.interp(t, truth[:, 0], truth[:, 1]),
                         np.interp(t, truth[:, 0], truth[:, 2]),
                         np.interp(t, truth[:, 0], np.unwrap(truth[:, 3]))])
        raw = np.array([np.interp(t, lio[:, 0], body[:, 0]),
                        np.interp(t, lio[:, 0], body[:, 1]), np.interp(t, lio[:, 0], yaw)])
        if raw_start is None:
            raw_start, aligned_start = raw.copy(), true.copy()
            angle = true[2]-raw[2]
        rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        standalone = (raw[:2]-raw_start[:2]) @ rotation.T+aligned_start[:2]
        effect = degradation.sample(*true[:2], .1)
        floating = effect['floating']
        sigma = .6 if floating else .02
        gps = true.copy()
        gps[:2] += random.normal(0., sigma, 2)+effect['bias']
        gps[2] = wrap(gps[2]+random.normal(0., math.radians(5. if floating else .5)))
        baseline = .515+random.normal(0., .04 if floating else .002)
        f.advance(t, raw)
        f.observe_gps(t, gps, 3 if floating else 4, 10 if floating else 20,
                      baseline, sigma**2+max(abs(v) for v in effect['bias'])**2,
                      5. if floating else .5)
        if f.x is not None:
            rows.append([t, *true[:2], *gps[:2], *standalone, *f.x[:2], float(floating)])
    data = np.array(rows)
    errors = {name: np.linalg.norm(data[:, col:col+2]-data[:, 1:3], axis=1)
              for name, col in [('GPS', 3), ('LIO', 5), ('融合', 7)]}
    result = dict(comparison={name: statistics(e) for name, e in errors.items()},
                  duration_s=float(times[-1]-times[0]), samples=len(data),
                  final_baseline=f.baseline_status, final_diagnostics=f.diagnostics(times[-1]),
                  limitations=['保存済みconservative全周LIOを使用。GNSSは新しく生成した仮定観測',
                               '既存のGNSS FIX制御の走行軌跡を固定した再生。融合制御の完走ではない',
                               '観測時刻順のコア評価で通信遅延・DDS欠落・閉ループ作用は対象外',
                               'LIO間を線形内挿。長い欠測の影響は別のROS走行試験で評価',
                               '真値のroll/pitchが保存されておらず、GNSSは車軸平面観測として生成'])
    np.save(output/'comparison.npy', data)
    (output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False})
    fig, axes = plt.subplots(2, 1, figsize=(10, 10), facecolor='#f5f8fb')
    for name, col, color in [('真値', 1, 'black'), ('GPS', 3, '#e6aa30'),
                             ('LIO', 5, '#cc483b'), ('融合', 7, '#227bb0')]:
        axes[0].plot(*data[:, col:col+2].T, label=name, color=color, lw=.8)
    axes[0].set(aspect='equal', xlabel='東 [m]', ylabel='北 [m]')
    axes[0].legend(ncol=4)
    for name, color in [('GPS', '#e6aa30'), ('LIO', '#cc483b'), ('融合', '#227bb0')]:
        axes[1].plot((data[:, 0]-data[0, 0])/60, errors[name], label=name, color=color, lw=.7)
    axes[1].set(xlabel='シミュレーション時間 [分]', ylabel='XY誤差 [m]')
    axes[1].legend()
    axes[1].grid(alpha=.25)
    fig.suptitle('全周記録の再生評価：厳しめLIO＋建物近傍GPS劣化', fontsize=18)
    fig.text(.08, .015, '走行軌跡を固定した再生。融合制御で完走した結果ではない。', fontsize=11)
    fig.tight_layout(rect=(0, .05, 1, .95))
    fig.savefig(output/'replay_comparison.png', dpi=150)
    plt.close(fig)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(evaluate(args.source, args.output), ensure_ascii=False, indent=2))
