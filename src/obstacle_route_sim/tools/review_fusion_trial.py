#!/usr/bin/env python3
"""同一走行のGNSS・LIO・融合を同じ時刻で比較し、経路逸脱も評価する."""
import argparse
import csv
import json
import math
from pathlib import Path
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def statistics(error: np.ndarray) -> dict:
    """誤差分布を欠落なく集計する."""
    return dict(rmse_m=float(np.sqrt(np.mean(error**2))),
                p95_m=float(np.percentile(abs(error), 95)), max_m=float(abs(error).max()))


def route_distance(xy: np.ndarray, route: np.ndarray) -> np.ndarray:
    """最近傍の連続線分への距離を求め、waypoint間隔による誤差を除く."""
    start, delta = route[:-1], np.diff(route, axis=0)
    length = np.sum(delta**2, axis=1)
    result = []
    for chunk in np.array_split(xy, max(1, len(xy)//500)):
        offset = chunk[:, None, :]-start
        fraction = np.clip(np.sum(offset*delta, axis=2)/np.maximum(length, 1e-12), 0., 1.)
        result.extend(np.min(np.linalg.norm(offset-fraction[:, :, None]*delta, axis=2), axis=1))
    return np.array(result)


def review(directory: Path) -> dict:
    """LIOだけ初回XY/yaw整合する。融合軌跡を真値で位置合わせしない."""
    with (directory/'trajectory.csv').open() as stream:
        truth_rows = list(csv.DictReader(stream))
        truth = np.array([[float(r[k]) for k in ['sim_s', 'x', 'y', 'yaw']]
                          for r in truth_rows])
    records = [json.loads(line) for line in (directory/'fusion.jsonl').read_text().splitlines()]
    inputs = [json.loads(line) for line in (directory/'fusion.inputs.jsonl').read_text().splitlines()]
    fused = np.array([[r['sim_s'], r['x'], r['y'], r['yaw']] for r in records])
    gps = np.array([[r['sim_s'], *r['pose']] for r in inputs if r['kind'] == 'gps'])
    lio = np.array([[r['sim_s'], *r['pose']] for r in inputs if r['kind'] == 'lio'])
    start = max(truth[0, 0], fused[0, 0], gps[0, 0], lio[0, 0])
    end = min(truth[-1, 0], fused[-1, 0], gps[-1, 0], lio[-1, 0])
    times = np.arange(start, end, .2)
    def sample(data: np.ndarray) -> np.ndarray:
        return np.column_stack([np.interp(times, data[:, 0], data[:, i]) for i in [1, 2]])
    target, fused_xy, gps_xy, lio_xy = map(sample, [truth, fused, gps, lio])
    yaw = np.interp(start, truth[:, 0], np.unwrap(truth[:, 3]))
    yaw -= np.interp(start, lio[:, 0], np.unwrap(lio[:, 3]))
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    lio_xy = (lio_xy-lio_xy[0]) @ rotation.T+target[0]
    # 長い欠測を線形補間だけで良く見せない。共通の有効時刻のみを精度比較する。
    valid = np.ones(len(times), dtype=bool)
    gaps = {}
    for name, data in [('truth', truth), ('gps', gps), ('lio', lio), ('fusion', fused)]:
        gaps[name] = float(np.diff(data[:, 0]).max())
        index = np.searchsorted(data[:, 0], times, side='right')
        index = np.clip(index, 1, len(data)-1)
        valid &= data[index, 0]-data[index-1, 0] <= .5
    errors = {name: np.linalg.norm(xy-target, axis=1)
              for name, xy in [('GPS', gps_xy), ('LIO', lio_xy), ('融合', fused_xy)]}
    true_yaw = np.interp(times, truth[:, 0], np.unwrap(truth[:, 3]))
    heading_errors = {}
    for name, data, offset in [('GPS', gps, 0.), ('LIO', lio, yaw), ('融合', fused, 0.)]:
        difference = np.interp(times, data[:, 0], np.unwrap(data[:, 3]))+offset-true_yaw
        heading_errors[name] = np.rad2deg(np.arctan2(np.sin(difference), np.cos(difference)))
    heading_statistics = {name: {key.replace('_m', '_deg'): value
                                for key, value in statistics(error[valid]).items()}
                          for name, error in heading_errors.items()}
    route = np.array(json.loads((directory/'trial.json').read_text())['points'])
    cross_track = route_distance(truth[:, 1:3], route)
    modes = {mode: sum(r['mode'] == mode for r in records) for mode in set(r['mode'] for r in records)}
    run = json.loads((directory/'result.json').read_text())
    running_start = next((t for t, state in run['states'] if state == 'RUNNING'), math.inf)
    finish = next((t for t, state in run['states'] if state == 'FINISHED'), math.inf)
    wall = np.array([float(r['wall_s']) for r in truth_rows])
    speed = np.linalg.norm(np.diff(truth[:, 1:3], axis=0), axis=1)/np.diff(truth[:, 0])
    yaw_rate = abs(np.diff(np.unwrap(truth[:, 3])))/np.diff(truth[:, 0])
    stationary = (speed < .02) & (yaw_rate < .05) & (wall[:-1] >= running_start) & (wall[1:] < finish)
    longest, current = 0., 0.
    for quiet, dt in zip(stationary, np.diff(wall)):
        current = current+dt if quiet else 0.
        longest = max(longest, current)
    log = (directory/'route_follower.log').read_text()
    indices = set(map(int, re.findall(r'Proceed to next waypoint index=(\d+)', log)))
    versions = set(map(int, re.findall(r'route_ver=(\d+)', log)))
    result = dict(comparison={name: statistics(error[valid]) for name, error in errors.items()},
                  heading_comparison=heading_statistics, common_samples=int(valid.sum()), excluded_samples=int((~valid).sum()),
                  max_input_gap_s=gaps, route_cross_track=statistics(cross_track),
                  truth_within_1m_fraction=float(np.mean(cross_track <= 1.)),
                  truth_max_yaw_rate_rad_s=float(yaw_rate.max()),
                  truth_rotation_anomaly_intervals=int(np.count_nonzero(yaw_rate > 2.)),
                  modes=modes, final_baseline=records[-1]['baseline'],
                  missing_waypoint_indices=sorted(set(range(1, len(route)))-indices),
                  route_versions=sorted(versions), longest_near_stationary_wall_s=longest,
                  final_diagnostics=records[-1],
                  limitations=['GPS/LIO単独値は同一融合走行の観測比較。単独制御の走行結果ではない',
                               'LIOは初回XY/yawだけで真値整合。融合・GPSは真値整合なし',
                               '0.5秒超の観測間隔は精度比較から除き、欠測数と最大間隔を併記',
                               '経路距離は車体中心。車体幅や障害物の幾何学的安全性とは別'])
    (directory/'fusion_review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False})
    fig, axes = plt.subplots(4, 1, figsize=(10, 16), facecolor='#f5f8fb')
    axes[0].plot(*route.T, '--', color='grey', label='予定経路')
    for name, xy, color in [('真値', target, 'black'), ('GPS', gps_xy, '#e6aa30'),
                             ('LIO', lio_xy, '#cc483b'), ('融合', fused_xy, '#227bb0')]:
        axes[0].plot(*xy.T, color=color, lw=1, label=name)
    axes[0].set(aspect='equal', xlabel='東 [m]', ylabel='北 [m]', title='同一走行・同時刻の自己位置比較')
    axes[0].legend(ncol=3)
    for name, color in [('GPS', '#e6aa30'), ('LIO', '#cc483b'), ('融合', '#227bb0')]:
        axes[1].plot((times-start)/60, np.where(valid, errors[name], np.nan), color=color, label=name)
    axes[1].set(xlabel='経過時間 [分]', ylabel='XY 誤差 [m]', title='真値との誤差（欠測区間は描画しない）')
    axes[1].legend()
    gps_records = [r for r in inputs if r['kind'] == 'gps']
    gps_time = np.array([r['sim_s'] for r in gps_records])
    axes[2].plot((gps_time-start)/60, [r['baseline_m']*1000 for r in gps_records],
                 color='#e6aa30', lw=.6, label='観測baseline')
    axes[2].plot((fused[:, 0]-start)/60, [r['baseline']['reference_m']*1000 for r in records],
                 color='#227bb0', label='推定基準距離')
    axes[2].set(xlabel='経過時間 [分]', ylabel='アンテナ間距離 [mm]', title='基準距離の適応推定')
    axes[2].legend()
    for name, color in [('GPS', '#e6aa30'), ('LIO', '#cc483b'), ('融合', '#227bb0')]:
        axes[3].plot((times-start)/60, np.where(valid, heading_errors[name], np.nan),
                     color=color, label=name, lw=.8)
    axes[3].set(xlabel='経過時間 [分]', ylabel='方位誤差 [度]', title='方位の最短角誤差')
    axes[3].legend()
    for axis in axes:
        axis.grid(alpha=.25)
    fig.suptitle('厳しめセンサ条件・GNSS/LIO 融合評価', fontsize=19)
    fig.text(.07, .015, f'車体中心の経路横ずれ最大 {cross_track.max():.2f} m｜'
             f'±1 m以内 {np.mean(cross_track <= 1.)*100:.1f}%\n'
             '融合位置を使って操舵。単独センサは同じ走行の観測比較。', fontsize=11)
    fig.tight_layout(rect=(0, .06, 1, .95))
    fig.savefig(directory/'fusion_review.png', dpi=150)
    plt.close(fig)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    print(json.dumps(review(parser.parse_args().directory), ensure_ascii=False, indent=2))
