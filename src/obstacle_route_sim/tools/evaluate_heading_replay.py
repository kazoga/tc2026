#!/usr/bin/env python3
"""保存済みの同一GNSS/LIO観測で方位修正を再評価する（閉ループではない）."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from gnss_lio_fusion.fusion_core import FusionFilter
from gnss_lio_fusion.motion_guard_core import MotionGuard


def evaluate(source: Path, output: Path, reselect_motion: bool = False) -> dict:
    """真値は採点だけに使い、旧融合出力と新しいコア再生を比較する."""
    output.mkdir(parents=True, exist_ok=False)
    records = [json.loads(line) for line in (source/'fusion.inputs.jsonl').read_text().splitlines()]
    previous = [json.loads(line) for line in (source/'fusion.jsonl').read_text().splitlines()]
    lio = np.array([[r['sim_s'], *r['pose']] for r in records if r['kind'] == 'lio'])
    lio[:, 3] = np.unwrap(lio[:, 3])
    motion = np.array([[r['sim_s'], *r['pose']] for r in records if r['kind'] == 'motion'])
    if len(motion):
        motion[:, 3] = np.unwrap(motion[:, 3])
    wheel = np.array([[r['sim_s'], *r['pose']] for r in records if r['kind'] == 'wheel'])
    if reselect_motion and not len(wheel):
        raise ValueError('運動再選択には車輪の保存記録が必要')
    if len(wheel):
        wheel = wheel[np.argsort(wheel[:, 0])]
        wheel[:, 3] = np.unwrap(wheel[:, 3])
    records.sort(key=lambda r: (r['sim_s'], 0 if r['kind'] == 'motion' else 1, r['kind']))
    old = np.array([[r['sim_s'], r['x'], r['y'], r['yaw']] for r in previous])
    with (source/'trajectory.csv').open() as stream:
        truth = np.array([[float(r[k]) for k in ['sim_s', 'x', 'y', 'yaw']]
                          for r in csv.DictReader(stream)])
    old[:, 3], truth[:, 3] = np.unwrap(old[:, 3]), np.unwrap(truth[:, 3])
    filter_ = FusionFilter()
    guard = MotionGuard()

    def interpolate(series: np.ndarray, stamp: float):
        if not len(series) or not series[0, 0] <= stamp <= series[-1, 0]:
            return None
        index = min(max(1, np.searchsorted(series[:, 0], stamp)), len(series)-1)
        if series[index, 0]-series[index-1, 0] > .4:
            return None
        return np.array([np.interp(stamp, series[:, 0], series[:, i]) for i in [1, 2, 3]])

    def advance(stamp: float, prediction: np.ndarray) -> None:
        if reselect_motion:
            prediction = guard.select(stamp, interpolate(lio, stamp), interpolate(wheel, stamp))
            if prediction is None:
                return
        filter_.advance(stamp, prediction)
        if reselect_motion and guard.source == 'WHEEL_FALLBACK':
            filter_.heading_recovery = True

    result_rows = []
    inferred = 0
    for record in records:
        t = record['sim_s']
        if record['kind'] in ['lio', 'motion', 'wheel']:
            if record['kind'] == ('motion' if len(motion) else 'lio'):
                advance(t, np.array(record['pose']))
            continue
        if not lio[0, 0] <= t <= lio[-1, 0]:
            continue
        raw = np.array([np.interp(t, lio[:, 0], lio[:, i]) for i in [1, 2, 3]])
        prediction = (np.array([np.interp(t, motion[:, 0], motion[:, i]) for i in [1, 2, 3]])
                      if len(motion) else raw)
        advance(t, prediction)
        fix = record['state'] == 4
        if 'heading_stddev_deg' not in record:
            inferred += 1
        filter_.observe_gps(t, np.array(record['pose']), record['state'], record['satellites'],
                            record['baseline_m'], record.get('position_variance', .0004 if fix else 1.8),
                            record.get('heading_stddev_deg', .5 if fix else 5.))
        if filter_.x is not None and max(old[0, 0], truth[0, 0]) <= t <= min(old[-1, 0], truth[-1, 0]):
            actual = [np.interp(t, truth[:, 0], truth[:, i]) for i in [1, 2, 3]]
            before = [np.interp(t, old[:, 0], old[:, i]) for i in [1, 2, 3]]
            result_rows.append([t, *actual, *before, *filter_.x, *record['pose'], *raw,
                                record['state']])
    data = np.array(result_rows)
    valid = np.ones(len(data), dtype=bool)
    for series in [truth, old, lio]:
        index = np.clip(np.searchsorted(series[:, 0], data[:, 0]), 1, len(series)-1)
        valid &= series[index, 0]-series[index-1, 0] <= .5
    def stats(error: np.ndarray) -> dict:
        return dict(rmse=float(np.sqrt(np.mean(error[valid]**2))),
                    p95=float(np.percentile(abs(error[valid]), 95)),
                    maximum=float(abs(error[valid]).max()))
    errors, positions = {}, {}
    for name, col in [('旧融合', 4), ('改修融合', 7), ('GPS', 10), ('LIO', 13)]:
        difference = data[:, col+2]-data[:, 3]
        if name == 'LIO':
            difference -= difference[0]
        errors[name] = np.rad2deg(np.arctan2(np.sin(difference), np.cos(difference)))
        if name != 'LIO':
            positions[name] = stats(np.linalg.norm(data[:, col:col+2]-data[:, 1:3], axis=1))
    result = dict(heading_error_deg={name: stats(e) for name, e in errors.items()},
                  position_error_m=positions, valid_samples=int(valid.sum()),
                  excluded_samples=int((~valid).sum()), inferred_quality_samples=inferred,
                  diagnostics=filter_.diagnostics(data[-1, 0]),
                  reselected_motion=reselect_motion, window_rejections=guard.window_rejections,
                  limitations=['保存観測のコア再生。新融合制御での走行結果ではない',
                               '旧ログに精度欄がない場合はFIX 0.5度/FLOAT 5度と位置分散を仮定',
                               'LIO方位のみ初回整合。GPSと融合は真値への方位合わせなし',
                               '運動再選択時は同時刻へ内挿し、0.4秒超の観測間隔は欠測扱い'])
    (output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    np.save(output/'comparison.npy', data)
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False})
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    for name, error in errors.items():
        axes[0].plot((data[:, 0]-data[0, 0])/60, np.where(valid, error, np.nan), label=name, lw=.7)
    for name, col in [('旧融合', 4), ('改修融合', 7), ('GPS', 10)]:
        error = np.linalg.norm(data[:, col:col+2]-data[:, 1:3], axis=1)
        axes[1].plot((data[:, 0]-data[0, 0])/60, np.where(valid, error, np.nan), label=name, lw=.7)
    axes[0].set(ylabel='最短角の方位誤差 [度]', title='同一観測の再生：位置と方位の独立更新')
    axes[1].set(ylabel='位置誤差 [m]', xlabel='経過時間 [分]')
    for axis in axes:
        axis.legend(); axis.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(output/'heading_replay.png', dpi=140)
    plt.close(fig)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reselect-motion', action='store_true', help='車輪記録から運動照合も再実行する')
    args = parser.parse_args()
    print(json.dumps(evaluate(args.source, args.output, args.reselect_motion), ensure_ascii=False, indent=2))
