#!/usr/bin/env python3
"""全周の推定を、固定座標ずれと内部変形に分けて評価する."""
import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from review_lio_noise_trials import read_columns, trajectory_metrics


def rigid_fit(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """拡大縮小・鏡映を許さず、単一の剛体変換を最小二乗で求める."""
    u, _, vt = np.linalg.svd((source-source.mean(0)).T @ (target-target.mean(0)))
    if np.linalg.det(u @ vt) < 0:
        u[:, -1] *= -1
    rotation = u @ vt
    return rotation, target.mean(0)-source.mean(0) @ rotation


def review(directory: Path, baseline: Path) -> dict:
    """全点を集計し、表示範囲外の点も誤差統計から除外しない."""
    truth = read_columns(directory/'trajectory.csv', ['sim_s', 'x', 'y', 'z', 'yaw'])
    lio = read_columns(directory/'lio_trajectory.csv', ['sim_s', 'x', 'y', 'z'])
    lio = lio[(lio[:, 0] >= truth[0, 0]) & (lio[:, 0] <= truth[-1, 0])]
    target = np.column_stack([np.interp(lio[:, 0], truth[:, 0], truth[:, i])
                              for i in [1, 2, 3]])
    target[:, 2] += .6
    rotation, translation = rigid_fit(lio[:, 1:4], target)
    aligned = lio[:, 1:4] @ rotation + translation
    errors = np.linalg.norm(aligned-target, axis=1)
    raw = np.frombuffer((directory/'fastlio_map.pcd').read_bytes().split(
        b'DATA binary\n', 1)[1], dtype='<f4').reshape(-1, 3)
    map_aligned = raw @ rotation + translation
    reference = np.load(directory/'reference_surface.npy')
    distance, _ = cKDTree(reference).query(map_aligned, workers=2)
    indices = set(map(int, re.findall(r'Proceed to next waypoint index=(\d+)',
                                      (directory/'route_follower.log').read_text())))
    with (directory/'route.csv').open() as stream:
        count = len(list(csv.DictReader(stream)))
    primary = json.loads((directory/'map_evaluation.json').read_text())
    result = dict(
        trial=directory.name, fair_trajectory=trajectory_metrics(directory),
        baseline_fair_trajectory=trajectory_metrics(baseline),
        waypoint_count=count, missing_waypoint_indices=sorted(set(range(1, count))-indices),
        truth_distance_m=float(np.linalg.norm(np.diff(truth[:, 1:3], axis=0), axis=1).sum()),
        lio_observed_duration_s=float(lio[-1, 0]-lio[0, 0]),
        maximum_lio_gap_s=float(np.diff(lio[:, 0]).max()),
        rigid_xyz_rmse_m=float(np.sqrt(np.mean(errors**2))),
        rigid_xyz_max_m=float(errors.max()),
        diagnostic_map_median_m=float(np.median(distance)),
        diagnostic_map_p95_m=float(np.percentile(distance, 95)),
        diagnostic_map_within_1m_fraction=float(np.mean(distance <= 1.)),
        diagnostic_map_bounds_min=map_aligned.min(0).tolist(),
        diagnostic_map_bounds_max=map_aligned.max(0).tolist(),
        map_points=len(raw), rotation_row_vectors=rotation.tolist(),
        translation=translation.tolist(),
        limitations=[
            '全軌跡を使う剛体整合は事後診断であり、走行中の位置精度ではない',
            '真値の IMU 高さはモデル原点＋0.6 m。roll/pitch lever arm 補正は未実施',
            '地図距離は生成元 SDF の表面標本に対する値。独立実測との比較ではない',
            '単一 seed の全周試験であり、実機の誤差分布・成功確率ではない'])
    (directory/'full_review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    np.save(directory/'diagnostic_map_enu.npy', map_aligned.astype(np.float32))
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False})
    fig, axes = plt.subplots(3, 1, figsize=(9, 15), facecolor='#f5f8fb')
    axes[0].plot(*target[:, :2].T, color='black', lw=2, label='真値')
    axes[0].plot(*aligned[:, :2].T, color='#d64232', lw=1, label='FAST-LIO（剛体整合後）')
    axes[0].set(aspect='equal', xlabel='東 [m]', ylabel='北 [m]',
                title='全周の軌跡形状：拡大縮小・局所補正なし')
    axes[0].legend()
    axes[1].plot((lio[:, 0]-lio[0, 0])/60, errors, color='#d64232', lw=1)
    axes[1].set(xlabel='推定開始からのシミュレーション時間 [分]', ylabel='XYZ 誤差 [m]',
                title='単一の剛体変換でも残る誤差')
    axes[1].grid(alpha=.3)
    stride = max(1, len(map_aligned)//150000)
    axes[2].scatter(*map_aligned[::stride, :2].T, s=.25, color='#498abd')
    axes[2].plot(*target[:, :2].T, color='black', lw=1, label='真値走行経路')
    # 全点の範囲を使い、表示間引きで極端な逸脱を隠さない。
    lower = np.minimum(map_aligned[:, :2].min(0), target[:, :2].min(0))
    upper = np.maximum(map_aligned[:, :2].max(0), target[:, :2].max(0))
    margin = np.maximum((upper-lower)*.03, 1.)
    axes[2].set(xlim=(lower[0]-margin[0], upper[0]+margin[0]),
                ylim=(lower[1]-margin[1], upper[1]+margin[1]), aspect='equal',
                xlabel='東 [m]', ylabel='北 [m]', title='点群地図の全範囲（剛体整合後）')
    axes[2].legend()
    fig.suptitle('厳しめ条件・全周 FAST-LIO 評価', fontsize=20)
    fig.text(.06, .012,
             f'点群 {len(raw):,} 点｜表面距離 P95：初期整合 '
             f'{primary["sample_distance_p95_m"]:.2f} m → 剛体整合 {np.percentile(distance, 95):.2f} m\n'
             'GNSS FIX で走行。剛体整合は評価用であり、制御には使用していない。', fontsize=10)
    fig.tight_layout(rect=(0, .05, 1, .96))
    fig.savefig(directory/'full_review.png', dpi=160)
    plt.close(fig)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--baseline', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(review(args.directory, args.baseline), ensure_ascii=False, indent=2))
