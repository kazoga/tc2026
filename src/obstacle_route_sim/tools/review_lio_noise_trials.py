#!/usr/bin/env python3
"""入力誤差条件を、絶対配置・剛体整合・短時間相対誤差に分けて比較する."""
import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read_columns(path: Path, columns: list[str]) -> np.ndarray:
    """保存済み実測列を読み出す."""
    with path.open() as stream:
        return np.array([[float(r[k]) for k in columns] for r in csv.DictReader(stream)])


def trajectory_metrics(directory: Path) -> dict:
    """固定した地図全体のずれを、相対推定精度の失敗と混同しない."""
    truth = read_columns(directory/'trajectory.csv', ['sim_s','x','y','yaw'])
    lio = read_columns(directory/'lio_trajectory.csv', ['sim_s','x','y','qx','qy','qz','qw'])
    lio = lio[(lio[:,0] >= truth[0,0]) & (lio[:,0] <= truth[-1,0])]
    if len(lio) < 20:
        return dict(valid=False, reason='LIO サンプル不足')
    target = np.column_stack([np.interp(lio[:,0],truth[:,0],truth[:,i]) for i in [1,2]])
    distance = float(np.linalg.norm(np.diff(target,axis=0),axis=1).sum())
    if distance < 10. or lio[-1,0]-lio[0,0] < 10.:
        return dict(valid=False, reason='走行距離10 mまたは比較時間10 s未満',
                    truth_distance_m=distance)
    x = lio[:,1:3]
    u, _, v = np.linalg.svd((x-x.mean(0)).T@(target-target.mean(0)))
    if np.linalg.det(u@v) < 0:
        u[:,-1] *= -1
    aligned = (x-x.mean(0))@(u@v)+target.mean(0)
    error = np.linalg.norm(aligned-target, axis=1)
    qx,qy,qz,qw = lio[:,3:].T
    yaw = np.unwrap(np.arctan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz)))
    truth_yaw = np.interp(lio[:,0],truth[:,0],np.unwrap(truth[:,3]))
    end = np.searchsorted(lio[:,0], lio[:,0]+5.)
    start = np.flatnonzero(end < len(lio))
    end = end[start]
    keep = np.linalg.norm(target[end]-target[start],axis=1) >= 1.
    start, end = start[keep], end[keep]
    def relative(points: np.ndarray, angles: np.ndarray) -> np.ndarray:
        delta = points[end]-points[start]
        a = angles[start]
        return np.column_stack([np.cos(a)*delta[:,0]+np.sin(a)*delta[:,1],
                                -np.sin(a)*delta[:,0]+np.cos(a)*delta[:,1]])
    rpe = np.linalg.norm(relative(x,yaw)-relative(target,truth_yaw),axis=1)
    return dict(valid=True, rigid_xy_rmse_m=float(np.sqrt(np.mean(error**2))),
                rigid_xy_max_m=float(error.max()),
                rpe_5s_rmse_m=float(np.sqrt(np.mean(rpe**2))) if len(rpe) else None,
                rpe_pairs=len(rpe))


def review(root: Path) -> list:
    """全 seed を含め、成功した試験だけを選別せず集計する."""
    rows = []
    for directory in sorted(root.iterdir()):
        if not directory.is_dir():
            continue
        record = (json.loads((directory/'result.json').read_text())
                  if (directory/'result.json').exists() else {})
        match = re.match(r'^(reference|field_assumed|conservative)_(\d+)', directory.name)
        if match is None:
            continue
        profile, seed = match.groups()
        noise = (json.loads((directory/'sensor_noise.json').read_text())
                 if (directory/'sensor_noise.json').exists() else
                 dict(profile=profile, seed=int(seed), input_points=0, output_points=0))
        row = dict(profile=noise['profile'], seed=noise['seed'],
                   trial=directory.name,
                   goal_pass=record.get('goal_pass',False),
                   absolute_xy_rmse_m=record.get('lio_quality',{}).get('xy_rmse_m'),
                   retained_fraction=noise['output_points']/max(1,noise['input_points']))
        if (directory/'lio_trajectory.csv').exists():
            row.update(trajectory_metrics(directory))
        else:
            row.update(valid=False,reason='初期化または記録未完了')
        rows.append(row)
    (root/'noise_comparison.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False})
    fig, axes = plt.subplots(2,1,figsize=(8,9),facecolor='#f5f8fb')
    profiles = ['reference','field_assumed','conservative']
    labels = ['基準','実環境仮定','厳しめ']
    for ax, key, title in zip(axes,['rigid_xy_rmse_m','rpe_5s_rmse_m'],
            ['剛体整合後の XY RMSE','5 秒間の相対移動 RMSE']):
        for i, profile in enumerate(profiles):
            values = [r[key] for r in rows if r['profile']==profile and r.get(key) is not None]
            if values:
                ax.scatter(i+np.linspace(-.08,.08,len(values)),values,s=45)
                ax.plot([i-.15,i+.15],[np.mean(values)]*2,color='black',lw=2)
        ax.set_xticks(range(3),labels)
        ax.set(ylabel='誤差 [m]（対数目盛）',title=title,yscale='log')
        ax.grid(axis='y',alpha=.3)
    fig.suptitle('FAST-LIO 入力ノイズ比較\n同一約 40 m 区間・各 3 seed',fontsize=17)
    failed = sum(not r.get('valid',False) for r in rows)
    fig.text(.06,.025,
        f'点は各走行、黒線は平均。無効走行 {failed} 試行は別記。\n'
        '剛体整合は評価用の後処理。実機ログによる校正は未実施。',
        fontsize=10)
    fig.tight_layout(rect=(0,.07,1,.92))
    fig.savefig(root/'noise_comparison.png',dpi=160)
    plt.close(fig)
    return rows


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    print(json.dumps(review(parser.parse_args().root),ensure_ascii=False,indent=2))
