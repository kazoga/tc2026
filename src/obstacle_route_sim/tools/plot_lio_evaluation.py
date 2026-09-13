#!/usr/bin/env python3
"""FAST-LIO の真値比較を記録と PNG に出力する."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from lio_evaluation_core import evaluate_lio


def plot(directory: Path) -> dict:
    """同時刻比較の結果を保存する."""
    rows=list(csv.DictReader((directory/'trajectory.csv').open()))
    truth=np.array([[float(r[k]) for k in ['sim_s','x','y','yaw']] for r in rows])
    rows=list(csv.DictReader((directory/'lio_trajectory.csv').open()))
    lio=np.array([[float(r[k]) for k in ['sim_s','x','y','z','qx','qy','qz','qw']] for r in rows])
    result,data=evaluate_lio(truth,lio)
    (directory/'lio_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    if not len(data):return result
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False})
    fig,axes=plt.subplots(2,1,figsize=(9,9),facecolor='#f5f8fb')
    origin=data[0,3:5]
    axes[0].plot(*(data[:,3:5]-origin).T,label='Gazebo 真値',color='#253d58',lw=2)
    axes[0].plot(*(data[:,1:3]-origin).T,label='FAST-LIO',color='#d35f46',lw=1.5)
    axes[0].set_aspect('equal',adjustable='datalim');axes[0].legend()
    axes[0].set(xlabel='東方向 [m]',ylabel='北方向 [m]')
    axes[1].plot(data[:,0]-data[0,0],np.linalg.norm(data[:,1:3]-data[:,3:5],axis=1),color='#d35f46')
    axes[1].axhline(.5,ls='--',color='#6c7e8d',label='最大誤差の試験基準 0.5 m')
    axes[1].set(xlabel='シミュレーション経過時間 [s]',ylabel='XY 誤差 [m]');axes[1].legend()
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle('Gazebo 上で動作する FAST-LIO の評価',fontsize=18)
    fig.text(.09,.035,f"XY RMSE {result['xy_rmse_m']:.3f} m ／ 最大 {result['max_xy_error_m']:.3f} m",fontsize=13)
    fig.text(.09,.012,'初回姿勢で整合。瞬時点群・仮定 IMU。GNSS 融合と実機精度は未検証。',fontsize=10)
    fig.tight_layout(rect=(0,.07,1,.96));fig.savefig(directory/'lio_comparison.png',dpi=160)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    print(json.dumps(plot(parser.parse_args().directory),ensure_ascii=False))
