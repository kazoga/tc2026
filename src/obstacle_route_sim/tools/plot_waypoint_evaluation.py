#!/usr/bin/env python3
"""実測したシミュレーション真値軌跡と経路ずれを画像化する."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString, Point, box


def plot(root: Path, output: Path) -> None:
    """元の時系列記録から比較画像と距離指標を生成する."""
    output.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False})
    runs=[('通常 FIX（ENU）','waypoint_eval/fixed','#237ab5'),
          ('建物近傍 FLOAT（ENU）','waypoint_eval/float','#db5d48'),
          ('障害物回避（緯度経度）','waypoint_llh_eval/obstacle','#319464')]
    fig,axes=plt.subplots(3,1,figsize=(10,12),facecolor='#f5f8fb')
    metrics={}
    for ax,(title,relative,color) in zip(axes,runs):
        source=root/relative
        trial=json.loads((source/'trial.json').read_text())
        result=json.loads((source/'result.json').read_text())
        waypoints=np.array(trial['points']);route=LineString(waypoints)
        rows=list(csv.DictReader((source/'trajectory.csv').open()))
        trajectory=np.array([[float(r['x']),float(r['y'])] for r in rows])
        errors=np.array([Point(p).distance(route) for p in trajectory])
        entry=dict(cross_track_rmse_m=float(np.sqrt(np.mean(errors**2))),
                   cross_track_max_m=float(errors.max()),outside_1m_samples=int((errors>1).sum()),
                   goal_pass=result['goal_pass'],goal_error_m=result['goal_error_m'])
        origin=waypoints[0]
        boundary=np.array(route.buffer(1).exterior.coords)-origin
        ax.fill(boundary[:,0],boundary[:,1],color='#e4ebf1',label='経路 ±1 m')
        ax.plot(*(waypoints-origin).T,'k.--',lw=1,ms=4,label='ウェイポイント')
        ax.plot(*(trajectory-origin).T,color=color,lw=2,label='Gazebo 真値軌跡')
        if 'obstacle' in relative:
            x,y=waypoints[10];obstacle=box(x-.3,y-.3,x+.3,y+.3)
            polygon=np.array(obstacle.exterior.coords)-origin
            ax.fill(polygon[:,0],polygon[:,1],color='#784e34',label='試験障害物')
            entry['min_center_obstacle_distance_m']=min(Point(p).distance(obstacle) for p in trajectory)
        metrics[relative]=entry
        ax.set_title(title+f" ｜ 最大経路ずれ {errors.max():.2f} m",loc='left',fontsize=14)
        ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2)
        ax.set_ylabel('北方向 [m]');ax.set_xlabel('開始点から東方向 [m]')
        ax.legend(loc='upper right',fontsize=9,ncol=2)
    fig.suptitle('調整版地図でのウェイポイント走行試験',fontsize=20,y=.98)
    fig.text(.06,.035,'40.465 m・22 点の局所試験。全コース完走・FAST-LIO 併用は未検証。',fontsize=11)
    fig.text(.06,.015,'FLOAT：水平 σ=0.6 m、各軸 bias→1.2 m、方位 σ=5°。実機測定値ではない。',fontsize=10)
    fig.tight_layout(rect=(0,.06,1,.95))
    fig.savefig(output/'comparison.png',dpi=160)
    (output/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2))
    print(json.dumps(metrics,ensure_ascii=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();plot(args.root,args.output)
