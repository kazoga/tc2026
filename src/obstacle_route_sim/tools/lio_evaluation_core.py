"""LIO と同時刻の真値を初期姿勢のみで整合し、推定誤差を評価する."""
import numpy as np


def evaluate_lio(truth: np.ndarray, lio: np.ndarray) -> tuple[dict, np.ndarray]:
    """truth=[t,x,y,yaw]、lio=[t,x,y,z,qx,qy,qz,qw] を比較する.

    軌跡全体を使った最適化は行わず、初回の位置と yaw のみで座標を合わせる。
    評価用の真値は走行制御や FAST-LIO に配信しない。
    """
    if len(truth)<2 or len(lio)<2 or not np.isfinite(lio).all():
        return dict(pass_lio=False,reason='有限の時系列データが不足'),np.empty((0,5))
    lio=lio[(lio[:,0]>=truth[0,0])&(lio[:,0]<=truth[-1,0])]
    if len(lio)<2:return dict(pass_lio=False,reason='時刻の重なりが不足'),np.empty((0,5))
    reference=np.column_stack([np.interp(lio[:,0],truth[:,0],truth[:,i]) for i in [1,2]])
    yaw=np.interp(lio[0,0],truth[:,0],np.unwrap(truth[:,3]))
    qx,qy,qz,qw=lio[0,4:8]
    yaw-=np.arctan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    aligned=(lio[:,1:3]-lio[0,1:3])@rotation.T+reference[0]
    errors=np.linalg.norm(aligned-reference,axis=1)
    duration=float(lio[-1,0]-lio[0,0])
    distance=float(np.linalg.norm(np.diff(reference,axis=0),axis=1).sum())
    rmse=float(np.sqrt(np.mean(errors**2)));maximum=float(errors.max())
    result=dict(pass_lio=bool(rmse<.2 and maximum<.5 and duration>=10 and distance>=10),
                xy_rmse_m=rmse,max_xy_error_m=maximum,final_xy_error_m=float(errors[-1]),
                samples=len(lio),sim_duration_s=duration,truth_distance_m=distance,
                criteria='RMSE < 0.2 m、最大 < 0.5 m、10 s / 10 m 以上',
                alignment='初回の XY と yaw のみ。roll/pitch とセンサ lever arm の厳密補正は未実施')
    return result,np.column_stack([lio[:,0],aligned,reference])
