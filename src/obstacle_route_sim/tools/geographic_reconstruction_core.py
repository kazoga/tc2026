"""航空写真の特徴候補と地理座標を扱う ROS 非依存処理."""

import math

import cv2
import numpy as np


def world_pixels(lat, lon, zoom: int):
    """緯度経度を Web Mercator のタイル画素座標へ変換する."""
    size = 256*2**zoom
    return ((np.asarray(lon)+180)/360*size,
            (1-np.arcsinh(np.tan(np.radians(lat)))/np.pi)/2*size)


def pixel_llh(x, y, zoom: int):
    """タイル画素座標から緯度経度へ戻す."""
    size = 256*2**zoom
    return (np.degrees(np.arctan(np.sinh(np.pi*(1-2*np.asarray(y)/size)))),
            np.asarray(x)/size*360-180)


def route_from_pixels(config: dict, registration: dict, raster: dict) -> np.ndarray:
    """公式図のトレースを座標付き写真へ移し、緯度経度列を返す."""
    pixels = np.asarray(config['official_route_pixels'], dtype=float)
    affine = np.asarray(registration['affine'], dtype=float)
    if affine.shape != (2,3) or not np.isfinite(affine).all():
        raise ValueError('有効な affine 変換が必要')
    mapped = np.c_[pixels,np.ones(len(pixels))]@affine.T
    lat,lon = pixel_llh(mapped[:,0]+raster['x0'],mapped[:,1]+raster['y0'],raster['z'])
    return np.c_[lat,lon]


def vehicle_candidates(rgb: np.ndarray, parking_mask: np.ndarray, mpp: float) -> list[dict]:
    """駐車場内の明るい矩形を車両候補とする。車種や存在時刻は推定しない."""
    if mpp <= 0:
        raise ValueError('mpp は正値が必要')
    hsv = cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    # 白線は幅と面積、建物は駐車場マスクで除く。暗色車両は検出対象外である。
    mask = ((hsv[:,:,2]>155)&(hsv[:,:,1]<100)&(parking_mask>0)).astype(np.uint8)*255
    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((2,2),np.uint8))
    contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    result=[]
    for contour in contours:
        center,size,angle = cv2.minAreaRect(contour)
        short,long = sorted(v*mpp for v in size)
        area = cv2.contourArea(contour)*mpp*mpp
        if not (1.05 <= short <= 2.4 and 2.4 <= long <= 5.5 and 2.5 <= area <= 11):
            continue
        if area/(short*long) < .60 or long/short < 1.35:
            continue
        result.append(dict(pixel=list(center),width_m=size[0]*mpp,depth_m=size[1]*mpp,
                           yaw_rad=-math.radians(angle),height_m=1.5,confidence='candidate',
                           evidence='parking-mask+bright-rectangle',area_m2=area))
    return sorted(result,key=lambda o:(o['pixel'][1],o['pixel'][0]))


def vegetation_strips(rgb: np.ndarray, exclusion: np.ndarray, mpp: float) -> list[dict]:
    """低木帯候補を緑色の細長い領域から抽出する。高さは未観測である."""
    f=rgb.astype(np.float32)
    green=(2*f[:,:,1]-f[:,:,0]-f[:,:,2])/np.maximum(f.sum(axis=2),45)
    mask=((green>.09)&(exclusion==0)).astype(np.uint8)*255
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    result=[]
    for contour in contours:
        center,size,angle=cv2.minAreaRect(contour)
        short,long=sorted(v*mpp for v in size)
        if .5 <= short <= 2.8 and 5 <= long <= 50 and long/short >= 4:
            fill=cv2.contourArea(contour)*mpp*mpp/(short*long)
            if fill>.4:
                result.append(dict(pixel=list(center),width_m=size[0]*mpp,
                    depth_m=size[1]*mpp,yaw_rad=-math.radians(angle),height_m=.8,
                    evidence='elongated-green-region',confidence='candidate'))
    return result


def filter_tree_candidates(rgb: np.ndarray, candidates: list[dict], exclusion: np.ndarray,
                           width: float, depth: float) -> list[dict]:
    """車両・屋根の色かぶりを樹冠として使わないよう候補を保守的に絞る."""
    h,w=rgb.shape[:2]
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    value=rgb.astype(np.float32)
    green=(2*value[:,:,1]-value[:,:,0]-value[:,:,2])/np.maximum(value.sum(axis=2),45)
    eligible=(green>.13)&(hsv[:,:,0]>=25)&(hsv[:,:,0]<=85)&(hsv[:,:,1]>45)
    eligible&=(hsv[:,:,2]>35)&(hsv[:,:,2]<210)&(exclusion==0)
    kept=[]
    for tree in candidates:
        x=int((tree['x']/width+.5)*w);y=int((.5-tree['y']/depth)*h)
        if not (0<=x<w and 0<=y<h) or exclusion[y,x]:continue
        r=max(2,int(min(tree['r'],2)/width*w))
        patch=eligible[max(0,y-r):min(h,y+r+1),max(0,x-r):min(w,x+r+1)]
        if patch.size and patch.mean()>.55 and tree['confidence']>.4:
            kept.append(dict(tree,verification='rgb-filtered-candidate'))
    return kept


def register_course_image(official: np.ndarray, aerial: np.ndarray,
                          controls: list[dict]) -> dict:
    """目視対応で探索範囲を制限し、SIFT/RANSAC で地図の affine を推定する."""
    from scipy.spatial import cKDTree

    a=np.array([c['official_pixel']+[1] for c in controls],float)
    b=np.array([c['gsi_pixel'] for c in controls],float)
    initial=np.linalg.lstsq(a,b,rcond=None)[0]
    sift=cv2.SIFT_create(nfeatures=25000)
    ka,da=sift.detectAndCompute(official,None)
    kb,db=sift.detectAndCompute(aerial,None)
    if da is None or db is None:raise ValueError('画像特徴が不足')
    pa=np.float32([k.pt for k in ka]);pb=np.float32([k.pt for k in kb])
    prediction=np.c_[pa,np.ones(len(pa))]@initial
    tree=cKDTree(pb);pairs=[]
    for i,near in enumerate(tree.query_ball_point(prediction,30)):
        if len(near)<2:continue
        distance=np.linalg.norm(db[near]-da[i],axis=1);order=np.argsort(distance)
        if distance[order[0]]<260 and distance[order[0]]<.72*distance[order[1]]:
            pairs.append((i,near[order[0]]))
    if len(pairs)<15:raise ValueError('対応点が不足')
    source=np.float32([pa[i] for i,j in pairs]);target=np.float32([pb[j] for i,j in pairs])
    cv2.setRNGSeed(42)
    affine,mask=cv2.estimateAffine2D(source,target,method=cv2.RANSAC,
        ransacReprojThreshold=2.5,maxIters=10000,confidence=.999)
    if affine is None or int(mask.sum())<15:raise ValueError('有効な画像位置合わせを確定できない')
    errors=np.linalg.norm(np.c_[source,np.ones(len(source))]@affine.T-target,axis=1)
    rms=float(np.sqrt(np.mean(errors[mask[:,0]>0]**2)))
    if rms>2.5 or np.linalg.det(affine[:,:2])<=0:raise ValueError('位置合わせが不安定')
    return dict(affine=affine.tolist(),inliers=int(mask.sum()),rmse_px=rms,
                source_points=source[mask[:,0]>0].tolist(),target_points=target[mask[:,0]>0].tolist())
