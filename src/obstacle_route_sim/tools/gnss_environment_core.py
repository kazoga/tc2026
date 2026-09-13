"""建物近傍での FIX/FLOAT と相関誤差を生成する ROS 非依存モデル."""
import math
import random

import cv2
import numpy as np


class BuildingDegradation:
    """距離とヒステリシスによる仮説モデル。電波伝搬計算ではない."""

    def __init__(self, polygons: list, enter_m: float = 12., exit_m: float = 15.,
                 sigma_m: float = .6, bias_m: float = 1.2, seed: int = 43) -> None:
        if not all(math.isfinite(v) for v in [enter_m, exit_m, sigma_m, bias_m]):
            raise ValueError('パラメータは有限値が必要')
        if not 0 <= enter_m < exit_m or min(sigma_m,bias_m)<0:
            raise ValueError('0 <= enter < exit、誤差は非負が必要')
        self.polygons=[np.asarray(p,np.float32) for p in polygons]
        self.enter,self.exit,self.sigma,self.amplitude=enter_m,exit_m,sigma_m,bias_m
        self.random=random.Random(seed)
        self.floating=False
        self.bias=[0.,0.]

    def sample(self,x: float,y: float,dt: float) -> dict:
        """建物離隔、状態、時系列相関を持つ水平 bias を返す."""
        distance=min((max(0.,-cv2.pointPolygonTest(p,(x,y),True))
                      for p in self.polygons),default=math.inf)
        if self.floating:
            self.floating=distance<self.exit
        else:
            self.floating=distance<=self.enter
        rho=math.exp(-max(0.,dt)/8.)
        for i in range(2):
            target=self.amplitude if self.floating else 0.
            self.bias[i]=rho*self.bias[i]+(1-rho)*target
        return dict(floating=self.floating,distance_m=distance,
                    bias=tuple(self.bias),sigma_m=self.sigma if self.floating else None)
