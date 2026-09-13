"""良好な観測のみでアンテナ間距離の基準を適応推定する."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class BaselineConfig:
    """距離の許容範囲は取付誤りを検出する広い事前範囲とする."""
    initial_m: float = .5
    minimum_m: float = .35
    maximum_m: float = .65
    min_satellites: int = 14
    initialize_s: float = 2.
    reacquire_s: float = 8.
    adapt_tau_s: float = 60.
    tolerance_min_m: float = .01
    tolerance_max_m: float = .03
    stable_sigma_m: float = .004


class AdaptiveBaseline:
    """異常観測への追従を防ぎ、安定した新しい取付値は別候補として再取得する."""

    def __init__(self, config: BaselineConfig | None = None) -> None:
        self.config = config or BaselineConfig()
        c = self.config
        if not (all(math.isfinite(v) for v in vars(c).values()) and c.min_satellites > 0
                and 0 < c.minimum_m < c.initial_m < c.maximum_m
                and 0 < c.tolerance_min_m <= c.tolerance_max_m
                and min(c.initialize_s, c.reacquire_s, c.adapt_tau_s, c.stable_sigma_m) > 0):
            raise ValueError('baseline設定の範囲・時定数が不正')
        self.reference = c.initial_m
        self.sigma = c.tolerance_min_m/3
        self.ready = False
        self.candidates = deque()
        self.last_stamp = None
        self.reacquisitions = 0

    def observe(self, stamp: float, value: float, fix: bool, satellites: int) -> dict:
        """判定値を返す。FLOAT・疎な観測・非有限値は学習しない."""
        c = self.config
        if not math.isfinite(stamp) or (self.last_stamp is not None and stamp <= self.last_stamp):
            return self.status(value, 0., 'out_of_order')
        dt = stamp-self.last_stamp if self.last_stamp is not None else 0.
        self.last_stamp = stamp
        valid = math.isfinite(value) and c.minimum_m <= value <= c.maximum_m
        if not valid or not fix or satellites < c.min_satellites:
            self.candidates.clear()
            return self.status(value, self.score(value) if valid else 0., 'frozen')
        if dt > .5:
            self.candidates.clear()
        self.candidates.append((stamp, value))
        while self.candidates and stamp-self.candidates[0][0] > c.reacquire_s+1:
            self.candidates.popleft()
        values = np.array([v for _, v in self.candidates])
        median = float(np.median(values))
        sigma = float(1.4826*np.median(abs(values-median)))
        stable = len(values) >= 15 and sigma <= c.stable_sigma_m
        duration = stamp-self.candidates[0][0]
        mode = 'learning' if not self.ready else 'tracking'
        tolerance = self.tolerance()
        if stable and not self.ready and duration >= c.initialize_s:
            self.reference, self.sigma, self.ready = median, max(.001, sigma), True
            mode = 'initialized'
        elif stable and self.ready and abs(median-self.reference) > tolerance:
            mode = 'candidate'
            if duration >= c.reacquire_s:
                self.reference, self.sigma = median, max(.001, sigma)
                self.reacquisitions += 1
                self.candidates.clear()
                mode = 'reacquired'
        elif self.ready and abs(value-self.reference) <= tolerance:
            alpha = 1-math.exp(-min(dt, .5)/c.adapt_tau_s)
            self.reference += alpha*(value-self.reference)
            # 外れ値で許容幅が際限なく大きくならないよう上限を固定する。
            self.sigma = min(c.tolerance_max_m/3,
                             max(.001, (1-alpha)*self.sigma+alpha*sigma))
        return self.status(value, self.score(value), mode)

    def tolerance(self) -> float:
        """学習ばらつきから有限の許容幅を返す."""
        c = self.config
        return float(np.clip(3*self.sigma, c.tolerance_min_m, c.tolerance_max_m))

    def score(self, value: float) -> float:
        """初期未確定時にも暫定品質を返し、起動を永久に待たない."""
        if not math.isfinite(value):
            return 0.
        return float(math.exp(-.5*((value-self.reference)/self.tolerance())**2))

    def status(self, value: float, score: float, mode: str) -> dict:
        """診断と永続保存に使用する有限の値を返す."""
        return dict(reference_m=self.reference, sigma_m=self.sigma, ready=self.ready,
                    deviation_m=value-self.reference if math.isfinite(value) else None,
                    score=score, mode=mode, reacquisitions=self.reacquisitions)
