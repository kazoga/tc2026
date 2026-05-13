"""信号認識の ROS 非依存判定ロジック."""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Iterable, Optional, Sequence


@dataclass(frozen=True)
class DetectionCandidate:
    """YOLO 検出候補."""

    class_id: int
    class_name: str
    score: float


@dataclass(frozen=True)
class SignalDecision:
    """信号認識の判定結果."""

    status: int
    selected_class_id: int
    selected_class_name: str
    selected_score: float
    is_green: bool
    history: tuple[bool, ...]


class TrafficSignalRecognitionCore:
    """直近の信号検出履歴から GO/STOP を判定する."""

    def __init__(
        self,
        confidence_threshold: float = 0.8,
        judge_count: int = 3,
        go_status: int = 1,
        stop_status: int = 2,
        unknown_class_id: int = 99,
        green_class_ids: Optional[Sequence[int]] = None,
        red_class_ids: Optional[Sequence[int]] = None,
        green_class_names: Optional[Sequence[str]] = None,
        red_class_names: Optional[Sequence[str]] = None,
        hold_go: bool = False,
    ) -> None:
        """信号認識 core を初期化する.

        Args:
            confidence_threshold (float): 採用する最小信頼度.
            judge_count (int): GO 判定に必要な連続 green 回数.
            go_status (int): GO として publish する値.
            stop_status (int): STOP として publish する値.
            unknown_class_id (int): 未検出時の内部 class id.
            green_class_ids (Optional[Sequence[int]]): green とみなす class id.
            red_class_ids (Optional[Sequence[int]]): red とみなす class id.
            green_class_names (Optional[Sequence[str]]): green とみなす class name.
            red_class_names (Optional[Sequence[str]]): red とみなす class name.
            hold_go (bool): 一度 GO 判定した後に GO を保持するか.
        """

        self.confidence_threshold = float(confidence_threshold)
        self.judge_count = max(1, int(judge_count))
        self.go_status = int(go_status)
        self.stop_status = int(stop_status)
        self.unknown_class_id = int(unknown_class_id)
        self.green_class_ids = set(green_class_ids or [1])
        self.red_class_ids = set(red_class_ids or [0])
        self.green_class_names = self._normalize_names(green_class_names or ['green'])
        self.red_class_names = self._normalize_names(red_class_names or ['red'])
        self.hold_go = bool(hold_go)

        self._green_history: Deque[bool] = deque(maxlen=self.judge_count)
        self._go_latched = False

    def reset(self) -> None:
        """判定履歴を初期化する."""

        self._green_history.clear()
        self._go_latched = False

    def update(self, candidates: Iterable[DetectionCandidate]) -> SignalDecision:
        """検出候補から GO/STOP を更新判定する.

        Args:
            candidates (Iterable[DetectionCandidate]): 1 フレーム分の検出候補.

        Returns:
            SignalDecision: 判定結果.
        """

        selected = self.select_best_candidate(candidates)
        is_green = self._is_green(selected) if selected is not None else False
        self._green_history.append(is_green)

        history_ready = len(self._green_history) >= self.judge_count
        green_continuous = history_ready and all(self._green_history)
        if green_continuous:
            self._go_latched = True

        if self._go_latched and self.hold_go:
            status = self.go_status
        else:
            status = self.go_status if green_continuous else self.stop_status

        if selected is None:
            return SignalDecision(
                status=status,
                selected_class_id=self.unknown_class_id,
                selected_class_name='unknown',
                selected_score=0.0,
                is_green=False,
                history=tuple(self._green_history),
            )

        return SignalDecision(
            status=status,
            selected_class_id=selected.class_id,
            selected_class_name=selected.class_name,
            selected_score=selected.score,
            is_green=is_green,
            history=tuple(self._green_history),
        )

    def select_best_candidate(
        self, candidates: Iterable[DetectionCandidate]
    ) -> Optional[DetectionCandidate]:
        """信頼度閾値以上かつ既知 class の候補から最大信頼度のものを返す.
        """

        best: Optional[DetectionCandidate] = None
        for candidate in candidates:
            if candidate.score < self.confidence_threshold:
                continue
            if not self._is_known_signal(candidate):
                continue
            if best is None or candidate.score > best.score:
                best = candidate
        return best

    def _is_known_signal(self, candidate: DetectionCandidate) -> bool:
        return self._is_green(candidate) or self._is_red(candidate)

    def _is_green(self, candidate: DetectionCandidate) -> bool:
        return (
            candidate.class_id in self.green_class_ids
            or self._normalize_name(candidate.class_name) in self.green_class_names
        )

    def _is_red(self, candidate: DetectionCandidate) -> bool:
        return (
            candidate.class_id in self.red_class_ids
            or self._normalize_name(candidate.class_name) in self.red_class_names
        )

    @staticmethod
    def _normalize_names(names: Sequence[str]) -> set[str]:
        return {TrafficSignalRecognitionCore._normalize_name(name) for name in names}

    @staticmethod
    def _normalize_name(name: str) -> str:
        return str(name).strip().lower()
