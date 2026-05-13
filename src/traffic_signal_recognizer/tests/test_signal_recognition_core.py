import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from traffic_signal_recognizer.signal_recognition_core import (
    DetectionCandidate,
    TrafficSignalRecognitionCore,
)


def test_green_three_times_publish_go() -> None:
    core = TrafficSignalRecognitionCore(confidence_threshold=0.8, judge_count=3)
    green = [DetectionCandidate(class_id=1, class_name='green', score=0.9)]

    assert core.update(green).status == 2
    assert core.update(green).status == 2
    decision = core.update(green)

    assert decision.status == 1
    assert decision.selected_class_id == 1
    assert decision.is_green is True


def test_red_or_missing_keeps_stop() -> None:
    core = TrafficSignalRecognitionCore(confidence_threshold=0.8, judge_count=3)
    red = [DetectionCandidate(class_id=0, class_name='red', score=0.95)]

    assert core.update(red).status == 2
    assert core.update([]).status == 2
    assert core.update(red).status == 2


def test_low_score_candidate_is_ignored() -> None:
    core = TrafficSignalRecognitionCore(confidence_threshold=0.8, judge_count=1)
    low_score_green = [DetectionCandidate(class_id=1, class_name='green', score=0.79)]

    decision = core.update(low_score_green)

    assert decision.status == 2
    assert decision.selected_class_name == 'unknown'
    assert decision.selected_score == 0.0


def test_hold_go_keeps_go_after_first_green_decision() -> None:
    core = TrafficSignalRecognitionCore(
        confidence_threshold=0.8,
        judge_count=2,
        hold_go=True,
    )
    green = [DetectionCandidate(class_id=1, class_name='green', score=0.9)]
    red = [DetectionCandidate(class_id=0, class_name='red', score=0.9)]

    assert core.update(green).status == 2
    assert core.update(green).status == 1
    assert core.update(red).status == 1
