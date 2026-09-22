"""robot_simulator が申告する駆動系契約が robot_navigator の要求を満たすことを確認する.

`robot_navigator` は `/motion_limits` を 0.5 秒の watchdog で監視し、途絶すると停止指令を
出し続ける。実機では ypspur_node が配信するが、シミュレーション時は robot_simulator が
同じ役割を担う。申告値が要求を下回ると走行できないため、契約の成立をテストで固定する。
"""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parents[1]))
from robot_navigator.braking import driver_compatible

# robot_navigator_node.py が InputWatchdog へ登録する motion_limits のタイムアウト [s]。
MOTION_LIMITS_TIMEOUT_SEC = 0.5
# robot_simulator_node.py の cycle_hz 既定値 [Hz]。積分ループと同じタイマーで配信する。
SIMULATOR_CYCLE_HZ = 10.0


def _repository_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _simulator_defaults() -> tuple[float, float, float]:
    """robot_simulator が申告する加減速の既定値を取得する."""

    pytest.importorskip('rclpy')
    pytest.importorskip('tf2_ros')
    from robot_navigator.robot_simulator_node import (
        DEFAULT_MAX_ACCEL_MPS2,
        DEFAULT_MAX_ANGULAR_ACCEL_RPS2,
        DEFAULT_MAX_DECEL_MPS2,
    )
    return (DEFAULT_MAX_ACCEL_MPS2, DEFAULT_MAX_DECEL_MPS2, DEFAULT_MAX_ANGULAR_ACCEL_RPS2)


def test_simulator_limits_satisfy_navigator_requirements() -> None:
    """シミュレータの申告値で driver_compatible が成立する."""

    accel, decel, angular_accel = _simulator_defaults()
    root = _repository_root()
    nav = yaml.safe_load(
        (root/'src/robot_navigator/params/default.yaml').read_text()
    )['robot_navigator']['ros__parameters']

    # robot_simulator_node.py の max_linear_mps / max_angular_rps の既定値。
    message = SimpleNamespace(
        max_linear_velocity=2.0,
        max_angular_velocity=3.0,
        linear_acceleration=accel,
        linear_deceleration=decel,
        angular_acceleration=angular_accel,
    )
    assert driver_compatible(message, nav['max_vel'], nav['max_w'], nav['max_acc_v'],
                             nav['max_acc_w'], nav['max_decel_v'])


def test_simulator_declares_same_contract_as_real_driver() -> None:
    """申告する加減速が実機 ypspur_node の設定値と一致する."""

    accel, decel, angular_accel = _simulator_defaults()
    root = _repository_root()
    cfg = yaml.safe_load(
        (root/'src/ypspur_ros2/config/default.yaml').read_text()
    )['/ypspur_node']['ros__parameters']

    assert accel == cfg['acceleration_max']['linear']
    assert decel == cfg['deceleration_max']['linear']
    assert angular_accel == cfg['acceleration_max']['angular']


def test_publish_period_keeps_watchdog_satisfied() -> None:
    """既定の配信周期が watchdog のタイムアウトに対して余裕を持つ."""

    period = 1.0/SIMULATOR_CYCLE_HZ
    assert period < MOTION_LIMITS_TIMEOUT_SEC
    # 1 回取りこぼしても途絶と判定されない程度の余裕を確保する。
    assert period*2 < MOTION_LIMITS_TIMEOUT_SEC
