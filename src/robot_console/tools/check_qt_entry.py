#!/usr/bin/env python3
"""既存模擬ROS graphに正式Qt入口を接続し、4タブと実受信表示を期限付き検査する."""
import argparse
import json
import os
from pathlib import Path
import sys


def main() -> None:
    """操作指令を送らず、正式入口のQtイベントループ中に表示結果を保存する."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = '86'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    from PyQt5 import QtCore, QtWidgets
    from robot_console import ui_qt_main

    original_exec = QtWidgets.QApplication.exec_
    result = {}

    def capture(app, map_tab: bool) -> None:
        window = next(widget for widget in app.topLevelWidgets() if hasattr(widget, 'tab_widget'))
        name = 'map.png' if map_tab else 'dashboard.png'
        window.grab().save(str(args.output/name))
        snapshot = window._core.build_snapshot()
        result.update(tabs=window.tab_widget.count(),
                      environment=snapshot.operation_state.environment,
                      latitude=snapshot.localization_state.latitude,
                      fused_yaw_deg=snapshot.fusion_state.yaw_deg,
                      mode=snapshot.fusion_state.mode)
        if not map_tab:
            window.tab_widget.setCurrentIndex(1)

    def bounded_exec(app) -> int:
        QtCore.QTimer.singleShot(10000, lambda: capture(app, False))
        QtCore.QTimer.singleShot(17000, lambda: capture(app, True))
        QtCore.QTimer.singleShot(19000, app.quit)
        return original_exec()

    QtWidgets.QApplication.exec_ = bounded_exec
    sys.argv = ['robot_console_qt', '--business-environment', 'デジタルツイン',
                '--ros-args', '-p', 'use_sim_time:=true', '-r', 'odom:=/ypspur_ros/odom']
    status = ui_qt_main.main(sys.argv)
    result['pass_test'] = bool(status == 0 and result.get('tabs') == 4
                               and result.get('environment') == 'デジタルツイン'
                               and result.get('latitude') is not None
                               and result.get('fused_yaw_deg') is not None)
    (args.output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result['pass_test']:
        raise RuntimeError('正式UI入口の表示確認が不合格')


if __name__ == '__main__':
    main()
