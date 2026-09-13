#!/usr/bin/env python3
"""共通launchをGazeboで起動し、既存PyQt5の開始操作と受信表示を試験する."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
import time
import threading

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import rclpy
from PyQt5 import QtWidgets

from icart_bringup.session_core import load_session
from robot_console.core.console_core import ConsoleCore
from robot_console.ros.console_node import RobotConsoleNode
from robot_console.ui_qt.dashboard_tab import DashboardTab
from robot_console.ui_qt.qt_environment import fix_qt_plugin_path_conflict
from ament_index_python.packages import get_package_prefix
sys.path.insert(0, str(Path(get_package_prefix('obstacle_route_sim'))/'lib/obstacle_route_sim'))
from evaluate_terrain_trial import Monitor


def evaluate(session: Path, output: Path, timeout: float) -> dict:
    """UIとシミュレータを同じ隔離domainで実行し、期限後に所有プロセスだけを回収する."""
    os.environ['ROS_DOMAIN_ID'] = '86'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    data = load_session(session, 'simulation')
    trial = json.loads((Path(data['trial_directory'])/'trial.json').read_text())
    output.mkdir(parents=True, exist_ok=False)
    (output/'trial.json').write_text(json.dumps(trial))
    fix_qt_plugin_path_conflict()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    rclpy.init(args=['--ros-args', '-r', 'odom:=/ypspur_ros/odom'])
    core = ConsoleCore(log_directory=str(output/'ui_logs'))
    core.update_business_mode('デジタルツイン', '自律走行')
    console = RobotConsoleNode(core)
    monitor = Monitor(trial, output, 0., True)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(console)
    executor.add_node(monitor)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    widget = DashboardTab()
    widget.manual_ops_card.manual_start_requested.connect(core.send_manual_start)
    widget.launch_control_card.update_plan(environment='デジタルツイン', drive_mode='自律走行',
        ordered_profile_ids=['icart_fused_stack'], profiles_by_id={p.profile_id: p for p in core._profiles})
    widget.resize(1500, 950)
    widget.show()
    result = {}
    core.update_selected_param('icart_fused_stack', str(session.resolve()))
    widget.launch_control_card.launch_requested.connect(lambda profile: core.request_launch(
        profile, overrides={'environment': 'simulation',
                            'fusion_log': str((output/'fusion.jsonl').resolve())}))
    for button in widget.findChildren(QtWidgets.QPushButton):
        if button.text() == '選択ノードを起動':
            button.click()
    started, clicked, screenshot = time.monotonic(), False, False
    next_ui = 0.
    snapshot = core.build_snapshot()
    try:
        while (time.monotonic()-started < timeout and
               snapshot.launch_profiles['icart_fused_stack'].status.name not in ['ERROR', 'STOPPED']):
            time.sleep(.01)
            if time.monotonic() >= next_ui:
                snapshot = core.build_snapshot()
                widget.update_snapshot(snapshot)
                app.processEvents()
                next_ui = time.monotonic()+.2
            if not clicked and time.monotonic()-started > 15 and snapshot.fusion_state.yaw_deg is not None:
                result['before_start_max_command_mps'] = max(
                    [abs(row[6]) for row in monitor.samples] or [0.])
                for button in widget.findChildren(QtWidgets.QPushButton):
                    if button.text() == 'manual_start = True 送信':
                        button.click()
                        clicked = True
            if clicked and not screenshot and time.monotonic()-started > 30:
                widget.grab().save(str(output/'console.png'))
                screenshot = True
            if (monitor.finished_sim is not None and monitor.samples
                    and monitor.samples[-1][1]-monitor.finished_sim >= 2.):
                break
        executor.shutdown(timeout_sec=3.)
        ros_thread.join(timeout=3.)
        result.update(monitor.result())
        result['ui_start_clicked'] = clicked
        snapshot = core.build_snapshot()
        result['ui_received'] = dict(rtk=snapshot.gps_state.rtk_state,
            fusion=snapshot.fusion_state.mode, heading_sigma_deg=snapshot.fusion_state.heading_sigma_deg,
            latitude=snapshot.localization_state.latitude, waypoint_count=len(trial['points']),
            odom_display=widget._odom_label.text())
        yaw_rates = [abs(math.atan2(math.sin(b[5]-a[5]), math.cos(b[5]-a[5])))
                     / (b[1]-a[1]) for a, b in zip(monitor.samples, monitor.samples[1:])
                     if b[1] > a[1]]
        result['truth_max_yaw_rate_rad_s'] = max(yaw_rates, default=0.)
        result['truth_rotation_anomaly_intervals'] = sum(rate > 2. for rate in yaw_rates)
        result['pose_publishers'] = [p.node_name for p in console.get_publishers_info_by_topic(
            '/localization/pose_enu')]
        result['cmd_publishers'] = [p.node_name for p in console.get_publishers_info_by_topic('/cmd_vel')]
        result['localization_mode'] = 'gnss_lio_fusion_with_wheel_guard'
        result['limitations'] = [
            '厳しめ模擬LiDAR/IMUと建物依存GNSS。実測校正は未実施',
            '接触監視は車体・支柱・GNSSアンテナ。車輪とキャスタは対象外',
            'DashboardTabのoffscreen操作。iPhone実機とMainWindow全体は未検証']
        result['integration_pass'] = bool(
            result.get('goal_pass') and clicked and snapshot.localization_state.latitude is not None
            and len(result['pose_publishers']) == 1 and len(result['cmd_publishers']) == 1
            and result.get('before_start_max_command_mps', 1.) < .001
            and result.get('counts', {}).get('scan', 0) > 100
            and result['truth_rotation_anomaly_intervals'] == 0)
    finally:
        executor.shutdown(timeout_sec=3.)
        ros_thread.join(timeout=3.)
        core.request_stop('icart_fused_stack')
        result['launch_status_after_stop'] = core.build_snapshot().launch_profiles[
            'icart_fused_stack'].status.name
        log_path = core.build_snapshot().log_paths.get('icart_fused_stack')
        log = Path(log_path).read_text() if log_path else ''
        (output/'stack.log').write_text(log)
        (output/'route_follower.log').write_text(log)
        with (output/'lio_trajectory.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['sim_s','x','y','z','qx','qy','qz','qw'])
            writer.writerows(monitor.lio_samples)
        result.setdefault('goal_pass', False)
        (output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        monitor.stream.close()
        monitor.destroy_node()
        console.destroy_node()
        rclpy.try_shutdown()
        widget.close()
        (output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=600.)
    args = parser.parse_args()
    result = evaluate(args.session, args.output, args.timeout)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result.get('integration_pass') else 1)
