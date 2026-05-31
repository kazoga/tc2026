#!/usr/bin/env python3
"""robot_console の実 GUI を操作して route stack を評価する.

Tkinter の座標クリックは使わず、UiMain の automation hook から Combobox、Entry、
Checkbutton、Button 相当の操作を実行する。
ローカルデスクトップまたは X11 転送が有効な環境での実行を前提とする。
"""

from __future__ import annotations

import argparse
import os
import threading
import time
from pathlib import Path
from typing import Iterable, Optional, Sequence

import rclpy
from rclpy.executors import MultiThreadedExecutor

from headless_route_stack_eval import (
    DEFAULT_LAUNCH_ORDER,
    STOPPED_STATUSES,
    EvalConfig,
    TopicMonitor,
    config_from_args,
    parse_launch_order,
    positive_float,
)
from robot_console.robot_console_node import RobotConsoleNode
from robot_console.ui_main import UiMain


class GuiRouteStackEvaluator:
    """UiMain の widget 操作を通して route stack を評価する。"""

    def __init__(
        self,
        config: EvalConfig,
        verify_log_open_buttons: bool = False,
        log_open_profiles: Sequence[str] = DEFAULT_LAUNCH_ORDER,
        show_drive_status_gui: bool = False,
    ) -> None:
        self._config = config
        self._verify_log_open_buttons = verify_log_open_buttons
        self._log_open_profiles = tuple(log_open_profiles)
        self._show_drive_status_gui = show_drive_status_gui
        self._console = RobotConsoleNode(
            console_log_directory=config.console_log_directory
        )
        self._monitor = TopicMonitor(config.goal_label, config)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._console)
        self._executor.add_node(self._monitor)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._ui = UiMain(self._console.core)
        self._seen_logs: dict[str, int] = {}
        self._result = 1
        self._goal_reached = False
        self._stop_started_at: Optional[float] = None
        self._timeout_deadline: Optional[float] = None

    def run(self) -> int:
        """GUI 評価を実行し、終了コードを返す。"""

        self._executor_thread.start()
        try:
            self._ui.automation_after(500, self._configure_launch_profiles)
            self._ui.run()
            return self._result
        finally:
            self._executor.shutdown()
            self._console.destroy_node()
            self._monitor.destroy_node()
            self._executor_thread.join(timeout=2.0)

    def _configure_launch_profiles(self) -> None:
        try:
            self._select_param("route_planner", self._config.route_planner_param)
            self._select_param("route_manager", self._config.route_manager_param)
            self._select_param("route_follower", self._config.route_follower_param)
            self._select_param("robot_navigator", self._config.robot_navigator_param)
            self._ui.automation_set_launch_override(
                "route_manager", "start_label", self._config.start_label
            )
            self._ui.automation_set_launch_override(
                "route_manager", "goal_label", self._config.goal_label
            )
            self._ui.automation_set_launch_override(
                "drive_mode_manager",
                "start_gui",
                "true" if self._show_drive_status_gui else "false",
            )
            self._ui.automation_set_launch_override(
                "drive_mode_manager", "joy_input", "joy_node"
            )
            self._ui.automation_set_launch_override(
                "robot_navigator", "cmd_vel_topic", "/cmd_vel/autonomous"
            )
            self._ui.automation_set_launch_override(
                "robot_navigator", "odom_topic", "/ypspur_ros/odom"
            )
            self._ui.automation_set_launch_override(
                "robot_navigator", "pose_enu_topic", "/localization/pose_enu"
            )
            self._ui.automation_set_simulator_enabled(
                "robot_navigator", self._config.simulator
            )
        except Exception as exc:  # pylint: disable=broad-except
            print(f"[gui-test] configure failed: {exc}", flush=True)
            self._result = 4
            self._ui.automation_request_close()
            return
        self._ui.automation_after(500, self._after_configured)

    def _after_configured(self) -> None:
        self._print_launch_selection()
        self._launch_profile_at(0)

    def _select_param(self, profile_id: str, requested: str) -> None:
        display = self._resolve_param_display(profile_id, requested)
        self._ui.automation_set_launch_param(profile_id, display)

    def _resolve_param_display(self, profile_id: str, requested: str) -> str:
        state = self._console.core.snapshot().launch_states.get(profile_id)
        if state is None:
            raise RuntimeError(f"profile が見つかりません: {profile_id}")
        if requested in state.param_display_map:
            return requested
        matches = [
            display
            for display, path in state.param_display_map.items()
            if path == requested or path.endswith("/" + requested) or display == requested
        ]
        if not matches:
            available = ", ".join(state.available_params) or "(none)"
            raise RuntimeError(
                f"{profile_id} のパラメータ {requested!r} が見つかりません: "
                f"{available}"
            )
        if len(matches) > 1:
            raise RuntimeError(
                f"{profile_id} のパラメータ {requested!r} が複数候補に一致します: "
                f"{matches}"
            )
        return matches[0]

    def _print_launch_selection(self) -> None:
        snapshot = self._console.core.snapshot()
        for profile_id in self._config.launch_order:
            state = snapshot.launch_states.get(profile_id)
            if state is None:
                continue
            print(
                f"[gui-test] profile={profile_id} "
                f"param={state.selected_param_display!r} "
                f"simulator={state.simulator_enabled} overrides={state.override_inputs}",
                flush=True,
            )

    def _launch_profile_at(self, index: int) -> None:
        if index >= len(self._config.launch_order):
            if self._config.manual_start:
                print("[gui-test] manual_start=True button invoke", flush=True)
                self._ui.automation_send_manual_start(True)
            self._timeout_deadline = time.monotonic() + self._config.timeout_sec
            self._ui.automation_after(500, self._poll_goal)
            return
        profile_id = self._config.launch_order[index]
        print(f"[gui-test] launch button invoke: {profile_id}", flush=True)
        self._ui.automation_launch_profile(profile_id)
        delay_ms = int(self._config.startup_wait_sec * 1000)
        self._ui.automation_after(delay_ms, lambda: self._launch_profile_at(index + 1))

    def _poll_goal(self) -> None:
        self._drain_console_logs()
        if self._monitor.goal_reached_time is not None:
            self._goal_reached = True
            delay_ms = int(self._config.post_goal_wait_sec * 1000)
            if self._verify_log_open_buttons:
                self._ui.automation_after(delay_ms, self._verify_log_open_buttons_once)
            else:
                self._ui.automation_after(delay_ms, self._start_stop_sequence)
            return
        assert self._timeout_deadline is not None
        if time.monotonic() >= self._timeout_deadline:
            print(
                f"[gui-test] timeout waiting for goal label {self._config.goal_label!r}",
                flush=True,
            )
            self._result = 2
            self._start_stop_sequence()
            return
        self._ui.automation_after(500, self._poll_goal)

    def _verify_log_open_buttons_once(self) -> None:
        all_ok = True
        for profile_id in self._log_open_profiles:
            path_text = self._ui.automation_get_log_file_path(profile_id)
            if not path_text:
                print(
                    f"[gui-test] log open button path missing: {profile_id}",
                    flush=True,
                )
                all_ok = False
                continue
            path = Path(path_text)
            if not path.exists() or path.stat().st_size <= 0:
                print(
                    f"[gui-test] log file invalid: {profile_id} path={path}",
                    flush=True,
                )
                all_ok = False
                continue
            try:
                self._ui.automation_open_log_file(profile_id)
            except Exception as exc:  # pylint: disable=broad-except
                print(
                    f"[gui-test] log open button failed: {profile_id}: {exc}",
                    flush=True,
                )
                all_ok = False
            else:
                print(
                    f"[gui-test] log open button invoked: {profile_id} path={path}",
                    flush=True,
                )
        if not all_ok and self._result == 1:
            self._result = 5
        self._start_stop_sequence()

    def _start_stop_sequence(self) -> None:
        print("[gui-test] stop button invoke for launched profiles", flush=True)
        self._stop_started_at = time.monotonic()
        for profile_id in reversed(self._config.launch_order):
            self._ui.automation_stop_profile(profile_id)
        self._ui.automation_after(500, self._poll_stopped)

    def _poll_stopped(self) -> None:
        self._drain_console_logs()
        if self._all_launch_profiles_stopped():
            self._print_remaining_launch_states()
            if self._goal_reached and self._result == 1:
                self._result = 0
            print("[gui-test] shutdown completed", flush=True)
            self._ui.automation_request_close()
            return
        assert self._stop_started_at is not None
        if time.monotonic() - self._stop_started_at >= self._config.stop_timeout_sec:
            self._print_remaining_launch_states()
            print("[gui-test] launched profiles did not stop cleanly", flush=True)
            if self._result == 1:
                self._result = 3
            self._ui.automation_request_close()
            return
        self._ui.automation_after(500, self._poll_stopped)

    def _all_launch_profiles_stopped(self) -> bool:
        snapshot = self._console.core.snapshot()
        for profile_id in self._config.launch_order:
            state = snapshot.launch_states.get(profile_id)
            if state is None:
                continue
            if state.status not in STOPPED_STATUSES:
                return False
            if state.process_id is not None or state.simulator_process_id is not None:
                return False
        return True

    def _print_remaining_launch_states(self) -> None:
        snapshot = self._console.core.snapshot()
        for profile_id in self._config.launch_order:
            state = snapshot.launch_states.get(profile_id)
            if state is None:
                continue
            print(
                f"[gui-test] stop state: {profile_id} status={state.status.name} "
                f"pid={state.process_id} sim_pid={state.simulator_process_id} "
                f"error={state.error_message!r}",
                flush=True,
            )

    def _drain_console_logs(self) -> None:
        snapshot = self._console.core.snapshot()
        keywords = (
            "ERROR",
            "Error",
            "error",
            "failed",
            "Failed",
            "Traceback",
            "started",
            "Using",
            "generated temporary yaml",
        )
        for profile_id, lines in snapshot.console_logs.items():
            start = self._seen_logs.get(profile_id, 0)
            for line in lines[start:]:
                if any(key in line for key in keywords):
                    print(f"[console:{profile_id}] {line}", flush=True)
            self._seen_logs[profile_id] = len(lines)


def build_arg_parser() -> argparse.ArgumentParser:
    """CLI 引数パーサを構築する。"""

    parser = argparse.ArgumentParser(
        description="robot_console の実 GUI を操作する route stack 評価ツール"
    )
    parser.add_argument("--start-label", default="10", help="route_manager の開始ラベル")
    parser.add_argument("--goal-label", default="30", help="route_manager の終了ラベル")
    parser.add_argument("--route-planner-param", default="tsukuba.yaml")
    parser.add_argument("--route-manager-param", default="tsukuba.yaml")
    parser.add_argument("--route-follower-param", default="default.yaml")
    parser.add_argument("--robot-navigator-param", default="default.yaml")
    parser.add_argument("--timeout-sec", type=positive_float, default=180.0)
    parser.add_argument("--post-goal-wait-sec", type=positive_float, default=10.0)
    parser.add_argument("--startup-wait-sec", type=positive_float, default=3.0)
    parser.add_argument("--stop-timeout-sec", type=positive_float, default=20.0)
    parser.add_argument("--summary-period-sec", type=positive_float, default=5.0)
    parser.add_argument("--cmd-vel-period-sec", type=positive_float, default=5.0)
    parser.add_argument(
        "--console-log-directory",
        default=os.environ.get("ROBOT_CONSOLE_LOG_DIR"),
        help=(
            "robot_console 管理の子プロセス stdout/stderr 保存先。"
            "未指定時は ROBOT_CONSOLE_LOG_DIR を参照します"
        ),
    )
    parser.add_argument(
        "--launch-order",
        type=parse_launch_order,
        default=DEFAULT_LAUNCH_ORDER,
        help="起動する profile ID のカンマ区切り一覧",
    )
    parser.add_argument(
        "--verify-log-open-buttons",
        action="store_true",
        help="goal 到達後にログファイルを開くボタンを invoke して検証する",
    )
    parser.add_argument(
        "--log-open-profiles",
        type=parse_launch_order,
        default=DEFAULT_LAUNCH_ORDER,
        help="ログファイルボタンを検証する profile ID のカンマ区切り一覧",
    )
    parser.add_argument(
        "--show-drive-status-gui",
        action="store_true",
        help="drive_mode_manager の専用状態 GUI も起動して表示確認する",
    )
    parser.add_argument("--no-simulator", action="store_true")
    parser.add_argument("--no-manual-start", action="store_true")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    """CLI エントリポイント。"""

    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if not os.environ.get("DISPLAY"):
        print(
            "[gui-test] DISPLAY が未設定です。"
            "ローカルデスクトップまたは X11 転送ありの環境で"
            "実行してください。",
            flush=True,
        )
        return 4
    config = config_from_args(args)
    rclpy.init()
    try:
        evaluator = GuiRouteStackEvaluator(
            config,
            verify_log_open_buttons=args.verify_log_open_buttons,
            log_open_profiles=args.log_open_profiles,
            show_drive_status_gui=args.show_drive_status_gui,
        )
        return evaluator.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
