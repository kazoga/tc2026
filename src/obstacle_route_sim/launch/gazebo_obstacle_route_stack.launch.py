#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gazebo 障害物回避・ルート復帰検証用の統合 launch."""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    """launch 引数文字列を bool に変換する."""

    return str(value).lower() in {"1", "true", "yes", "on"}


def _write_yaml(path: Path, data: dict[str, Any]) -> None:
    """YAML ファイルを UTF-8 で書き出す."""

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as yaml_file:
        yaml.safe_dump(data, yaml_file, allow_unicode=True, sort_keys=False)


def _generate_route_files(
    road_type: str,
    road_width: float,
    waypoint_step: float,
    generated_route_dir: Path,
) -> tuple[Path, Path, int]:
    """Gazebo 道路と同じ座標系の route_planner 入力を生成する."""

    scripts_dir = Path(get_package_prefix("obstacle_route_sim")) / "lib" / "obstacle_route_sim"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))

    from generate_waypoints import write_waypoint_csv

    fixed_dir = generated_route_dir / "fixed"
    route_csv = fixed_dir / f"{road_type}_w{int(road_width)}.csv"
    fixed_dir.mkdir(parents=True, exist_ok=True)
    waypoint_count = write_waypoint_csv(
        road_type=road_type,
        width=road_width,
        output=str(route_csv),
        step_m=waypoint_step,
    )
    route_config = generated_route_dir / "route_config.yaml"
    _write_yaml(
        route_config,
        {
            "blocks": [
                {
                    "type": "fixed",
                    "name": f"gazebo_{road_type}_w{int(road_width)}",
                    "segment_id": f"fixed/{route_csv.name}",
                }
            ]
        },
    )
    return route_config, route_csv, waypoint_count


def _setup_stack(context: LaunchContext, *args: Any, **kwargs: Any) -> list:
    """launch 引数を解決し、統合検証用 action を構築する."""

    del args, kwargs

    road_type = LaunchConfiguration("road_type").perform(context)
    road_width = float(LaunchConfiguration("road_width").perform(context))
    waypoint_step = float(LaunchConfiguration("waypoint_step").perform(context))
    generated_route_dir = Path(LaunchConfiguration("generated_route_dir").perform(context))
    generated_config_dir = Path(LaunchConfiguration("generated_config_dir").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    stack_use_sim_time = False
    start_drive_status_gui = _as_bool(LaunchConfiguration("start_drive_status_gui").perform(context))

    route_config, _route_csv, waypoint_count = _generate_route_files(
        road_type=road_type,
        road_width=road_width,
        waypoint_step=waypoint_step,
        generated_route_dir=generated_route_dir,
    )
    goal_label = str(max(waypoint_count - 1, 0))

    route_planner_params = generated_config_dir / "route_planner_gazebo.yaml"
    _write_yaml(
        route_planner_params,
        {
            "route_planner": {
                "ros__parameters": {
                    "config_yaml_path": str(route_config),
                    "csv_base_dir": str(generated_route_dir),
                    "use_sim_time": stack_use_sim_time,
                }
            }
        },
    )

    route_manager_params = generated_config_dir / "route_manager_gazebo.yaml"
    _write_yaml(
        route_manager_params,
        {
            "route_manager": {
                "ros__parameters": {
                    "start_label": "0",
                    "goal_label": goal_label,
                    "checkpoint_labels": [""],
                    "planner_timeout_sec": 5.0,
                    "planner_retry_count": 2,
                    "planner_connect_timeout_sec": 10.0,
                    "state_publish_rate_hz": 1.0,
                    "image_encoding_check": False,
                    "report_stuck_timeout_sec": 5.0,
                    "offset_step_max_m": 1.0,
                    "use_sim_time": stack_use_sim_time,
                }
            }
        },
    )

    route_follower_params = str(
        Path(get_package_share_directory("route_follower")) / "params" / "default.yaml"
    )
    robot_navigator_params = str(
        Path(get_package_share_directory("robot_navigator")) / "params" / "default.yaml"
    )
    obstacle_monitor_params = str(
        Path(get_package_share_directory("obstacle_monitor")) / "params" / "default.yaml"
    )
    drive_mode_params = str(
        Path(get_package_share_directory("drive_mode_manager")) / "params" / "default.yaml"
    )

    obstacle_route_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("obstacle_route_sim"),
                    "launch",
                    "sim_obstacle_route.launch.py",
                ]
            )
        ),
        launch_arguments={
            "road_type": road_type,
            "road_width": str(road_width),
            "enable_pylons": LaunchConfiguration("enable_pylons").perform(context),
            "pylon_seed": LaunchConfiguration("pylon_seed").perform(context),
            "enable_route_blocker": LaunchConfiguration("enable_route_blocker").perform(context),
            "route_blocker_distance": LaunchConfiguration("route_blocker_distance").perform(context),
            "use_sim_time": str(use_sim_time).lower(),
            "spawn_robot": "true",
            "robot_x": LaunchConfiguration("robot_x").perform(context),
            "robot_y": LaunchConfiguration("robot_y").perform(context),
            "robot_z": LaunchConfiguration("robot_z").perform(context),
            "generated_world_dir": LaunchConfiguration("generated_world_dir").perform(context),
            "start_gazebo_gui": LaunchConfiguration("start_gazebo_gui").perform(context),
        }.items(),
    )

    actions = [
        obstacle_route_sim_launch,
        Node(
            package="route_planner",
            executable="route_planner",
            name="route_planner",
            output="screen",
            emulate_tty=True,
            parameters=[str(route_planner_params)],
            remappings=[("get_route", "/get_route"), ("update_route", "/update_route")],
        ),
        Node(
            package="route_manager",
            executable="route_manager",
            name="route_manager",
            output="screen",
            emulate_tty=True,
            parameters=[str(route_manager_params)],
            remappings=[
                ("active_route", "/active_route"),
                ("route_state", "/route_state"),
                ("mission_info", "/mission_info"),
                ("manager_status", "/manager_status"),
                ("report_stuck", "/report_stuck"),
                ("get_route", "/get_route"),
                ("update_route", "/update_route"),
            ],
        ),
        Node(
            package="route_follower",
            executable="route_follower",
            name="route_follower",
            output="screen",
            emulate_tty=True,
            parameters=[
                route_follower_params,
                {"start_immediately": True, "use_sim_time": stack_use_sim_time},
            ],
            remappings=[
                ("active_route", "/active_route"),
                ("amcl_pose", "/amcl_pose"),
                ("obstacle_avoidance_hint", "/obstacle_avoidance_hint"),
                ("manual_start", "/manual_start"),
                ("sig_recog", "/sig_recog"),
                ("recog_flag", "/recog_flag"),
                ("active_target", "/active_target"),
                ("follower_state", "/follower_state"),
                ("report_stuck", "/report_stuck"),
            ],
        ),
        Node(
            package="obstacle_monitor",
            executable="obstacle_monitor",
            name="obstacle_monitor",
            output="screen",
            parameters=[obstacle_monitor_params, {"use_sim_time": stack_use_sim_time}],
            remappings=[
                ("scan", "/scan"),
                ("obstacle_avoidance_hint", "/obstacle_avoidance_hint"),
                ("sensor_viewer", "/sensor_viewer"),
                ("amcl_pose", "/amcl_pose"),
                ("active_target", "/active_target"),
            ],
        ),
        Node(
            package="robot_navigator",
            executable="robot_navigator",
            name="robot_navigator",
            output="screen",
            emulate_tty=True,
            parameters=[robot_navigator_params, {"use_sim_time": stack_use_sim_time}],
            remappings=[
                ("scan", "/scan"),
                ("odom", "/ypspur_ros/odom"),
                ("amcl_pose", "/amcl_pose"),
                ("active_target", "/active_target"),
                ("cmd_vel", "/cmd_vel/autonomous"),
                ("direction_marker", "/direction_marker"),
                ("obstacle_avoidance_hint", "/obstacle_avoidance_hint"),
            ],
        ),
        Node(
            package="drive_mode_manager",
            executable="ps3_joy_sim_node",
            name="ps3_joy_sim_node",
            output="screen",
            parameters=[drive_mode_params, {"use_sim_time": stack_use_sim_time}],
        ),
        Node(
            package="drive_mode_manager",
            executable="manual_teleop_node",
            name="manual_teleop_node",
            output="screen",
            parameters=[drive_mode_params, {"use_sim_time": stack_use_sim_time}],
        ),
        Node(
            package="drive_mode_manager",
            executable="drive_cmd_mux_node",
            name="drive_cmd_mux_node",
            output="screen",
            parameters=[drive_mode_params, {"use_sim_time": stack_use_sim_time}],
        ),
    ]

    if start_drive_status_gui:
        actions.append(
            Node(
                package="drive_mode_manager",
                executable="drive_status_gui_node",
                name="drive_status_gui_node",
                output="screen",
                parameters=[drive_mode_params, {"use_sim_time": stack_use_sim_time}],
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """LaunchDescription を生成する."""

    default_tmp = os.path.join("/tmp", "obstacle_route_sim")
    return LaunchDescription(
        [
            DeclareLaunchArgument("road_type", default_value="crank"),
            DeclareLaunchArgument("road_width", default_value="5.0"),
            DeclareLaunchArgument("enable_pylons", default_value="true"),
            DeclareLaunchArgument("pylon_seed", default_value="0"),
            DeclareLaunchArgument("enable_route_blocker", default_value="true"),
            DeclareLaunchArgument("route_blocker_distance", default_value="8.0"),
            DeclareLaunchArgument("waypoint_step", default_value="3.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robot_x", default_value="1.0"),
            DeclareLaunchArgument("robot_y", default_value="0.0"),
            DeclareLaunchArgument("robot_z", default_value="0.16"),
            DeclareLaunchArgument("generated_world_dir", default_value=default_tmp),
            DeclareLaunchArgument("generated_route_dir", default_value=f"{default_tmp}/routes"),
            DeclareLaunchArgument("generated_config_dir", default_value=f"{default_tmp}/config"),
            DeclareLaunchArgument("start_gazebo_gui", default_value="true"),
            DeclareLaunchArgument("start_drive_status_gui", default_value="true"),
            OpaqueFunction(function=_setup_stack),
        ]
    )
