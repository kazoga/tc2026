# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    param_file = PathJoinSubstitution([
        FindPackageShare('obstacle_monitor'),
        'params',
        'default.yaml',
    ])

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LiDAR入力トピック名',
    )
    hint_topic_arg = DeclareLaunchArgument(
        'hint_topic',
        default_value='/obstacle_avoidance_hint',
        description='障害物回避ヒント出力トピック名',
    )
    viewer_topic_arg = DeclareLaunchArgument(
        'viewer_topic',
        default_value='/sensor_viewer',
        description='デバッグ画像出力トピック名',
    )
    pose_enu_topic_arg = DeclareLaunchArgument(
        'pose_enu_topic',
        default_value='/localization/pose_enu',
        description='ENU自己位置入力トピック名',
    )
    active_target_topic_arg = DeclareLaunchArgument(
        'active_target_topic',
        default_value='/active_target',
        description='現在の目標Poseトピック名',
    )

    scan_topic = LaunchConfiguration('scan_topic')
    hint_topic = LaunchConfiguration('hint_topic')
    viewer_topic = LaunchConfiguration('viewer_topic')
    pose_enu_topic = LaunchConfiguration('pose_enu_topic')
    active_target_topic = LaunchConfiguration('active_target_topic')

    node = Node(
        package='obstacle_monitor',
        executable='obstacle_monitor',
        name='obstacle_monitor',
        output='screen',
        parameters=[param_file],
        remappings=[
            ('scan', scan_topic),
            ('obstacle_avoidance_hint', hint_topic),
            ('sensor_viewer', viewer_topic),
            ('localization/pose_enu', pose_enu_topic),
            ('active_target', active_target_topic),
        ],
    )

    return LaunchDescription([
        scan_topic_arg,
        hint_topic_arg,
        viewer_topic_arg,
        pose_enu_topic_arg,
        active_target_topic_arg,
        node,
    ])
