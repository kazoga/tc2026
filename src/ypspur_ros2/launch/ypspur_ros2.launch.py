"""ypspur_node 起動用 launch file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('ypspur_ros2')
    default_config = os.path.join(pkg_share, 'config', 'default.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to parameter YAML file',
    )

    node = Node(
        package='ypspur_ros2',
        executable='ypspur_node',
        name='ypspur_node',
        parameters=[LaunchConfiguration('config')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([config_arg, node])
