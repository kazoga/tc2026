"""rtk_gps_um982_node 起動用 launch file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('rtk_gps_um982')
    default_config = os.path.join(pkg_share, 'config', 'default.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to parameter YAML file',
    )

    node = Node(
        package='rtk_gps_um982',
        executable='rtk_gps_um982_node',
        name='rtk_gps_um982_node',
        namespace='rtk_gps',
        parameters=[LaunchConfiguration('config')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([config_arg, node])
