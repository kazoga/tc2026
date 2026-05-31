from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('drive_mode_manager')
    default_param = PathJoinSubstitution([pkg_share, 'params', 'default.yaml'])

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param,
        description='ps3_joy_sim_node のパラメータファイル',
    )

    ps3_joy_sim_node = Node(
        package='drive_mode_manager',
        executable='ps3_joy_sim_node',
        name='ps3_joy_sim_node',
        output='screen',
        parameters=[LaunchConfiguration('param_file')],
    )

    return LaunchDescription([
        param_file_arg,
        ps3_joy_sim_node,
    ])
