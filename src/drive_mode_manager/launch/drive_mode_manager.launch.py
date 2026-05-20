from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('drive_mode_manager')
    default_param = PathJoinSubstitution([pkg_share, 'params', 'default.yaml'])

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param,
        description='drive_mode_manager ノード群のパラメータファイル',
    )
    start_gui_arg = DeclareLaunchArgument(
        'start_gui', default_value='true', description='drive_status_gui_node を起動する'
    )

    param_file = LaunchConfiguration('param_file')
    manual_teleop_node = Node(
        package='drive_mode_manager',
        executable='manual_teleop_node',
        name='manual_teleop_node',
        output='screen',
        parameters=[param_file],
    )
    drive_cmd_mux_node = Node(
        package='drive_mode_manager',
        executable='drive_cmd_mux_node',
        name='drive_cmd_mux_node',
        output='screen',
        parameters=[param_file],
    )
    drive_status_gui_node = Node(
        package='drive_mode_manager',
        executable='drive_status_gui_node',
        name='drive_status_gui_node',
        output='screen',
        parameters=[param_file],
        condition=IfCondition(LaunchConfiguration('start_gui')),
    )

    return LaunchDescription([
        param_file_arg,
        start_gui_arg,
        manual_teleop_node,
        drive_cmd_mux_node,
        drive_status_gui_node,
    ])
