from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    joy_input_arg = DeclareLaunchArgument(
        'joy_input',
        default_value='joy_node',
        description='Joy 入力源。joy_node または ps3_joy_sim',
    )

    param_file = LaunchConfiguration('param_file')
    joy_input = LaunchConfiguration('joy_input')
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", joy_input, "' == 'joy_node'"])),
    )
    ps3_joy_sim_node = Node(
        package='drive_mode_manager',
        executable='ps3_joy_sim_node',
        name='ps3_joy_sim_node',
        output='screen',
        parameters=[param_file],
        condition=IfCondition(PythonExpression(["'", joy_input, "' == 'ps3_joy_sim'"])),
    )
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
        joy_input_arg,
        joy_node,
        ps3_joy_sim_node,
        manual_teleop_node,
        drive_cmd_mux_node,
        drive_status_gui_node,
    ])
