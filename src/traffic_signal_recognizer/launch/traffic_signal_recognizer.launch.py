from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('traffic_signal_recognizer')
    default_param = PathJoinSubstitution([pkg_share, 'params', 'default.yaml'])

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param,
        description='traffic_signal_recognizerノードのパラメータファイル',
    )

    recognizer_node = Node(
        package='traffic_signal_recognizer',
        executable='traffic_signal_recognizer',
        name='traffic_signal_recognizer',
        output='screen',
        parameters=[LaunchConfiguration('param_file')],
    )

    return LaunchDescription([param_file_arg, recognizer_node])
