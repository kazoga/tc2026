"""融合のみを起動する。実機ドライバや走行モードは自動起動しない."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('projection_params', description='ENU原点を含むROSパラメータYAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='gnss_lio_fusion', executable='fusion_node', name='gnss_lio_fusion',
             parameters=[PathJoinSubstitution([FindPackageShare('gnss_lio_fusion'),
                                               'params', 'default.yaml']),
                         LaunchConfiguration('projection_params'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}], output='screen'),
    ])
