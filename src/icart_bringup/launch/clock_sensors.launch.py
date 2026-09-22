"""GNSSとMID360だけで時刻同期を検証。車輪・融合・自律走行は起動しない。"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    directory = Path(LaunchConfiguration('session_directory').perform(context)).expanduser().resolve()
    for name in ['um982.yaml', 'livox.json']:
        if not (directory/name).is_file():
            raise ValueError('実機sessionの設定がありません: '+str(directory/name))
    return [
        Node(package='rtk_gps_um982', executable='rtk_gps_um982_node',
             name='rtk_gps_um982_node', namespace='rtk_gps', output='screen',
             parameters=[str(directory/'um982.yaml'), {'time_sync.enabled': True,
                 'time_sync.chrony_socket': '/run/chrony/um982.sock',
                 'stamp_source': 'gnss_utc', 'transport_delay_ms': 0, 'use_sim_time': False}]),
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             namespace='mid360', output='screen',
             parameters=[{'xfer_format': 1, 'multi_topic': 0, 'data_src': 0, 'publish_freq': 10.,
                 'output_data_type': 0, 'frame_id': 'mid360_frame',
                 'user_config_path': str(directory/'livox.json'), 'use_sim_time': False}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session_directory', description='実機で生成済みsessionディレクトリ'),
        OpaqueFunction(function=setup),
    ])
