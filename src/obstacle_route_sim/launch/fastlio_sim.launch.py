"""IMU 追加済み Gazebo world と FAST-LIO を隔離した仮想環境で起動する."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def resource_path(context) -> list:
    """world と同じディレクトリの相対メッシュ URI を解決する."""
    directory=str(Path(LaunchConfiguration('world').perform(context)).resolve().parent)
    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
            directory+os.pathsep+os.environ.get('GZ_SIM_RESOURCE_PATH',''))]


def generate_launch_description() -> LaunchDescription:
    """走行指令を出さず、センサと推定 odometry のみ起動する."""
    config=PathJoinSubstitution([FindPackageShare('obstacle_route_sim'),'params','fastlio_gazebo.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('world',description='prepare_fastlio_trial.py で生成した trial.sdf'),
        DeclareLaunchArgument('domain_id',default_value='86'),
        DeclareLaunchArgument('partition',default_value='icart_fastlio_sim'),
        DeclareLaunchArgument('noise_profile',default_value='field_assumed'),
        DeclareLaunchArgument('noise_seed',default_value='1'),
        SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST'),
        SetEnvironmentVariable('GZ_PARTITION',LaunchConfiguration('partition')),
        OpaqueFunction(function=resource_path),
        ExecuteProcess(cmd=['gz','sim','-s','-r',LaunchConfiguration('world')],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/sim/mid360/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/mid360/livox/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],output='screen'),
        Node(package='obstacle_route_sim',executable='lio_sensor_adapter_node.py',
             parameters=[{'use_sim_time':True, 'noise_profile':LaunchConfiguration('noise_profile'),
                          'noise_seed':LaunchConfiguration('noise_seed')}],output='screen'),
        Node(package='fast_lio',executable='fastlio_mapping',parameters=[config],
             remappings=[('/Odometry','/lio/odometry')],output='screen'),
    ])
