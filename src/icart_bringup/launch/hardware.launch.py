"""実機ドライバのみを起動する。FAST-LIO・mux・手動操作は共通bringupが所有する。"""
from pathlib import Path
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from icart_bringup.hardware_core import read_yaml, geometry, validate_runtime


def setup(context):
    filename = Path(LaunchConfiguration('hardware_config').perform(context))
    config = read_yaml(filename)
    validate_runtime(config)
    directory = filename.parent
    geo = geometry(config)
    share = Path(get_package_share_directory('ypspur_ros2'))
    wheel = config['wheel']
    coordinator = ExecuteProcess(cmd=['ypspur-coordinator', '-d', wheel['serial_port'],
                                     '-p', str(Path(wheel['coordinator_param']).expanduser())], output='screen')
    wheel_node = Node(package='ypspur_ros2', executable='ypspur_node', name='ypspur_node',
             parameters=[str(share/'config/default.yaml')],
             remappings=[('cmd_vel', '/cmd_vel'), ('odom', '/ypspur_ros/odom')], output='screen')
    actions = [coordinator, TimerAction(period=2., actions=[wheel_node]),
        Node(package='urg_node', executable='urg_node_driver', name='urg_node',
             parameters=[{'serial_port': config['urg']['serial_port'],
                          'serial_baud': config['urg']['baud'], 'laser_frame_id': 'laser',
                          'ip_address': '', 'angle_min': config['urg']['angle_min'],
                          'angle_max': config['urg']['angle_max']}], output='screen'),
        Node(package='rtk_gps_um982', executable='rtk_gps_um982_node', name='rtk_gps_um982_node',
             namespace='rtk_gps', parameters=[str(directory/'um982.yaml')], output='screen'),
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node', namespace='mid360',
             parameters=[{'xfer_format': 1, 'multi_topic': 0, 'data_src': 0, 'publish_freq': 10.,
                          'output_data_type': 0, 'frame_id': 'mid360_frame',
                          'user_config_path': str(directory/'livox.json')}], output='screen')]
    if config['camera']['enabled']:
        camera = config['camera']
        actions.append(Node(package='usb_cam', executable='usb_cam_node_exe', name='usb_cam', namespace='usb_cam',
            parameters=[{'video_device': camera['device'], 'image_width': camera['width'],
                         'image_height': camera['height'], 'framerate': camera['framerate'],
                         'pixel_format': camera['pixel_format'], 'camera_frame_id': 'camera_optical_frame'}],
            remappings=[('image_raw', '/usb_cam/image_raw')], output='screen'))
    for frame, xyz, angles in [
            ('mid360_frame', geo['lidar'], geo['rpy_deg']),
            ('mid360_imu', geo['imu'], geo['rpy_deg']),
            ('gnss_main', geo['master'], [0., 0., 0.]),
            ('gnss_sub', geo['slave'], [0., 0., 0.]),
            ('laser', config['urg']['xyz'], config['urg']['rpy_deg'])]:
        args = ['--frame-id', 'base_link', '--child-frame-id', frame]
        for key, value in zip(['x', 'y', 'z'], xyz):
            args += ['--'+key, str(value)]
        for key, value in zip(['roll', 'pitch', 'yaw'], angles):
            args += ['--'+key, str(math.radians(value))]
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name='base_to_'+frame, arguments=args))
    actions += [RegisterEventHandler(OnProcessExit(target_action=action,
                 on_exit=[EmitEvent(event=Shutdown(reason='実機ドライバ終了'))]))
                for action in [*list(actions), wheel_node] if isinstance(action, ExecuteProcess)]
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('hardware_config'), OpaqueFunction(function=setup)])
