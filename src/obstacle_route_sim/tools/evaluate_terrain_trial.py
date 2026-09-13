#!/usr/bin/env python3
"""Gazebo と既存走行スタックを期限付きで起動して真値軌跡を計測する."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time

import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory
from lio_evaluation_core import evaluate_lio
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from tc_route_msgs.msg import FollowerState, ObstacleAvoidanceHint
from rtk_gps_um982_msgs.msg import RtkStatus
from tf2_msgs.msg import TFMessage


def default_fastlio_config() -> Path:
    """ソース実行と通常 install の両方でパラメータを解決する."""
    local = Path(__file__).resolve().parents[1]/'params/fastlio_gazebo.yaml'
    if local.is_file():
        return local
    return Path(get_package_share_directory('obstacle_route_sim'))/'params/fastlio_gazebo.yaml'


class Monitor(Node):
    """モデル名で真値を選び、理想自己位置を配信して独立計測する."""

    def __init__(self, trial: dict, output: Path, dropout: float, gnss: bool = False) -> None:
        super().__init__('terrain_trial_monitor')
        self.trial = trial
        self.started = time.monotonic()
        self.dropout = dropout
        self.gnss = gnss
        self.gnss_count = 0
        self.rtk_states = {}
        self.lio_samples = []
        self.imu_count = 0
        self.imu_first = None
        self.scan_metadata = None
        self.latest_scan = None
        self.blocked_scan = None
        self.hint_count = 0
        self.blocked_hint_count = 0
        self.create_subscription(Odometry, '/lio/odometry', self.on_lio, 10)
        self.create_subscription(Imu, '/sim/mid360/imu', self.on_imu, qos_profile_sensor_data)
        self.localization_errors = []
        self.counts = dict(pose=0, scan=0, points=0, cmd=0, odom=0)
        self.pose = None
        self.command = (0.0, 0.0)
        self.speed = 0.0
        self.max_speed = 0.0
        self.contact_events = 0
        self.contact_messages = 0
        self.finished = None
        self.finished_sim = None
        self.states = []
        self.samples = []
        self.last_write = 0.0
        self.stream = (output/'trajectory.csv').open('w', newline='')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(['wall_s','sim_s','x','y','z','yaw','cmd_v','cmd_w','odom_v'])
        # GNSS/融合試験では未使用の真値publisher自体を作らない。
        self.publisher = (None if gnss else self.create_publisher(
            PoseWithCovarianceStamped, '/localization/pose_enu', 10))
        self.create_subscription(TFMessage, '/truth', self.on_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/pose_enu',
                                 self.on_localization, 10)
        self.create_subscription(RtkStatus, '/rtk_gps/rtk_status', self.on_rtk, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Odometry, '/ypspur_ros/odom', self.on_odom, 10)
        self.create_subscription(FollowerState, '/follower_state', self.on_state, 10)
        self.create_subscription(Contacts, '/body_contacts', self.on_contacts, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(ObstacleAvoidanceHint, '/obstacle_avoidance_hint',
                                 self.on_hint, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/mid360/livox/lidar/points',
                                 lambda m: self.count('points'), qos_profile_sensor_data)

    def on_scan(self, message: LaserScan) -> None:
        """URG の実受信スキャンを保持し、回避判定時の観測を保存可能にする."""
        self.count('scan')
        self.scan_metadata = dict(frame_id=message.header.frame_id,
            samples=len(message.ranges), angle_min=message.angle_min,
            angle_max=message.angle_max, angle_increment=message.angle_increment,
            range_min=message.range_min, range_max=message.range_max)
        self.latest_scan = dict(**self.scan_metadata,
            sim_s=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
            ranges=[float(r) if math.isfinite(r) else None for r in message.ranges])

    def on_hint(self, message: ObstacleAvoidanceHint) -> None:
        """回避器の出力と、直近に受信したスキャンを対応付けて記録する."""
        self.hint_count += 1
        if message.front_blocked:
            self.blocked_hint_count += 1
            if self.blocked_scan is None and self.latest_scan is not None:
                self.blocked_scan = dict(scan=self.latest_scan,
                    front_clearance_m=float(message.front_clearance_m),
                    hint_sim_s=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                    note='判定受信時の直近スキャン。同一 stamp の厳密同期ではない')

    def on_lio(self, message: Odometry) -> None:
        p=message.pose.pose.position
        q=message.pose.pose.orientation
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        self.lio_samples.append([stamp,p.x,p.y,p.z,q.x,q.y,q.z,q.w])

    def on_imu(self, message: Imu) -> None:
        self.imu_count += 1
        if self.imu_first is None:
            a=message.linear_acceleration
            self.imu_first=[a.x,a.y,a.z]

    def on_rtk(self, message: RtkStatus) -> None:
        key=str(message.rtk_state)
        self.rtk_states[key]=self.rtk_states.get(key,0)+1

    def on_localization(self, message: PoseWithCovarianceStamped) -> None:
        if self.gnss:
            self.gnss_count += 1
            if self.pose:
                p = message.pose.pose.position
                self.localization_errors.append(math.hypot(p.x-self.pose[0], p.y-self.pose[1]))

    def count(self, key: str) -> None:
        self.counts[key] += 1

    def on_contacts(self, message: Contacts) -> None:
        self.contact_messages += 1
        self.contact_events += len(message.contacts)

    def on_cmd(self, message: Twist) -> None:
        self.count('cmd')
        self.command = (message.linear.x, message.angular.z)

    def on_odom(self, message: Odometry) -> None:
        self.count('odom')
        self.speed = message.twist.twist.linear.x
        self.max_speed = max(self.max_speed, abs(self.speed))

    def on_state(self, message: FollowerState) -> None:
        if not self.states or self.states[-1][1] != message.state:
            self.states.append((time.monotonic()-self.started, message.state))
        if message.state == 'FINISHED' and self.finished is None:
            self.finished = time.monotonic()
            self.finished_sim = self.samples[-1][1] if self.samples else None

    def on_pose(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.child_frame_id not in ['icart_mini', 'terrain3d_world::icart_mini']:
                continue
            self.count('pose')
            stamp = transform.header.stamp
            seconds = stamp.sec + stamp.nanosec*1e-9
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = 'map'
            p = transform.transform.translation
            q = transform.transform.rotation
            pose.pose.pose.position.x, pose.pose.pose.position.y = p.x, p.y
            pose.pose.pose.position.z = p.z
            pose.pose.pose.orientation = q
            if not self.gnss and (self.dropout <= 0 or time.monotonic()-self.started < self.dropout):
                self.publisher.publish(pose)
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.pose = (p.x,p.y,p.z,yaw)
            elapsed = time.monotonic()-self.started
            if elapsed-self.last_write >= .1:
                row = (elapsed,seconds,*self.pose,*self.command,self.speed)
                self.writer.writerow(row)
                self.samples.append(row)
                self.last_write = elapsed

    def result(self) -> dict:
        """ラベル到達だけを合格とせず、真値距離と停止も評価する."""
        goal = self.trial['goal']
        error = math.hypot(self.pose[0]-goal[0],self.pose[1]-goal[1]) if self.pose else None
        tail = [r for r in self.samples if r[0] >= time.monotonic()-self.started-2]
        stopped = len(tail) >= 5 and all(abs(r[6])<.01 and abs(r[7])<.01
                                       and abs(r[8])<.02 for r in tail)
        drift = math.hypot(tail[-1][2]-tail[0][2],tail[-1][3]-tail[0][3]) if tail else None
        after_dropout = [r for r in self.samples if self.dropout > 0 and r[0] > self.dropout+2]
        dropout_motion = sum(math.hypot(b[2]-a[2],b[3]-a[3])
                             for a,b in zip(after_dropout,after_dropout[1:]))
        dropout_pass = (self.dropout > 0 and len(after_dropout) >= 20
                        and after_dropout[-1][0]-after_dropout[0][0] >= 2
                        and any(abs(r[8]) > .1 for r in self.samples if r[0] < self.dropout)
                        and dropout_motion < .05 and stopped
                        and all(abs(r[6]) < .01 and abs(r[7]) < .01 for r in after_dropout))
        return dict(dropout_stop_pass=dropout_pass,
                    localization_mode='gnss' if self.gnss else 'truth',
                    gnss_pose_messages=self.gnss_count,
                    localization_xy_rmse_m=(math.sqrt(sum(e*e for e in self.localization_errors)
                        /len(self.localization_errors)) if self.localization_errors else None),
                    counts=self.counts, states=self.states, final_pose=self.pose,
                    goal_error_m=error, max_odom_speed_mps=self.max_speed,
                    stopped_last_2s=stopped, drift_last_2s_m=drift,
                    body_contact_events=self.contact_events,
                    body_contact_messages=self.contact_messages,
                    goal_pass=bool(self.finished and error is not None and error<.7 and stopped
                                   and drift is not None and drift<.05
                                   and all(self.counts.values()) and self.contact_events == 0),
                    dropout_wall_s=self.dropout, elapsed_wall_s=time.monotonic()-self.started,
                    motion_after_dropout_grace_m=dropout_motion if self.dropout > 0 else None,
                    nonzero_cmd_after_dropout_grace=sum(abs(r[6])>.01 or abs(r[7])>.01
                                                       for r in after_dropout),
                    limitations=['GNSS 誤差は未校正の仮定。SLAM・測位品質による切替は未検証' if self.gnss
                                 else '真値自己位置。GNSS/SLAM を検証していない',
                                 '接触監視は車体・支柱・GNSSアンテナ。車輪とキャスタは対象外'])


def evaluate(args: argparse.Namespace) -> int:
    """全プロセスを専用 group で起動し、失敗時も終了処理する."""
    output = args.world.resolve()
    trial = json.loads((output/'trial.json').read_text())
    config = output/'route_config.yaml'
    config.write_text(yaml.safe_dump(dict(blocks=[dict(type='fixed', name='terrain',
                                                      segment_id='route.csv')])))
    projection_file = output/'projection.yaml'
    projection_file.write_text(yaml.safe_dump({'/**': {'ros__parameters':
        dict(trial.get('projection', {}), projection_id='terrain_trial')}}))
    # 実機側の既定 DDS domain と Gazebo partition から隔離する。
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['GZ_PARTITION'] = 'terrain_trial_'+str(os.getpid())
    environment = dict(os.environ)
    environment['GZ_SIM_RESOURCE_PATH'] = str(output)+':'+environment.get('GZ_SIM_RESOURCE_PATH','')
    environment['ROS_LOG_DIR'] = str(output/'ros')
    processes = []
    streams = []

    def start(name: str, command: list[str]) -> None:
        stream = (output/(name+'.log')).open('w')
        streams.append(stream)
        processes.append((name, subprocess.Popen(command, env=environment, stdout=stream,
                                                 stderr=subprocess.STDOUT, start_new_session=True)))

    def node(package: str, executable: str, params: dict, remaps: dict = None) -> None:
        parameter_file = output/(executable+'.yaml')
        parameter_file.write_text(yaml.safe_dump({'/**': {'ros__parameters':params}}))
        command = ['ros2','run',package,executable,'--ros-args','--params-file',str(parameter_file)]
        for source, destination in (remaps or {}).items():
            command += ['-r',source+':='+destination]
        start(executable,command)

    rclpy.init()
    monitor = Monitor(trial,output,args.dropout,args.gnss)
    result = {}
    try:
        start('gazebo',['gz','sim','-s','-r','--physics-engine',args.physics_engine,
                        str(output/'trial.sdf')])
        if args.bridge:
            bridge = args.bridge
        else:
            from ament_index_python.packages import get_package_prefix
            bridge = str(Path(get_package_prefix('ros_gz_bridge'))
                         / 'lib/ros_gz_bridge/parameter_bridge')
        start('bridge',[bridge,
            '/model/icart_mini/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/ypspur_ros/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/body_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/mid360/livox/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            *(['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
               '/sim/mid360/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'] if args.fastlio else []),
            '--ros-args','-r','/model/icart_mini/pose:=/truth'])
        if args.fastlio:
            shutil.copyfile(args.fastlio_config, output/'fastlio_params.yaml')
            if args.record_lio_map:
                start('lio_map_recorder', ['python3',
                    str(Path(__file__).parent/'lio_map_recorder_node.py'),
                    '--output', str(output/'lio_map_scans')])
            start('lio_adapter', ['python3', str(Path(__file__).parent/'lio_sensor_adapter_node.py'),
                '--ros-args', '-p', 'noise_profile:='+args.lio_noise_profile,
                '-p', 'noise_seed:='+str(args.lio_noise_seed),
                '-p', 'noise_log:='+str(output/'sensor_noise.json')])
            start('fastlio', [str(args.fastlio.resolve()), '--ros-args',
                '--params-file',str(output/'fastlio_params.yaml'),
                '-r','/Odometry:=/lio/odometry'])
            # 落下・姿勢整定と静止 IMU 初期化を待ち、走行開始前から記録する。
            warmup = time.monotonic()+8.
            warmup_deadline=warmup+30.
            while time.monotonic()<warmup or len(monitor.lio_samples)<10:
                rclpy.spin_once(monitor,timeout_sec=.05)
                if time.monotonic()>warmup_deadline:
                    raise RuntimeError('FAST-LIO の静止初期化が期限内に完了しない')
        if args.gnss:
            origin = trial.get('projection', dict(origin_latitude=36.082628231,
                origin_longitude=140.076144739, origin_altitude=67.078))
            node('obstacle_route_sim', 'gnss_simulator_node.py',
                 dict(**origin, dropout_sec=float(args.dropout), start_wall_sec=monitor.started,
                      horizontal_sigma_m=args.gnss_sigma,
                      buildings_json=str(output/"world.json") if args.building_gnss else "",
                      float_enter_m=args.float_enter, float_exit_m=args.float_enter+3.,
                      heading_reference='antenna_baseline' if args.fusion else 'vehicle_forward',
                      baseline_sigma_m=.002 if args.fusion else 0.,
                      baseline_float_sigma_m=.04 if args.fusion else 0.,
                      baseline_shift_m=args.baseline_shift,
                      baseline_shift_after_s=args.baseline_shift_after,
                      heading_fault_deg=args.heading_fault, heading_fault_after_s=args.heading_fault_after,
                      heading_fault_duration_s=args.heading_fault_duration))
            node('geo_pose_converter', 'geo_pose_converter_node', origin,
                 {'gnss/pose_enu': '/gnss/pose_enu' if args.fusion else '/localization/pose_enu'})
            if args.fusion:
                node('gnss_lio_fusion', 'fusion_node',
                     dict(**origin, use_sim_time=True, gnss_heading_offset_deg=180.,
                          output_log=str(output/'fusion.jsonl')))
        if args.contact_probe:
            probe = monitor.create_publisher(Twist,'/cmd_vel',10)
            def probe_tick() -> None:
                command = Twist()
                command.linear.x = .3 if not monitor.contact_events else 0.0
                probe.publish(command)
            monitor.create_timer(.05,probe_tick)
        else:
            node('route_planner','route_planner',
                 dict(config_yaml_path=str(config),csv_base_dir=str(output),
                      projection_config_path=str(projection_file)))
            node('route_manager','route_manager',
                 dict(start_label='0',goal_label=str(len(trial['points'])-1),checkpoint_labels=['']))
            node('route_follower','route_follower',dict(start_immediately=True))
            node('obstacle_monitor','obstacle_monitor',{})
            node('robot_navigator','robot_navigator',dict(log_csv_path=str(output/'control.csv')),
                 dict(odom='/ypspur_ros/odom',cmd_vel='/cmd_vel/autonomous'))
            node('drive_mode_manager','drive_cmd_mux_node',dict(initial_mode='autonomous'),
                 {'cmd_vel/autonomous':'/cmd_vel/fusion_limited'} if args.fusion else None)
        deadline = time.monotonic()+args.timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(monitor,timeout_sec=.05)
            failures = [(name,p.returncode) for name,p in processes if p.poll() is not None]
            if failures:
                result['process_failures'] = failures
                break
            if monitor.finished and time.monotonic()-monitor.finished >= 4:
                break
            if args.contact_probe and monitor.contact_events > 0:
                break
        result.update(monitor.result())
        if args.fusion:
            result['localization_mode'] = 'gnss_lio_fusion'
        result["rtk_state_counts"] = monitor.rtk_states
        result['lio_messages'] = len(monitor.lio_samples)
        result['imu_messages'] = monitor.imu_count
        result['imu_first_acceleration'] = monitor.imu_first
        result['urg_scan'] = monitor.scan_metadata
        result['obstacle_hint_messages'] = monitor.hint_count
        result['front_blocked_hint_messages'] = monitor.blocked_hint_count
        if monitor.blocked_scan is not None:
            (output/'urg_blocked_scan.json').write_text(
                json.dumps(monitor.blocked_scan, ensure_ascii=False, indent=2))
        if args.fastlio:
            with (output/'lio_trajectory.csv').open('w') as stream:
                writer=csv.writer(stream)
                writer.writerow(['sim_s','x','y','z','qx','qy','qz','qw'])
                writer.writerows(monitor.lio_samples)
            truth_array=np.array([[r[1],r[2],r[3],r[5]] for r in monitor.samples])
            quality,_=evaluate_lio(truth_array,np.array(monitor.lio_samples))
            result['lio_quality']=quality
        result['contact_probe_pass'] = monitor.contact_events > 0 if args.contact_probe else None
    finally:
        # 終了指令を仮想 bridge に送ってから起動した group のみを終了する。
        stop = monitor.create_publisher(Twist,'/cmd_vel',10)
        stop.publish(Twist())
        for _, process in reversed(processes):
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for _, process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        result['processes_reaped'] = all(p.poll() is not None for _,p in processes)
        monitor.stream.close()
        monitor.destroy_node()
        rclpy.shutdown()
        for stream in streams:
            stream.close()
        (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    return 0 if (not args.fastlio or result.get('lio_quality',{}).get('pass_lio',False)) and not result.get('process_failures') and (result.get('goal_pass')
        or result.get('contact_probe_pass') or result.get('dropout_stop_pass')) else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fastlio',type=Path,help='FAST-LIO 実行ファイル。指定時に IMU/clock を bridge')
    parser.add_argument('--lio-noise-profile', choices=['reference','field_assumed','conservative'],
                        default='field_assumed')
    parser.add_argument('--lio-noise-seed', type=int, default=1)
    parser.add_argument('--record-lio-map', action='store_true',
                        help='FAST-LIO 登録済み点群を 1 Hz で分割保存する')
    parser.add_argument('--fastlio-config', type=Path, default=default_fastlio_config())
    parser.add_argument('--physics-engine', default='gz-physics-dartsim-plugin')
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=90)
    parser.add_argument('--dropout', type=float, default=0)
    parser.add_argument('--gnss', action='store_true')
    parser.add_argument('--fusion', action='store_true', help='GNSS/LIO融合位置で閉ループ走行する')
    parser.add_argument('--heading-fault', type=float, default=0.)
    parser.add_argument('--heading-fault-after', type=float, default=20.)
    parser.add_argument('--heading-fault-duration', type=float, default=0.)
    parser.add_argument('--baseline-shift', type=float, default=0.)
    parser.add_argument('--baseline-shift-after', type=float, default=0.)
    parser.add_argument('--building-gnss',action='store_true')
    parser.add_argument('--float-enter',type=float,default=12.)
    parser.add_argument('--gnss-sigma', type=float, default=.02)
    parser.add_argument('--bridge')
    parser.add_argument('--domain-id', type=int, choices=range(1,233), default=86)
    parser.add_argument('--contact-probe',action='store_true',
                        help='走行スタックを起動せず仮想障害物へ接近して接触監視を陽性確認する')
    arguments = parser.parse_args()
    if arguments.record_lio_map and not arguments.fastlio:
        parser.error('--record-lio-map には --fastlio が必要')
    if arguments.fusion and not (arguments.fastlio and arguments.gnss):
        parser.error('--fusion には --fastlio と --gnss が必要')
    raise SystemExit(evaluate(arguments))
