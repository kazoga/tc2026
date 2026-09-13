#!/usr/bin/env python3
"""Gazebo センサを整形し、選択した誤差モデルを加えて LIO 入力を配信する."""
import math
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from lio_noise_core import SensorNoise


def finite_xyzi(points: np.ndarray, minimum: float, maximum: float) -> np.ndarray:
    """未反射点の Inf/NaN を除外し、センサ有効範囲だけ残す."""
    values=np.column_stack([points[name].reshape(-1) for name in ['x','y','z','intensity']])
    ranges=np.linalg.norm(values[:,:3],axis=1)
    keep=np.isfinite(values).all(axis=1)&(ranges>=minimum)&(ranges<=maximum)
    return values[keep].astype(np.float32)


class LioSensorAdapter(Node):
    """LiDAR 時刻を保持し、IMU 時刻差はモデルで与える。点内時刻は追加しない."""
    def __init__(self) -> None:
        super().__init__('lio_sensor_adapter')
        self.settle=float(self.declare_parameter('settle_sec',3.).value)
        self.maximum=float(self.declare_parameter('max_range_m',70.).value)
        profile = str(self.declare_parameter('noise_profile', 'field_assumed').value)
        seed = int(self.declare_parameter('noise_seed', 1).value)
        self.noise = SensorNoise(profile, seed)
        self.noise_log = str(self.declare_parameter('noise_log', '').value)
        self.noise_identity = dict(profile=profile, seed=seed)
        self.get_logger().info(f'センサ誤差モデル: {profile}, seed={seed}')
        self.first=None
        self.cloud_pub=self.create_publisher(PointCloud2,'/sim/lio/points',10)
        self.imu_pub=self.create_publisher(Imu,'/sim/lio/imu',100)
        self.create_subscription(PointCloud2,'/mid360/livox/lidar/points',self.cloud,qos_profile_sensor_data)
        self.create_subscription(Imu,'/sim/mid360/imu',self.imu,qos_profile_sensor_data)
        self.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                     for i,name in enumerate(['x','y','z','intensity'])]
        self.count=0

    def ready(self,stamp) -> bool:
        now=stamp.sec+stamp.nanosec*1e-9
        if self.first is None or now<self.first:self.first=now
        return now-self.first>=self.settle

    def imu(self,message: Imu) -> None:
        if not self.ready(message.header.stamp):
            return
        if self.noise_identity['profile'] == 'reference':
            self.imu_pub.publish(message)
            return
        a, g = message.linear_acceleration, message.angular_velocity
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        accel, gyro, offset = self.noise.imu(np.array([a.x,a.y,a.z]),
                                            np.array([g.x,g.y,g.z]), stamp)
        a.x, a.y, a.z = map(float, accel)
        g.x, g.y, g.z = map(float, gyro)
        for i in [0,4,8]:
            message.linear_acceleration_covariance[i] += self.noise.profile.accel_sigma**2
            message.angular_velocity_covariance[i] += self.noise.profile.gyro_sigma**2
        ns = round((stamp+offset)*1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 1000000000)
        self.imu_pub.publish(message)

    def cloud(self,message: PointCloud2) -> None:
        if not self.ready(message.header.stamp):return
        points=point_cloud2.read_points(message,field_names=['x','y','z','intensity'])
        valid=finite_xyzi(points,.6,self.maximum)
        valid = self.noise.cloud(valid)
        output=point_cloud2.create_cloud(message.header,self.fields,valid)
        self.cloud_pub.publish(output)
        if self.count==0:self.get_logger().info(f'LIO 有効点群: {len(valid)} 点。LiDAR 時刻を保持する')
        self.count+=1
        if self.noise_log and self.count % 50 == 1:
            Path(self.noise_log).write_text(json.dumps(
                dict(**self.noise_identity, **self.noise.metadata()), indent=2))


def main() -> None:
    """launch の停止通知と ROS context の先行終了を扱う."""
    rclpy.init()
    node = LioSensorAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.noise_log:
            Path(node.noise_log).write_text(json.dumps(
                dict(**node.noise_identity, **node.noise.metadata()), indent=2))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
