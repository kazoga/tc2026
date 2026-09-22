"""FAST-LIO raw worldを水平化。IMU/body座標は変更しない。"""
from collections import deque
import copy
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from .gravity_core import GravityAlignment
from .mount_core import quaternion_rotation


def seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def quaternion(matrix):
    # Symmetric eigenproblem is stable at 180 degrees too. ROS order x,y,z,w.
    r = matrix
    k = np.array([
        [r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
        [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
        [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
        [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]]) / 3.
    q = np.linalg.eigh(k)[1][:, -1]
    return q if q[3] >= 0 else -q


def transform_odometry(msg, matrix):
    out = copy.deepcopy(msg)
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    pos = matrix @ np.array([p.x, p.y, p.z])
    quat = quaternion(matrix @ quaternion_rotation([q.x, q.y, q.z, q.w]))
    out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, pos)
    out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = map(float, quat)
    jac = np.zeros((6, 6)); jac[:3, :3] = jac[3:, 3:] = matrix
    cov = np.array(msg.pose.covariance).reshape(6, 6)
    out.pose.covariance = (jac @ cov @ jac.T).ravel().tolist()
    out.header.frame_id = 'lio_level'
    # twist and cloud_registered_body remain in the IMU child frame.
    return out


class GravityNode(Node):
    def __init__(self):
        super().__init__('lio_gravity_alignment')
        self.core = GravityAlignment(self.declare_parameter('stationary_duration_s', 2.).value)
        self.imus = deque(maxlen=400)
        self.wheels = deque(maxlen=100)
        self.last_receive = time.monotonic()
        self.publisher_gid = None
        self.pub = self.create_publisher(Odometry, '/lio/odometry', 30)
        self.ready_pub = self.create_publisher(Bool, '/lio/alignment_ready', 10)
        self.status_pub = self.create_publisher(String, '/lio/alignment_status', 10)
        self.create_subscription(Imu, '/mid360/livox/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ypspur_ros/odom', self.on_wheel, 30)
        self.create_subscription(Odometry, '/lio/odometry_raw', self.on_lio, 30)
        self.create_timer(.2, self.status, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_imu(self, msg):
        self.imus.append(msg)

    def on_wheel(self, msg):
        self.wheels.append(msg)

    def status(self):
        if time.monotonic()-self.last_receive > 1.5:
            self.core.reset()
        ready = self.core.matrix is not None
        self.ready_pub.publish(Bool(data=ready))
        self.status_pub.publish(String(data=json.dumps(dict(ready=ready, reason=self.core.reason,
            rotation=self.core.matrix.tolist() if ready else None), ensure_ascii=False)))

    def on_lio(self, msg, info=None):
        if info is not None:
            gid = bytes(info.publisher_gid)
            if self.publisher_gid is not None and gid != self.publisher_gid:
                self.core.reset()
            self.publisher_gid = gid
        self.last_receive = time.monotonic()
        t = seconds(msg)
        if not self.imus or not self.wheels:
            return
        imu = min(self.imus, key=lambda m: abs(seconds(m)-t))
        wheel = min(self.wheels, key=lambda m: abs(seconds(m)-t))
        if abs(seconds(imu)-t) > .05 or abs(seconds(wheel)-t) > .1:
            # Never bridge a missing initialization observation window.
            self.core.samples.clear()
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        try:
            matrix = self.core.observe(t, msg.header.frame_id, [p.x, p.y, p.z],
                [q.x, q.y, q.z, q.w],
                [getattr(imu.linear_acceleration, k) for k in 'xyz'],
                [getattr(imu.angular_velocity, k) for k in 'xyz'],
                [wheel.twist.twist.linear.x, wheel.twist.twist.angular.z])
        except ValueError:
            self.core.reset()
            matrix = None
        self.status()
        if matrix is not None:
            self.pub.publish(transform_odometry(msg, matrix))


def main(args=None):
    rclpy.init(args=args)
    node = GravityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
