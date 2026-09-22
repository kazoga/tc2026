"""隔離ROS domainで実際のsubscribe/publish経路と初期化ゲートを検証。"""
import os
import subprocess
import sys


def test_transport_waits_for_stationary_window_and_reinitializes():
    script = r'''
import time, math
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from gnss_lio_fusion.gravity_node import GravityNode
rclpy.init()
align=GravityNode();src=Node('alignment_test_source');ex=SingleThreadedExecutor();ex.add_node(align);ex.add_node(src)
p=src.create_publisher(Odometry,'/lio/odometry_raw',10)
w=src.create_publisher(Odometry,'/ypspur_ros/odom',10)
i=src.create_publisher(Imu,'/mid360/livox/imu',10)
received=[];ready=[]
sub=src.create_subscription(Odometry,'/lio/odometry',received.append,10)
subr=src.create_subscription(Bool,'/lio/alignment_ready',lambda m:ready.append(m.data),10)
start=time.monotonic();last=0
while time.monotonic()-start<5.:
 now=time.monotonic();elapsed=now-start
 if now-last>.05:
  stamp=src.get_clock().now().to_msg();odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='raw' if elapsed<4 else 'new_raw';odom.pose.pose.orientation.w=1.
  wheel=Odometry();wheel.header.stamp=stamp
  imu=Imu();imu.header.stamp=stamp;imu.linear_acceleration.x=-math.sin(.4);imu.linear_acceleration.z=math.cos(.4)
  w.publish(wheel);i.publish(imu);p.publish(odom);last=now
 ex.spin_once(timeout_sec=.005)
 if elapsed<1.8:assert not received
 if 3.5<elapsed<3.9:assert ready[-1] and received and received[-1].header.frame_id=='lio_level'
assert ready[-1] is False
assert received
ex.shutdown();src.destroy_node();align.destroy_node();rclpy.shutdown()
'''
    env = dict(os.environ, ROS_DOMAIN_ID='198', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    subprocess.run([sys.executable, '-c', script], env=env, check=True, timeout=20)
