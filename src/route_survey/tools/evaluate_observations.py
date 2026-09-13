#!/usr/bin/env python3
"""段差・欠測・Joyを含む模擬ROS観測で記録と保存を検証する（Gazeboとは別検査）."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from route_survey.recorder_node import Recorder


def evaluate(output: Path, projection: Path) -> None:
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p',
                    'output_directory:='+str(output.resolve()),'-p',
                    'projection_config:='+str(projection.resolve())])
    recorder=Recorder();sim=Node('survey_observation_simulator')
    pubs={
        'clock':sim.create_publisher(Clock,'/clock',10),
        'pose':sim.create_publisher(PoseWithCovarianceStamped,'/localization/pose_enu',10),
        'odom':sim.create_publisher(Odometry,'/lio/odometry',10),
        'joy':sim.create_publisher(Joy,'/joy',10)}
    from sensor_msgs.msg import PointCloud2
    pubs['cloud']=sim.create_publisher(PointCloud2,'/cloud_registered_body',10)
    executor=rclpy.executors.SingleThreadedExecutor();executor.add_node(recorder);executor.add_node(sim)
    rng=np.random.default_rng(4)
    x,y=np.meshgrid(np.arange(-.59,.6,.04),np.arange(-2.99,3,.04))
    z=np.where(y>=1.,.12,np.where(y<=-1.,-.12,0.))-.6
    points=np.column_stack([x.ravel(),y.ravel(),z.ravel()])
    try:
        for i in range(220):
            t=i*.1+1;sec=int(t);nano=int(round((t-sec)*1e9))
            header=Header();header.stamp.sec=sec;header.stamp.nanosec=nano
            pubs['clock'].publish(Clock(clock=header.stamp))
            for _ in range(3):executor.spin_once(timeout_sec=.003)
            pose=PoseWithCovarianceStamped();pose.header=header;pose.header.frame_id='map'
            d=max(0,i-10)*.1
            pose.pose.pose.position.x=min(d,8.);pose.pose.pose.position.y=max(0.,d-8.)
            yaw=0 if d<=8 else math.pi/2
            pose.pose.pose.orientation.z=math.sin(yaw/2);pose.pose.pose.orientation.w=math.cos(yaw/2)
            pose.pose.covariance[0]=pose.pose.covariance[7]=.01
            odom=Odometry();odom.header=header;odom.pose=pose.pose
            pubs['pose'].publish(pose);pubs['odom'].publish(odom)
            for _ in range(4):executor.spin_once(timeout_sec=.003)
            if i%2==0:
                cloud=points+rng.normal(0,.003,points.shape)
                # 後半は左0.5 m以遠が見えない。欠測を空きと誤認しないことを確認する。
                if i>140:cloud=cloud[cloud[:,1]<.5]
                h=Header(stamp=header.stamp,frame_id='body')
                pubs['cloud'].publish(point_cloud2.create_cloud_xyz32(h,cloud.astype(np.float32)))
            joy=Joy();joy.header=header;joy.buttons=[0]*17;joy.axes=[0.]*6
            for at,button in [(10,0),(75,1),(180,2),(215,3)]:
                if at<=i<at+3:joy.buttons[button]=1
            pubs['joy'].publish(joy)
            for _ in range(6):executor.spin_once(timeout_sec=.003)
        recorder.flush()
    finally:
        executor.shutdown();recorder.destroy_node();sim.destroy_node();rclpy.try_shutdown()
    data=json.loads((output/'survey.json').read_text());rows=data['waypoints']
    result=dict(waypoints=len(rows),reasons=[r['reason'] for r in rows],
                line_stops=sum(r['line_is_stop'] for r in rows),
                signal_stops=sum(r['signal_is_stop'] for r in rows),
                max_left_m=max(r['left_is_open'] for r in rows),
                max_right_m=max(r['right_is_open'] for r in rows),cloud_points=len(data['cloud']))
    result['pass']=bool(result['line_stops']==1 and result['signal_stops']==1 and
                        'turn' in result['reasons'] and 'distance' in result['reasons'] and
                        result['max_left_m']<=.726 and result['max_right_m']<=.726 and
                        result['cloud_points']>0 and not data['active'])
    (output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--projection',type=Path,required=True);a=p.parse_args();evaluate(a.output,a.projection)
