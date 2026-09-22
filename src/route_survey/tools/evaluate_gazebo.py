#!/usr/bin/env python3
"""既存デジタルツインをJoyで操縦し、採取ノードのROS接続を期限付き評価する."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


def evaluate(session: Path, output: Path, timeout: float) -> None:
    import yaml
    output.mkdir(parents=True, exist_ok=False)
    s=yaml.safe_load(session.read_text())
    projection=(session.parent/s['projection_params']).resolve()
    children=[];logs=[]
    os.environ['ROS_DOMAIN_ID']='86'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    def launch(args,name):
        log=(output/(name+'.log')).open('w');logs.append(log)
        child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child)
    launch(['ros2','launch','icart_bringup','bringup.launch.py','environment:=simulation',
            'session:='+str(session.resolve()),'initial_drive_mode:=manual'],'world')
    launch(['ros2','run','drive_mode_manager','manual_teleop_node'],'teleop')
    launch(['ros2','run','route_survey','recorder','--ros-args','-p','use_sim_time:=true',
            '-p','output_directory:='+str((output/'survey').resolve()),
            '-p','projection_config:='+str(projection)],'recorder')
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=Node('survey_gazebo_test');pub=node.create_publisher(Joy,'/joy',10)
    observed={'pose':0,'cloud':0,'status':{}}
    def pose(m):observed['pose']+=1
    def cloud(m):observed['cloud']+=1
    def status(m):observed['status']=json.loads(m.data)
    node.create_subscription(PoseWithCovarianceStamped,'/localization/pose_enu',pose,10)
    node.create_subscription(PointCloud2,'/cloud_registered_body',cloud,10)
    node.create_subscription(String,'/route_survey/status',status,10)
    wall=time.monotonic();ready=None;drive=None
    try:
        while time.monotonic()-wall<timeout:
            rclpy.spin_once(node,timeout_sec=.02)
            now=node.get_clock().now().nanoseconds*1e-9
            if observed['pose'] and ready is None:ready=now
            m=Joy();m.axes=[0.]*6;m.buttons=[0]*17
            m.header.stamp=node.get_clock().now().to_msg()
            if ready is not None:
                m.buttons[4]=1
                if now-ready<3.5:m.buttons[16]=1
                else:
                    if drive is None:drive=now
                    t=now-drive
                    if t<1.:m.buttons[0]=1
                    if 1.<t<35.:
                        m.axes[1]=.4
                        if 15<t<20:m.axes[0]=.2
                    if 10<t<11:m.buttons[1]=1
                    if 25<t<26:m.buttons[2]=1
                    if 35<t<36:m.buttons[3]=1
                    if t>38:break
            pub.publish(m)
            time.sleep(.03)
        pub.publish(Joy(axes=[0.]*6,buttons=[0]*17))
        observed['elapsed_wall_s']=time.monotonic()-wall
    finally:
        for child in reversed(children):
            if child.poll() is None:child.send_signal(signal.SIGINT)
        for child in reversed(children):
            try:child.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=4)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
        node.destroy_node();rclpy.try_shutdown()
    saved=output/'survey/survey.json'
    if saved.exists():
        data=json.loads(saved.read_text());rows=data['waypoints']
        observed.update(waypoints=len(rows),line_stops=sum(r['line_is_stop'] for r in rows),
                        signal_stops=sum(r['signal_is_stop'] for r in rows),
                        reasons=[r['reason'] for r in rows],map_points=len(data['cloud']))
        observed['pass']=bool(len(rows)>=3 and observed['line_stops'] and
                              observed['signal_stops'] and not data['active'] and observed['cloud'])
    else:observed['pass']=False
    (output/'result.json').write_text(json.dumps(observed,indent=2))
    print(json.dumps(observed,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--session',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--timeout',type=float,default=360)
    a=p.parse_args();evaluate(a.session,a.output,a.timeout)
