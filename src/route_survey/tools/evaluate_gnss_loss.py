#!/usr/bin/env python3
"""実融合ノードと採取ノードへ模擬GNSS/LIO観測を配信して劣化・欠落を確認する."""
import argparse
import json
import math
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix, Joy
from nav_msgs.msg import Odometry
from rtk_gps_um982_msgs.msg import RtkStatus
from geo_pose_converter.geo_core import (EnuPoint, ProjectionConfig, enu_to_llh,
                                         load_projection_config_from_yaml)
from gnss_lio_fusion.fusion_node import FusionNode
from route_survey.recorder_node import Recorder


def evaluate(output: Path, projection_file: Path) -> None:
    projection=load_projection_config_from_yaml(str(projection_file))
    args=['--ros-args','-p','use_sim_time:=true','-p','buffer_s:=0.1',
          '-p','output_directory:='+str(output.resolve()),
          '-p','projection_config:='+str(projection_file.resolve())]
    for k in ['origin_latitude','origin_longitude','origin_altitude','map_yaw_offset_rad']:
        args += ['-p', k+':='+str(getattr(projection,k))]
    rclpy.init(args=args)
    fusion=FusionNode();recorder=Recorder();source=Node('survey_gnss_loss_test')
    clock=source.create_publisher(Clock,'/clock',10)
    lio=source.create_publisher(Odometry,'/lio/odometry',10)
    wheel=source.create_publisher(Odometry,'/ypspur_ros/odom',10)
    fix=source.create_publisher(NavSatFix,'/rtk_gps/fix',10)
    status=source.create_publisher(RtkStatus,'/rtk_gps/rtk_status',10)
    joy=source.create_publisher(Joy,'/joy',10)
    executor=rclpy.executors.SingleThreadedExecutor()
    for n in [fusion,recorder,source]:executor.add_node(n)
    try:
        for _ in range(30):executor.spin_once(timeout_sec=.01)
        for i in range(1,421):
            t=i*.1;c=Clock();c.clock.sec=int(t);c.clock.nanosec=int(round((t-int(t))*1e9));clock.publish(c)
            od=Odometry();od.header.stamp=c.clock;od.pose.pose.position.x=.505*t
            od.pose.pose.position.y=.003*t;od.pose.pose.position.z=.6;od.pose.pose.orientation.w=1.
            lio.publish(od);od.pose.pose.position.z=0.;wheel.publish(od)
            if not 15<=t<=25:
                floating=8<=t<15
                q=RtkStatus();q.header.stamp=c.clock;q.rtk_state=3 if floating else 4
                q.num_satellites=8 if floating else 20;q.hdop=2.5 if floating else .7
                q.baseline_length_m=.54 if floating else .5;q.correction_age_s=4. if floating else .2
                q.heading_deg=90.;q.heading_stddev_deg=5. if floating else .5
                p=enu_to_llh(EnuPoint(.5*t+(.6*math.sin(t) if floating else 0.),
                                     1.5 if floating else 0.,.7),projection)
                g=NavSatFix();g.status.status=0;g.header.stamp=c.clock;g.latitude=p.latitude;g.longitude=p.longitude;g.altitude=p.altitude
                g.position_covariance[0]=g.position_covariance[4]=1. if floating else .0004
                status.publish(q);fix.publish(g)
            j=Joy();j.header.stamp=c.clock;j.buttons=[0]*17;j.axes=[0.]*6
            for begin,button in [(3.,0),(12.,1),(23.,2),(40.,3)]:
                if begin<=t<begin+.3:j.buttons[button]=1
            joy.publish(j)
            for _ in range(24):executor.spin_once(timeout_sec=.001)
        recorder.flush()
    finally:
        executor.shutdown()
        for n in [fusion,recorder,source]:n.destroy_node()
        rclpy.try_shutdown()
    data=json.loads((output/'survey.json').read_text());traces=data['traces']
    gap=[p for p in traces['fused'] if 17<p['stamp']<24]
    rows=recorder.survey.rows
    during=[p for p in rows if 17<p['stamp']<25]
    result=dict(gnss_samples=len(traces['gnss']), fused_samples=len(traces['fused']),
                gnss_states=sorted({p['quality']['state'] for p in traces['gnss'] if p['quality']}),
                fusion_modes=sorted({p['mode'] for p in traces['fused']}),
                samples_during_gnss_loss=len(gap),waypoints_during_gnss_loss=len(during),
                total_waypoints=len(rows), signal_stop_during_loss=any(p['signal_is_stop'] for p in during),
                limitations='GNSS/LIOは模擬観測。融合・採取は実ROSノード。Gazebo/FAST-LIO本体の新規評価ではない。')
    result['pass']=bool(len(gap)>10 and during and result['signal_stop_during_loss'] and
                         3 in result['gnss_states'] and 4 in result['gnss_states'] and
                         not any(15<=p['stamp']<=25 for p in traces['gnss']) and
                         all(p['x'] is not None for p in traces['gnss']))
    (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2))
    if not result['pass']:raise RuntimeError('GNSS劣化採取試験が不合格')


if __name__=='__main__':
    os.environ['ROS_DOMAIN_ID']='87';os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--projection',type=Path,required=True);a=p.parse_args();evaluate(a.output,a.projection)
