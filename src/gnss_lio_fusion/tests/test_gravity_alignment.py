import math
from collections import deque
from types import SimpleNamespace
import numpy as np
import pytest
from gnss_lio_fusion.gravity_core import GravityAlignment, level_rotation
from gnss_lio_fusion.mount_core import rotation, quaternion_rotation, base_from_sensor
from gnss_lio_fusion.gravity_node import quaternion, transform_odometry
from nav_msgs.msg import Odometry


def test_fixed_alignment_recovers_3d_base_for_all_headings():
    tilt = rotation(.2, -.35, 0)
    mount = rotation(-.01, .47, 0)
    core = GravityAlignment()
    for t in np.arange(0, 2.2, .1):
        core.observe(t, 'raw', [0,0,0], quaternion(tilt.T @ mount),
                     mount.T @ [0,0,1], [0,0,0], [0,0])
    assert core.matrix @ tilt.T @ [0,0,1] == pytest.approx([0,0,1], abs=1e-9)
    fixed = core.matrix.copy()
    lever = np.array([-.01,.02,.37])
    for i,yaw in enumerate(np.linspace(-math.pi,math.pi,30)):
        base = rotation(0,0,yaw); pos=np.array([3.,4.,.2])
        msg=Odometry();msg.header.frame_id='raw';msg.header.stamp.sec=4+i
        rp=tilt.T @ (pos+base@lever);q=quaternion(tilt.T@base@mount)
        msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=map(float,rp)
        msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w=map(float,q)
        # Moving data cannot update the established vertical direction.
        core.observe(2.2+i*.1,'raw',rp,q,[1,0,0],[1,2,3],[1,1])
        assert core.matrix == pytest.approx(fixed)
        out=transform_odometry(msg,fixed)
        p=out.pose.pose.position;q=out.pose.pose.orientation
        got,rpy=base_from_sensor([p.x,p.y,p.z],[q.x,q.y,q.z,q.w],lever,[-.01,.47,0])
        # Minimum rotation can retain a constant yaw, so check tilt and relative distances.
        assert rpy[:2] == pytest.approx([0,0],abs=1e-8)
        assert np.linalg.norm(got[:2]) == pytest.approx(5.,abs=1e-8)
        assert got[2] == pytest.approx(.2,abs=1e-8)
        assert out.header.stamp==msg.header.stamp and out.child_frame_id==msg.child_frame_id


def test_moving_or_discontinuous_data_cannot_initialize_and_reset_waits():
    core=GravityAlignment()
    for t in np.arange(0,4,.1):
        assert core.observe(t,'raw',[t,0,0],[0,0,0,1],[0,0,9.81],[0,0,0],[.4,0]) is None
    for t in np.arange(4,6.2,.1):core.observe(t,'raw',[4,0,0],[0,0,0,1],[0,0,9.81],[0,0,0],[0,0])
    assert core.matrix is not None
    assert core.observe(1.,'raw',[0,0,0],[0,0,0,1],[0,0,1],[0,0,0],[0,0]) is None
    assert core.observe(4.,'new',[0,0,0],[0,0,0,1],[0,0,1],[0,0,0],[0,0]) is None


def test_covariance_and_quaternion_rotation():
    msg=Odometry();msg.pose.pose.orientation.w=1.
    msg.pose.covariance=np.diag([1.,2.,3.,4.,5.,6.]).ravel().tolist()
    r=rotation(.2,.4,math.pi);out=transform_odometry(msg,r);q=out.pose.pose.orientation
    assert quaternion_rotation([q.x,q.y,q.z,q.w])==pytest.approx(r)
    assert np.linalg.eigvalsh(np.array(out.pose.covariance).reshape(6,6))==pytest.approx([1,2,3,4,5,6])


def test_alignment_lease_failure_clears_filter_and_stops_commands():
    from gnss_lio_fusion.fusion_node import FusionNode
    from gnss_lio_fusion.fusion_core import FusionFilter
    from gnss_lio_fusion.motion_guard_core import MotionGuard
    from geometry_msgs.msg import Twist
    import time
    published=[]
    n=SimpleNamespace(values={'require_gravity_alignment':True}, alignment_ready=True,
        alignment_received=time.monotonic()-2, filter=FusionFilter(),motion=MotionGuard(),
        lios=deque([1]),wheels=deque([1]),events=[1],statuses={1:1},
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000)),
        cmd_pub=SimpleNamespace(publish=published.append))
    n.filter.x=np.ones(3)
    n.reset_alignment_state=lambda: FusionNode.reset_alignment_state(n)
    n.alignment_available=lambda: FusionNode.alignment_available(n)
    cmd=Twist();cmd.linear.x=.5;cmd.angular.z=.5
    FusionNode.on_command(n,cmd)
    assert published[-1].linear.x==published[-1].angular.z==0
    assert n.filter.x is None and not n.lios and not n.events and not n.alignment_ready


def test_stationary_imu_noise_uses_window_dispersion_but_rejects_vibration():
    quiet=GravityAlignment();vibrating=GravityAlignment()
    for i,t in enumerate(np.arange(0,2.2,.1)):
        a=np.array([.04 if i==10 else .005*(-1)**i,0,1.])
        quiet.observe(t,'raw',[0,0,0],[0,0,0,1],a,[0,0,0],[0,0])
        vibrating.observe(t,'raw',[0,0,0],[0,0,0,1],[.08*(-1)**i,0,1],[0,0,0],[0,0])
    assert quiet.matrix is not None
    assert vibrating.matrix is None
