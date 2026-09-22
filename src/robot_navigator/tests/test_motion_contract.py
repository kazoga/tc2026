from pathlib import Path
import math
import sys
from types import SimpleNamespace as NS
import pytest
pytest.importorskip('rclpy')
sys.path.insert(0, str(Path(__file__).parents[1]))
from tc_route_msgs.msg import MotionLimits
from geometry_msgs.msg import Pose, Twist
from robot_navigator.robot_navigator_node import RobotNavigator
from robot_navigator.input_watchdog_core import InputWatchdog


def test_contract_invalid_stale_and_missing_force_zero():
    output=[]
    guard=InputWatchdog({'motion_limits': .5})
    node=NS(max_v=1., max_w=1., max_a_v=.7, max_a_w=.6,max_decel_v=1.5,
        input_watchdog=guard, get_clock=lambda: NS(now=lambda:NS(nanoseconds=10_000_000_000)),
        _input_time_seconds=lambda: 10., get_logger=lambda: NS(error=lambda *a,**k:None,warn=lambda *a:None),
        _stale_inputs=(), integral_w=0.,prev_yaw_error=0., prev_cmd_vel=Twist(),
        cmd_pub=NS(publish=output.append))
    RobotNavigator.on_timer(node)
    assert output[-1] == Twist()  # no contract
    msg=MotionLimits(max_linear_velocity=1.,max_angular_velocity=1.,linear_acceleration=.7,
        linear_deceleration=1.5,angular_acceleration=.6)
    msg.header.stamp.sec=10
    RobotNavigator.on_motion_limits(node,msg)
    assert guard.stale_inputs(10.)==()
    assert guard.stale_inputs(10.5)==('motion_limits',)
    for brake,stamp in [(1.,10),(math.nan,10),(1.5,9),(1.5,11)]:
        msg.linear_deceleration=brake;msg.header.stamp.sec=stamp
        RobotNavigator.on_motion_limits(node,msg)
        RobotNavigator.on_timer(node)
        assert output[-1] == Twist()
        assert guard.stale_inputs(10.)==('motion_limits',)


def controller():
    return NS(prev_cmd_vel=Twist(),max_v=1.,max_w=1.,max_a_v=.7,max_a_w=.6,
        max_decel_v=1.5,braking_delay_sec=.2,dt=.05,pos_tol=.5,ang_tol=.25,
        _within_goal_pos_tolerance=False,_within_goal_ang_tolerance=False,
        integral_w=0.,integral_w_limit=1000.,prev_yaw_error=0.,kp_w=.65,ki_w=.001,kd_w=.02,
        obstacle_distance=None,min_obstacle_distance=.5,
        quaternion_to_yaw=RobotNavigator.quaternion_to_yaw,normalize_angle=RobotNavigator.normalize_angle)


def test_navigator_early_goal_braking_and_turning_momentum():
    current=Pose();current.orientation.w=1.
    target=Pose();target.position.x=.55;target.orientation.w=1.
    velocity=Twist();velocity.linear.x=1.
    node=controller()
    cmd,_=RobotNavigator.compute_time_optimal_cmd_vel(node,current,velocity,target)
    assert cmd.linear.x==0.  # before the old 0.5 m arrival tolerance
    target.position.x=5.;target.position.y=5.
    node=controller();node.obstacle_distance=1.
    cmd,_=RobotNavigator.compute_time_optimal_cmd_vel(node,current,velocity,target)
    assert cmd.linear.x==0.  # heading scaling cannot hide 1 m/s actual speed
