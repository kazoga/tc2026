"""Manual integration check: private IPC/DDS, coordinator --without-device only.
Run after sourcing install/setup.bash. Never connects to a serial device.
"""
import ctypes
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time

# Set before importing ROS. Topics are private in addition to a separate domain.
os.environ['ROS_DOMAIN_ID']='219'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import rclpy
from geometry_msgs.msg import Twist
from tc_route_msgs.msg import MotionLimits

ROOT=Path(__file__).resolve().parents[3]
key=1900000+os.getpid()
libc=ctypes.CDLL(None,use_errno=True)
assert libc.msgget(key,0) == -1, 'Private IPC key already in use'
lib=ctypes.CDLL(str(ROOT/'install/ypspur_ros2/lib/libypspur.so'))
lib.YPSpur_initex.argtypes=[ctypes.c_int]
lib.YP_get_vref.argtypes=[ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
lib.YP_get_vref.restype=ctypes.c_double
processes=[]
with tempfile.TemporaryDirectory(prefix='icart-p01-') as tmp:
    logpath=Path(tmp)/'process.log'
    with logpath.open('w') as log:
        try:
            coordinator=subprocess.Popen([shutil.which('ypspur-coordinator'), '--without-device',
                '--msq-key',str(key),'-p',str(ROOT/'src/ypspur_ros2/config/icart-middle.param')],stdout=log,stderr=log)
            processes.append(coordinator)
            deadline=time.monotonic()+5
            while libc.msgget(key,0)==-1:
                assert coordinator.poll() is None, logpath.read_text()
                assert time.monotonic()<deadline, logpath.read_text()
                time.sleep(.02)
            driver=subprocess.Popen([str(ROOT/'install/ypspur_ros2/lib/ypspur_ros2/ypspur_node'),
                '--ros-args','--params-file',str(ROOT/'src/ypspur_ros2/config/default.yaml'),
                '-p',f'ipc.key:={key}','-r','cmd_vel:=/p01_offline/cmd_vel',
                '-r','motion_limits:=/p01_offline/motion_limits','-r','odom:=/p01_offline/odom'],stdout=log,stderr=log)
            processes.append(driver)
            assert lib.YPSpur_initex(key)>=0
            rclpy.init();node=rclpy.create_node('p01_offline_probe')
            received=[]
            sub=node.create_subscription(MotionLimits,'/p01_offline/motion_limits',received.append,10)
            pub=node.create_publisher(Twist,'/p01_offline/cmd_vel',10)
            deadline=time.monotonic()+5
            while not received or pub.get_subscription_count()==0:
                rclpy.spin_once(node,timeout_sec=.02)
                assert driver.poll() is None, logpath.read_text()
                assert time.monotonic()<deadline, logpath.read_text()
            assert received[-1].linear_acceleration==.7
            assert received[-1].linear_deceleration==1.5
            samples=[]
            def phase(label,target,duration):
                start=time.monotonic()
                while time.monotonic()-start<duration:
                    if target is not None:
                        msg=Twist();msg.linear.x=target;pub.publish(msg)
                    rclpy.spin_once(node,timeout_sec=.01)
                    ref=ctypes.c_double();w=ctypes.c_double()
                    assert lib.YP_get_vref(ctypes.byref(ref),ctypes.byref(w))>=0
                    samples.append((label,time.monotonic()-start,ref.value))
                    time.sleep(.01)
            phase('acceleration',1.,1.8)
            phase('braking',0.,1.)
            phase('reverse_acceleration',-.5,1.)
            phase('direction_change',.5,1.4)
            phase('watchdog',None,1.2)
            def value(label,t):
                return min((r for r in samples if r[0]==label),key=lambda r:abs(r[1]-t))[2]
            # Wide timing tolerances; distinguish .7 from 1.5, not scheduler jitter.
            assert .25<value('acceleration',.5)<.45, value('acceleration',.5)
            assert .4<value('braking',.3)<.7, value('braking',.3)
            assert abs(value('braking',.9))<.01
            assert -.45<value('reverse_acceleration',.5)<-.25
            assert abs(value('watchdog',1.1))<.01
            assert driver.poll() is None,logpath.read_text()
            driver.send_signal(signal.SIGINT); driver.wait(timeout=3)
            # A driver asking for more braking than the actual loaded .param
            # permits must fail before advertising a usable motion contract.
            rejected=subprocess.run([str(ROOT/'install/ypspur_ros2/lib/ypspur_ros2/ypspur_node'),
                '--ros-args','-p',f'ipc.key:={key}','-p','deceleration_max.linear:=1.6'],
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=5)
            assert rejected.returncode != 0 and 'ceiling' in rejected.stdout, rejected.stdout
            print(json.dumps({'mode':'without-device','acceleration_at_0_5s':value('acceleration',.5),
                'braking_at_0_3s':value('braking',.3),'reverse_at_0_5s':value('reverse_acceleration',.5),
                'watchdog_final':value('watchdog',1.1),'limits_messages':len(received),
                'above_icart_ceiling_rejected':True},indent=2))
            node.destroy_node();rclpy.shutdown()
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:process.wait(timeout=3)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
            queue=libc.msgget(key,0)
            if queue>=0:libc.msgctl(queue,0,None)
