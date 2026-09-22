#!/usr/bin/env python3
"""Move local human geometry in Gazebo using simulation time and ground truth."""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from tf2_msgs.msg import TFMessage
from ament_index_python.packages import get_package_prefix

# Gazebo vendor packages install bindings outside the ROS site-packages directory.
for package in ('gz_transport_vendor', 'gz_msgs_vendor'):
    sys.path.append(str(Path(get_package_prefix(package))/'opt'/package/'lib/python'))

from gz.transport13 import Node as TransportNode
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_v_pb2 import Pose_V

from pedestrian_core import Crowd, Ground, sensor_reach, model_sdf


class PedestrianSimulator(Node):
    def __init__(self):
        super().__init__('pedestrian_simulator')
        world_file = self.declare_parameter('world_sdf', '').value
        ground_file = self.declare_parameter('world_json', '').value
        density = self.declare_parameter('density', .1).value
        seed = self.declare_parameter('seed', 42).value
        world_name, reach = sensor_reach(world_file)
        self.crowd = Crowd(Ground(json.loads(Path(ground_file).read_text())), reach, density, seed)
        self.transport = TransportNode()
        self.service = '/world/'+world_name
        self.ready = False
        self.transport_started = time.monotonic()
        self.robot = None
        self.truth_time = None
        self.previous_time = None
        self.create_subscription(TFMessage, '/truth', self.on_truth, 10)
        self.add_on_set_parameters_callback(self.parameters_changed)
        self.create_timer(.05, self.tick)
        self.get_logger().info(f'歩行者密度 {density} 人/100m²; センサ半径 {reach:.2f} m; 上限200人')

    def parameters_changed(self, parameters):
        for p in parameters:
            if p.name == 'density':
                if type(p.value) not in (float, int) or not math.isfinite(p.value) or p.value < 0:
                    return SetParametersResult(successful=False, reason='density >= 0 (people / 100 m²)')
            elif p.name in ('seed', 'world_sdf', 'world_json'):
                return SetParametersResult(successful=False, reason=p.name+' requires restart')
        for p in parameters:
            if p.name == 'density':
                self.crowd.set_density(p.value)
        return SetParametersResult(successful=True)

    def on_truth(self, message):
        for t in message.transforms:
            if t.child_frame_id == 'icart_mini' or t.child_frame_id.endswith('::icart_mini'):
                self.robot = (t.transform.translation.x, t.transform.translation.y)
                self.truth_time = self.get_clock().now().nanoseconds*1e-9

    def request(self, suffix, message):
        ok, response = self.transport.request(self.service+suffix, message, type(message), Boolean, 1000)
        if not ok or not response.data:
            # A timeout may already have applied the command: stop rather than duplicate entities.
            raise RuntimeError('Gazebo pedestrian service failed: '+suffix)

    def tick(self):
        if time.monotonic()-self.transport_started < 1.:
            return
        if not self.ready:
            available = set(self.transport.service_list())
            self.ready = all(self.service+suffix in available
                             for suffix in ('/create', '/remove', '/set_pose_vector'))
            self.previous_time = None
            return
        now = self.get_clock().now().nanoseconds*1e-9
        previous, self.previous_time = self.previous_time, now
        if (self.robot is None or previous is None or self.truth_time is None
                or not 0 <= now-self.truth_time < .5):
            return
        births, exits = self.crowd.step(self.robot, now-previous)
        for name in exits:
            self.request('/remove', Entity(name=name, type=Entity.MODEL))
            del self.crowd.walkers[name]
        for w in births:
            self.request('/create', EntityFactory(sdf=model_sdf(w), allow_renaming=False))
            self.crowd.walkers[w.name] = w
        poses = Pose_V()
        for w in self.crowd.walkers.values():
            p = poses.pose.add(name=w.name)
            p.position.x, p.position.y, p.position.z = w.x, w.y, w.z
            p.orientation.z, p.orientation.w = math.sin(w.heading/2), math.cos(w.heading/2)
        if poses.pose:
            self.request('/set_pose_vector', poses)


def main():
    rclpy.init()
    node = PedestrianSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
