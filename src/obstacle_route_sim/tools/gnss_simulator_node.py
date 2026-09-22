#!/usr/bin/env python3
"""Gazebo 真値から遅延・誤差・途絶を持つ UM982 互換トピックを配信する."""

import json
import math
from pathlib import Path
import random
import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rtk_gps_um982_msgs.msg import RtkStatus
from geo_pose_converter.geo_core import EnuPoint, ProjectionConfig, enu_to_llh


class GnssSimulator(Node):
    """位置・方位の誤差は未校正の試験条件であり、受信機精度の保証ではない."""

    def __init__(self) -> None:
        super().__init__('gnss_simulator')
        defaults = dict(origin_latitude=36.082628231, origin_longitude=140.076144739,
                        origin_altitude=67.078, horizontal_sigma_m=.02,
                        vertical_sigma_m=.04, heading_sigma_deg=.5, latency_sec=.1,
                        rate_hz=10.0, seed=42, start_wall_sec=0.0, dropout_sec=0.0, dropout_duration_sec=0.0, buildings_json="",
                        float_enter_m=12., float_exit_m=15., float_sigma_m=.6,
                        float_bias_m=1.2, float_heading_sigma_deg=5.,
                        baseline_nominal_m=.5, baseline_sigma_m=0., baseline_float_sigma_m=0.,
                        baseline_shift_m=0., baseline_shift_after_s=0.,
                        heading_fault_deg=0., heading_fault_after_s=0., heading_fault_duration_s=0.,
                        heading_reference='vehicle_forward', start_fix_radius_m=0.)
        self.values = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for key in ['horizontal_sigma_m', 'vertical_sigma_m', 'heading_sigma_deg',
                    'latency_sec', 'dropout_sec', 'dropout_duration_sec',
                    'baseline_sigma_m', 'baseline_float_sigma_m', 'baseline_shift_after_s',
                    'start_fix_radius_m']:
            if not math.isfinite(self.values[key]) or self.values[key] < 0:
                raise ValueError(key+' は有限の非負値が必要')
        if not all(math.isfinite(self.values[k]) for k in ['baseline_nominal_m', 'baseline_shift_m']):
            raise ValueError('baseline距離は有限値が必要')
        if min(self.values['baseline_nominal_m'],
               self.values['baseline_nominal_m']+self.values['baseline_shift_m']) <= 0:
            raise ValueError('変更前後のbaseline距離は正の値が必要')
        if not math.isfinite(self.values['rate_hz']) or self.values['rate_hz'] <= 0:
            raise ValueError('rate_hz は正の有限値が必要')
        if self.values['heading_reference'] not in ['vehicle_forward', 'antenna_baseline']:
            raise ValueError('heading_referenceはvehicle_forwardまたはantenna_baselineが必要')
        self.projection = ProjectionConfig(**{k: self.values[k] for k in
            ['origin_latitude', 'origin_longitude', 'origin_altitude']})
        self.random = random.Random(self.values['seed'])
        self.environment = None
        if self.values['buildings_json']:
            from gnss_environment_core import BuildingDegradation
            world = json.loads(Path(self.values['buildings_json']).read_text())
            self.environment = BuildingDegradation(
                [o['poly'] for o in world['obstacles'] if o['source']=='building'],
                self.values['float_enter_m'], self.values['float_exit_m'],
                self.values['float_sigma_m'], self.values['float_bias_m'], self.values['seed']+1)
        self.previous_float = False
        self.truth = None
        self.last_truth = 0.0
        self.started = self.values['start_wall_sec'] or time.monotonic()
        self.queue = deque()
        self.fix_pub = self.create_publisher(NavSatFix, '/rtk_gps/fix', 10)
        self.slave_pub = self.create_publisher(NavSatFix, '/rtk_gps/slave_fix', 10)
        self.heading_pub = self.create_publisher(Imu, '/rtk_gps/heading', 10)
        self.status_pub = self.create_publisher(RtkStatus, '/rtk_gps/rtk_status', 10)
        self.ntrip_pub = self.create_publisher(String, '/rtk_gps/ntrip_status', 10)
        self.create_subscription(TFMessage, '/truth', self.on_truth, 10)
        self.create_timer(1/self.values['rate_hz'], self.sample)
        self.create_timer(.01, self.deliver)
        self.create_timer(1., self.publish_ntrip_status)
        self.get_logger().info('仮想 GNSS を開始。真値欠落時は配信を停止する')

    def publish_ntrip_status(self) -> None:
        """基地局診断を DISABLED で配信する。

        シムに NTRIP 接続は存在しない。無配信にすると UI 側が「未受信」となり、
        実機で診断が途絶した異常と区別できないため、補正を使わない構成を表す
        DISABLED を明示する。受信中を装う値は作らない。
        """

        self.ntrip_pub.publish(String(data=json.dumps(dict(
            state='DISABLED', host='', port=0, mountpoint='', station_id='',
            station_label='', site='', transport_connected=False,
            rtcm_bytes_total=0, rtcm_bytes_per_s=0., last_rtcm_age_s=None,
            reconnect_count=0, last_error=''))))

    def on_truth(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.child_frame_id in ['icart_mini', 'terrain3d_world::icart_mini']:
                self.truth = transform
                self.last_truth = time.monotonic()

    def sample(self) -> None:
        now = time.monotonic()
        elapsed = now-self.started
        start = self.values['dropout_sec']
        duration = self.values['dropout_duration_sec']
        if start > 0 and elapsed >= start and (duration == 0 or elapsed < start+duration):
            self.queue.clear()
            return
        if self.truth is None or now-self.last_truth >= .5:
            self.queue.clear()
            return
        t = self.truth.transform
        q = t.rotation
        # roll/pitch を含むアンテナ lever arm。マスターは車軸の真上である。
        def rotate(x: float, z: float) -> tuple[float, float, float]:
            return ((1-2*(q.y*q.y+q.z*q.z))*x+2*(q.x*q.z+q.w*q.y)*z,
                    2*(q.x*q.y+q.w*q.z)*x+2*(q.y*q.z-q.w*q.x)*z,
                    2*(q.x*q.z-q.w*q.y)*x+(1-2*(q.x*q.x+q.y*q.y))*z)
        master = rotate(0, .7)
        physical_baseline = self.values.get('baseline_nominal_m', .5)
        if elapsed >= self.values.get('baseline_shift_after_s', 0.):
            physical_baseline += self.values.get('baseline_shift_m', 0.)
        slave = rotate(-physical_baseline, .7)
        yaw = math.atan2(master[1]-slave[1], master[0]-slave[0])
        # UM982の無補正headingはANT1(master)→ANT2(slave)。既存試験は前方基準を保持する。
        if self.values.get('heading_reference', 'vehicle_forward') == 'antenna_baseline':
            yaw += math.pi
        effect = self.environment.sample(t.translation.x,t.translation.y,
            1/self.values['rate_hz']) if getattr(self,'environment',None) else None
        # A fixed region around the first valid robot position, not a region that follows it.
        if getattr(self, 'start_xy', None) is None:
            self.start_xy = (t.translation.x, t.translation.y)
        radius = self.values.get('start_fix_radius_m', 0.)
        if radius > 0 and math.dist(self.start_xy, (t.translation.x, t.translation.y)) <= radius:
            effect = None
        floating = bool(effect and effect['floating'])
        heading_sigma = self.values.get('float_heading_sigma_deg',5.) if floating else self.values['heading_sigma_deg']
        yaw += math.radians(self.random.gauss(0, heading_sigma))
        if (self.values.get('heading_fault_after_s', 0.) <= elapsed <
                self.values.get('heading_fault_after_s', 0.)+self.values.get('heading_fault_duration_s', 0.)):
            yaw += math.radians(self.values.get('heading_fault_deg', 0.))
        if floating != getattr(self,'previous_float',False):
            self.get_logger().info('GNSS 状態: '+('FLOAT' if floating else 'FIX'))
            self.previous_float = floating
        sigma = effect['sigma_m'] if floating else self.values['horizontal_sigma_m']
        vertical = self.values['vertical_sigma_m']
        noise = [self.random.gauss(0, s) for s in [sigma, sigma, vertical]]
        if effect:
            noise[0] += effect['bias'][0]
            noise[1] += effect['bias'][1]
        horizontal_variance = sigma*sigma + (max(abs(v) for v in effect['bias'])**2 if effect else 0.)
        fixes = []
        for frame, offset in [('gnss_master', master), ('gnss_slave', slave)]:
            point = enu_to_llh(EnuPoint(t.translation.x+offset[0]+noise[0],
                                      t.translation.y+offset[1]+noise[1],
                                      t.translation.z+offset[2]+noise[2]), self.projection)
            fix = NavSatFix()
            fix.header.stamp = self.truth.header.stamp
            fix.header.frame_id = frame
            fix.latitude, fix.longitude, fix.altitude = point.latitude, point.longitude, point.altitude
            fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            fix.status.service = NavSatStatus.SERVICE_GPS
            fix.position_covariance = [horizontal_variance, 0., 0., 0., horizontal_variance, 0., 0., 0., vertical*vertical]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            fixes.append(fix)
        status = RtkStatus()
        status.header = fixes[0].header
        status.rtk_state = RtkStatus.STATE_RTK_FLOAT if floating else RtkStatus.STATE_RTK_FIX
        status.rtk_state_raw = 'SIM_RTK_FLOAT' if floating else 'SIM_RTK_FIX'
        status.num_satellites = 10 if floating else 20
        status.hdop = 2.5 if floating else .7
        status.heading_deg = (90-math.degrees(yaw)) % 360
        status.heading_stddev_deg = heading_sigma
        # 共通のGPS位置誤差とは別に、受信機の相対baseline観測誤差を模擬する。
        baseline_sigma = self.values.get('baseline_float_sigma_m', 0.) if floating else self.values.get('baseline_sigma_m', 0.)
        status.baseline_length_m = physical_baseline+(self.random.gauss(0., baseline_sigma)
                                                    if baseline_sigma > 0 else 0.)
        status.latitude, status.longitude, status.altitude = (
            fixes[0].latitude, fixes[0].longitude, fixes[0].altitude)
        imu = Imu()
        imu.header = fixes[0].header
        imu.orientation.z, imu.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        imu.orientation_covariance[8] = math.radians(heading_sigma)**2
        imu.angular_velocity_covariance[0] = -1.
        imu.linear_acceleration_covariance[0] = -1.
        self.queue.append((now+self.values['latency_sec'], fixes, status, imu))

    def deliver(self) -> None:
        while self.queue and self.queue[0][0] <= time.monotonic():
            _, fixes, status, imu = self.queue.popleft()
            self.status_pub.publish(status)
            self.heading_pub.publish(imu)
            self.fix_pub.publish(fixes[0])
            self.slave_pub.publish(fixes[1])


def main() -> None:
    rclpy.init()
    node = GnssSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
