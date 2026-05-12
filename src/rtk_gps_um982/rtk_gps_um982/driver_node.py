"""UM982 RTK GNSS ドライバノード (rclpy)。"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix

from rtk_gps_um982_msgs.msg import RtkStatus
from um982 import UM982Client

from rtk_gps_um982 import converters


class Um982DriverNode(Node):

    def __init__(self) -> None:
        super().__init__('rtk_gps_um982_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial.port', '/dev/ttyUSB0'),
                ('serial.baud', 115200),
                ('output_rate_hz', 10),
                ('frame_id', 'gps_link'),
                ('stamp_source', 'gnss_utc'),
                ('transport_delay_ms', 0),
                ('ntrip.enabled', False),
                ('ntrip.host', ''),
                ('ntrip.port', 2101),
                ('ntrip.mountpoint', ''),
                ('ntrip.user', ''),
                ('ntrip.password', ''),
                ('publish.navsatfix', True),
                ('publish.imu_heading', True),
                ('publish.rtk_status', True),
                ('min_fix_for_publish', 'standalone'),
                ('hdop_sigma', 1.0),
            ],
        )

        p = self.get_parameter
        self._port = p('serial.port').value
        self._baud = p('serial.baud').value
        self._output_rate = p('output_rate_hz').value
        self._frame_id = p('frame_id').value
        self._stamp_source = p('stamp_source').value
        self._transport_delay_ms = p('transport_delay_ms').value
        self._min_fix = p('min_fix_for_publish').value
        self._hdop_sigma = float(p('hdop_sigma').value)

        self._pub_fix = (
            self.create_publisher(NavSatFix, '~/fix', 10)
            if p('publish.navsatfix').value else None
        )
        self._pub_imu = (
            self.create_publisher(Imu, '~/heading', 10)
            if p('publish.imu_heading').value else None
        )
        self._pub_status = (
            self.create_publisher(RtkStatus, '~/rtk_status', 10)
            if p('publish.rtk_status').value else None
        )

        self._client = UM982Client(
            port=self._port,
            baud=self._baud,
            output_rate=self._output_rate,
        )
        self._client.start()
        self._client.set_position_callback(self._on_position)

        if p('ntrip.enabled').value:
            host = p('ntrip.host').value
            port = p('ntrip.port').value
            mountpoint = p('ntrip.mountpoint').value
            user = p('ntrip.user').value
            password = p('ntrip.password').value
            if not host or not mountpoint:
                self.get_logger().error(
                    'ntrip.enabled=true but ntrip.host / ntrip.mountpoint is empty'
                )
            else:
                self.get_logger().info(
                    f'Starting NTRIP: {host}:{port}/{mountpoint} as {user or "(anonymous)"}'
                )
                self._client.start_ntrip(
                    host=host, port=port, mountpoint=mountpoint,
                    user=user, password=password,
                )

        self.get_logger().info(
            f'rtk_gps_um982_node up (port={self._port} baud={self._baud} '
            f'rate={self._output_rate}Hz stamp={self._stamp_source})'
        )

    def _on_position(self, pos) -> None:
        if not pos.is_valid:
            return
        if not converters.passes_fix_filter(pos.rtk_state, self._min_fix):
            return

        stamp = converters.make_stamp(
            self._stamp_source,
            pos.timestamp,
            self._transport_delay_ms,
            self.get_clock(),
        )

        if self._pub_fix is not None:
            self._pub_fix.publish(
                converters.position_to_navsatfix(pos, stamp, self._frame_id, self._hdop_sigma)
            )
        if self._pub_imu is not None:
            self._pub_imu.publish(
                converters.position_to_imu(pos, stamp, self._frame_id)
            )
        if self._pub_status is not None:
            self._pub_status.publish(
                converters.position_to_rtk_status(
                    pos, stamp, self._frame_id, self._client.get_rtcm_bytes()
                )
            )

    def destroy_node(self) -> bool:
        try:
            self._client.stop()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'UM982Client.stop() raised: {exc}')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Um982DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
