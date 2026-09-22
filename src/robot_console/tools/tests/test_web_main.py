"""HTML版の起動引数と、GNSS診断のDDS→HTTP配信を確認する。"""

import json
import time
from types import SimpleNamespace
from urllib.request import urlopen

import pytest

rclpy = pytest.importorskip('rclpy')

from rtk_gps_um982_msgs.msg import RtkStatus  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from robot_console import web_main  # noqa: E402


def test_parse_args_separates_ros_arguments():
    args = web_main._parse_args([
        '--port', '9000', '--ros-args',
        '-r', 'rtk_gps/rtk_status:=/receiver/rtk_status',
        '--', '--console-log-directory', '/tmp/console-test',
    ])

    assert args.port == 9000
    assert args.console_log_directory == '/tmp/console-test'
    assert args.host == '127.0.0.1'


def test_parse_args_rejects_unknown_application_arguments():
    with pytest.raises(SystemExit) as error:
        web_main._parse_args(['--unknown-option'])
    assert error.value.code == 2


@pytest.mark.parametrize('gnss_namespace', [
    '/rtk_gps',
    '/rtk_gps/rtk_gps_um982_node',
    '/custom_receiver',
])
def test_main_delivers_gnss_diagnostics_to_http(monkeypatch, gnss_namespace):
    """シムの既定名・実機のprivate名・独自namespaceから両診断を受信する。"""

    monkeypatch.setenv('ROS_DOMAIN_ID', '194')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    if rclpy.ok():
        rclpy.shutdown()
    server_type = web_main.WebObservationServer
    servers = []

    def create_server(*args, **kwargs):
        server = server_type(*args, **kwargs)
        servers.append(server)
        return server

    def check_diagnostics(_seconds):
        producer = rclpy.create_node('test_web_gnss_producer')
        rtk_pub = producer.create_publisher(RtkStatus, f'{gnss_namespace}/rtk_status', 10)
        ntrip_pub = producer.create_publisher(String, f'{gnss_namespace}/ntrip_status', 10)
        host, port = servers[0].address
        try:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                rtk_pub.publish(RtkStatus(rtk_state=RtkStatus.STATE_RTK_FIX,
                                          rtk_state_raw='NARROW_INT', num_satellites=24))
                ntrip_pub.publish(String(data=json.dumps({
                    'state': 'RECEIVING', 'host': 'test', 'port': 2101,
                    'mountpoint': 'MOCK', 'rtcm_bytes_per_s': 250.0,
                    'last_rtcm_age_s': 0.2,
                })))
                with urlopen(f'http://{host}:{port}/snapshot.json', timeout=2) as response:
                    payload = json.load(response)
                if (payload['gps']['num_satellites'] == 24
                        and 'MOCK' in json.dumps(payload['gnss_details'])):
                    break
                time.sleep(0.05)
            else:
                pytest.fail(f'GNSS diagnostics did not reach HTTP from {gnss_namespace}')
            assert payload['gps']['status_freshness'] == 'OK'
        finally:
            producer.destroy_node()
        raise KeyboardInterrupt

    monkeypatch.setattr(web_main, 'WebObservationServer', create_server)
    monkeypatch.setattr(web_main, 'time', SimpleNamespace(sleep=check_diagnostics))
    argv = ['--port', '0']
    if gnss_namespace != '/rtk_gps':
        argv += ['--ros-args',
                 '-r', f'rtk_gps/rtk_status:={gnss_namespace}/rtk_status',
                 '-r', f'rtk_gps/ntrip_status:={gnss_namespace}/ntrip_status']

    assert web_main.main(argv) == 0
    assert not rclpy.ok()
