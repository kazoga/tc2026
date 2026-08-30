"""web/server.py の単体テスト（実HTTPリクエストで検証する）。"""

import json
import urllib.error
import urllib.request

import pytest
from PIL import Image

from robot_console.core.image_store import ImageStore
from robot_console.core.snapshot_model import ConsoleSnapshot, OperationStateView
from robot_console.web.server import WebObservationServer


@pytest.fixture
def server():
    snapshot = ConsoleSnapshot(operation_state=OperationStateView(phase='走行中'))
    image_store = ImageStore()
    image_store.set('route_map', Image.new('RGB', (4, 4), color='green'))

    instance = WebObservationServer(lambda: snapshot, image_store, host='127.0.0.1', port=0)
    instance.start()
    yield instance
    instance.stop()


def _get(server, path: str):
    host, port = server.address
    url = f'http://{host}:{port}{path}'
    with urllib.request.urlopen(url, timeout=5) as response:
        return response.status, response.headers.get('Content-Type'), response.read()


def test_snapshot_json_endpoint_returns_expected_phase(server):
    status, content_type, body = _get(server, '/snapshot.json')

    assert status == 200
    assert content_type.startswith('application/json')
    payload = json.loads(body)
    assert payload['operation']['phase'] == '走行中'


def test_map_state_json_endpoint_responds(server):
    status, content_type, body = _get(server, '/map_state.json')

    assert status == 200
    assert content_type.startswith('application/json')
    json.loads(body)  # パース可能であること


def test_sensor_panels_json_endpoint_responds(server):
    status, _content_type, body = _get(server, '/sensor_panels.json')

    assert status == 200
    payload = json.loads(body)
    assert 'panels' in payload


def test_health_json_endpoint_responds(server):
    status, _content_type, body = _get(server, '/health.json')

    assert status == 200
    json.loads(body)


def test_image_endpoint_returns_png_for_known_panel(server):
    status, content_type, body = _get(server, '/images/route_map')

    assert status == 200
    assert content_type == 'image/png'
    assert body[:8] == b'\x89PNG\r\n\x1a\n'  # PNGシグネチャ


def test_image_endpoint_returns_404_for_unknown_panel(server):
    with pytest.raises(urllib.error.HTTPError) as exc_info:
        _get(server, '/images/unknown_panel')
    assert exc_info.value.code == 404


def test_static_index_html_is_served_at_root(server):
    status, content_type, body = _get(server, '/')

    assert status == 200
    assert content_type == 'text/html'
    assert b'robot_console' in body


def test_static_unknown_path_returns_404(server):
    with pytest.raises(urllib.error.HTTPError) as exc_info:
        _get(server, '/no-such-file.html')
    assert exc_info.value.code == 404


def test_path_traversal_is_rejected(server):
    with pytest.raises(urllib.error.HTTPError) as exc_info:
        _get(server, '/../server.py')
    # urllibが送信前にパスを正規化するため、リクエストパスとしては404/403いずれかになり得る。
    assert exc_info.value.code in (403, 404)


@pytest.mark.parametrize('method', ['POST', 'PUT', 'DELETE', 'PATCH'])
def test_write_methods_are_rejected_with_405(server, method):
    host, port = server.address
    url = f'http://{host}:{port}/snapshot.json'
    request = urllib.request.Request(url, method=method, data=b'{}')
    with pytest.raises(urllib.error.HTTPError) as exc_info:
        urllib.request.urlopen(request, timeout=5)
    assert exc_info.value.code == 405
    assert exc_info.value.headers.get('Allow') == 'GET'


def test_start_is_idempotent(server):
    address_before = server.address
    server.start()
    assert server.address == address_before
