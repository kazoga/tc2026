"""HTML遠隔観測UI向けの読み取り専用HTTPサーバ。

`robot_console_gui_screen_function_design.md` 8章・
`robot_console_gui_architecture_design.md` 15章の方針に従い、以下のみを提供する。

* ``GET /snapshot.json`` ``GET /map_state.json`` ``GET /sensor_panels.json``
  ``GET /health.json`` : `web.json_codec` が組み立てたJSON
* ``GET /images/{panel_id}`` : `ImageStore` の最新画像をPNGで返す
* それ以外のGET : `static_root` 配下の閲覧専用ページを配信する

GET以外のメソッド（POST/PUT/DELETE等）は405を返し、操作系APIを一切提供しない。
"""

from __future__ import annotations

import io
import json
import mimetypes
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable, Optional, Tuple
from urllib.parse import urlparse

from ..core.image_store import ImageStore
from ..core.snapshot_model import ConsoleSnapshot
from .json_codec import (
    build_health_payload,
    build_map_state_payload,
    build_sensor_panels_payload,
    build_snapshot_payload,
)

DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 8765

SnapshotProvider = Callable[[], ConsoleSnapshot]

_JSON_ROUTES = {
    '/snapshot.json': build_snapshot_payload,
    '/map_state.json': build_map_state_payload,
    '/sensor_panels.json': build_sensor_panels_payload,
    '/health.json': build_health_payload,
}


def _default_static_root() -> Path:
    """`web/static/`（本モジュールと同じパッケージ配下）を返す。"""

    return Path(__file__).resolve().parent / 'static'


class _ObservationHTTPServer(ThreadingHTTPServer):
    """snapshot_provider / image_store / static_root を保持するHTTPサーバ。"""

    daemon_threads = True

    def __init__(
        self,
        address: Tuple[str, int],
        handler_cls: type,
        *,
        snapshot_provider: SnapshotProvider,
        image_store: ImageStore,
        static_root: Path,
    ) -> None:
        super().__init__(address, handler_cls)
        self.snapshot_provider = snapshot_provider
        self.image_store = image_store
        self.static_root = static_root


class _ObservationRequestHandler(BaseHTTPRequestHandler):
    """読み取り専用エンドポイントのみを提供するリクエストハンドラ。"""

    server: _ObservationHTTPServer  # type: ignore[assignment]
    protocol_version = 'HTTP/1.1'

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandlerの命名規則に合わせる
        path = urlparse(self.path).path
        if path in _JSON_ROUTES:
            payload = _JSON_ROUTES[path](self.server.snapshot_provider())
            self._write_json(payload)
            return
        if path.startswith('/images/'):
            self._write_image(path[len('/images/'):])
            return
        self._write_static(path)

    def do_POST(self) -> None:  # noqa: N802
        self._write_method_not_allowed()

    def do_PUT(self) -> None:  # noqa: N802
        self._write_method_not_allowed()

    def do_PATCH(self) -> None:  # noqa: N802
        self._write_method_not_allowed()

    def do_DELETE(self) -> None:  # noqa: N802
        self._write_method_not_allowed()

    def _write_method_not_allowed(self) -> None:
        body = 'HTML observation UI is read-only.'.encode('utf-8')
        self.send_response(405)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Allow', 'GET')
        self.end_headers()
        self.wfile.write(body)

    def _write_json(self, payload: object) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _write_image(self, panel_id: str) -> None:
        image = self.server.image_store.get(panel_id)
        if image is None:
            self._write_not_found()
            return
        buffer = io.BytesIO()
        image.save(buffer, format='PNG')
        body = buffer.getvalue()
        self.send_response(200)
        self.send_header('Content-Type', 'image/png')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _write_static(self, path: str) -> None:
        if path == '/':
            path = '/index.html'
        relative = path.lstrip('/')
        static_root = self.server.static_root.resolve()
        candidate = (static_root / relative).resolve()
        if candidate != static_root and static_root not in candidate.parents:
            self._write_forbidden()
            return
        if not candidate.is_file():
            self._write_not_found()
            return
        content_type, _ = mimetypes.guess_type(str(candidate))
        body = candidate.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', content_type or 'application/octet-stream')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _write_not_found(self) -> None:
        body = b'Not Found'
        self.send_response(404)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _write_forbidden(self) -> None:
        body = b'Forbidden'
        self.send_response(403)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002
        # BaseHTTPRequestHandlerの既定実装はstderrへ逐次出力するため抑制する。
        return


class WebObservationServer:
    """HTML遠隔観測UI用HTTPサーバの起動・停止を管理する。"""

    def __init__(
        self,
        snapshot_provider: SnapshotProvider,
        image_store: Optional[ImageStore] = None,
        *,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        static_root: Optional[Path] = None,
    ) -> None:
        self._snapshot_provider = snapshot_provider
        self._image_store = image_store if image_store is not None else ImageStore()
        self._host = host
        self._port = port
        self._static_root = static_root or _default_static_root()
        self._httpd: Optional[_ObservationHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    @property
    def image_store(self) -> ImageStore:
        """本サーバが画像配信に使う `ImageStore` を返す。"""

        return self._image_store

    @property
    def address(self) -> Optional[Tuple[str, int]]:
        """実際にbindされたアドレス（起動前は None）。"""

        if self._httpd is None:
            return None
        return self._httpd.server_address

    def start(self) -> None:
        """バックグラウンドスレッドでサーバを起動する。既に起動中の場合は何もしない。"""

        if self._httpd is not None:
            return
        self._httpd = _ObservationHTTPServer(
            (self._host, self._port),
            _ObservationRequestHandler,
            snapshot_provider=self._snapshot_provider,
            image_store=self._image_store,
            static_root=self._static_root,
        )
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """サーバを停止する。起動していない場合は何もしない。"""

        if self._httpd is None:
            return
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self._httpd = None
        self._thread = None
