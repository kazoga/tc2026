#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""LLH自己位置をOpenStreetMap上に三角アイコンで表示するROS 2ノード.

route_geo_projector_node が publish する /localization/pose_llh
(tc_geo_msgs/GeoPoseWithQuality) を購読し、ブラウザ上のOpenStreetMapに
現在位置と方位を重畳表示する。

表示アイコン:
  - 底辺:高さ = 1:2 の二等辺三角形
  - 頂角から底辺への垂線方向が heading_deg を表す
  - heading_deg は真北基準・時計回り[deg]

注意:
  - 地図描画には Leaflet CDN と OpenStreetMap タイルを使う。
  - そのため、通常はブラウザ表示時にインターネット接続が必要。
"""

from __future__ import annotations

import json
import math
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tc_geo_msgs.msg import GeoPoseWithQuality


class PoseStore:
    """HTTPサーバスレッドとROSスレッドの間で最新姿勢を共有するクラス."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pose: Optional[dict[str, Any]] = None

    def update(self, pose: dict[str, Any]) -> None:
        """最新姿勢を更新する."""
        with self._lock:
            self._pose = dict(pose)

    def get(self) -> Optional[dict[str, Any]]:
        """最新姿勢を取得する."""
        with self._lock:
            if self._pose is None:
                return None
            return dict(self._pose)


class LlhOsmViewerNode(Node):
    """GeoPoseWithQualityを購読してWeb地図ビューアへ表示するノード."""

    def __init__(self) -> None:
        super().__init__('llh_osm_viewer')

        self.declare_parameter('pose_llh_topic', '/localization/pose_llh')
        self.declare_parameter('http_host', '127.0.0.1')
        self.declare_parameter('http_port', 8765)
        self.declare_parameter('open_browser', True)
        self.declare_parameter('poll_interval_ms', 200)
        self.declare_parameter('initial_zoom', 19)
        self.declare_parameter('default_latitude', 36.082)
        self.declare_parameter('default_longitude', 140.111)
        self.declare_parameter('default_zoom', 17)
        self.declare_parameter('triangle_height_px', 48.0)

        self._pose_store = PoseStore()
        self._http_server: Optional[ThreadingHTTPServer] = None

        self._pose_llh_topic = str(self.get_parameter('pose_llh_topic').value)
        self._http_host = str(self.get_parameter('http_host').value)
        self._http_port = int(self.get_parameter('http_port').value)
        self._open_browser = bool(self.get_parameter('open_browser').value)
        self._poll_interval_ms = int(self.get_parameter('poll_interval_ms').value)
        self._initial_zoom = int(self.get_parameter('initial_zoom').value)
        self._default_latitude = float(self.get_parameter('default_latitude').value)
        self._default_longitude = float(self.get_parameter('default_longitude').value)
        self._default_zoom = int(self.get_parameter('default_zoom').value)
        self._triangle_height_px = float(self.get_parameter('triangle_height_px').value)

        qos_stream = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            GeoPoseWithQuality,
            self._pose_llh_topic,
            self._on_pose_llh,
            qos_stream,
        )

        self._http_thread = threading.Thread(
            target=self._serve_http,
            daemon=True,
        )
        self._http_thread.start()

        viewer_url = f'http://{self._http_host}:{self._http_port}/'
        self.get_logger().info(
            'llh_osm_viewer node started: '
            f'topic={self._pose_llh_topic}, url={viewer_url}'
        )

        if self._open_browser:
            # HTTPサーバ起動直後にブラウザがアクセスすると失敗する場合があるため、
            # 1秒遅延してから開く。
            browser_timer = threading.Timer(
                1.0,
                lambda: webbrowser.open(viewer_url),
            )
            browser_timer.daemon = True
            browser_timer.start()

    def destroy_node(self) -> bool:
        """ノード破棄時にHTTPサーバも停止する."""
        if self._http_server is not None:
            self.get_logger().info('Shutting down llh_osm_viewer HTTP server.')
            self._http_server.shutdown()
            self._http_server.server_close()
            self._http_server = None
        return super().destroy_node()

    def _on_pose_llh(self, msg: GeoPoseWithQuality) -> None:
        """LLH姿勢メッセージ受信時の処理."""
        latitude = float(msg.pose.point.latitude)
        longitude = float(msg.pose.point.longitude)

        if not math.isfinite(latitude) or not math.isfinite(longitude):
            self.get_logger().warn('Ignored non-finite latitude/longitude.')
            return

        if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
            self.get_logger().warn(
                'Ignored out-of-range latitude/longitude: '
                f'lat={latitude}, lon={longitude}'
            )
            return

        has_heading = bool(msg.pose.has_heading)
        heading_deg = float(msg.pose.heading_deg) if has_heading else 0.0

        if not math.isfinite(heading_deg):
            heading_deg = 0.0
            has_heading = False

        heading_deg = heading_deg % 360.0

        stamp_sec = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1.0e-9
        )

        self._pose_store.update(
            {
                'latitude': latitude,
                'longitude': longitude,
                'altitude': float(msg.pose.point.altitude),
                'has_altitude': bool(msg.pose.point.has_altitude),
                'heading_deg': heading_deg,
                'has_heading': has_heading,
                'status_text': str(msg.status_text),
                'fix_quality': int(msg.fix_quality),
                'fusion_status': int(msg.fusion_status),
                'source': int(msg.source),
                'stamp': stamp_sec,
                'frame_id': str(msg.header.frame_id),
                'child_frame_id': str(msg.pose.child_frame_id),
            }
        )

    def _serve_http(self) -> None:
        """OpenStreetMapビューア用HTTPサーバを起動する."""
        node = self

        class Handler(BaseHTTPRequestHandler):
            """llh_osm_viewer用HTTPハンドラ."""

            def do_GET(self) -> None:  # noqa: N802
                """GETリクエストを処理する."""
                if self.path == '/' or self.path.startswith('/index.html'):
                    self._send_html()
                    return

                if self.path.startswith('/pose'):
                    self._send_pose()
                    return

                self.send_response(404)
                self.end_headers()

            def log_message(self, fmt: str, *args: Any) -> None:
                """HTTPアクセスログを抑制する."""
                return

            def _send_html(self) -> None:
                """ビューアHTMLを返す."""
                html = _make_html(
                    poll_interval_ms=node._poll_interval_ms,
                    initial_zoom=node._initial_zoom,
                    default_latitude=node._default_latitude,
                    default_longitude=node._default_longitude,
                    default_zoom=node._default_zoom,
                    triangle_height_px=node._triangle_height_px,
                )
                body = html.encode('utf-8')

                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def _send_pose(self) -> None:
                """最新姿勢をJSONで返す."""
                body = json.dumps(
                    {'pose': node._pose_store.get()},
                    ensure_ascii=False,
                ).encode('utf-8')

                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

        try:
            self._http_server = ThreadingHTTPServer(
                (self._http_host, self._http_port),
                Handler,
            )
            self._http_server.serve_forever()
        except OSError as exc:
            self.get_logger().error(
                'Failed to start llh_osm_viewer HTTP server: '
                f'{exc}'
            )


def _make_html(
    poll_interval_ms: int,
    initial_zoom: int,
    default_latitude: float,
    default_longitude: float,
    default_zoom: int,
    triangle_height_px: float,
) -> str:
    """LeafletでOpenStreetMap表示を行うHTMLを生成する."""
    config_json = json.dumps(
        {
            'pollIntervalMs': poll_interval_ms,
            'initialZoom': initial_zoom,
            'defaultLatitude': default_latitude,
            'defaultLongitude': default_longitude,
            'defaultZoom': default_zoom,
            'triangleHeightPx': triangle_height_px,
        },
        ensure_ascii=False,
    )

    return f"""<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <title>LLH OpenStreetMap Viewer</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">

  <link
    rel="stylesheet"
    href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    crossorigin=""
  />
  <script
    src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
    crossorigin="">
  </script>

  <style>
    html, body {{
      height: 100%;
      margin: 0;
      font-family: sans-serif;
    }}

    #map {{
      height: 100%;
      width: 100%;
    }}

    #status {{
      position: absolute;
      z-index: 1000;
      left: 12px;
      bottom: 12px;
      min-width: 300px;
      padding: 10px 12px;
      background: rgba(255, 255, 255, 0.92);
      border-radius: 8px;
      box-shadow: 0 1px 8px rgba(0, 0, 0, 0.25);
      font-size: 13px;
      line-height: 1.45;
    }}
  </style>
</head>

<body>
<div id="map"></div>
<div id="status">Waiting for /localization/pose_llh ...</div>

<script>
'use strict';

const CONFIG = {config_json};

const map = L.map('map').setView(
  [CONFIG.defaultLatitude, CONFIG.defaultLongitude],
  CONFIG.defaultZoom
);

L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
  maxZoom: 22,
  attribution: '&copy; OpenStreetMap contributors'
}}).addTo(map);

const statusBox = document.getElementById('status');

let triangleLayer = null;
let centerMarker = null;
let firstPoseReceived = false;
let latestPose = null;

/**
 * heading_degを地図上の単位ベクトルに変換する。
 *
 * Leafletのピクセル座標は x:右向き, y:下向き。
 * heading_degは真北基準・時計回りなので、
 *   0deg: 画面上方向
 *  90deg: 画面右方向
 */
function headingToPixelUnit(headingDeg) {{
  const rad = headingDeg * Math.PI / 180.0;
  return {{
    x: Math.sin(rad),
    y: -Math.cos(rad)
  }};
}}

/**
 * 底辺:高さ = 1:2 の二等辺三角形を作る。
 *
 * heightPx = 頂角から底辺中央までの長さ
 * basePx = heightPx / 2
 *
 * 頂角 -> 底辺中央の向きが heading_deg。
 * つまり、三角形の先端が進行方向を向く。
 */
function makeTriangleLatLngs(lat, lon, headingDeg) {{
  const center = L.latLng(lat, lon);
  const centerPt = map.latLngToLayerPoint(center);

  const heightPx = CONFIG.triangleHeightPx;
  const basePx = heightPx / 2.0;
  const halfBasePx = basePx / 2.0;

  const forward = headingToPixelUnit(headingDeg);
  const right = {{
    x: -forward.y,
    y: forward.x
  }};

  // 現在位置を三角形の重心付近に置く。
  // 頂点A=(+2/3 h), 底辺B/C=(-1/3 h) とすれば重心が中心になる。
  const apex = L.point(
    centerPt.x + forward.x * (heightPx * 2.0 / 3.0),
    centerPt.y + forward.y * (heightPx * 2.0 / 3.0)
  );

  const baseCenter = L.point(
    centerPt.x - forward.x * (heightPx / 3.0),
    centerPt.y - forward.y * (heightPx / 3.0)
  );

  const leftBase = L.point(
    baseCenter.x - right.x * halfBasePx,
    baseCenter.y - right.y * halfBasePx
  );

  const rightBase = L.point(
    baseCenter.x + right.x * halfBasePx,
    baseCenter.y + right.y * halfBasePx
  );

  return [
    map.layerPointToLatLng(apex),
    map.layerPointToLatLng(leftBase),
    map.layerPointToLatLng(rightBase)
  ];
}}

function redrawPose() {{
  if (latestPose === null) {{
    return;
  }}

  const lat = latestPose.latitude;
  const lon = latestPose.longitude;
  const headingDeg = latestPose.has_heading ? latestPose.heading_deg : 0.0;

  const latLngs = makeTriangleLatLngs(lat, lon, headingDeg);

  if (triangleLayer === null) {{
    triangleLayer = L.polygon(latLngs, {{
      color: '#d62728',
      weight: 2,
      fillColor: '#d62728',
      fillOpacity: 0.65
    }}).addTo(map);
  }} else {{
    triangleLayer.setLatLngs(latLngs);
  }}

  if (centerMarker === null) {{
    centerMarker = L.circleMarker([lat, lon], {{
      radius: 4,
      color: '#000000',
      weight: 1,
      fillColor: '#ffffff',
      fillOpacity: 1.0
    }}).addTo(map);
  }} else {{
    centerMarker.setLatLng([lat, lon]);
  }}

  if (!firstPoseReceived) {{
    map.setView([lat, lon], CONFIG.initialZoom);
    firstPoseReceived = true;
  }}
}}

function updateStatus(pose) {{
  const headingText = pose.has_heading
    ? `${{pose.heading_deg.toFixed(1)}} deg`
    : 'N/A';

  const altitudeText = pose.has_altitude
    ? `${{pose.altitude.toFixed(2)}} m`
    : 'N/A';

  statusBox.innerHTML =
    `<b>/localization/pose_llh</b><br>` +
    `lat: ${{pose.latitude.toFixed(8)}}<br>` +
    `lon: ${{pose.longitude.toFixed(8)}}<br>` +
    `alt: ${{altitudeText}}<br>` +
    `heading: ${{headingText}}<br>` +
    `status: ${{pose.status_text || ''}}<br>` +
    `source: ${{pose.source}}, fix: ${{pose.fix_quality}}, fusion: ${{pose.fusion_status}}<br>` +
    `frame: ${{pose.frame_id || ''}} / ${{pose.child_frame_id || ''}}`;
}}

async function pollPose() {{
  try {{
    const response = await fetch('/pose', {{ cache: 'no-store' }});
    const data = await response.json();

    if (data.pose !== null) {{
      latestPose = data.pose;
      redrawPose();
      updateStatus(data.pose);
    }}
  }} catch (error) {{
    statusBox.textContent = `Failed to fetch pose: ${{error}}`;
  }}
}}

setInterval(pollPose, CONFIG.pollIntervalMs);

// ズームやパンでピクセル座標系が変わるため、三角形を再計算する。
map.on('zoomend moveend', () => {{
  redrawPose();
}});

pollPose();
</script>
</body>
</html>
"""


def main(args: Optional[list[str]] = None) -> None:
    """エントリポイント."""
    rclpy.init(args=args)
    node = LlhOsmViewerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
