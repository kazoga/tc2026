#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""OSM の歩行路を bbox で取得し、ブラウザ上で選択・座標取得できるビューア。

機能:
- 始点(lat1, lon1)と終点(lat2, lon2)を対角とする bbox を作成
- bbox 内の歩行路 way を Overpass API から取得
- ローカル HTML ページを起動して Leaflet で描画
- 線分クリックで詳細表示
- 選択された線分の XML / JSON / CSV をダウンロード可能
- 選択された線分の緯度経度列を画面表示

注意:
- インターネット接続が必要です
- 地図表示には Leaflet CDN を利用します
- Overpass API の混雑状況によっては応答が遅い場合があります
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import threading
import urllib.parse
import webbrowser
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Sequence, Tuple

import requests


OVERPASS_URL = "https://overpass-api.de/api/interpreter"


@dataclass(frozen=True)
class BoundingBox:
    """緯度経度の bbox を表す。"""

    south: float
    west: float
    north: float
    east: float

    @property
    def center(self) -> Tuple[float, float]:
        """bbox 中心を返す。"""
        return ((self.south + self.north) / 2.0, (self.west + self.east) / 2.0)


def build_bbox(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
) -> BoundingBox:
    """2点を対角とする bbox を返す。"""
    return BoundingBox(
        south=min(lat1, lat2),
        west=min(lon1, lon2),
        north=max(lat1, lat2),
        east=max(lon1, lon2),
    )


def build_overpass_query(bbox: BoundingBox, highway_values: Sequence[str]) -> str:
    """Overpass QL を生成する。"""
    escaped_values = "|".join(highway_values)
    return f"""
[out:json][timeout:25];
(
  way["highway"~"^{escaped_values}$"]({bbox.south},{bbox.west},{bbox.north},{bbox.east});
);
out tags geom;
""".strip()


def fetch_pedestrian_ways(
    bbox: BoundingBox,
    highway_values: Sequence[str],
    timeout_sec: int = 60,
) -> Dict:
    """Overpass API から歩行路 way を取得する。"""
    query = build_overpass_query(bbox, highway_values)
    response = requests.get(
        OVERPASS_URL,
        params={"data": query},
        timeout=timeout_sec,
        headers={"User-Agent": "osm_pedestrian_bbox_viewer/1.0"},
    )
    response.raise_for_status()
    payload = response.json()

    features: List[Dict] = []
    for element in payload.get("elements", []):
        if element.get("type") != "way":
            continue
        geometry = element.get("geometry", [])
        if len(geometry) < 2:
            continue

        features.append(
            {
                "id": element["id"],
                "type": "way",
                "tags": element.get("tags", {}),
                "geometry": geometry,
            }
        )

    return {
        "bbox": {
            "south": bbox.south,
            "west": bbox.west,
            "north": bbox.north,
            "east": bbox.east,
        },
        "center": list(bbox.center),
        "highway_values": list(highway_values),
        "feature_count": len(features),
        "features": features,
    }


def way_to_osm_xml(feature: Dict) -> str:
    """1 本の way を簡易 OSM XML として出力する。"""
    geometry = feature["geometry"]
    tags = feature.get("tags", {})
    way_id = int(feature["id"])

    lines: List[str] = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<osm version="0.6" generator="osm_pedestrian_bbox_viewer">',
    ]

    node_ids: List[int] = []
    for index, point in enumerate(geometry, start=1):
        node_id = -index
        node_ids.append(node_id)
        lat = point["lat"]
        lon = point["lon"]
        lines.append(f'  <node id="{node_id}" lat="{lat}" lon="{lon}" />')

    lines.append(f'  <way id="{way_id}">')
    for node_id in node_ids:
        lines.append(f'    <nd ref="{node_id}" />')
    for key, value in tags.items():
        escaped_key = xml_escape(str(key))
        escaped_value = xml_escape(str(value))
        lines.append(f'    <tag k="{escaped_key}" v="{escaped_value}" />')
    lines.append("  </way>")
    lines.append("</osm>")
    lines.append("")
    return "\n".join(lines)


def way_to_csv(feature: Dict) -> str:
    """1 本の way を CSV 化する。"""
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(["index", "lat", "lon"])
    for index, point in enumerate(feature["geometry"]):
        writer.writerow([index, point["lat"], point["lon"]])
    return output.getvalue()


def xml_escape(text: str) -> str:
    """XML エスケープを行う。"""
    return (
        text.replace("&", "&amp;")
        .replace('"', "&quot;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace("'", "&apos;")
    )


def build_html_page(initial_data: Dict) -> str:
    """表示用 HTML を生成する。"""
    data_json = json.dumps(initial_data, ensure_ascii=False)

    return f"""<!DOCTYPE html>
<html lang="ja">
<head>
  <meta charset="utf-8" />
  <title>OSM Pedestrian BBox Viewer</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />

  <link
    rel="stylesheet"
    href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY="
    crossorigin=""
  />
  <script
    src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
    integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo="
    crossorigin=""
  ></script>

  <style>
    html, body {{
      margin: 0;
      padding: 0;
      height: 100%;
      font-family: sans-serif;
    }}

    #app {{
      display: grid;
      grid-template-columns: 1fr 360px;
      height: 100%;
    }}

    #map {{
      height: 100%;
      width: 100%;
    }}

    #sidebar {{
      border-left: 1px solid #ccc;
      padding: 12px;
      overflow: auto;
      background: #fafafa;
    }}

    h1 {{
      font-size: 18px;
      margin-top: 0;
    }}

    .meta {{
      font-size: 13px;
      line-height: 1.5;
      margin-bottom: 12px;
      padding: 8px;
      background: #fff;
      border: 1px solid #ddd;
    }}

    .section {{
      margin-bottom: 16px;
      background: #fff;
      border: 1px solid #ddd;
      padding: 10px;
    }}

    .button-row {{
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 10px;
      margin-bottom: 10px;
    }}

    button {{
      padding: 8px 10px;
      cursor: pointer;
      border: 1px solid #999;
      background: #f5f5f5;
      border-radius: 4px;
    }}

    button:hover {{
      background: #eaeaea;
    }}

    pre {{
      white-space: pre-wrap;
      word-break: break-word;
      background: #f7f7f7;
      border: 1px solid #ddd;
      padding: 8px;
      font-size: 12px;
      max-height: 260px;
      overflow: auto;
    }}

    table {{
      border-collapse: collapse;
      width: 100%;
      font-size: 12px;
    }}

    th, td {{
      border: 1px solid #ddd;
      padding: 6px;
      text-align: left;
      vertical-align: top;
    }}

    .small {{
      font-size: 12px;
      color: #555;
    }}

    .selected-badge {{
      display: inline-block;
      padding: 2px 6px;
      background: #d22;
      color: white;
      border-radius: 999px;
      font-size: 11px;
      margin-left: 6px;
    }}
  </style>
</head>
<body>
  <div id="app">
    <div id="map"></div>
    <div id="sidebar">
      <h1>OSM 歩行路ビューア</h1>

      <div class="meta">
        <div><strong>取得件数:</strong> <span id="feature-count"></span></div>
        <div><strong>bbox:</strong> <span id="bbox-text"></span></div>
        <div><strong>highway:</strong> <span id="highway-text"></span></div>
        <div class="small">線分をクリックすると詳細とダウンロードボタンが表示されます。</div>
      </div>

      <div class="section" id="selection-section">
        <div id="selection-empty">まだ線分が選択されていません。</div>
        <div id="selection-content" style="display:none;">
          <div id="selection-title"></div>
          <div class="button-row" id="download-buttons"></div>

          <h3>タグ</h3>
          <pre id="tags-view"></pre>

          <h3>緯度経度一覧</h3>
          <pre id="coords-view"></pre>
        </div>
      </div>
    </div>
  </div>

  <script>
    const INITIAL_DATA = {data_json};

    const map = L.map('map');
    const featureLayers = new Map();
    let selectedWayId = null;

    const defaultStyle = {{
      color: '#d11',
      weight: 4,
      opacity: 0.85
    }};

    const selectedStyle = {{
      color: '#0057ff',
      weight: 6,
      opacity: 1.0
    }};

    function setMeta(data) {{
      document.getElementById('feature-count').textContent = data.feature_count;
      const bbox = data.bbox;
      document.getElementById('bbox-text').textContent =
        `south=${{bbox.south}}, west=${{bbox.west}}, north=${{bbox.north}}, east=${{bbox.east}}`;
      document.getElementById('highway-text').textContent = data.highway_values.join(', ');
    }}

    function escapeHtml(text) {{
      return text
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;')
        .replaceAll("'", '&#39;');
    }}

    function downloadText(filename, content, mimeType) {{
      const blob = new Blob([content], {{ type: mimeType }});
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = filename;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
    }}

    async function fetchDownload(path) {{
      const response = await fetch(path);
      if (!response.ok) {{
        throw new Error(`download failed: ${{response.status}}`);
      }}
      return await response.text();
    }}

    function showSelectedFeature(feature) {{
      selectedWayId = feature.id;

      for (const [wayId, layer] of featureLayers.entries()) {{
        layer.setStyle(wayId === feature.id ? selectedStyle : defaultStyle);
      }}

      const title = document.getElementById('selection-title');
      title.innerHTML = `<strong>way ${{feature.id}}</strong><span class="selected-badge">選択中</span>`;

      document.getElementById('selection-empty').style.display = 'none';
      document.getElementById('selection-content').style.display = '';

      const tags = feature.tags || {{}};
      document.getElementById('tags-view').textContent = JSON.stringify(tags, null, 2);

      const coordsText = feature.geometry
        .map((p, i) => `${{i}}: lat=${{p.lat}}, lon=${{p.lon}}`)
        .join('\\n');
      document.getElementById('coords-view').textContent = coordsText;

      const buttonRow = document.getElementById('download-buttons');
      buttonRow.innerHTML = '';

      const xmlButton = document.createElement('button');
      xmlButton.textContent = 'XML ダウンロード';
      xmlButton.onclick = async () => {{
        const text = await fetchDownload(`/download/xml?way_id=${{feature.id}}`);
        downloadText(`way_${{feature.id}}.osm.xml`, text, 'application/xml;charset=utf-8');
      }};
      buttonRow.appendChild(xmlButton);

      const jsonButton = document.createElement('button');
      jsonButton.textContent = 'JSON ダウンロード';
      jsonButton.onclick = async () => {{
        const text = await fetchDownload(`/download/json?way_id=${{feature.id}}`);
        downloadText(`way_${{feature.id}}.json`, text, 'application/json;charset=utf-8');
      }};
      buttonRow.appendChild(jsonButton);

      const csvButton = document.createElement('button');
      csvButton.textContent = 'CSV ダウンロード';
      csvButton.onclick = async () => {{
        const text = await fetchDownload(`/download/csv?way_id=${{feature.id}}`);
        downloadText(`way_${{feature.id}}.csv`, text, 'text/csv;charset=utf-8');
      }};
      buttonRow.appendChild(csvButton);
    }}

    function initMap(data) {{
      L.tileLayer('https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
        maxZoom: 21,
        attribution: '&copy; OpenStreetMap contributors'
      }}).addTo(map);

      const bbox = data.bbox;
      const bounds = [
        [bbox.south, bbox.west],
        [bbox.north, bbox.east]
      ];

      if (data.features.length > 0) {{
        const allLatLngs = [];
        for (const feature of data.features) {{
          const latlngs = feature.geometry.map(p => [p.lat, p.lon]);
          latlngs.forEach(ll => allLatLngs.push(ll));

          const polyline = L.polyline(latlngs, defaultStyle).addTo(map);
          polyline.bindTooltip(`way ${{feature.id}}`, {{ sticky: true }});
          polyline.on('click', () => showSelectedFeature(feature));
          featureLayers.set(feature.id, polyline);
        }}

        const featureBounds = L.latLngBounds(allLatLngs);
        map.fitBounds(featureBounds.pad(0.05));
      }} else {{
        map.fitBounds(bounds);
      }}

      L.rectangle(bounds, {{
        color: '#333',
        weight: 1,
        fill: false,
        dashArray: '4,4'
      }}).addTo(map);
    }}

    setMeta(INITIAL_DATA);
    initMap(INITIAL_DATA);
  </script>
</body>
</html>
"""


class AppState:
    """HTTP サーバが参照するアプリ状態。"""

    def __init__(self, data: Dict, html: str) -> None:
        self.data = data
        self.html = html
        self.feature_by_id = {
            int(feature["id"]): feature for feature in self.data.get("features", [])
        }

    def get_feature(self, way_id: int) -> Dict:
        """way_id に対応する feature を返す。"""
        if way_id not in self.feature_by_id:
            raise KeyError(f"way_id not found: {way_id}")
        return self.feature_by_id[way_id]


def make_handler(app_state: AppState):
    """状態を閉じ込めた HTTP handler を生成する。"""

    class RequestHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            parsed = urllib.parse.urlparse(self.path)

            if parsed.path == "/":
                self._send_text(
                    200,
                    app_state.html,
                    "text/html; charset=utf-8",
                )
                return

            if parsed.path.startswith("/download/"):
                try:
                    self._handle_download(parsed)
                except KeyError:
                    self._send_text(404, "way_id not found", "text/plain; charset=utf-8")
                except ValueError:
                    self._send_text(
                        400,
                        "invalid way_id",
                        "text/plain; charset=utf-8",
                    )
                return

            self._send_text(404, "Not Found", "text/plain; charset=utf-8")

        def log_message(self, format: str, *args) -> None:
            """標準出力のアクセスログを抑制する。"""
            return

        def _handle_download(self, parsed: urllib.parse.ParseResult) -> None:
            query = urllib.parse.parse_qs(parsed.query)
            if "way_id" not in query or not query["way_id"]:
                raise ValueError("missing way_id")

            way_id = int(query["way_id"][0])
            feature = app_state.get_feature(way_id)

            if parsed.path == "/download/xml":
                content = way_to_osm_xml(feature)
                self._send_text(
                    200,
                    content,
                    "application/xml; charset=utf-8",
                )
                return

            if parsed.path == "/download/json":
                content = json.dumps(feature, ensure_ascii=False, indent=2)
                self._send_text(
                    200,
                    content,
                    "application/json; charset=utf-8",
                )
                return

            if parsed.path == "/download/csv":
                content = way_to_csv(feature)
                self._send_text(
                    200,
                    content,
                    "text/csv; charset=utf-8",
                )
                return

            self._send_text(404, "Not Found", "text/plain; charset=utf-8")

        def _send_text(self, status: int, body: str, content_type: str) -> None:
            encoded = body.encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)

    return RequestHandler


def parse_highway_values(text: str) -> List[str]:
    """カンマ区切り highway 指定を配列へ変換する。"""
    values = [item.strip() for item in text.split(",") if item.strip()]
    if not values:
        raise ValueError("highway values must not be empty")
    return values


def parse_args() -> argparse.Namespace:
    """CLI 引数を解釈する。"""
    parser = argparse.ArgumentParser(
        description="bbox 内の OSM 歩行路を取得してブラウザ表示する"
    )
    parser.add_argument("--lat1", type=float, required=True, help="始点緯度")
    parser.add_argument("--lon1", type=float, required=True, help="始点経度")
    parser.add_argument("--lat2", type=float, required=True, help="終点緯度")
    parser.add_argument("--lon2", type=float, required=True, help="終点経度")
    parser.add_argument(
        "--highway",
        type=str,
        default="footway,path,pedestrian,steps",
        help="取得する highway 値のカンマ区切り指定",
    )
    parser.add_argument(
        "--host",
        type=str,
        default="127.0.0.1",
        help="HTTP サーバの listen host",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="HTTP サーバの listen port",
    )
    parser.add_argument(
        "--open-browser",
        action="store_true",
        help="起動後にブラウザを自動で開く",
    )
    return parser.parse_args()


def main() -> None:
    """エントリポイント。"""
    args = parse_args()
    highway_values = parse_highway_values(args.highway)

    bbox = build_bbox(args.lat1, args.lon1, args.lat2, args.lon2)
    data = fetch_pedestrian_ways(bbox, highway_values)
    html = build_html_page(data)
    app_state = AppState(data, html)

    handler = make_handler(app_state)
    server = ThreadingHTTPServer((args.host, args.port), handler)
    url = f"http://{args.host}:{args.port}/"

    print("OSM pedestrian bbox viewer")
    print(f"URL: {url}")
    print(f"feature_count: {data['feature_count']}")
    print(
        "bbox: "
        f"south={bbox.south}, west={bbox.west}, north={bbox.north}, east={bbox.east}"
    )
    print(f"highway: {', '.join(highway_values)}")

    if args.open_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
