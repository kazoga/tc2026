"""PyQt5側の地図表示Widget（`robot_console_ui_renewal_input.md` 8.4節 候補A）。

`QWebEngineView` にLeaflet + OpenStreetMapタイルの地図HTMLを埋め込み、現在位置・
目標waypointをマーカー表示する。HTML遠隔観測UI（`web/static/app.js`）と同じ
Leaflet 1.9.4 + OSMタイルを用いて表示内容を揃える。waypoint列・route区間の重畳は
map_model（未実装）が確定するまでの間表示できないため、現在位置・目標waypointの
マーカーのみ表示する。
"""

from __future__ import annotations

import json
from typing import Optional

from PyQt5 import QtWidgets
from PyQt5.QtWebEngineWidgets import QWebEngineView

from robot_console.core.snapshot_model import LocalizationStateView, TargetView

DEFAULT_LATITUDE = 36.083
DEFAULT_LONGITUDE = 140.113
DEFAULT_ZOOM = 18

_MAP_HTML_TEMPLATE = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" crossorigin="">
<style>
html, body, #map { margin: 0; padding: 0; width: 100%; height: 100%; background: #000; }
</style>
</head>
<body>
<div id="map"></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" crossorigin=""></script>
<script>
var DEFAULT_ZOOM = __DEFAULT_ZOOM__;
var map = L.map('map').setView([__DEFAULT_LAT__, __DEFAULT_LON__], DEFAULT_ZOOM);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
  attribution: '&copy; OpenStreetMap contributors',
}).addTo(map);

var currentMarker = null;
var targetMarker = null;
var hasCentered = false;

function updateMarkers(current, target) {
  if (current && current.latitude !== null && current.longitude !== null) {
    var latlng = [current.latitude, current.longitude];
    if (currentMarker === null) {
      currentMarker = L.circleMarker(latlng, {
        radius: 8, color: '#4fc3f7', fillColor: '#4fc3f7', fillOpacity: 0.9,
      }).bindTooltip('現在位置').addTo(map);
    } else {
      currentMarker.setLatLng(latlng);
    }
    if (!hasCentered) {
      map.setView(latlng, DEFAULT_ZOOM);
      hasCentered = true;
    }
  }
  if (target && target.latitude !== null && target.longitude !== null) {
    var tlatlng = [target.latitude, target.longitude];
    if (targetMarker === null) {
      targetMarker = L.circleMarker(tlatlng, {
        radius: 6, color: '#f9a825', fillColor: '#f9a825', fillOpacity: 0.9,
      }).bindTooltip('目標waypoint').addTo(map);
    } else {
      targetMarker.setLatLng(tlatlng);
    }
  }
}

setTimeout(function () { map.invalidateSize(); }, 0);
window.addEventListener('resize', function () { map.invalidateSize(); });
</script>
</body>
</html>
"""


def build_map_html(
    *,
    default_latitude: float = DEFAULT_LATITUDE,
    default_longitude: float = DEFAULT_LONGITUDE,
    default_zoom: int = DEFAULT_ZOOM,
) -> str:
    """地図初期表示用のHTML文字列を組み立てる。"""

    return (
        _MAP_HTML_TEMPLATE.replace('__DEFAULT_LAT__', str(default_latitude))
        .replace('__DEFAULT_LON__', str(default_longitude))
        .replace('__DEFAULT_ZOOM__', str(default_zoom))
    )


def build_update_markers_script(
    localization: LocalizationStateView,
    target: TargetView,
) -> str:
    """自己位置・目標waypointをLeaflet地図へ反映するJavaScriptを組み立てる。"""

    current_payload = {
        'latitude': localization.latitude,
        'longitude': localization.longitude,
    }
    target_payload = {
        'latitude': target.latitude,
        'longitude': target.longitude,
    }
    return 'updateMarkers({}, {});'.format(json.dumps(current_payload), json.dumps(target_payload))


class MapView(QWebEngineView):
    """LeafletベースのOSM地図をPyQt5画面へ埋め込むWidget。"""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.setHtml(build_map_html())

    def update_map(self, localization: LocalizationStateView, target: TargetView) -> None:
        """`ConsoleSnapshot` の自己位置・目標waypointをマーカーへ反映する。"""

        self.page().runJavaScript(build_update_markers_script(localization, target))
