"""PyQt5側の地図表示Widget（`robot_console_ui_renewal_input.md` 8.4節 候補A）。

`QWebEngineView` にLeaflet + OpenStreetMapタイルの地図HTMLを埋め込み、現在位置・
目標waypoint・route waypoint列を表示する。HTML遠隔観測UI（`web/static/app.js`）と
同じLeaflet 1.9.4 + OSMタイルを用いて表示内容を揃える。waypoint列は
`RouteView.waypoints`（緯度経度を持つもののみ）から描画し、`current_index`を
境に走行済み/未走行を色分けする。
"""

from __future__ import annotations

import json
from typing import Optional

from PyQt5 import QtWidgets
from PyQt5.QtWebEngineWidgets import QWebEngineView

from robot_console.core.snapshot_model import LocalizationStateView, RouteView, TargetView

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

var routeWaypointMarkers = [];
var routeTraveledPolyline = null;
var routeUntraveledPolyline = null;
var knownRouteWaypointCount = -1;
var ROUTE_TRAVELED_COLOR = '#757575';
var ROUTE_UNTRAVELED_COLOR = '#66bb6a';

function updateRoute(waypoints, currentIndex) {
  var validWaypoints = waypoints.filter(function (waypoint) {
    return waypoint.latitude !== null && waypoint.longitude !== null;
  });

  if (validWaypoints.length !== knownRouteWaypointCount) {
    for (var i = 0; i < routeWaypointMarkers.length; i += 1) {
      map.removeLayer(routeWaypointMarkers[i]);
    }
    routeWaypointMarkers = validWaypoints.map(function (waypoint) {
      return L.circleMarker([waypoint.latitude, waypoint.longitude], {
        radius: 3, weight: 1, fillOpacity: 0.9,
      }).addTo(map);
    });
    knownRouteWaypointCount = validWaypoints.length;
  }

  for (var j = 0; j < validWaypoints.length; j += 1) {
    var waypoint = validWaypoints[j];
    var traveled = waypoint.index < currentIndex;
    var color = traveled ? ROUTE_TRAVELED_COLOR : ROUTE_UNTRAVELED_COLOR;
    routeWaypointMarkers[j].setLatLng([waypoint.latitude, waypoint.longitude]);
    routeWaypointMarkers[j].setStyle({ color: color, fillColor: color });
  }

  if (routeTraveledPolyline === null) {
    routeTraveledPolyline = L.polyline([], {
      color: ROUTE_TRAVELED_COLOR, weight: 3,
    }).addTo(map);
    routeUntraveledPolyline = L.polyline([], {
      color: ROUTE_UNTRAVELED_COLOR, weight: 3,
    }).addTo(map);
  }

  var traveledLatLngs = validWaypoints
    .filter(function (waypoint) { return waypoint.index <= currentIndex; })
    .map(function (waypoint) { return [waypoint.latitude, waypoint.longitude]; });
  var untraveledLatLngs = validWaypoints
    .filter(function (waypoint) { return waypoint.index >= currentIndex; })
    .map(function (waypoint) { return [waypoint.latitude, waypoint.longitude]; });
  routeTraveledPolyline.setLatLngs(traveledLatLngs);
  routeUntraveledPolyline.setLatLngs(untraveledLatLngs);
}

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


def build_update_route_script(route: RouteView) -> str:
    """route waypoint列をLeaflet地図へ反映するJavaScriptを組み立てる。"""

    waypoints_payload = [
        {'index': waypoint.index, 'latitude': waypoint.latitude, 'longitude': waypoint.longitude}
        for waypoint in route.waypoints
    ]
    return 'updateRoute({}, {});'.format(json.dumps(waypoints_payload), json.dumps(route.current_index))


class MapView(QWebEngineView):
    """LeafletベースのOSM地図をPyQt5画面へ埋め込むWidget。"""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self._page_loaded = False
        self.loadFinished.connect(self._on_load_finished)
        self.setHtml(build_map_html())

    def _on_load_finished(self, ok: bool) -> None:
        """`setHtml()` の非同期読み込み完了を記録する。

        読み込み完了前に`update_map`/`update_route`が呼ばれると、Leaflet側の
        `updateMarkers`/`updateRoute`関数がまだ定義されておらず
        `ReferenceError`になるため、完了までは更新を無視する
        （次回のQTimerポーリングで改めて反映される）。
        """

        self._page_loaded = bool(ok)

    def update_map(self, localization: LocalizationStateView, target: TargetView) -> None:
        """`ConsoleSnapshot` の自己位置・目標waypointをマーカーへ反映する。"""

        if not self._page_loaded:
            return
        self.page().runJavaScript(build_update_markers_script(localization, target))

    def update_route(self, route: RouteView) -> None:
        """`ConsoleSnapshot` のroute waypoint列をLeaflet地図へ反映する。"""

        if not self._page_loaded:
            return

        self.page().runJavaScript(build_update_route_script(route))
