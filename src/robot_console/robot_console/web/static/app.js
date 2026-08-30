// robot_console HTML遠隔観測UI（閲覧専用）。
//
// /snapshot.json をポーリングして状態サマリ・GPS・センサ一覧・鮮度一覧・地図
// マーカーを更新し、/images/{panel_id} を別周期でポーリングして画像を更新する。
// 本ページは観測専用であり、書き込み系のfetch（POST/PUT/DELETE）は行わない。
// 地図はLeaflet + OpenStreetMapタイルを使うため、閲覧時にインターネット接続が
// 必要（robot_console_ui_renewal_input.md 8.4節 候補A）。waypoint列・route
// polylineはmap_model（未実装）が確定するまでの間表示できないため、現在位置・
// 目標位置のマーカーのみ表示する。

const SNAPSHOT_POLL_MS = 1000;
const IMAGE_POLL_MS = 1500;

const DEFAULT_LATITUDE = 36.083;
const DEFAULT_LONGITUDE = 140.113;
const DEFAULT_ZOOM = 18;

const FRESHNESS_COLORS = {
  OK: '#2e7d32',
  STALE: '#f9a825',
  LOST: '#c62828',
  UNKNOWN: '#757575',
};

let knownPanelIds = [];
let leafletMap = null;
let currentPositionMarker = null;
let targetPositionMarker = null;
let hasCenteredMap = false;

function freshnessColor(level) {
  return FRESHNESS_COLORS[level] || FRESHNESS_COLORS.UNKNOWN;
}

function initMap() {
  leafletMap = L.map('leaflet-map').setView([DEFAULT_LATITUDE, DEFAULT_LONGITUDE], DEFAULT_ZOOM);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; OpenStreetMap contributors',
  }).addTo(leafletMap);

  // グリッドレイアウト内では初期化時にコンテナの実寸がまだ確定していないことが
  // あり、その場合Leafletがタイルを1枚も要求しない。少し遅らせてサイズを
  // 再計算させることで確実にタイルを読み込ませる。
  setTimeout(() => leafletMap.invalidateSize(), 0);
  window.addEventListener('resize', () => leafletMap.invalidateSize());
}

function updateMapMarkers(snapshot) {
  if (leafletMap === null) {
    return;
  }

  const current = snapshot.localization;
  if (current.latitude !== null && current.longitude !== null) {
    const latlng = [current.latitude, current.longitude];
    if (currentPositionMarker === null) {
      currentPositionMarker = L.circleMarker(latlng, {
        radius: 8,
        color: '#4fc3f7',
        fillColor: '#4fc3f7',
        fillOpacity: 0.9,
      })
        .bindTooltip('現在位置')
        .addTo(leafletMap);
    } else {
      currentPositionMarker.setLatLng(latlng);
    }
    if (!hasCenteredMap) {
      leafletMap.setView(latlng, DEFAULT_ZOOM);
      hasCenteredMap = true;
    }
  }

  const target = snapshot.target;
  if (target.latitude !== null && target.longitude !== null) {
    const latlng = [target.latitude, target.longitude];
    if (targetPositionMarker === null) {
      targetPositionMarker = L.circleMarker(latlng, {
        radius: 6,
        color: '#f9a825',
        fillColor: '#f9a825',
        fillOpacity: 0.9,
      })
        .bindTooltip('目標waypoint')
        .addTo(leafletMap);
    } else {
      targetPositionMarker.setLatLng(latlng);
    }
  }
}

function appendField(dl, label, value) {
  const dt = document.createElement('dt');
  dt.textContent = label;
  const dd = document.createElement('dd');
  dd.textContent = value;
  dl.appendChild(dt);
  dl.appendChild(dd);
}

function renderSummary(snapshot) {
  const fields = document.getElementById('summary-fields');
  fields.innerHTML = '';
  appendField(fields, '運行フェーズ', snapshot.operation.phase);
  appendField(
    fields,
    '業務モード',
    `${snapshot.operation.environment} / ${snapshot.operation.drive_mode}`,
  );
  appendField(fields, '進捗', `${(snapshot.operation.route_progress * 100).toFixed(1)}%`);
  appendField(
    fields,
    'WP',
    `${snapshot.operation.current_waypoint || '-'} -> ${snapshot.operation.next_waypoint || '-'}`,
  );
  appendField(fields, 'route_follower', snapshot.follower.state);
  appendField(fields, 'localization source', snapshot.localization.source);
}

function renderGpsSummary(snapshot) {
  const fields = document.getElementById('gps-fields');
  fields.innerHTML = '';
  appendField(fields, 'RTK', snapshot.gps.rtk_state);
  appendField(fields, 'Satellites', `${snapshot.gps.num_satellites} sat`);
  appendField(fields, 'HDOP', snapshot.gps.hdop.toFixed(2));
  appendField(fields, 'Correction', `${snapshot.gps.correction_age_s.toFixed(2)} s`);
  appendField(fields, 'Heading', `${snapshot.gps.heading_deg.toFixed(1)} deg`);
  appendField(fields, 'Localization freshness', snapshot.localization.freshness);
}

function renderSensorGrid(allPanels) {
  // route_mapは専用のLeaflet地図で表示するため、センサ・画像パネルの
  // グリッドには含めない（PyQt5側のlocalization_sensor_tab.pyと同様の扱い）。
  const panels = allPanels.filter((panel) => panel.panel_id !== 'route_map');
  const grid = document.getElementById('sensor-grid');
  const panelIds = panels.map((panel) => panel.panel_id);
  const sameOrder =
    panelIds.length === knownPanelIds.length &&
    panelIds.every((id, index) => id === knownPanelIds[index]);

  if (!sameOrder) {
    grid.innerHTML = '';
    for (const panel of panels) {
      const card = document.createElement('figure');
      card.className = 'sensor-card';

      const img = document.createElement('img');
      img.id = `panel-image-${panel.panel_id}`;
      img.alt = panel.title;

      const caption = document.createElement('figcaption');
      caption.id = `panel-caption-${panel.panel_id}`;

      card.appendChild(img);
      card.appendChild(caption);
      grid.appendChild(card);
    }
    knownPanelIds = panelIds;
  }

  for (const panel of panels) {
    const caption = document.getElementById(`panel-caption-${panel.panel_id}`);
    if (caption === null) {
      continue;
    }
    caption.textContent = `${panel.title} / ${panel.topic || '-'} / ${panel.freshness}`;
    caption.style.color = freshnessColor(panel.freshness);
  }
}

function renderHealthTable(profiles) {
  const tbody = document.querySelector('#health-table tbody');
  tbody.innerHTML = '';
  for (const item of profiles) {
    const row = document.createElement('tr');

    const profileCell = document.createElement('td');
    profileCell.textContent = item.profile_id;
    const categoryCell = document.createElement('td');
    categoryCell.textContent = item.category;
    const statusCell = document.createElement('td');
    statusCell.textContent = item.status;
    const healthCell = document.createElement('td');
    healthCell.textContent = item.health;
    healthCell.style.color = freshnessColor(item.health);

    row.appendChild(profileCell);
    row.appendChild(categoryCell);
    row.appendChild(statusCell);
    row.appendChild(healthCell);
    tbody.appendChild(row);
  }
}

function renderMapCaption(snapshot) {
  const caption = document.getElementById('map-caption');
  const source = snapshot.localization.source;
  const freshness = snapshot.localization.freshness;
  caption.textContent = `source: ${source} / freshness: ${freshness}`;
  caption.style.color = freshnessColor(freshness);
}

async function pollSnapshot() {
  try {
    const response = await fetch('/snapshot.json', { cache: 'no-store' });
    if (response.ok) {
      const snapshot = await response.json();
      renderSummary(snapshot);
      renderGpsSummary(snapshot);
      renderMapCaption(snapshot);
      updateMapMarkers(snapshot);
      renderSensorGrid(snapshot.sensor_panels);
      renderHealthTable(snapshot.health);
    }
  } catch (error) {
    console.error('snapshot取得に失敗しました', error);
  } finally {
    setTimeout(pollSnapshot, SNAPSHOT_POLL_MS);
  }
}

function pollImages() {
  const cacheBuster = Date.now();
  for (const panelId of knownPanelIds) {
    const img = document.getElementById(`panel-image-${panelId}`);
    if (img !== null) {
      img.src = `/images/${panelId}?t=${cacheBuster}`;
    }
  }
  setTimeout(pollImages, IMAGE_POLL_MS);
}

initMap();
pollSnapshot();
pollImages();
