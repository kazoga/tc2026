// robot_console HTML遠隔観測UI（閲覧専用）。
//
// /snapshot.json をポーリングして状態サマリ・GPS・センサ一覧・鮮度一覧を表示し、
// /images/{panel_id} を別周期でポーリングして画像を更新する。
// 本ページは観測専用であり、書き込み系のfetch（POST/PUT/DELETE）は行わない。

const SNAPSHOT_POLL_MS = 1000;
const IMAGE_POLL_MS = 1500;

const FRESHNESS_COLORS = {
  OK: '#2e7d32',
  STALE: '#f9a825',
  LOST: '#c62828',
  UNKNOWN: '#757575',
};

let knownPanelIds = [];

function freshnessColor(level) {
  return FRESHNESS_COLORS[level] || FRESHNESS_COLORS.UNKNOWN;
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

function renderSensorGrid(panels) {
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
  const mapImage = document.getElementById('map-image');
  mapImage.src = `/images/route_map?t=${cacheBuster}`;
  for (const panelId of knownPanelIds) {
    const img = document.getElementById(`panel-image-${panelId}`);
    if (img !== null) {
      img.src = `/images/${panelId}?t=${cacheBuster}`;
    }
  }
  setTimeout(pollImages, IMAGE_POLL_MS);
}

pollSnapshot();
pollImages();
