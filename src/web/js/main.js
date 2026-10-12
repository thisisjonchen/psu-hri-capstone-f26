'use strict';

const xySources = [
  ['groundTruth', 'Ground truth', []],
  ['ukf', 'UKF', [6, 3]],
  ['camera', 'Camera pose', [2, 3]],
  ['uwb', 'UWB + ToF', [8, 2, 2, 2]],
  ['fused', 'Fused position', []]
];

const roomFrame = frame => typeof frame === 'string' ? frame.replace(/^\/+/, '') : '';
const droneColor = id => id === 'drone1' ? '#0072b2' : '#d55e00';
function roomTick(value, index, ticks) {
  // Exact viewport limits can land very close to the first/last regular tick.
  const direction = index === 0 ? 1 : index === ticks.length - 1 ? -1 : 0;
  if (direction && ticks.length > 2 && Math.abs(value - ticks[index + direction]) <
      0.75 * Math.abs(ticks[index + direction] - ticks[index + 2 * direction])) return '';
  return Number(value.toFixed(2));
}

class FleetMap {
  constructor() {
    this.room = null;
    this.targets = {};
    this.zoom = 1;
    this.anchorReports = {};
    this.anchorStamps = {};
    this.layoutUsable = false;
    const datasets = Object.values(drones).flatMap(drone => xySources.flatMap(([key, label, dash]) =>
      [0, 1, 2].map(i => ({
        droneId: drone.id, source: key, key: key + i, label: 'Drone ' + drone.id.slice(-1) + ' · ' + label,
        data: [], borderColor: drone.id === 'drone1' ? '#0072b2' : '#d55e00',
        borderDash: dash, borderWidth: 2.5, pointRadius: 0, fill: false, lineTension: 0
      }))));
    datasets.push({ key: 'anchors', label: 'UWB anchors', data: [], borderColor: '#58677b',
      backgroundColor: '#58677b', pointStyle: 'rect', pointRadius: 5, borderDash: [4, 4],
      borderWidth: 1, fill: false, lineTension: 0 });
    Object.keys(drones).forEach(id => datasets.push({ key: 'target', targetDrone: id,
      label: 'Drone ' + id.slice(-1) + ' · Target', data: [], showLine: false,
      pointStyle: 'circle', pointRadius: 7, pointHoverRadius: 8,
      pointBackgroundColor: '#fff', pointBorderColor: droneColor(id), pointBorderWidth: 2 }));
    this.chart = new Chart(document.getElementById('fleet-xyChart').getContext('2d'), {
      type: 'line', data: { datasets }, plugins: [{
        afterLayout: chart => this.equalizeScale(chart),
        afterDatasetsDraw: chart => this.drawAnchorLabels(chart)
      }], options: {
        responsive: true, maintainAspectRatio: false, animation: { duration: 0 },
        scales: {
          xAxes: [{ id: 'room-x', type: 'linear', display: false, ticks: { min: 0, max: 1, maxRotation: 0,
            callback: roomTick },
            afterFit: scale => {
              scale.height = scale.minSize.height = 45;
              scale.paddingLeft = scale.paddingRight = 0;
            },
            scaleLabel: { display: true, labelString: 'X position (m)' } }],
          yAxes: [{ id: 'room-y', type: 'linear', display: false, ticks: { min: 0, max: 1,
            callback: roomTick },
            afterFit: scale => {
              // Reserve a steady gutter so changing tick labels cannot move the map.
              scale.width = scale.minSize.width = 55;
              scale.paddingTop = scale.paddingBottom = 8;
            },
            scaleLabel: { display: true, labelString: 'Y position (m)' } }]
        }, legend: { display: false }, onClick: event => this.previewClick(event)
      }
    });
    document.getElementById('room-fit').addEventListener('click', () => { this.zoom = 1; this.render(); });
    document.getElementById('room-zoom-in').addEventListener('click', () => this.setZoom(this.zoom * 1.25));
    document.getElementById('room-zoom-out').addEventListener('click', () => this.setZoom(this.zoom / 1.25));
    document.getElementById('target-preview').addEventListener('submit', event => {
      event.preventDefault();
      const x = document.getElementById('target-x').value;
      const y = document.getElementById('target-y').value;
      this.previewTarget(document.getElementById('target-drone').value,
        x.trim() ? Number(x) : NaN, y.trim() ? Number(y) : NaN);
    });
    document.getElementById('target-clear').addEventListener('click', () => {
      const id = document.getElementById('target-drone').value;
      drones[id].publish('cancelRoomTarget', {});
      delete this.targets[id];
      this.render();
    });
    document.getElementById('target-send').addEventListener('click', () => this.sendTarget());
    document.getElementById('target-stop').addEventListener('click', () => {
      drones[document.getElementById('target-drone').value].publish('cancelRoomTarget', {});
      document.getElementById('target-command-status').textContent = 'Target stopped.';
    });
    document.getElementById('target-drone').addEventListener('change', () => {
      const target = this.targets[document.getElementById('target-drone').value];
      document.getElementById('target-x').value = target ? target.x.toFixed(2) : '';
      document.getElementById('target-y').value = target ? target.y.toFixed(2) : '';
      this.render();
    });
    this.refreshAnchors();
  }

  makeRoom(frame, positions) {
    const margin = 0.5;
    const anchors = positions.map(anchor => ({ ...anchor }));
    const xMin = Math.min(...anchors.map(a => a.x)), xMax = Math.max(...anchors.map(a => a.x));
    const yMin = Math.min(...anchors.map(a => a.y)), yMax = Math.max(...anchors.map(a => a.y));
    const cx = anchors.reduce((sum, a) => sum + a.x, 0) / 4;
    const cy = anchors.reduce((sum, a) => sum + a.y, 0) / 4;
    anchors.sort((a, b) => Math.atan2(a.y - cy, a.x - cx) - Math.atan2(b.y - cy, b.x - cx));
    // A room outline needs four distinct convex corners, not collinear anchors.
    if (!anchors.every((a, i) => {
      const b = anchors[(i + 1) % 4], c = anchors[(i + 2) % 4];
      return (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x) > 0;
    })) throw new Error('Check the four anchor positions');
    return { frame, margin, anchors, xMin, xMax, yMin, yMax };
  }

  updateAnchors(id, message) {
    const receivedAt = Date.now();
    const stamp = message.header && message.header.stamp;
    const time = stamp && stamp.secs + stamp.nsecs / 1e9;
    const frame = roomFrame(message.header && message.header.frame_id);
    const ids = message.anchor_ids, positions = message.positions;
    try {
      if (!frame || !Number.isFinite(time) || time <= 0 || !Array.isArray(ids) || !Array.isArray(positions) ||
          ids.length !== positions.length || ids.length > 4 ||
          !ids.every(value => typeof value === 'string' && /^[0-9a-f]{4}$/i.test(value)) ||
          !positions.every(p => p && [p.x, p.y, p.z].every(Number.isFinite))) {
        throw new Error('Invalid anchor report');
      }
      const names = ids.map(value => value.toUpperCase());
      if (new Set(names).size !== names.length) throw new Error('Duplicate UWB anchor IDs');
      // Old/repeated ROS samples must not keep a stopped discovery stream alive.
      if (this.anchorStamps[id] !== undefined && time <= this.anchorStamps[id]) return;
      const anchors = positions.map((p, i) => ({ name: names[i], x: p.x, y: p.y, z: p.z }));
      const previous = this.anchorReports[id];
      const report = { frame, anchors, receivedAt };
      if (anchors.length === 4) {
        report.completeRoom = this.makeRoom(frame, anchors);
        report.completeAt = receivedAt;
      } else if (previous && previous.completeRoom && this.matchesRoom(previous.completeRoom, report)) {
        // Briefly retain the last complete observation, without refreshing its age
        // from a partial report or assembling a room from different acquisitions.
        report.completeRoom = previous.completeRoom;
        report.completeAt = previous.completeAt;
      }
      this.anchorStamps[id] = time;
      this.anchorReports[id] = report;
    } catch (error) {
      this.anchorReports[id] = { error: 'Drone ' + id.slice(-1) + ': ' + error.message, receivedAt };
    }
    this.refreshAnchors();
  }

  matchesRoom(room, report) {
    return room.frame === report.frame && report.anchors.every(anchor => {
      const known = room.anchors.find(a => a.name === anchor.name);
      return known && ['x', 'y', 'z'].every(axis => Math.abs(known[axis] - anchor[axis]) <= 0.02);
    });
  }

  refreshAnchors() {
    const wasUsable = this.layoutUsable;
    const now = Date.now();
    const live = Object.entries(this.anchorReports).filter(([id, report]) =>
      drones[id].connected && now - report.receivedAt <= 1500);
    const complete = live.filter(([, report]) => report.completeRoom && now - report.completeAt <= 1500);
    const candidate = complete.length ? complete[0][1].completeRoom : null;
    const bad = live.find(([, report]) => report.error || (candidate && !this.matchesRoom(candidate, report)));
    const calibrating = calibrationActive();
    this.layoutUsable = !!candidate && !bad && !calibrating;
    if (wasUsable && !this.layoutUsable) Object.values(drones).forEach(drone => drone.publish('cancelRoomTarget', {}));
    let status;
    if (calibrating) {
      status = 'Anchor calibration in progress. Map and room targets paused.';
    } else if (bad) {
      status = bad[1].error || 'Anchor layouts disagree between drones';
      status += '. Map updates and target editing are paused.';
    } else if (candidate) {
      if (!this.room || this.room.anchors.length !== candidate.anchors.length || !this.matchesRoom(this.room, candidate)) {
        if (this.room) {
          Object.values(drones).forEach(drone => drone.publish('cancelRoomTarget', {}));
          // Do not carry measured markers or target drafts across a layout change.
          Object.values(drones).forEach(drone => xySources.forEach(([key]) => {
            [0, 1, 2].forEach(i => { drone.series[key + i] = []; });
          }));
          document.getElementById('target-x').value = '';
          document.getElementById('target-y').value = '';
        }
        this.room = candidate;
        this.targets = {};
        this.zoom = 1;
        this.chart.options.scales.xAxes[0].display = true;
        this.chart.options.scales.yAxes[0].display = true;
      }
      const r = this.room;
      status = 'Live UWB anchors · ' + r.frame + ' · ' + (r.xMax - r.xMin).toFixed(1) + ' × ' +
        (r.yMax - r.yMin).toFixed(1) + ' m · Equal X/Y scale';
    } else {
      status = Object.values(drones).some(drone => drone.connected)
        ? 'Waiting for a complete report of four UWB corner anchors'
        : 'Connect a drone to discover the UWB anchor layout';
      const counts = live.filter(([, report]) => !report.error).map(([id, report]) =>
        'Drone ' + id.slice(-1) + ': ' + report.anchors.length + '/4');
      if (counts.length) status += ' · ' + counts.join(', ');
    }
    if (!this.layoutUsable && this.room) status += ' · Last observed layout retained';
    document.getElementById('room-status').textContent = status;
    ['room-fit', 'room-zoom-in', 'room-zoom-out'].forEach(id => { document.getElementById(id).disabled = !this.layoutUsable; });
    document.getElementById('target-submit').disabled = !this.layoutUsable;
    document.getElementById('target-clear').disabled = !this.room;
    this.render();
  }

  hasMatchingReport(id) {
    const report = this.anchorReports[id];
    return report && !report.error && report.anchors.length && Date.now() - report.receivedAt <= 1500 &&
      this.matchesRoom(this.room, report);
  }

  setZoom(zoom) {
    this.zoom = Math.max(0.5, Math.min(4, zoom));
    this.render();
  }

  equalizeScale(chart) {
    if (!this.room || !chart.chartArea) return;
    const width = chart.chartArea.right - chart.chartArea.left;
    const height = chart.chartArea.bottom - chart.chartArea.top;
    if (width <= 0 || height <= 0) return;
    const r = this.room;
    const metersPerPixel = Math.max((r.xMax - r.xMin + 2 * r.margin) / width,
      (r.yMax - r.yMin + 2 * r.margin) / height) / this.zoom;
    [['room-x', (r.xMin + r.xMax) / 2, width], ['room-y', (r.yMin + r.yMax) / 2, height]].forEach(([id, center, pixels]) => {
      const scale = chart.scales[id];
      Object.assign(scale.options.ticks, { min: center - pixels * metersPerPixel / 2,
        max: center + pixels * metersPerPixel / 2, maxTicksLimit: 9 });
      scale.min = scale.options.ticks.min;
      scale.max = scale.options.ticks.max;
      // Chart.js 2.7 has already laid out its axes here. Rebuild labels using the
      // final plot dimensions so grid lines, hit testing and datasets agree.
      scale.buildTicks();
      scale.convertTicksToLabels();
      scale._ticks = scale.ticks.map(label => ({ label, major: false }));
    });
  }

  drawAnchorLabels(chart) {
    if (!this.room) return;
    const ctx = chart.ctx, area = chart.chartArea;
    ctx.save();
    ctx.beginPath(); ctx.rect(area.left, area.top, area.right - area.left, area.bottom - area.top); ctx.clip();
    ctx.font = '12px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.fillStyle = '#58677b';
    this.room.anchors.forEach(anchor => {
      ctx.fillText(anchor.name, chart.scales['room-x'].getPixelForValue(anchor.x) + 9,
        chart.scales['room-y'].getPixelForValue(anchor.y) - 9);
    });
    ctx.restore();
  }

  previewClick(event) {
    if (!this.layoutUsable) return;
    const pixel = Chart.helpers.getRelativePosition(event, this.chart);
    const area = this.chart.chartArea;
    if (pixel.x < area.left || pixel.x > area.right || pixel.y < area.top || pixel.y > area.bottom) return;
    this.previewTarget(document.getElementById('target-drone').value,
      this.chart.scales['room-x'].getValueForPixel(pixel.x), this.chart.scales['room-y'].getValueForPixel(pixel.y));
  }

  previewTarget(id, x, y) {
    if (!this.layoutUsable || !drones[id]) return;
    const anchors = this.room.anchors;
    const inside = [x, y].every(Number.isFinite) && anchors.every((a, i) => {
      const b = anchors[(i + 1) % anchors.length];
      return (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x) >= -1e-9;
    });
    if (!inside) {
      document.getElementById('target-status').textContent = 'Choose an X/Y position inside the room.';
      return;
    }
    this.targets[id] = { x, y };
    document.getElementById('target-x').value = x.toFixed(2);
    document.getElementById('target-y').value = y.toFixed(2);
    this.render();
  }

  canSendTarget(id) {
    const drone = drones[id];
    return !!(this.layoutUsable && this.hasMatchingReport(id) && this.targets[id] && drone.connected &&
      drone.field('flightMode').textContent === 'FLYING' && drone.fusedPose &&
      drone.fusionReadyAt !== null && Date.now() - drone.fusionReadyAt <= 500 &&
      drone.fusedReceivedAt !== null && Date.now() - drone.fusedReceivedAt <= 500 &&
      roomFrame(drone.fusedPose.header.frame_id) === this.room.frame);
  }

  sendTarget() {
    const id = document.getElementById('target-drone').value;
    if (!this.canSendTarget(id)) return;
    const drone = drones[id], target = this.targets[id];
    drone.publish('roomTarget', { header: drone.fusedPose.header,
      pose: { position: { x: target.x, y: target.y, z: drone.fusedPose.pose.pose.position.z },
        orientation: { x: 0, y: 0, z: 0, w: 1 } } });
    document.getElementById('target-command-status').textContent = 'Target sent to Drone ' + id.slice(-1) + '.';
  }

  render() {
    document.getElementById('map-frame').classList.toggle('map-unavailable', !this.layoutUsable);
    document.getElementById('map-placeholder').hidden = this.layoutUsable;
    document.getElementById('fleet-xyChart').setAttribute('aria-hidden', String(!this.layoutUsable));
    ['map-legend', 'map-sources'].forEach(id => { document.getElementById(id).hidden = !this.layoutUsable; });
    const selected = document.getElementById('target-drone').value;
    document.getElementById('target-send').disabled = !this.canSendTarget(selected);
    document.getElementById('target-stop').disabled = !drones[selected].connected;
    this.chart.data.datasets.forEach(dataset => {
      if (dataset.key === 'anchors') {
        dataset.data = this.room ? [...this.room.anchors, this.room.anchors[0]] : [];
      } else if (dataset.key === 'target') {
        dataset.data = this.layoutUsable && this.targets[dataset.targetDrone] ? [this.targets[dataset.targetDrone]] : [];
      } else {
        const drone = drones[dataset.droneId];
        dataset.data = this.layoutUsable && this.hasMatchingReport(drone.id) && roomFrame(drone.xyFrames[dataset.source]) === this.room.frame
          ? drone.series[dataset.key].map(point => ({ ...point })) : [];
      }
    });
    document.getElementById('target-status').textContent = Object.entries(this.targets).map(([id, p]) =>
      'Drone ' + id.slice(-1) + ': X ' + p.x.toFixed(2) + ', Y ' + p.y.toFixed(2) + ' m').join(' · ');
    const omitted = this.room ? Object.values(drones).flatMap(drone => xySources.filter(([key]) =>
      drone.series[key + '0'].length && roomFrame(drone.xyFrames[key]) !== this.room.frame).map(([key, label]) =>
      'Drone ' + drone.id.slice(-1) + ' ' + label + ' (' + (drone.xyFrames[key] || 'frame unknown') + ')')) : [];
    this.chart.update();
    const xTicks = this.chart.options.scales.xAxes[0].ticks;
    const yTicks = this.chart.options.scales.yAxes[0].ticks;
    const outside = this.chart.data.datasets.filter(dataset => dataset.droneId && dataset.key === dataset.source + '0' && dataset.data.length)
      .filter(dataset => {
        const x = (dataset.data[0].x + dataset.data[1].x) / 2;
        const y = (dataset.data[0].y + dataset.data[1].y) / 2;
        return x < xTicks.min || x > xTicks.max || y < yTicks.min || y > yTicks.max;
      }).map(dataset => dataset.label);
    document.getElementById('map-stream-status').textContent = [
      omitted.length ? 'Hidden until aligned to ' + this.room.frame + ': ' + omitted.join(', ') : '',
      outside.length ? 'Outside current view: ' + outside.join(', ') : ''
    ].filter(Boolean).join('. ');
  }

  clearDrone(id) {
    // Remove measured markers while preserving this drone's proposed destination.
    xySources.forEach(([key]) => { [0, 1, 2].forEach(i => { drones[id].series[key + i] = []; }); });
    delete this.anchorReports[id];
    delete this.anchorStamps[id];
    this.refreshAnchors();
  }
}

// Each bridge uses the same topic names; all ROS and chart state belongs to its session.
class DroneSession {
  constructor(id) {
    this.id = id;
    this.host = id + '.local';
    this.detectionReceivedAt = null;
    this.localizationReceivedAt = null;
    this.localizationStatusAt = null;
    this.localizationStamp = null;
    this.fusedPose = null;
    this.fusedStamp = null;
    this.fusedReceivedAt = null;
    this.fusionReadyAt = null;
    this.fusionGeneration = null;
    this.modeReceivedAt = null;
    this.calibrationLocked = false;
    this.root = document.getElementById(id);
    this.ros = null;
    this.publishers = {};
    this.subscribers = [];
    this.heartbeat = null;
    this.positionMode = false;
    this.startTime = null;
    this.latestTime = 0;
    this.verticalHeightSamples = [];
    this.verticalSampleAt = null;
    this.analysis = false;
    this.series = {};
    this.xyFrames = {};
    this.heightChart = this.makeChart('heightChart', this.heightDatasets(), 'Height (m)', 0, 0.6);
    this.speedChart = this.makeChart('speedChart', [
      this.dataset('speed', 'Horizontal speed', '#e46555'),
      this.dataset('verticalSpeed', 'Vertical velocity', '#49368c')
    ], 'Speed (m/s)', undefined, undefined);
    xySources.forEach(([key]) => {
      [0, 1, 2].forEach(i => { this.series[key + i] = []; });
    });
    ['height', 'speed'].forEach(kind => {
      this.field(kind + 'Details').addEventListener('toggle', () => {
        if (!this.field(kind + 'Details').open) return;
        this[kind + 'Chart'].resize();
        this.render(kind);
      });
    });
    this.field('camera').addEventListener('error', () => {
      // Stop retrying a missing stream until the next connection.
      if (this.field('camera').getAttribute('src') !== 'nocamera.svg') {
        this.field('camera').src = 'nocamera.svg';
      }
    });
  }

  field(name) {
    if (name === 'camera') return document.getElementById(this.id + '-camera');
    if (name === 'detection') return document.getElementById(this.id + '-detection');
    return this.root.querySelector('[data-field="' + name + '"]');
  }
  get connected() { return !!(this.ros && this.ros.isConnected); }

  setDetection(text, state) {
    const status = this.field('detection');
    status.textContent = text;
    status.dataset.state = state;
  }

  updateDetection(message) {
    const valid = typeof message.camera_online === 'boolean' &&
      Array.isArray(message.tag_ids) && Array.isArray(message.zones) &&
      message.tag_ids.length === message.zones.length &&
      message.tag_ids.every(id => Number.isInteger(id) && id >= 0) &&
      message.zones.every(zone => typeof zone === 'string' && zone.trim());
    if (!valid || !message.camera_online) {
      this.detectionReceivedAt = null;
      this.setDetection('Drone Offline', 'offline');
      return;
    }
    this.detectionReceivedAt = Date.now();
    this.setDetection(message.tag_ids.length ? message.tag_ids.map((id, i) =>
      message.zones[i] + ' · Tag ' + id).join(', ') : 'No tag detected',
      message.tag_ids.length ? 'detected' : 'none');
  }

  refreshDetection() {
    if (this.detectionReceivedAt !== null && Date.now() - this.detectionReceivedAt > 1500) {
      this.detectionReceivedAt = null;
      this.setDetection('Drone Offline', 'offline');
    }
  }

  clearLocalization(text = 'Localization unavailable') {
    this.localizationReceivedAt = null;
    this.field('localizationPosition').textContent = '—';
    this.field('localizationStatus').textContent = text;
    this.field('localizationStatus').dataset.state = 'stale';
    [0, 1, 2].forEach(i => { this.series['uwb' + i] = []; });
    if (fleetMap) fleetMap.render();
  }

  updateLocalization(message) {
    const p = message.pose && message.pose.pose && message.pose.pose.position;
    const stamp = message.header && message.header.stamp;
    const time = stamp && stamp.secs + stamp.nsecs / 1e9;
    if (!p || ![p.x, p.y, p.z, time].every(Number.isFinite) || time <= 0 ||
        !message.header.frame_id) return this.clearLocalization('Invalid position');
    // Repeated or older sensor stamps must not keep a stopped stream alive.
    if (this.localizationStamp !== null && time <= this.localizationStamp) return;
    this.localizationStamp = time;
    this.xyFrames.uwb = message.header.frame_id;
    this.localizationReceivedAt = Date.now();
    this.field('localizationPosition').textContent = 'X ' + p.x.toFixed(2) + ' · Y ' + p.y.toFixed(2) + ' · Z ' + p.z.toFixed(2) + ' m';
    this.field('localizationStatus').textContent = 'Ready · ' + message.header.frame_id;
    this.field('localizationStatus').dataset.state = 'ready';
    // Draw a cross, since UWB provides no heading measurement.
    this.series.uwb0 = [{ x: p.x - 0.08, y: p.y }, { x: p.x + 0.08, y: p.y }];
    this.series.uwb1 = [{ x: p.x, y: p.y - 0.08 }, { x: p.x, y: p.y + 0.08 }];
    this.series.uwb2 = [];
    if (fleetMap) fleetMap.render();
  }

  updateLocalizationStatus(message) {
    const status = message.status && message.status[0];
    if (!status || !Number.isInteger(status.level) || typeof status.message !== 'string') return;
    this.localizationStatusAt = Date.now();
    if (status.level !== 0) this.clearLocalization(status.message || 'Localization unavailable');
  }

  refreshLocalization() {
    if ((this.localizationReceivedAt !== null && Date.now() - this.localizationReceivedAt > 500) ||
        (this.localizationStatusAt !== null && Date.now() - this.localizationStatusAt > 1500)) {
      this.clearLocalization('Localization stale');
    }
    if ((this.fusedReceivedAt !== null && Date.now() - this.fusedReceivedAt > 500) ||
        (this.fusionReadyAt !== null && Date.now() - this.fusionReadyAt > 500)) this.clearFusion('Fused position stale');
  }

  clearFusion(text) {
    this.fusedPose = null;
    this.fusedReceivedAt = this.fusionReadyAt = null;
    this.field('fusedPosition').textContent = '—';
    this.field('fusionStatus').textContent = text;
    [0, 1, 2].forEach(i => { this.series['fused' + i] = []; });
    if (fleetMap) fleetMap.render();
  }

  updateFusion(message) {
    const p = message.pose && message.pose.pose && message.pose.pose.position;
    const q = message.pose && message.pose.pose && message.pose.pose.orientation;
    const stamp = message.header && message.header.stamp;
    const time = stamp && stamp.secs + stamp.nsecs / 1e9;
    if (!p || !q || ![p.x, p.y, p.z, q.x, q.y, q.z, q.w, time].every(Number.isFinite) ||
        q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-12 || time <= 0 || !message.header.frame_id)
      return this.clearFusion('Invalid fused position');
    if (this.fusedStamp !== null && time <= this.fusedStamp) return;
    this.fusedPose = message;
    this.fusedStamp = time;
    this.fusedReceivedAt = Date.now();
    this.field('fusedPosition').textContent = 'X ' + p.x.toFixed(2) + ' · Y ' + p.y.toFixed(2) + ' · Z ' + p.z.toFixed(2) + ' m';
    this.updateXY('fused', message.pose.pose, message.header.frame_id);
  }

  dataset(key, label, color, extra = {}) {
    if (!this.series[key]) this.series[key] = [];
    return Object.assign({
      key, label, data: this.series[key].slice(), borderColor: color,
      backgroundColor: color, borderWidth: 1.5, pointRadius: 0,
      fill: false, lineTension: 0
    }, extra);
  }

  heightDatasets() {
    if (this.analysis) return [
      this.dataset('error', 'UKF − ground truth', '#d1513a'),
      this.dataset('sigmaPlus', '+1 sigma', '#49368c', { fill: '+1', backgroundColor: '#49368c22' }),
      this.dataset('sigmaMinus', '−1 sigma', '#49368c')
    ];
    return [
      this.dataset('range', 'Raw range', '#e46555'),
      this.dataset('ukfHeight', 'UKF height', '#49368c'),
      this.dataset('ukfPlus', 'UKF +sigma', '#49368c', { band: true, borderWidth: 0, fill: '+1', backgroundColor: '#49368c22' }),
      this.dataset('ukfMinus', 'UKF −sigma', '#49368c', { band: true, borderWidth: 0 }),
      this.dataset('ema', 'EMA height', '#fc46ad'),
      this.dataset('truthHeight', 'Ground truth', '#172337')
    ];
  }

  makeChart(field, datasets, axisLabel, min, max) {
    return new Chart(this.field(field).getContext('2d'), {
      type: 'line', data: { datasets }, options: {
        responsive: true, maintainAspectRatio: false, animation: { duration: 0 },
        scales: {
          yAxes: [{ ticks: { min, max }, scaleLabel: { display: true, labelString: axisLabel } }],
          xAxes: [{ type: 'linear', display: false, ticks: { min: 0, max: 5 } }]
        },
        legend: { labels: { filter: (item, data) => {
          const dataset = data.datasets[item.datasetIndex];
          return !dataset.band && dataset.data.length > 0;
        } } }
      }
    });
  }

  setStatus(text, connected = false) {
    const state = connected ? 'connected' : text === 'Connecting…' ? 'connecting' : text === 'Connection error' ? 'error' : 'disconnected';
    [document.getElementById(this.id + '-connectionStatus'), this.field('connectionStatus')].forEach(status => {
      status.dataset.state = state;
      status.querySelector('[data-connection-text]').textContent = text;
      const icon = status.querySelector('[data-state-icon]');
      icon.dataset.state = state;
      icon.textContent = { connected: '●', connecting: '', error: '!', disconnected: '○' }[state];
      icon.setAttribute('aria-label', text);
      icon.title = text;
      status.setAttribute('aria-label', 'Drone ' + this.id.slice(-1) + ' ' + text);
    });
  }

  resetTelemetry() {
    this.xyFrames = {};
    this.startTime = null;
    this.latestTime = 0;
    this.positionMode = false;
    this.verticalHeightSamples = [];
    this.verticalSampleAt = null;
    this.detectionReceivedAt = null;
    this.setDetection('Drone Offline', 'offline');
    this.localizationStamp = null;
    this.localizationStatusAt = null;
    this.fusedStamp = null;
    this.fusionGeneration = null;
    this.clearFusion(this.connected ? 'Waiting for fusion' : 'Drone offline');
    this.clearLocalization(this.connected ? 'Waiting for localization' : 'Drone offline');
    ['battery', 'flightMode', 'height', 'speed', 'verticalSpeed'].forEach(name => {
      this.field(name).textContent = '—';
      this.field(name).classList.remove('alert-success', 'alert-danger');
    });
    this.field('camera').src = 'nocamera.svg';
    Object.keys(this.series).forEach(key => { this.series[key] = []; });
    [this.heightChart, this.speedChart].forEach(chart => {
      chart.data.datasets.forEach(dataset => { dataset.data = []; });
      Object.assign(chart.options.scales.xAxes[0].ticks, { min: 0, max: 5 });
      chart.update();
    });
    if (fleetMap) fleetMap.clearDrone(this.id);
  }

  connect() {
    // A pending socket is also a session; repeated clicks must not create duplicates.
    if (this.ros) return;
    this.resetTelemetry();
    this.setStatus('Connecting…');
    const ros = new ROSLIB.Ros();
    this.ros = ros;
    ros.on('connection', () => {
      if (this.ros !== ros) return;
      this.setStatus('Connected', true);
      this.setupTopics(ros);
      this.field('camera').src = 'http://' + this.host + ':8080/stream?topic=/raspicam_node/image&quality=70&type=ros_compressed';
      updateLayout();
    });
    ros.on('error', error => {
      if (this.ros !== ros) return;
      console.error(this.id + ' ROS connection error', error);
      this.disconnect('Connection error');
    });
    ros.on('close', () => {
      if (this.ros !== ros) return;
      this.disconnect();
    });
    ros.connect('ws://' + this.host + ':9090');
  }

  disconnect(status = 'Disconnected') {
    const ros = this.ros;
    // Invalidate first, so callbacks from a replaced socket cannot update this drone.
    this.ros = null;
    if (this.heartbeat !== null) clearInterval(this.heartbeat);
    this.heartbeat = null;
    if (ros && ros.isConnected) {
      this.subscribers.forEach(topic => topic.unsubscribe());
      Object.values(this.publishers).forEach(topic => topic.unadvertise());
    }
    this.subscribers = [];
    this.publishers = {};
    if (ros) ros.close();
    this.resetTelemetry();
    this.setStatus(status);
    updateLayout();
  }

  setupTopics(ros) {
    const publisher = (key, name, messageType) => {
      this.publishers[key] = new ROSLIB.Topic({ ros, name, messageType });
    };
    publisher('mode', '/pidrone/desired/mode', 'pidrone_pkg/Mode');
    publisher('heartbeat', '/pidrone/heartbeat/web_interface', 'std_msgs/Empty');
    publisher('positionMode', '/pidrone/position_control', 'std_msgs/Bool');
    publisher('twist', '/pidrone/desired/twist', 'geometry_msgs/Twist');
    publisher('pose', '/pidrone/desired/pose', 'geometry_msgs/Pose');
    publisher('roomTarget', '/pidrone/desired/room_target', 'geometry_msgs/PoseStamped');
    publisher('cancelRoomTarget', '/pidrone/desired/cancel_room_target', 'std_msgs/Empty');
    publisher('reset', '/pidrone/reset_transform', 'std_msgs/Empty');
    publisher('map', '/pidrone/map', 'std_msgs/Empty');
    this.publish('heartbeat', {});
    this.heartbeat = setInterval(() => this.publish('heartbeat', {}), 1000);

    const subscribe = (name, messageType, callback) => {
      const topic = new ROSLIB.Topic({ ros, name, messageType, queue_length: 2, throttle_rate: 80 });
      topic.subscribe(message => { if (this.ros === ros && this.connected) callback(message); });
      this.subscribers.push(topic);
    };
    subscribe('/pidrone/apriltag/detections', 'pidrone_pkg/TagDetection', message => this.updateDetection(message));
    subscribe('/pidrone/localization/pose', 'geometry_msgs/PoseWithCovarianceStamped', message => this.updateLocalization(message));
    subscribe('/pidrone/localization/status', 'diagnostic_msgs/DiagnosticArray', message => this.updateLocalizationStatus(message));
    subscribe('/pidrone/localization/fused', 'geometry_msgs/PoseWithCovarianceStamped', message => this.updateFusion(message));
    subscribe('/pidrone/localization/generation', 'std_msgs/UInt32', message => {
      if (!Number.isInteger(message.data)) return;
      if (this.fusionGeneration !== null && this.fusionGeneration !== message.data) {
        this.fusedStamp = this.localizationStamp = null;
        this.clearFusion('Localization reference changed');
        delete fleetMap.anchorStamps[this.id];
        delete fleetMap.anchorReports[this.id];
        fleetMap.refreshAnchors();
      }
      this.fusionGeneration = message.data;
    });
    subscribe('/pidrone/localization/ready', 'std_msgs/Bool', message => {
      this.fusionReadyAt = message.data ? Date.now() : null;
      if (!message.data) this.clearFusion('Room localization unavailable');
      if (fleetMap) fleetMap.render();
    });
    subscribe('/pidrone/localization/fusion_status', 'diagnostic_msgs/DiagnosticArray', message => {
      const status = message.status && message.status[0];
      if (!status || typeof status.message !== 'string') return;
      if (status.level !== 0) this.clearFusion(status.message);
      this.field('fusionStatus').textContent = status.message;
    });
    subscribe('/pidrone/localization/anchors', 'pidrone_pkg/AnchorLayout', message => fleetMap.updateAnchors(this.id, message));
    subscribe('/pidrone/battery', 'pidrone_pkg/Battery', message => {
      if (!Number.isFinite(message.vbat)) return;
      const battery = this.field('battery');
      battery.textContent = message.vbat.toFixed(1) + ' V' + (message.vbat <= 11.3 ? ' EMPTY!' : '');
      battery.classList.toggle('alert-danger', message.vbat <= 11.3);
      battery.classList.toggle('alert-success', message.vbat > 11.3);
    });
    subscribe('/pidrone/mode', 'pidrone_pkg/Mode', message => {
      this.field('flightMode').textContent = message.mode;
      this.modeReceivedAt = Date.now();
    });
    subscribe('/pidrone/localization/calibration_locked', 'std_msgs/Bool', message => {
      if (typeof message.data !== 'boolean') return;
      this.calibrationLocked = message.data;
      fleetMap.refreshAnchors();
    });

    subscribe('/pidrone/position_control', 'std_msgs/Bool', message => {
      this.positionMode = !!message.data;
    });
    subscribe('/pidrone/range', 'sensor_msgs/Range', message => {
      const time = this.time(message);
      const valid = Number.isFinite(message.range) && message.range >= 0 &&
        (message.min_range === undefined || message.range >= message.min_range) &&
        (message.max_range === undefined || message.range <= message.max_range);
      if (valid) this.addPoint('range', time, message.range);
      this.field('height').textContent = valid ? message.range.toFixed(2) + ' m' : '—';
      this.updateVerticalSpeed(time, valid ? message.range : NaN);
      this.render('height');
    });
    subscribe('/pidrone/picamera/twist', 'geometry_msgs/TwistStamped', message => {
      const speed = Math.hypot(message.twist.linear.x, message.twist.linear.y);
      this.addPoint('speed', this.time(message), speed);
      if (Number.isFinite(speed)) {
        this.field('speed').textContent = speed.toFixed(2) + ' m/s';
      }
      this.render('speed');
    });
    const ukf = message => {
      const time = this.time(message);
      const pose = message.pose_with_covariance.pose;
      const sigma = Math.sqrt(message.pose_with_covariance.covariance[14]);
      this.addPoint('ukfHeight', time, pose.position.z);
      this.addPoint('ukfPlus', time, pose.position.z + sigma);
      this.addPoint('ukfMinus', time, pose.position.z - sigma);
      this.updateXY('ukf', pose, message.header && message.header.frame_id);
      this.render('height');
    };
    subscribe('/pidrone/state/ukf_2d', 'pidrone_pkg/State', ukf);
    subscribe('/pidrone/state/ukf_7d', 'pidrone_pkg/State', ukf);
    subscribe('/pidrone/state/ema', 'pidrone_pkg/State', message => {
      this.addPoint('ema', this.time(message), message.pose_with_covariance.pose.position.z);
      this.render('height');
    });
    subscribe('/pidrone/state/ground_truth', 'pidrone_pkg/StateGroundTruth', message => {
      this.addPoint('truthHeight', this.time(message), message.pose.position.z);
      this.updateXY('groundTruth', message.pose, message.header && message.header.frame_id);
      this.render('height');
    });
    subscribe('/pidrone/picamera/pose', 'geometry_msgs/PoseStamped', message => {
      this.updateXY('camera', message.pose, message.header && message.header.frame_id);
    });
    subscribe('/pidrone/ukf_stats', 'pidrone_pkg/UkfStats', message => {
      const time = this.time(message);
      this.addPoint('error', time, message.error);
      this.addPoint('sigmaPlus', time, message.stddev);
      this.addPoint('sigmaMinus', time, -message.stddev);
      this.render('height');
    });
  }

  time(message) {
    const stamp = message.header.stamp;
    const now = stamp.secs + stamp.nsecs / 1e9;
    if (!Number.isFinite(now) || now <= 0) return NaN;
    if (this.startTime === null) this.startTime = now;
    const time = now - this.startTime;
    this.latestTime = Math.max(this.latestTime, time);
    return time;
  }

  clearVerticalSpeed() {
    this.verticalHeightSamples = [];
    this.verticalSampleAt = null;
    this.series.verticalSpeed = [];
    this.field('verticalSpeed').textContent = '—';
    this.render('speed');
  }

  refreshVerticalSpeed() {
    if (this.verticalSampleAt !== null && Date.now() - this.verticalSampleAt > 500) this.clearVerticalSpeed();
  }

  updateVerticalSpeed(time, height) {
    if (!Number.isFinite(time) || !Number.isFinite(height)) return this.clearVerticalSpeed();
    const previous = this.verticalHeightSamples.at(-1);
    // Duplicate/out-of-order sensor stamps cannot describe a new velocity.
    if (previous && time <= previous.time) return;
    if (previous && time - previous.time > 0.5) this.clearVerticalSpeed();
    this.verticalSampleAt = Date.now();
    this.verticalHeightSamples.push({ time, height });
    this.verticalHeightSamples = this.verticalHeightSamples.filter(sample => sample.time >= time - 0.3).slice(-20);
    const samples = this.verticalHeightSamples;
    if (samples.length < 2 || time - samples[0].time < 0.1) return;
    // Fit a short height/time slope instead of amplifying noise between two frames.
    const meanTime = samples.reduce((sum, sample) => sum + sample.time, 0) / samples.length;
    const meanHeight = samples.reduce((sum, sample) => sum + sample.height, 0) / samples.length;
    const numerator = samples.reduce((sum, sample) => sum + (sample.time - meanTime) * (sample.height - meanHeight), 0);
    const denominator = samples.reduce((sum, sample) => sum + (sample.time - meanTime) ** 2, 0);
    const velocity = numerator / denominator;
    this.addPoint('verticalSpeed', time, velocity);
    const rounded = Math.round(Math.abs(velocity) * 100) / 100;
    const direction = rounded === 0 ? '' : velocity > 0 ? ' ↑' : ' ↓';
    this.field('verticalSpeed').textContent = rounded.toFixed(2) + ' m/s' + direction;
    this.render('speed');
  }

  addPoint(key, time, value) {
    if (!Number.isFinite(time) || !Number.isFinite(value)) return;
    const series = this.series[key] || [];
    series.push({ x: time, y: value });
    // Bound every stream, including stopped timestamps.
    this.series[key] = series.filter(point => point.x >= this.latestTime - 5).slice(-1000);
  }

  render(kind) {
    const chart = this[kind + 'Chart'];
    chart.data.datasets.forEach(dataset => {
      dataset.data = this.series[dataset.key].filter(point => point.x >= this.latestTime - 5);
    });
    chart.options.scales.xAxes[0].ticks.min = Math.max(0, this.latestTime - 5);
    chart.options.scales.xAxes[0].ticks.max = Math.max(5, this.latestTime);
    chart.update();
  }

  updateXY(key, pose, frame) {
    const { position: p, orientation: q } = pose;
    if (![p.x, p.y, q.w, q.x, q.y, q.z].every(Number.isFinite)) return;
    this.xyFrames[key] = frame;
    const rotation = new Quaternion([q.w, q.x, q.y, q.z]);
    [[0.09, 0.09, 0], [0.09, -0.09, 0], [0, 0.15, 0]].forEach((vector, i) => {
      const rotated = rotation.rotateVector(vector);
      this.series[key + i] = [
        { x: p.x - (i === 2 ? 0 : rotated[0]), y: p.y - (i === 2 ? 0 : rotated[1]) },
        { x: p.x + rotated[0], y: p.y + rotated[1] }
      ];
    });
    if (fleetMap) fleetMap.render();
  }

  toggleAnalysis(button) {
    this.analysis = !this.analysis;
    button.textContent = this.analysis ? 'Standard view' : 'UKF analysis';
    button.setAttribute('aria-pressed', String(this.analysis));
    this.heightChart.data.datasets = this.heightDatasets();
    Object.assign(this.heightChart.options.scales.yAxes[0].ticks, this.analysis ? { min: undefined, max: undefined } : { min: 0, max: 0.6 });
    this.render('height');
  }

  publish(key, data) {
    // ROSLIB queues sends while disconnected; flight commands must never be queued.
    if (!this.connected || !this.publishers[key]) return false;
    this.publishers[key].publish(new ROSLIB.Message(data));
    return true;
  }

  setMode(position) {
    if (this.publish('positionMode', { data: position })) this.positionMode = position;
  }

  twist(x = 0, y = 0, z = 0, yaw = 0) {
    this.publish('twist', { linear: { x, y, z }, angular: { x: 0, y: 0, z: yaw } });
  }

  pose(x = 0, y = 0, z = 0) {
    this.publish('pose', { position: { x, y, z }, orientation: { x: 0, y: 0, z: 0, w: 1 } });
  }

  command(action) {
    if (!this.connected) return;
    if ((action === 'arm' || action === 'takeoff') && calibrationActive()) return;
    if (action === 'position') return this.setMode(true);
    if (action === 'velocity') return this.setMode(false);
    if (action === 'reset' || action === 'map') return this.publish(action, {});
    if (action === 'stop') return this.twist();
    if (action === 'arm' || action === 'disarm' || action === 'takeoff') {
      this.twist();
      if (this.positionMode) this.pose();
      return this.publish('mode', { mode: { arm: 'ARMED', disarm: 'DISARMED', takeoff: 'FLYING' }[action] });
    }
    if (action === 'up' || action === 'down') return this.pose(0, 0, action === 'up' ? 0.05 : -0.05);
    if (action === 'yawLeft' || action === 'yawRight') return this.twist(0, 0, 0, action === 'yawLeft' ? -50 : 50);
    const offset = { left: [-0.1, 0], right: [0.1, 0], forward: [0, 0.1], backward: [0, -0.1] }[action];
    if (offset) return this.positionMode ? this.pose(...offset, 0) : this.twist(...offset, 0);
  }
}

const drones = {};
let fleetMap = null;
let anchorCalibration = null;
function calibrationActive() {
  return Object.values(drones).some(drone => drone.calibrationLocked) ||
    !!(anchorCalibration && anchorCalibration.active());
}
const heldKeys = new Map();
const movementKeys = { j: 'left', l: 'right', i: 'forward', k: 'backward', w: 'up', s: 'down', a: 'yawLeft', d: 'yawRight' };
const actionKeys = { ';': 'arm', t: 'takeoff', r: 'reset', p: 'position', v: 'velocity', m: 'map' };

function sessionsFor(target) { return target === 'both' ? Object.values(drones) : [drones[target]]; }
function currentTarget() {
  const connected = Object.values(drones).filter(drone => drone.connected);
  return connected.length === 1 ? connected[0].id : 'both';
}
function selectedConnection() { return document.querySelector('input[name="droneSelection"]:checked').value; }
function feedback(message) {
  const status = document.getElementById('commandFeedback');
  status.textContent = message;
  status.hidden = !message;
}

function updateLayout() {
  const connected = Object.values(drones).filter(drone => drone.connected);
  const grid = document.getElementById('fleetGrid');
  const countChanged = Number(grid.dataset.count) !== connected.length;
  grid.dataset.count = String(connected.length);
  Object.values(drones).forEach(drone => {
    [drone.heightChart, drone.speedChart].forEach(chart => chart.resize());
  });
  document.getElementById('controlsHeading').textContent = connected.length === 2 ? 'Shared controls' : 'Flight controls';
  if (fleetMap) fleetMap.chart.resize();
  if (countChanged) {
    stopHeldKeys();
    feedback('');
  }
}

function connectSelected() {
  stopHeldKeys();
  const selected = sessionsFor(selectedConnection());
  // Connecting the selected drone adds it to the existing fleet.
  selected.forEach(drone => drone.connect());
}

function dispatch(action, target = currentTarget()) {
  if ((action === 'arm' || action === 'takeoff') && calibrationActive()) {
    feedback('Flight inhibited until anchor calibration is completed or cancelled.');
    return [];
  }
  const sessions = sessionsFor(target);
  const available = sessions.filter(drone => drone.connected);
  // Stop and disarm can still reach a remaining drone if its partner is offline.
  if (!available.length || (available.length !== sessions.length && action !== 'disarm' && action !== 'stop')) {
    feedback('Command not sent. Connect ' + sessions.filter(drone => !drone.connected).map(drone => drone.id).join(' and ') + ' first.');
    return [];
  }
  available.forEach(drone => drone.command(action));
  const label = {
    arm: 'Arm', disarm: 'Disarm', takeoff: 'Takeoff', stop: 'Stop motion',
    position: 'Position mode',
    velocity: 'Velocity mode', reset: 'Reset position hold', map: 'Toggle mapping',
    up: 'Up', down: 'Down', yawLeft: 'Yaw left', yawRight: 'Yaw right',
    left: 'Left', right: 'Right', forward: 'Forward', backward: 'Backward'
  }[action];
  feedback(label + ' sent to ' + available.map(drone => drone.id).join(' and ') + '.');
  return available;
}

function stopHeldKeys() {
  const sessions = new Set();
  heldKeys.forEach(targets => targets.forEach(drone => sessions.add(drone)));
  sessions.forEach(drone => drone.command('stop'));
  heldKeys.clear();
}

function isFormControl(target) {
  return target && (target.isContentEditable || !!target.closest('input, textarea, select, summary'));
}

document.addEventListener('DOMContentLoaded', () => {
  Chart.defaults.global.defaultFontFamily = '-apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
  Chart.defaults.global.defaultFontSize = 12;
  Chart.defaults.global.defaultFontColor = '#58677b';
  ['drone1', 'drone2'].forEach(id => { drones[id] = new DroneSession(id); });
  fleetMap = new FleetMap();
  anchorCalibration = new AnchorCalibration();
  setInterval(() => {
    Object.values(drones).forEach(drone => { drone.refreshDetection(); drone.refreshVerticalSpeed(); drone.refreshLocalization(); });
    fleetMap.refreshAnchors();
    anchorCalibration.refresh();
  }, 250);
  updateLayout();
  document.addEventListener('click', event => {
    const button = event.target.closest('button');
    if (!button) return;
    if (button.dataset.connect === 'selected') connectSelected();
    else if (button.dataset.connect) sessionsFor(button.dataset.connect).forEach(drone => drone.connect());
    if (button.dataset.disconnect) {
      stopHeldKeys();
      sessionsFor(button.dataset.disconnect === 'selected' ? selectedConnection() : button.dataset.disconnect).forEach(drone => drone.disconnect());
    }
    if (button.dataset.analysis) drones[button.dataset.analysis].toggleAnalysis(button);
    if (!button.dataset.command) return;
    const controller = button.closest('[data-controller]');
    const scope = controller.dataset.controller;
    const action = button.dataset.command;
    const target = scope === 'shared' ? (action === 'disarm' ? 'both' : currentTarget()) : scope;
    if (action === 'disarm' || action === 'stop') stopHeldKeys();
    dispatch(action, target);
  });
});

document.addEventListener('keydown', event => {
  const key = event.key.toLowerCase();
  if (event.ctrlKey || event.metaKey || event.altKey) return;
  if (key === ' ') {
    event.preventDefault();
    stopHeldKeys();
    dispatch('disarm', 'both');
    return;
  }
  if (isFormControl(event.target)) return;
  if (movementKeys[key]) {
    event.preventDefault();
    if (event.repeat && !heldKeys.has(key)) return;
    const targets = dispatch(movementKeys[key]);
    if (targets.length) heldKeys.set(key, targets);
  } else if (actionKeys[key] && !event.repeat) {
    event.preventDefault();
    dispatch(actionKeys[key]);
  }
});

document.addEventListener('keyup', event => {
  const key = event.key.toLowerCase();
  // Release the drones that received the press, even if focus or selection changed.
  if (!heldKeys.has(key)) return;
  heldKeys.get(key).forEach(drone => drone.command('stop'));
  heldKeys.delete(key);
});
window.addEventListener('blur', stopHeldKeys);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopHeldKeys(); });
window.addEventListener('beforeunload', () => {
  stopHeldKeys();
  Object.values(drones).forEach(drone => drone.disconnect());
});
