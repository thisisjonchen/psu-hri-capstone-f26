'use strict';

const xySources = [
  ['groundTruth', 'Ground truth', []],
  ['camera', 'Camera pose', [2, 3]],
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
    this.calibratedRoom = null;
    this.calibrationRevision = null;
    this.targets = {};
    this.selectedTarget = null;
    this.drag = null;
    this.targetFeedback = '';
    this.zoom = 1;
    this.anchorReports = {};
    this.anchorStamps = {};
    this.tagReports = {};
    this.waypointReports = {};
    this.tagRevision = 0;
    this.layoutUsable = false;
    const datasets = Object.values(drones).flatMap(drone => xySources.flatMap(([key, label, dash]) =>
      [0, 1, 2].map(i => ({
        droneId: drone.id, source: key, key: key + i, label: 'Drone ' + drone.id.slice(-1) + ' · ' + label,
        data: [], borderColor: drone.id === 'drone1' ? '#0072b2' : '#d55e00',
        borderDash: dash, borderWidth: 2.5, pointRadius: 0, fill: false, lineTension: 0
      }))));
    datasets.push({ key: 'apriltags', label: 'Mapped floor AprilTags', data: [], showLine: false,
      backgroundColor: '#198a71', borderColor: '#198a71', pointStyle: 'rectRot',
      pointRadius: 7, pointHoverRadius: 9, fill: false });
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
        afterDatasetsDraw: chart => { this.drawAnchorLabels(chart); this.drawTagLabels(chart); this.drawTargetLabels(chart); }
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
        }, legend: { display: false }
      }
    });
    document.getElementById('room-fit').addEventListener('click', () => { this.zoom = 1; this.render(); });
    document.getElementById('room-zoom-in').addEventListener('click', () => this.setZoom(this.zoom * 1.25));
    document.getElementById('room-zoom-out').addEventListener('click', () => this.setZoom(this.zoom / 1.25));
    const canvas = document.getElementById('fleet-xyChart');
    canvas.addEventListener('pointerdown', event => this.startTargetDrag(event));
    canvas.addEventListener('pointermove', event => this.moveTargetDrag(event));
    canvas.addEventListener('pointerup', event => this.finishTargetDrag(event));
    canvas.addEventListener('pointercancel', () => this.cancelTargetDrag());
    canvas.addEventListener('lostpointercapture', () => this.cancelTargetDrag());
    canvas.addEventListener('keydown', event => this.adjustTarget(event));
    canvas.addEventListener('blur', () => this.cancelTargetDrag());
    document.getElementById('target-clear').addEventListener('click', () => {
      const id = this.selectedTarget;
      if (!id) return;
      drones[id].publish('cancelRoomTarget', {});
      delete this.targets[id];
      this.targetFeedback = '';
      this.render();
    });
    document.getElementById('target-send').addEventListener('click', () => this.sendTarget());
    document.getElementById('target-stop').addEventListener('click', () => {
      if (!this.selectedTarget) return;
      drones[this.selectedTarget].publish('cancelRoomTarget', {});
      document.getElementById('target-command-status').textContent = 'Target stopped.';
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

  updateCalibratedLayout(layout) {
    if (!layout) {
      this.calibratedRoom = null;
      this.calibrationRevision = null;
    } else if (layout.revision !== this.calibrationRevision) {
      const positions = layout.positions;
      if (typeof layout.revision !== 'string' || !layout.revision || roomFrame(layout.frame) !== 'map' ||
          !Array.isArray(positions) || positions.length !== 4 ||
          !positions.every(p => p && /^[0-9a-f]{4}$/i.test(p.id) &&
            [p.x, p.y, p.z].every(Number.isFinite)) ||
          new Set(positions.map(p => p.id.toUpperCase())).size !== 4) return;
      try {
        this.calibratedRoom = this.makeRoom('map', positions.map(p =>
          ({ name: p.id.toUpperCase(), x: p.x, y: p.y, z: p.z })));
        this.calibrationRevision = layout.revision;
      } catch (error) { return; }
    }
    this.calibrationVerified = !layout || layout.verified !== false;
    this.refreshAnchors();
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
    const candidate = this.calibratedRoom || (complete.length ? complete[0][1].completeRoom : null);
    const bad = live.find(([, report]) => report.error || (candidate && !this.matchesRoom(candidate, report)));
    const awaitingVerification = this.calibratedRoom && !this.calibrationVerified && !complete.length;
    this.layoutUsable = !!candidate && !bad && !awaitingVerification;
    if (wasUsable && !this.layoutUsable) Object.values(drones).forEach(drone => drone.publish('cancelRoomTarget', {}));
    // A Bluetooth-verified outline can be displayed before tags report it.
    if (candidate && (this.calibratedRoom || !bad)) {
      if (!this.room || this.room.anchors.length !== candidate.anchors.length || !this.matchesRoom(this.room, candidate)) {
        if (this.room) {
          Object.values(drones).forEach(drone => drone.publish('cancelRoomTarget', {}));
          // Do not carry measured markers or target drafts across a layout change.
          Object.values(drones).forEach(drone => xySources.forEach(([key]) => {
            [0, 1, 2].forEach(i => { drone.series[key + i] = []; });
          }));
          this.chart.data.datasets.forEach(dataset => { dataset.lastData = []; });
          this.tagReports = {};
          this.waypointReports = {};
        }
        this.room = candidate;
        this.targets = {};
        this.selectedTarget = null;
        this.targetFeedback = '';
        this.zoom = 1;
        this.chart.options.scales.xAxes[0].display = true;
        this.chart.options.scales.yAxes[0].display = true;
      }
    }
    let status;
    if (bad) {
      status = bad[1].error || 'Anchor layouts disagree between drones';
      status += '. Map updates and target editing are paused.';
    } else if (candidate) {
      status = '';
    } else {
      status = Object.values(drones).some(drone => drone.connected)
        ? 'Waiting for a complete report of four UWB corner anchors'
        : '';
      const counts = live.filter(([, report]) => !report.error).map(([id, report]) =>
        'Drone ' + id.slice(-1) + ': ' + report.anchors.length + '/4');
      if (counts.length) status += ' · ' + counts.join(', ');
    }
    if (!this.layoutUsable && this.room && !awaitingVerification) status += ' · Last observed layout retained';
    document.getElementById('room-status').textContent = status;
    document.getElementById('room-status').hidden = !status;
    ['room-fit', 'room-zoom-in', 'room-zoom-out'].forEach(id => { document.getElementById(id).disabled = !this.room; });
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
      const label = anchorLabel(anchor.name);
      const x = Math.min(chart.scales['room-x'].getPixelForValue(anchor.x) + 9,
        area.right - ctx.measureText(label).width - 4);
      const y = Math.max(area.top + 14, chart.scales['room-y'].getPixelForValue(anchor.y) - 9);
      ctx.fillText(label, x, y);
    });
    ctx.restore();
  }

  updateTagMap(id, message, navigation = false) {
    const header = message.header || {};
    const stamp = header.stamp && header.stamp.secs + header.stamp.nsecs / 1e9;
    const tags = message.tag_ids, zones = message.zones;
    const points = message.positions, counts = message.sample_counts;
    const frame = roomFrame(header.frame_id);
    if (!frame || !Number.isFinite(stamp) || stamp <= 0 || !Array.isArray(tags) ||
        !Array.isArray(zones) || !Array.isArray(points) || !Array.isArray(counts) ||
        tags.length !== zones.length || tags.length !== points.length || tags.length !== counts.length ||
        new Set(tags).size !== tags.length ||
        !tags.every(tag => Number.isInteger(tag) && tag >= 0) ||
        !zones.every(zone => typeof zone === 'string') ||
        !counts.every(count => Number.isInteger(count) && count > 0) ||
        !points.every(point => point && [point.x, point.y, point.z].every(Number.isFinite))) return;
    if (navigation && (!Number.isInteger(message.generation) || !Array.isArray(message.observed_at) ||
        message.observed_at.length !== tags.length)) return;
    const reports = navigation ? this.waypointReports : this.tagReports;
    if (reports[id] && stamp <= reports[id].stamp) return;
    const previous = reports[id];
    reports[id] = { frame, stamp, receivedAt: Date.now(), generation: message.generation, tags: tags.map((tag, i) => {
      const observed = message.observed_at && message.observed_at[i];
      const value = { id: tag, zone: zones[i], x: points[i].x, y: points[i].y,
        z: points[i].z, count: counts[i], observedAt: observed && observed.secs + observed.nsecs / 1e9 };
      const old = previous && previous.frame === frame && previous.tags.find(p => p.id === tag);
      // A snapshot can update another tag while retaining this cached position.
      // Track changes locally; ROS clocks on the two drones need not agree.
      const unchanged = old && ['zone', 'x', 'y', 'z', 'count'].every(key => old[key] === value[key]);
      value.revision = unchanged ? old.revision : ++this.tagRevision;
      return value;
    }) };
    if (!navigation) this.render();
  }

  mappedTags() {
    if (!this.room || !this.layoutUsable) return [];
    const groups = new Map();
    Object.keys(drones).forEach(id => {
      const report = this.tagReports[id];
      if (!drones[id].connected || !report || report.frame !== this.room.frame ||
          !this.hasMatchingReport(id)) return;
      report.tags.forEach(tag => {
        const previous = groups.get(tag.id);
        if (!previous || tag.revision > previous.revision) groups.set(tag.id, tag);
      });
    });
    // Lifetime sample counts and averaging can pin a moved tag to its old spot.
    return Array.from(groups.values()).sort((a, b) => a.id - b.id)
      .map(tag => ({ id: tag.id, zone: tag.zone, x: tag.x, y: tag.y }));
  }

  drawTagLabels(chart) {
    if (!this.room) return;
    const ctx = chart.ctx, area = chart.chartArea;
    ctx.save();
    ctx.beginPath(); ctx.rect(area.left, area.top, area.right - area.left, area.bottom - area.top); ctx.clip();
    ctx.font = '12px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    this.mappedTags().forEach(tag => {
      const text = tag.zone + ' · Tag ' + tag.id + ' (' + tag.x.toFixed(2) + ', ' + tag.y.toFixed(2) + ')';
      const px = chart.scales['room-x'].getPixelForValue(tag.x);
      const py = chart.scales['room-y'].getPixelForValue(tag.y);
      const x = Math.min(area.right - ctx.measureText(text).width - 4, Math.max(area.left + 4, px + 10));
      const y = Math.max(area.top + 14, py - 10);
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 3; ctx.strokeText(text, x, y);
      ctx.fillStyle = '#14765e'; ctx.fillText(text, x, y);
    });
    ctx.restore();
  }

  targetInsideRoom(x, y) {
    return this.layoutUsable && [x, y].every(Number.isFinite) && this.room.anchors.every((a, i, anchors) => {
      const b = anchors[(i + 1) % anchors.length];
      return (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x) >= -1e-9;
    });
  }

  markerPosition(id) {
    // Prefer the fused position, then the other displayed pose sources.
    const marker = [...this.chart.data.datasets].reverse().find(dataset =>
      dataset.droneId === id && dataset.key === dataset.source + '0' && dataset.data.length === 2);
    if (!marker) return null;
    return { x: (marker.data[0].x + marker.data[1].x) / 2,
      y: (marker.data[0].y + marker.data[1].y) / 2 };
  }

  targetPixel(point) {
    return { x: this.chart.scales['room-x'].getPixelForValue(point.x),
      y: this.chart.scales['room-y'].getPixelForValue(point.y) };
  }

  targetPoint(event) {
    const pixel = Chart.helpers.getRelativePosition(event, this.chart);
    const area = this.chart.chartArea;
    return { pixel, x: this.chart.scales['room-x'].getValueForPixel(pixel.x),
      y: this.chart.scales['room-y'].getValueForPixel(pixel.y),
      inPlot: pixel.x >= area.left && pixel.x <= area.right && pixel.y >= area.top && pixel.y <= area.bottom };
  }

  hitTarget(pixel) {
    // Prefer an existing target over telemetry; break overlapping-marker ties
    // using the current map selection.
    for (const targetsOnly of [true, false]) {
      const hits = Object.keys(drones).map(id => {
        const point = targetsOnly ? this.targets[id] : this.markerPosition(id);
        const p = point && this.targetPixel(point);
        return { id, distance: p ? Math.hypot(p.x - pixel.x, p.y - pixel.y) : Infinity };
      }).filter(hit => hit.distance <= 22).sort((a, b) =>
        Math.abs(a.distance - b.distance) < 2 ? (a.id === this.selectedTarget ? -1 : b.id === this.selectedTarget ? 1 : 0) : a.distance - b.distance);
      if (hits.length) return hits[0].id;
    }
    return null;
  }

  startTargetDrag(event) {
    if (!this.layoutUsable || this.drag || event.button !== 0) return;
    const point = this.targetPoint(event);
    if (!point.inPlot) return;
    event.preventDefault();
    const canvas = document.getElementById('fleet-xyChart');
    canvas.focus({ preventScroll: true });
    const hit = this.hitTarget(point.pixel);
    const id = hit || this.selectedTarget || currentTarget();
    if (!drones[id]) return;
    this.selectedTarget = id;
    this.targetFeedback = '';
    document.getElementById('target-command-status').textContent = '';
    this.drag = { id: this.selectedTarget, pointerId: event.pointerId, room: this.room,
      start: point.pixel, hit: !!hit, point: null };
    canvas.setPointerCapture(event.pointerId);
    this.render();
  }

  moveTargetDrag(event) {
    if (!this.drag || event.pointerId !== this.drag.pointerId) return;
    const point = this.targetPoint(event);
    if (!this.drag.point && Math.hypot(point.pixel.x - this.drag.start.x, point.pixel.y - this.drag.start.y) < 4) return;
    this.drag.point = point;
    this.render();
  }

  finishTargetDrag(event) {
    if (!this.drag || event.pointerId !== this.drag.pointerId) return;
    const drag = this.drag, point = this.targetPoint(event);
    this.drag = null;
    if (this.room === drag.room && this.layoutUsable) {
      // A tap on a marker selects it; a drag or tap on empty space sets a target.
      if (drag.point || !drag.hit) {
        if (point.inPlot && this.targetInsideRoom(point.x, point.y)) this.previewTarget(drag.id, point.x, point.y);
        else this.targetFeedback = 'Drop inside the room. Target unchanged.';
      }
    }
    const canvas = document.getElementById('fleet-xyChart');
    if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
    this.render();
  }

  cancelTargetDrag() {
    if (!this.drag) return;
    const pointerId = this.drag.pointerId;
    this.drag = null;
    const canvas = document.getElementById('fleet-xyChart');
    if (canvas.hasPointerCapture(pointerId)) canvas.releasePointerCapture(pointerId);
    this.render();
  }

  adjustTarget(event) {
    if (event.key === 'Escape') { this.cancelTargetDrag(); return; }
    const step = event.shiftKey ? 0.5 : 0.1;
    const offset = { ArrowLeft: [-step, 0], ArrowRight: [step, 0],
      ArrowUp: [0, step], ArrowDown: [0, -step] }[event.key];
    if (!offset || !this.layoutUsable || this.drag) return;
    event.preventDefault(); event.stopPropagation();
    const id = this.selectedTarget || currentTarget();
    if (!drones[id]) return;
    const point = this.targets[id] || this.markerPosition(id);
    if (point) this.previewTarget(id, point.x + offset[0], point.y + offset[1]);
  }

  previewTarget(id, x, y) {
    if (!this.layoutUsable || !drones[id]) return;
    if (!this.targetInsideRoom(x, y)) {
      this.targetFeedback = 'Choose a destination inside the room.';
      this.render();
      return;
    }
    this.selectedTarget = id;
    this.targets[id] = { x, y };
    this.targetFeedback = '';
    document.getElementById('target-command-status').textContent = '';
    this.render();
  }

  drawTargetLabels(chart) {
    if (!this.room) return;
    const ctx = chart.ctx, area = chart.chartArea;
    ctx.save();
    ctx.beginPath(); ctx.rect(area.left, area.top, area.right - area.left, area.bottom - area.top); ctx.clip();
    ctx.font = '12px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    Object.keys(drones).forEach(id => {
      const marker = this.markerPosition(id);
      const dataset = chart.data.datasets.find(d => d.targetDrone === id);
      [[marker, 'Drone ' + id.slice(-1)], [dataset.data[0], 'D' + id.slice(-1) + ' target']].forEach(([point, label]) => {
        if (!point) return;
        const p = this.targetPixel(point);
        const x = Math.max(area.left + 4, Math.min(p.x + 12, area.right - ctx.measureText(label).width - 4));
        const y = Math.max(area.top + 14, p.y - 12);
        ctx.strokeStyle = '#fff'; ctx.lineWidth = 3; ctx.strokeText(label, x, y);
        ctx.fillStyle = droneColor(id); ctx.fillText(label, x, y);
      });
    });
    ctx.restore();
  }

  canSendTarget(id) {
    const drone = drones[id];
    return !!(drone && !this.drag && this.layoutUsable && this.hasMatchingReport(id) && this.targets[id] && drone.connected &&
      drone.field('flightMode').textContent === 'FLYING' && drone.fusedPose &&
      drone.fusionReadyAt !== null && Date.now() - drone.fusionReadyAt <= 500 &&
      drone.fusedReceivedAt !== null && Date.now() - drone.fusedReceivedAt <= 500 &&
      roomFrame(drone.fusedPose.header.frame_id) === this.room.frame);
  }

  sendTarget() {
    const id = this.selectedTarget;
    if (!this.canSendTarget(id)) return;
    const drone = drones[id], target = this.targets[id];
    drone.publish('roomTarget', { header: drone.fusedPose.header,
      pose: { position: { x: target.x, y: target.y, z: drone.fusedPose.pose.pose.position.z },
        orientation: { x: 0, y: 0, z: 0, w: 1 } } });
    document.getElementById('target-command-status').textContent = 'Target sent to Drone ' + id.slice(-1) + '.';
  }

  render() {
    const frame = document.getElementById('map-frame');
    // Once discovered, keep the map visible for the rest of this page session.
    const visible = !!this.room || frame.classList.contains('map-ready');
    frame.classList.toggle('map-ready', visible);
    frame.classList.toggle('map-unavailable', !visible);
    document.getElementById('map-placeholder').hidden = visible;
    document.getElementById('fleet-xyChart').setAttribute('aria-hidden', String(!visible));
    document.getElementById('map-legend').hidden = !visible;
    if (this.drag && (!this.layoutUsable || this.drag.room !== this.room)) {
      this.cancelTargetDrag();
      return;
    }
    const selected = this.selectedTarget;
    document.getElementById('fleet-xyChart').classList.toggle('target-dragging', !!this.drag);
    document.getElementById('fleet-xyChart').setAttribute('aria-disabled', String(!this.layoutUsable));
    document.getElementById('target-clear').disabled = !selected || !this.targets[selected] || !!this.drag;
    document.getElementById('target-selection').textContent = selected
      ? 'Drone ' + selected.slice(-1) + (this.drag ? ' · Choosing destination…' : this.targets[selected] ? ' · Target preview' : ' · Tap or drag to choose a destination')
      : '';
    ['target-selection', 'target-clear', 'target-send', 'target-stop'].forEach(id => {
      document.getElementById(id).hidden = !selected;
    });
    document.getElementById('target-send').disabled = !this.canSendTarget(selected);
    document.getElementById('target-stop').disabled = !selected || !drones[selected].connected;
    this.chart.data.datasets.forEach(dataset => {
      if (dataset.key === 'apriltags') {
        dataset.data = this.mappedTags().map(tag => ({ x: tag.x, y: tag.y }));
      } else if (dataset.key === 'anchors') {
        dataset.data = this.room ? [...this.room.anchors, this.room.anchors[0]] : [];
      } else if (dataset.key === 'target') {
        const dragging = this.drag && this.drag.id === dataset.targetDrone && this.drag.point;
        const point = dragging || this.targets[dataset.targetDrone];
        dataset.data = this.layoutUsable && point ? [point] : [];
        dataset.pointRadius = dataset.targetDrone === selected ? 10 : 7;
        dataset.pointBorderColor = dragging && (!dragging.inPlot || !this.targetInsideRoom(dragging.x, dragging.y))
          ? '#b84339' : droneColor(dataset.targetDrone);
      } else {
        const drone = drones[dataset.droneId];
        const aligned = this.room && drone.connected && roomFrame(drone.xyFrames[dataset.source]) === this.room.frame;
        const points = drone.series[dataset.key];
        const receivedAt = drone.xyReceivedAt[dataset.source];
        const live = aligned && this.layoutUsable && this.hasMatchingReport(drone.id) && points.length > 0 &&
          receivedAt !== undefined && Date.now() - receivedAt <= 500;
        // Keep display-only coordinates through gaps; never renew flight freshness.
        if (!aligned) dataset.lastData = [];
        else if (live) dataset.lastData = points.map(point => ({ ...point }));
        dataset.data = dataset.lastData || [];
        dataset.stale = dataset.data.length > 0 && !live;
        dataset.borderColor = droneColor(drone.id);
      }
    });
    document.getElementById('target-status').textContent = this.targetFeedback;
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
    this.chart.data.datasets.filter(dataset => dataset.droneId === id).forEach(dataset => { dataset.lastData = []; });
    delete this.anchorReports[id];
    delete this.anchorStamps[id];
    delete this.tagReports[id];
    delete this.waypointReports[id];
    this.refreshAnchors();
  }
}

// Each bridge uses the same topic names; all ROS and chart state belongs to its session.
class DroneSession {
  constructor(id) {
    this.id = id;
    this.host = id + '.local';
    this.detectionReceivedAt = null;
    this.payloadReceivedAt = null;
    this.payloadStamp = null;
    this.fusedPose = null;
    this.fusedStamp = null;
    this.fusedReceivedAt = null;
    this.fusionReadyAt = null;
    this.fusionGeneration = null;
    this.modeReceivedAt = null;
    this.routineReadyAt = null;
    this.root = document.getElementById(id);
    this.ros = null;
    this.publishers = {};
    this.subscribers = [];
    this.heartbeat = null;
    this.waitUntil = 0;
    this.retryTimer = null;
    this.positionMode = false;
    this.startTime = null;
    this.latestTime = 0;
    this.verticalHeightSamples = [];
    this.verticalSampleAt = null;
    this.series = {};
    this.xyFrames = {};
    this.xyReceivedAt = {};
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
      this.clearPayload('Payload camera unavailable');
      // Stop retrying a missing stream until the next connection.
      if (this.field('camera').getAttribute('src') !== 'nocamera.svg') {
        this.field('camera').src = 'nocamera.svg';
      }
    });
  }

  field(name) {
    if (name === 'camera') return document.getElementById(this.id + '-camera');
    if (name === 'detection') return document.getElementById(this.id + '-detection');
    if (['payloadStatus', 'payloadOverlay', 'payloadBox', 'payloadCenter'].includes(name)) return document.getElementById(this.id + '-' + name);
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
      this.setDetection('No tag data', 'offline');
      return;
    }
    this.detectionReceivedAt = Date.now();
    this.setDetection(message.tag_ids.length ? message.tag_ids.map((id, i) =>
      message.zones[i] + ' · Tag ' + id).join(', ') : 'No AprilTag detected',
      message.tag_ids.length ? 'detected' : 'none');
  }

  refreshDetection() {
    if (this.detectionReceivedAt !== null && Date.now() - this.detectionReceivedAt > 1500) {
      this.detectionReceivedAt = null;
      this.setDetection('No tag data', 'offline');
    }
    if (this.payloadReceivedAt !== null && Date.now() - this.payloadReceivedAt > 750) this.clearPayload();
  }

  clearPayload(text = 'No payload data') {
    this.payloadReceivedAt = null;
    this.field('payloadOverlay').setAttribute('visibility', 'hidden');
    this.field('payloadStatus').textContent = text;
    this.field('payloadStatus').dataset.state = 'offline';
  }

  updatePayload(message) {
    const stamp = message.header && message.header.stamp;
    if (!stamp || !Number.isInteger(stamp.secs) || stamp.secs <= 0 ||
        !Number.isInteger(stamp.nsecs) || stamp.nsecs < 0 || stamp.nsecs >= 1e9 ||
        typeof message.camera_online !== 'boolean' || typeof message.detected !== 'boolean') return this.clearPayload();
    if (!message.camera_online) return this.clearPayload('Payload camera unavailable');
    const { image_width: width, image_height: height, bbox_x: x, bbox_y: y,
      bbox_width: w, bbox_height: h, center_x: cx, center_y: cy, area } = message;
    if (![width, height].every(n => Number.isInteger(n) && n > 0) ||
        (message.detected && (![x, y, w, h].every(Number.isInteger) ||
          ![cx, cy, area].every(Number.isFinite) || x < 0 || y < 0 || w <= 0 || h <= 0 ||
          x + w > width || y + h > height || cx < x || cx >= x + w || cy < y || cy >= y + h ||
          area <= 0 || area > w * h))) return this.clearPayload();
    const time = stamp.secs + stamp.nsecs / 1e9;
    // Cached reports must not keep a frozen camera observation alive.
    if (this.payloadStamp !== null && time <= this.payloadStamp) return;
    this.payloadStamp = time;
    this.payloadReceivedAt = Date.now();
    const status = this.field('payloadStatus');
    status.textContent = message.detected ? 'Yellow payload detected' : 'No yellow payload detected';
    status.dataset.state = message.detected ? 'detected' : 'none';
    const overlay = this.field('payloadOverlay');
    overlay.setAttribute('visibility', message.detected && this.field('camera').getAttribute('src') !== 'nocamera.svg' ? 'visible' : 'hidden');
    if (!message.detected) return;
    // Match the video's object-fit:contain layout, including letterboxing.
    // ROS observations and the MJPEG video arrive independently: display aid only.
    overlay.setAttribute('viewBox', '0 0 ' + width + ' ' + height);
    const box = this.field('payloadBox');
    Object.entries({ x, y, width: w, height: h }).forEach(([name, value]) => box.setAttribute(name, value));
    this.field('payloadCenter').setAttribute('cx', cx);
    this.field('payloadCenter').setAttribute('cy', cy);
  }

  refreshLocalization() {
    if ((this.fusedReceivedAt !== null && Date.now() - this.fusedReceivedAt > 500) ||
        (this.fusionReadyAt !== null && Date.now() - this.fusionReadyAt > 500)) this.clearFusion();
  }

  clearFusion() {
    this.fusedPose = null;
    this.fusedReceivedAt = this.fusionReadyAt = null;
    // Retain the displayed coordinates through gaps; control must use fresh data.
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
      return this.clearFusion();
    if (this.fusedStamp !== null && time <= this.fusedStamp) return;
    this.fusedPose = message;
    this.fusedStamp = time;
    this.fusedReceivedAt = Date.now();
    this.field('fusedPosition').textContent = 'X ' + p.x.toFixed(2) + ' · Y ' + p.y.toFixed(2) + ' · Z ' + p.z.toFixed(2) + ' m';
    this.updateXY('fused', message.pose.pose, message.header.frame_id);
  }

  dataset(key, label, color) {
    if (!this.series[key]) this.series[key] = [];
    return {
      key, label, data: this.series[key].slice(), borderColor: color,
      backgroundColor: color, borderWidth: 1.5, pointRadius: 0,
      fill: false, lineTension: 0
    };
  }

  heightDatasets() {
    return [
      this.dataset('range', 'Raw range', '#e46555'),
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
          return dataset.data.length > 0;
        } } }
      }
    });
  }

  setStatus(text, connected = false) {
    const state = connected ? 'connected' : ['Connecting…', 'Waiting for nodes…'].includes(text) ? 'connecting'
      : ['Connection error', 'Connection timed out', 'Nodes failed to start'].includes(text) ? 'error' : 'disconnected';
    const status = document.getElementById(this.id + '-connectionStatus');
    status.dataset.state = state;
    const button = document.getElementById(this.id + '-connectionToggle');
    const active = state === 'connected' || state === 'connecting';
    button.hidden = connected;
    button.textContent = state === 'connecting' ? 'Cancel connection' : 'Connect';
    button.setAttribute('aria-label', button.textContent + ' Drone ' + this.id.slice(-1));
    this.field('telemetry').hidden = !connected;
    status.disabled = !active;
    status.querySelector('[data-connection-action]').textContent = connected ? 'Disconnect' : 'Cancel';
    if (connected && document.activeElement === button) status.focus();
    if (!active && document.activeElement === status) button.focus();
    const icon = status.querySelector('[data-state-icon]');
    status.querySelector('[data-connection-label]').textContent = {
      connected: 'Connected', connecting: 'Connecting', error: 'Error', disconnected: 'Offline'
    }[state];
    icon.title = text;
    status.title = active ? (connected ? 'Disconnect' : 'Cancel connection') : text;
    status.setAttribute('aria-label', 'Drone ' + this.id.slice(-1) + ' ' + text + (active ? '. ' + status.title : ''));
  }

  resetTelemetry() {
    this.xyFrames = {};
    this.xyReceivedAt = {};
    this.startTime = null;
    this.latestTime = 0;
    this.positionMode = false;
    this.verticalHeightSamples = [];
    this.verticalSampleAt = null;
    this.detectionReceivedAt = null;
    this.setDetection('No tag data', 'offline');
    this.payloadStamp = null;
    this.clearPayload();
    this.fusedStamp = null;
    this.fusionGeneration = null;
    this.clearFusion();
    ['battery', 'flightMode', 'height', 'speed', 'verticalSpeed', 'fusedPosition'].forEach(name => {
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

  connect(waitForNodes = false) {
    // A pending socket is also a session; repeated clicks must not create duplicates.
    if (this.ros || this.retryTimer !== null) return;
    if (this.expireConnection()) return;
    // Connect also starts the drone's nodes, so retry until its rosbridge is up.
    if (!this.waitUntil) this.waitUntil = Date.now() + (waitForNodes ? 90000 : 15000);
    this.resetTelemetry();
    this.setStatus('Connecting…');
    const ros = new ROSLIB.Ros();
    this.ros = ros;
    ros.on('connection', () => {
      if (this.ros !== ros || this.expireConnection()) return;
      this.waitUntil = 0;
      this.setStatus('Connected', true);
      this.setupTopics(ros);
      this.field('camera').src = 'http://' + this.host + ':8080/stream?topic=/raspicam_node/image&quality=70&type=ros_compressed';
      updateLayout();
    });
    ros.on('error', error => {
      if (this.ros !== ros || this.retry(ros)) return;
      console.error(this.id + ' ROS connection error', error);
      this.failConnection('Connection error');
    });
    ros.on('close', () => {
      if (this.ros !== ros || this.retry(ros)) return;
      if (this.waitUntil) this.failConnection('Connection error');
      else this.disconnect();
    });
    ros.connect('ws://' + this.host + ':9090');
  }

  failConnection(status) {
    this.disconnect(status);
    document.dispatchEvent(new CustomEvent('drone-connect-failed', { detail: { id: this.id, message: status } }));
  }

  expireConnection() {
    if (!this.waitUntil || Date.now() < this.waitUntil) return false;
    this.failConnection('Connection timed out');
    return true;
  }

  retry(ros) {
    if (this.expireConnection()) return true;
    if (ros.isConnected || !(Date.now() < this.waitUntil)) return false;
    const until = this.waitUntil;
    this.disconnect('Waiting for nodes…');
    this.waitUntil = until;
    this.retryTimer = setTimeout(() => {
      if (this.waitUntil !== until) return;
      this.retryTimer = null;
      this.connect();
    }, 2000);
    return true;
  }

  disconnect(status = 'Disconnected') {
    if ((syncStart && syncStart.ids.includes(this.id)) ||
        (routineFlight && routineFlight.ids.includes(this.id))) awaitSyncStart(null);
    clearTimeout(this.retryTimer);
    this.retryTimer = null;
    this.waitUntil = 0;
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
    publishCoordinationSelection();
    updateLayout();
  }

  setupTopics(ros) {
    const publisher = (key, name, messageType) => {
      this.publishers[key] = new ROSLIB.Topic({ ros, name, messageType });
    };
    publisher('mode', '/pidrone/desired/mode', 'pidrone_pkg/Mode');
    publisher('heartbeat', '/pidrone/heartbeat/web_interface', 'std_msgs/Empty');
    publisher('coordination', '/pidrone/coordination/paired', 'std_msgs/Bool');
    publisher('positionMode', '/pidrone/position_control', 'std_msgs/Bool');
    publisher('twist', '/pidrone/desired/twist', 'geometry_msgs/Twist');
    publisher('pose', '/pidrone/desired/pose', 'geometry_msgs/Pose');
    publisher('roomTarget', '/pidrone/desired/room_target', 'geometry_msgs/PoseStamped');
    publisher('cancelRoomTarget', '/pidrone/desired/cancel_room_target', 'std_msgs/Empty');
    publisher('reset', '/pidrone/reset_transform', 'std_msgs/Empty');
    publisher('map', '/pidrone/map', 'std_msgs/Empty');
    publisher('routineStart', '/pidrone/routine/start', 'std_msgs/Empty');
    publisher('stageRelease', '/pidrone/routine/stage_release', 'std_msgs/String');
    // ROSLIB advertises on first publish, and the routine would miss that first start.
    this.publishers.routineStart.advertise();
    this.publishers.stageRelease.advertise();
    this.publish('heartbeat', {});
    this.heartbeat = setInterval(() => this.publish('heartbeat', {}), 1000);

    const subscribe = (name, messageType, callback) => {
      const topic = new ROSLIB.Topic({ ros, name, messageType, queue_length: 2, throttle_rate: 80 });
      topic.subscribe(message => { if (this.ros === ros && this.connected) callback(message); });
      this.subscribers.push(topic);
    };
    subscribe('/pidrone/apriltag/detections', 'pidrone_pkg/TagDetection', message => this.updateDetection(message));
    subscribe('/pidrone/payload/detection', 'pidrone_pkg/PayloadDetection', message => this.updatePayload(message));
    subscribe('/pidrone/apriltag/map', 'pidrone_pkg/TagMap', message => fleetMap.updateTagMap(this.id, message));
    subscribe('/pidrone/apriltag/waypoints', 'pidrone_pkg/TagWaypointMap', message => {
      if (message.map) fleetMap.updateTagMap(this.id, { ...message.map,
        observed_at: message.observed_at, generation: message.generation }, true);
    });
    subscribe('/pidrone/localization/fused', 'geometry_msgs/PoseWithCovarianceStamped', message => this.updateFusion(message));
    subscribe('/pidrone/localization/generation', 'std_msgs/UInt32', message => {
      if (!Number.isInteger(message.data)) return;
      if (this.fusionGeneration !== null && this.fusionGeneration !== message.data) {
        this.fusedStamp = null;
        this.xyReceivedAt = {};
        this.clearFusion();
        // A generation change invalidates control data. A confirmed anchor-layout
        // change clears display-only markers in refreshAnchors().
        delete fleetMap.anchorStamps[this.id];
        delete fleetMap.anchorReports[this.id];
        delete fleetMap.tagReports[this.id];
        delete fleetMap.waypointReports[this.id];
        fleetMap.refreshAnchors();
      }
      this.fusionGeneration = message.data;
    });
    subscribe('/pidrone/localization/ready', 'std_msgs/Bool', message => {
      this.fusionReadyAt = message.data ? Date.now() : null;
      if (!message.data) this.clearFusion();
      if (fleetMap) fleetMap.render();
    });
    subscribe('/pidrone/localization/fusion_status', 'diagnostic_msgs/DiagnosticArray', message => {
      const status = message.status && message.status[0];
      if (!status || typeof status.message !== 'string') return;
      if (status.level !== 0) this.clearFusion();
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
      publishCoordinationSelection();
    });

    subscribe('/pidrone/routine/ready', 'std_msgs/Empty', () => {
      this.routineReadyAt = Date.now();
      releaseSyncStart();
    });
    subscribe('/pidrone/routine/stage_ready', 'std_msgs/String', message => {
      receiveRoutineStage(this.id, message);
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
    subscribe('/pidrone/state', 'pidrone_pkg/State', message => {
      const velocity = message.twist_with_covariance.twist.linear;
      const speed = Math.hypot(velocity.x, velocity.y);
      this.addPoint('speed', this.time(message), speed);
      this.field('speed').textContent = Number.isFinite(speed) ? speed.toFixed(2) + ' m/s' : '—';
      this.render('speed');
    });
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
    this.xyReceivedAt[key] = Date.now();
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
const heldKeys = new Map();
const movementKeys = { j: 'left', l: 'right', i: 'forward', k: 'backward', w: 'up', s: 'down', a: 'yawLeft', d: 'yawRight' };
const actionKeys = { ';': 'arm', t: 'takeoff', r: 'reset', p: 'position', v: 'velocity', m: 'map' };

function sessionsFor(target) { return target === 'both' ? Object.values(drones) : [drones[target]]; }
function currentTarget() {
  const connected = Object.values(drones).filter(drone => drone.connected);
  return connected.length === 1 ? connected[0].id : 'both';
}
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
  if (fleetMap) fleetMap.chart.resize();
  if (countChanged) {
    stopHeldKeys();
    feedback('');
  }
}

function connectDrone(id) {
  stopHeldKeys();
  const selected = sessionsFor(id);
  // Connecting the selected drone adds it to the existing fleet. Its terminal
  // (routines.js) starts the nodes, which aligns map +Y.
  selected.forEach(drone => {
    if (drone.ros || drone.waitUntil) return;
    document.dispatchEvent(new CustomEvent('drone-connect', { detail: drone.id }));
    drone.connect(true);
  });
}

// Routines wait before takeoff (Drone.wait_for_start) until every drone of one
// Execute posts ready; then all of them are released together.
let syncStart = null;
let routineFlight = null;
let routineSerial = 0;
function makeAprilTagPlan(ids, tags) {
  const room = fleetMap.room;
  if (!room || !fleetMap.layoutUsable) throw new Error('A verified room map is required.');
  if (Object.values(drones).filter(drone => drone.connected).length !== ids.length) {
    throw new Error('Connect only the drone(s) selected for this demo before takeoff.');
  }
  const plan = { ids: ids.slice(), tag_ids: tags.slice(), frame: room.frame,
    anchors: room.anchors.map(a => ({ ...a })), generations: {}, starts: {}, points: [] };
  for (const id of ids) {
    const drone = drones[id];
    if (!drone.connected || !drone.fusedPose || drone.fusedReceivedAt === null ||
        Date.now() - drone.fusedReceivedAt > 500 || !fleetMap.hasMatchingReport(id) ||
        !Number.isInteger(drone.fusionGeneration) || drone.field('flightMode').textContent !== 'DISARMED') {
      throw new Error(id + ' needs fresh localization in the shared room and DISARMED mode.');
    }
    const p = drone.fusedPose.pose.pose.position;
    plan.starts[id] = [p.x, p.y];
    plan.generations[id] = drone.fusionGeneration;
  }
  for (const id of tags) {
    const observations = ids.flatMap(name => {
      const report = fleetMap.waypointReports[name], drone = drones[name];
      if (!report || report.frame !== plan.frame || report.generation !== drone.fusionGeneration) return [];
      const now = Math.max(drone.fusedStamp + (Date.now() - drone.fusedReceivedAt) / 1000,
        report.stamp + (Date.now() - report.receivedAt) / 1000);
      return report.tags.filter(tag => tag.id === id && tag.count >= 3 && Number.isFinite(tag.observedAt) &&
        tag.observedAt > 0 && now - tag.observedAt >= 0 && now - tag.observedAt <= 120);
    });
    if (!observations.length) throw new Error('Tag ' + id + ' needs a confirmed observation within the last 120 seconds.');
    if (observations.some(tag => Math.hypot(tag.x - observations[0].x, tag.y - observations[0].y) > .15)) {
      throw new Error('Drones disagree on the position of tag ' + id + '.');
    }
    const tag = observations.sort((a, b) => b.observedAt - a.observedAt)[0];
    plan.points.push({ id, x: tag.x, y: tag.y, stamp: tag.observedAt });
  }
  return plan;
}

function awaitSyncStart(ids, stages = [], plan = null) {
  routineFlight = null;
  syncStart = ids && { ids: ids.slice(), stages: stages.slice(), plan, dispatched: false,
    run: Date.now().toString(36) + '-' + (++routineSerial), expires: Date.now() + 120000 };
  if (ids) ids.forEach(id => { drones[id].routineReadyAt = null; });
}
function confirmSyncStart() {
  if (syncStart) { syncStart.dispatched = true; releaseSyncStart(); }
}
function releaseSyncStart() {
  if (!syncStart || !syncStart.dispatched) return;
  if (Date.now() > syncStart.expires) { awaitSyncStart(null); return; }
  const group = syncStart.ids.map(id => drones[id]);
  if (!group.every(drone => drone.connected && drone.routineReadyAt !== null &&
      Date.now() - drone.routineReadyAt >= 0 && Date.now() - drone.routineReadyAt <= 500 &&
      drone.field('flightMode').textContent === 'DISARMED' && drone.modeReceivedAt !== null &&
      Date.now() - drone.modeReceivedAt >= 0 && Date.now() - drone.modeReceivedAt <= 1500)) return;
  if (syncStart.plan) {
    // Recheck just before release; readiness can take a while. Keep the original
    // frozen coordinates rather than adopting a different path while waiting.
    try {
      const current = makeAprilTagPlan(syncStart.ids, syncStart.plan.tag_ids);
      if (current.ids.some(id => current.generations[id] !== syncStart.plan.generations[id] ||
          Math.hypot(...current.starts[id].map((v, i) => v - syncStart.plan.starts[id][i])) > .06) ||
          current.points.some((p, i) => Math.hypot(p.x - syncStart.plan.points[i].x, p.y - syncStart.plan.points[i].y) > .06) ||
          syncStart.plan.points.some(p => group.some(d => d.fusedStamp - p.stamp > 120))) {
        throw new Error('Tag route changed while waiting; run again after checking the map.');
      }
    } catch (error) { awaitSyncStart(null); feedback(error.message); return; }
  }
  publishCoordinationSelection();
  const run = syncStart.run;
  const plan = syncStart.plan;
  if (syncStart.stages.length) {
    routineFlight = { ids: syncStart.ids, run, stages: syncStart.stages, ready: {} };
  }
  syncStart = null;
  group.forEach(drone => drone.publish('stageRelease', { data: JSON.stringify({ run, stage: 'start', ...(plan ? { plan } : {}) }) }));
  group.forEach(drone => drone.publish('routineStart', {}));
  feedback('Synchronized start sent to ' + group.map(drone => drone.id).join(' and ') + '.');
}

function receiveRoutineStage(id, message) {
  const flight = routineFlight;
  if (!flight || !flight.ids.includes(id)) return;
  let ready;
  try { ready = JSON.parse(message.data); } catch (_) { return; }
  if (!ready || ready.run !== flight.run || ready.stage !== flight.stages[0]) return;
  flight.ready[id] = Date.now();
  const group = flight.ids.map(name => drones[name]);
  if (!group.every(drone => drone.connected && flight.ready[drone.id] !== undefined &&
      Date.now() - flight.ready[drone.id] >= 0 && Date.now() - flight.ready[drone.id] <= 500 &&
      drone.field('flightMode').textContent === 'FLYING' && drone.modeReceivedAt !== null &&
      Date.now() - drone.modeReceivedAt >= 0 && Date.now() - drone.modeReceivedAt <= 1500)) return;
  const stage = flight.stages.shift();
  flight.ready = {};
  if (!flight.stages.length) routineFlight = null;
  group.forEach(drone => drone.publish('stageRelease', { data: JSON.stringify({ run: flight.run, stage }) }));
  feedback('Synchronized ' + stage + ' sent to ' + group.map(drone => drone.id).join(' and ') + '.');
}

function dispatch(action, target = currentTarget()) {
  if (action === 'disarm' || action === 'stop') awaitSyncStart(null);
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

// Keep one selection across staggered arming. Onboard PIDs latch it when armed.
let coordinationPaired = null;
let peerRelayEnabled = null;
let peerRelayBusy = false;
let peerRelayAvailable = false;
function canChangePeerRelay() {
  return Object.values(drones).filter(drone => drone.connected).every(drone =>
    drone.modeReceivedAt !== null && Date.now() - drone.modeReceivedAt >= 0 &&
    Date.now() - drone.modeReceivedAt <= 1500 &&
    drone.field('flightMode').textContent === 'DISARMED');
}

function renderPeerRelay() {
  const button = document.getElementById('peer-relay-toggle');
  button.textContent = 'Peer relay: ' + (peerRelayEnabled === null ? 'loading' : peerRelayEnabled ? 'on' : 'off');
  button.setAttribute('aria-pressed', String(peerRelayEnabled !== false));
  button.disabled = peerRelayBusy || !peerRelayAvailable || !canChangePeerRelay();
  document.getElementById('peer-relay-status').textContent = !peerRelayAvailable
    ? 'Peer relay server unavailable.'
    : peerRelayEnabled
      ? 'Change before arming. Off disables automatic spacing control.'
      : 'Automatic spacing control off for the next flight. Routine synchronization stays on.';
  button.title = document.getElementById('peer-relay-status').textContent;
}

async function updatePeerRelay(enabled) {
  if (peerRelayBusy) return;
  const changing = typeof enabled === 'boolean';
  if (changing && (!peerRelayAvailable || !canChangePeerRelay())) return;
  peerRelayBusy = true;
  renderPeerRelay();
  try {
    const options = { signal: AbortSignal.timeout(2000) };
    if (changing) Object.assign(options, {
      method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Routine-Request': '1' },
      body: JSON.stringify({ enabled })
    });
    const response = await fetch('/api/peer-relay', options);
    const state = await response.json();
    if (!response.ok || typeof state.enabled !== 'boolean') throw new Error('Relay setting unavailable');
    peerRelayEnabled = state.enabled;
    peerRelayAvailable = true;
    publishCoordinationSelection();
  } catch (_) {
    peerRelayAvailable = false;
    if (changing) feedback('Could not confirm the peer relay setting. Wait for the server to reconnect.');
  } finally {
    peerRelayBusy = false;
    renderPeerRelay();
  }
}

function publishCoordinationSelection() {
  renderPeerRelay();
  const connected = Object.values(drones).filter(drone => drone.connected);
  if (!connected.length) { coordinationPaired = null; return; }
  const fresh = drone => drone.modeReceivedAt !== null &&
    Date.now() - drone.modeReceivedAt >= 0 && Date.now() - drone.modeReceivedAt <= 1500;
  if (!connected.every(fresh)) return;
  if (connected.every(drone => drone.field('flightMode').textContent === 'DISARMED')) {
    coordinationPaired = peerRelayEnabled !== false && connected.length === 2;
  }
  if (coordinationPaired === null) return;
  connected.forEach(drone => drone.publish('coordination', { data: coordinationPaired }));
}

function isFormControl(target) {
  return target && (target.isContentEditable || !!target.closest('input, textarea, select, summary'));
}

document.addEventListener('DOMContentLoaded', () => {
  const fullscreenButton = document.getElementById('fullscreen-toggle');
  fullscreenButton.hidden = !document.fullscreenEnabled;
  const updateFullscreen = () => {
    const active = !!document.fullscreenElement;
    const label = active ? 'Exit full screen' : 'Full screen';
    fullscreenButton.setAttribute('aria-label', label);
    fullscreenButton.title = label;
    fullscreenButton.setAttribute('aria-pressed', String(active));
  };
  document.addEventListener('fullscreenchange', updateFullscreen);
  fullscreenButton.addEventListener('click', async () => {
    fullscreenButton.disabled = true;
    try {
      if (document.fullscreenElement) await document.exitFullscreen();
      else await document.documentElement.requestFullscreen();
    } catch (_) {
      feedback('Could not change full screen. Try your browser’s full-screen control.');
    } finally {
      fullscreenButton.disabled = false;
      updateFullscreen();
    }
  });
  updateFullscreen();
  Chart.defaults.global.defaultFontFamily = '-apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
  Chart.defaults.global.defaultFontSize = 12;
  Chart.defaults.global.defaultFontColor = '#58677b';
  ['drone1', 'drone2'].forEach(id => { drones[id] = new DroneSession(id); });
  fleetMap = new FleetMap();
  anchorCalibration = new AnchorCalibration();
  document.getElementById('peer-relay-toggle').addEventListener('click', () => updatePeerRelay(!peerRelayEnabled));
  updatePeerRelay();
  setInterval(() => updatePeerRelay(), 2000);
  setInterval(() => {
    publishCoordinationSelection();
    Object.values(drones).forEach(drone => { drone.expireConnection(); drone.refreshDetection(); drone.refreshVerticalSpeed(); drone.refreshLocalization(); });
    fleetMap.refreshAnchors();
    anchorCalibration.refresh();
  }, 250);
  updateLayout();
  document.addEventListener('click', event => {
    const button = event.target.closest('button');
    if (!button) return;
    if (button.dataset.connectionToggle) {
      const drone = drones[button.dataset.connectionToggle];
      if (drone.ros || drone.waitUntil) {
        stopHeldKeys();
        drone.disconnect();
        document.dispatchEvent(new CustomEvent('drone-disconnect', { detail: drone.id }));
      } else connectDrone(drone.id);
    }
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
  // Drone terminals receive every key, including Space; Esc leaves the terminal.
  if (event.target && event.target.closest && event.target.closest('.script-terminal')) return;
  if (isFormControl(event.target)) return;
  if (key === ' ') {
    event.preventDefault();
    stopHeldKeys();
    dispatch('disarm', 'both');
    return;
  }
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
