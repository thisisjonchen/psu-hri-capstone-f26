'use strict';
// Flight sensor logs for the Logs tab. Pre-Arm starts a recording with a baseline;
// it continues through arm and takeoff and stops at disarm. Logging only: nothing here publishes.

// Raw IMU scaling (FlightRaw.msg). The gyro unit and axis signs follow Betaflight's MSP
// convention and the processed /pidrone/imu axes; confirm them by rotating the drone by hand.
const ACCEL_COUNTS_PER_G = 2048;  // params/multiwii.yaml
const GRAVITY = 9.8;              // flight_controller_node.py
const RAD_TO_DEG = 180 / Math.PI;
const LOG_DISPLAY_POINTS = 1200;  // per series; CSV keeps every sample
const LOG_COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4'];

// [key, topic, type]. Subscribed without throttling while a recording runs.
const LOG_TOPICS = [
  ['imu', '/pidrone/imu', 'sensor_msgs/Imu'],
  ['flight_raw', '/pidrone/flight_raw', 'pidrone_pkg/FlightRaw'],
  ['range', '/pidrone/range', 'sensor_msgs/Range'],
  ['state', '/pidrone/state', 'pidrone_pkg/State'],
  ['flow', '/pidrone/picamera/twist', 'geometry_msgs/TwistStamped'],
  ['flow_quality', '/pidrone/picamera/flow_quality', 'pidrone_pkg/FlowQuality'],
  ['uwb', '/pidrone/localization/uwb', 'geometry_msgs/PoseWithCovarianceStamped'],
  ['fused', '/pidrone/localization/fused', 'geometry_msgs/PoseWithCovarianceStamped'],
  ['commands', '/pidrone/fly_commands', 'pidrone_pkg/RC']
];

// Staged graphs plot `${id}.${stage}.${series}`; the others plot `${id}.${series}`.
const LOG_GRAPHS = [
  { id: 'accel', title: 'Acceleration', unit: 'm/s²', staged: true, series: [['x', 'x (right)'], ['y', 'y (forward)'], ['z', 'z (up)']],
    note: { raw: 'FC accelerometer, scaled only', processed: '/pidrone/imu: offset and gravity removed' } },
  { id: 'rate', title: 'Angular rate', unit: 'deg/s', staged: true, series: [['x', 'Roll rate'], ['y', 'Pitch rate'], ['z', 'Yaw rate']],
    note: { raw: 'FC gyro', processed: '/pidrone/imu: differenced attitude' } },
  { id: 'attitude', title: 'Attitude', unit: 'deg', series: [['roll', 'Roll'], ['pitch', 'Pitch'], ['yaw', 'Yaw']],
    note: 'FC attitude estimate (no raw version)' },
  { id: 'height', title: 'Height', unit: 'm', staged: true, series: [['height', 'Height']],
    note: { raw: 'ToF slant range', processed: 'Fused height' } },
  { id: 'velocity', title: 'Horizontal velocity', unit: 'm/s', staged: true, series: [['x', 'x (right)'], ['y', 'y (forward)']],
    note: { raw: 'Optical flow, body frame', processed: 'Fused velocity, body frame' } },
  { id: 'odometry', title: 'Odometry position', unit: 'm', series: [['x', 'x'], ['y', 'y']],
    note: 'Local odometry: the position that hold uses' },
  { id: 'position', title: 'UWB position', unit: 'm', staged: true, series: [['x', 'x'], ['y', 'y']],
    note: { raw: 'UWB fix, map frame', processed: 'Fused position, map frame' } },
  { id: 'throttle', title: 'Throttle command', unit: 'PWM', series: [['throttle', 'Throttle']],
    note: 'Command sent to the FC' },
  { id: 'commands', title: 'Attitude commands', unit: 'PWM', series: [['roll', 'Roll'], ['pitch', 'Pitch'], ['yaw', 'Yaw']],
    note: 'Commands sent to the FC (1500 = level)' },
  { id: 'motors', title: 'Motor outputs', unit: 'PWM', series: [['m1', 'Motor 1'], ['m2', 'Motor 2'], ['m3', 'Motor 3'], ['m4', 'Motor 4']],
    note: 'Outputs from the FC to the ESCs' },
  { id: 'flow', title: 'Optical-flow blocks', unit: 'blocks', series: [['blocks', 'Blocks']],
    note: 'Blocks used per camera frame (0 = rejected or zero)' },
  { id: 'flowstatus', title: 'Optical-flow status', unit: 'state', series: [['status', 'Status']], stepped: true,
    note: 'Zero means motion below one pixel per frame, reported as 0 m/s',
    ticks: { min: 0, max: 2, stepSize: 1, callback: value => FLOW_STATUS[value] || '' } },
  { id: 'interval', title: 'Sample interval', unit: 'ms', series: [['imu', 'IMU'], ['flight_raw', 'Raw IMU'], ['range', 'ToF'], ['flow', 'Camera'], ['uwb', 'UWB']],
    note: 'Time between consecutive samples' }
];
// FlowQuality.msg: rejected frames are invalid; valid frames with no blocks saw under one pixel of motion.
const FLOW_STATUS = ['rejected', 'zero', 'measured'];
const LOG_UNITS = Object.fromEntries(LOG_GRAPHS.map(graph => [graph.id, graph.unit]));

function stampSeconds(message) {
  const stamp = message && message.header && message.header.stamp;
  return stamp ? stamp.secs + stamp.nsecs / 1e9 : NaN;
}

// Roll, pitch, yaw (deg) of the FC's quaternion_from_euler(roll, pitch, heading).
function eulerDegrees(q) {
  return [
    Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
    Math.asin(Math.max(-1, Math.min(1, 2 * (q.w * q.y - q.z * q.x)))),
    Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
  ].map(angle => angle * RAD_TO_DEG);
}

// Keep each bucket's minimum and maximum so the noise envelope survives thinning.
function decimate(points, limit) {
  if (points.length <= limit) return points;
  const buckets = Math.floor(limit / 2), size = points.length / buckets, result = [];
  for (let b = 0; b < buckets; b++) {
    let low = Math.floor(b * size), high = low;
    for (let i = low + 1; i < Math.floor((b + 1) * size); i++) {
      if (points[i].y < points[low].y) low = i;
      if (points[i].y > points[high].y) high = i;
    }
    result.push(points[Math.min(low, high)]);
    if (low !== high) result.push(points[Math.max(low, high)]);
  }
  return result;
}

class FlightLog {
  constructor() { this.clear(); }

  clear() {
    this.state = 'idle';  // idle | baseline | flight | stopped
    this.series = {};
    this.events = [];
    this.origin = null;   // first sensor stamp (drone clock)
    this.offset = null;   // drone clock minus browser clock, from the latest stamped sample
    this.lastStamp = {};
    this.receivedAt = {};
    this.lastMode = null;
    this.startedAt = null;
    this.stopReason = '';
    this.version = 0;
  }

  get recording() { return this.state === 'baseline' || this.state === 'flight'; }
  get duration() {
    return Object.values(this.series).reduce((latest, points) => Math.max(latest, points.at(-1).x), 0);
  }

  start() {
    this.clear();
    this.state = 'baseline';
    this.startedAt = Date.now();
  }

  stop(reason) {
    if (!this.recording) return;
    this.event('stop', this.droneTime());
    this.state = 'stopped';
    this.stopReason = reason;
    this.version++;
  }

  fresh(topic, maxAge) {
    return this.receivedAt[topic] !== undefined && Date.now() - this.receivedAt[topic] <= maxAge;
  }

  // Time since the recording began, from the message's own stamp; sensor topics also log their interval.
  sample(topic, message, interval = true) {
    const stamp = stampSeconds(message);
    if (!this.recording || !Number.isFinite(stamp) || stamp <= 0) return NaN;
    this.receivedAt[topic] = Date.now();
    this.offset = stamp - Date.now() / 1000;
    if (this.origin === null) this.origin = stamp;
    const previous = this.lastStamp[topic];
    this.lastStamp[topic] = stamp;
    const time = stamp - this.origin;
    if (interval && previous !== undefined && stamp > previous) this.add('interval.' + topic, time, (stamp - previous) * 1000);
    return time;
  }

  // Unstamped messages (commands, mode) are placed using the latest clock offset.
  droneTime() {
    return this.offset === null || this.origin === null ? NaN : Date.now() / 1000 + this.offset - this.origin;
  }

  add(key, time, value) {
    if (!this.recording || !Number.isFinite(time) || !Number.isFinite(value)) return;
    (this.series[key] = this.series[key] || []).push({ x: time, y: value });
    this.version++;
  }

  addAll(prefix, time, values) {
    Object.entries(values).forEach(([name, value]) => this.add(prefix + '.' + name, time, value));
  }

  event(label, time) {
    if (Number.isFinite(time)) this.events.push({ t: time, label });
  }

  updateMode(mode) {
    if (!this.recording || mode === this.lastMode) return;
    if (mode === 'ARMED' || mode === 'FLYING') {
      this.event(mode === 'ARMED' ? 'arm' : 'takeoff', this.droneTime());
      this.state = 'flight';
      this.version++;
    } else if (mode === 'DISARMED' && this.state === 'flight') this.stop('Disarmed');
    this.lastMode = mode;
  }

  receive(key, m) {
    if (!this.recording) return;
    if (key === 'imu') {
      const t = this.sample('imu', m), a = m.linear_acceleration, w = m.angular_velocity;
      this.addAll('accel.processed', t, { x: a.x, y: a.y, z: a.z });
      this.addAll('rate.processed', t, { x: w.x * RAD_TO_DEG, y: w.y * RAD_TO_DEG, z: w.z * RAD_TO_DEG });
      const [roll, pitch, yaw] = eulerDegrees(m.orientation);
      this.addAll('attitude', t, { roll, pitch, yaw });
    } else if (key === 'flight_raw') {
      // Rotate into the processed body axes (x right, y forward); no offset, gravity or filtering.
      const t = this.sample('flight_raw', m), [ax, ay, az] = m.accel, [gx, gy, gz] = m.gyro;
      const scale = GRAVITY / ACCEL_COUNTS_PER_G;
      this.addAll('accel.raw', t, { x: -ay * scale, y: ax * scale, z: az * scale });
      this.addAll('rate.raw', t, { x: gx, y: -gy, z: -gz });
      this.addAll('motors', t, { m1: m.motors[0], m2: m.motors[1], m3: m.motors[2], m4: m.motors[3] });
    } else if (key === 'range') {
      const t = this.sample('range', m);
      if (m.range >= m.min_range && m.range <= m.max_range) this.add('height.raw.height', t, m.range);
    } else if (key === 'state') {
      const t = this.sample('state', m, false), p = m.pose_with_covariance.pose.position, v = m.twist_with_covariance.twist.linear;
      this.add('height.processed.height', t, p.z);
      this.addAll('velocity.processed', t, { x: v.x, y: v.y });
      this.addAll('odometry', t, { x: p.x, y: p.y });
    } else if (key === 'flow') {
      const t = this.sample('flow_twist', m, false), v = m.twist.linear;
      this.addAll('velocity.raw', t, { x: v.x, y: v.y });
    } else if (key === 'flow_quality') {
      const t = this.sample('flow', m);
      this.add('flow.blocks', t, m.valid ? m.blocks : 0);
      this.add('flowstatus.status', t, !m.valid ? 0 : m.blocks > 0 ? 2 : 1);
    } else if (key === 'uwb' || key === 'fused') {
      const t = this.sample(key, m, key === 'uwb'), p = m.pose.pose.position;
      this.addAll(key === 'uwb' ? 'position.raw' : 'position.processed', t, { x: p.x, y: p.y });
    } else if (key === 'commands') {
      const t = this.droneTime();
      this.add('throttle.throttle', t, m.throttle);
      this.addAll('commands', t, { roll: m.roll, pitch: m.pitch, yaw: m.yaw });
    }
  }

  csv() {
    const rows = [];
    Object.entries(this.series).forEach(([key, points]) => {
      const unit = LOG_UNITS[key.split('.')[0]];
      points.forEach(point => rows.push([point.x, key, point.y, unit]));
    });
    this.events.forEach(event => rows.push([event.t, 'event', event.label, '']));
    rows.sort((a, b) => a[0] - b[0]);
    return 'time_s,signal,value,unit\n' + rows.map(([t, ...rest]) => [t.toFixed(4), ...rest].join(',')).join('\n') + '\n';
  }
}

// Shades the pre-arm baseline and marks arm, takeoff and stop on every graph.
const logMarkers = {
  beforeDatasetsDraw(chart) {
    const log = chart.options.flightLog && chart.options.flightLog();
    if (!log || !log.events) return;
    const x = chart.scales['log-x'], area = chart.chartArea, ctx = chart.ctx;
    const arm = log.events.find(event => event.label === 'arm');
    ctx.save();
    ctx.fillStyle = '#58677b14';
    const baselineEnd = x.getPixelForValue(arm ? arm.t : log.duration);
    ctx.fillRect(area.left, area.top, Math.max(0, Math.min(baselineEnd, area.right) - area.left), area.bottom - area.top);
    ctx.strokeStyle = '#58677b';
    ctx.fillStyle = '#58677b';
    ctx.font = '10px ' + Chart.defaults.global.defaultFontFamily;
    ctx.setLineDash([4, 3]);
    log.events.forEach((event, i) => {
      const px = x.getPixelForValue(event.t);
      if (px < area.left || px > area.right) return;
      ctx.beginPath(); ctx.moveTo(px, area.top); ctx.lineTo(px, area.bottom); ctx.stroke();
      ctx.fillText(event.label, px + 3, area.top + 10 + (i % 2) * 11);  // Stagger close events.
    });
    ctx.restore();
  }
};

class LogsView {
  constructor(drones, onShowTelemetry) {
    this.drones = drones;
    this.onShowTelemetry = onShowTelemetry;
    this.selected = 'drone1';
    this.stage = 'raw';
    this.visible = false;
    this.rendered = null;
    this.el = id => document.getElementById(id);
    this.charts = LOG_GRAPHS.map(graph => this.makeChart(graph));
    this.el('view-telemetry').addEventListener('click', () => this.show(false));
    this.el('view-logs').addEventListener('click', () => this.show(true));
    ['drone1', 'drone2'].forEach(id => this.el('log-' + id).addEventListener('click', () => { this.selected = id; this.refresh(true); }));
    ['raw', 'processed'].forEach(stage => this.el('log-' + stage).addEventListener('click', () => { this.stage = stage; this.refresh(true); }));
    this.el('log-csv').addEventListener('click', () => this.downloadCsv());
    this.el('log-png').addEventListener('click', () => this.downloadPng());
    this.refresh(true);
  }

  get log() { return this.drones[this.selected].log; }

  makeChart(graph) {
    const article = document.createElement('article');
    article.className = 'log-chart';
    const title = document.createElement('h4');
    title.textContent = graph.title + ' (' + graph.unit + ')';
    const note = document.createElement('p');
    const wrap = document.createElement('div');
    wrap.className = 'canvas-wrap';
    const canvas = document.createElement('canvas');
    canvas.setAttribute('aria-label', graph.title + ' graph');
    wrap.appendChild(canvas);
    [title, note, wrap].forEach(child => article.appendChild(child));
    this.el('log-grid').appendChild(article);
    const chart = new Chart(canvas.getContext('2d'), {
      type: 'line',
      data: { datasets: graph.series.map(([, label], i) => ({
        label, data: [], borderColor: LOG_COLORS[i], backgroundColor: LOG_COLORS[i],
        borderWidth: 1.5, pointRadius: 0, pointHitRadius: 4, fill: false, lineTension: 0, steppedLine: !!graph.stepped
      })) },
      options: {
        responsive: true, maintainAspectRatio: false, animation: { duration: 0 },
        legend: { display: graph.series.length > 1, labels: { boxWidth: 10, fontSize: 11 } },
        tooltips: { mode: 'nearest', intersect: false },
        scales: {
          xAxes: [{ id: 'log-x', type: 'linear', ticks: { min: 0, max: 10 }, gridLines: { color: '#e4eaf2' },
            scaleLabel: { display: true, labelString: 'Time since Pre-Arm (s)' } }],
          yAxes: [{ id: 'log-y', ticks: graph.ticks || {}, gridLines: { color: '#e4eaf2' } }]
        },
        flightLog: () => this.log
      },
      plugins: [logMarkers]
    });
    return { graph, chart, note, canvas };
  }

  show(logs) {
    this.visible = logs;
    this.el('view-telemetry').setAttribute('aria-selected', String(!logs));
    this.el('view-logs').setAttribute('aria-selected', String(logs));
    this.el('control-map').hidden = logs;
    this.el('flight-logs').hidden = !logs;
    if (logs) this.resize();
    else this.onShowTelemetry();
    this.refresh(true);
  }

  resize() {
    if (this.visible) this.charts.forEach(({ chart }) => chart.resize());
  }

  status(log) {
    const clock = seconds => Math.floor(seconds / 60) + ':' + String(Math.floor(seconds % 60)).padStart(2, '0');
    if (log.state === 'baseline') return 'Recording pre-arm baseline · ' + clock((Date.now() - log.startedAt) / 1000);
    if (log.state === 'flight') return 'Recording flight · ' + clock(log.duration);
    if (log.state === 'stopped') return 'Stopped (' + log.stopReason + ') · ' + clock(log.duration) + ' recorded';
    return 'No recording. Pre-Arm records the next flight.';
  }

  // Called every 250 ms; redraws only when the visible log changed.
  refresh(force = false) {
    const log = this.log;
    ['drone1', 'drone2'].forEach(id => this.el('log-' + id).setAttribute('aria-selected', String(id === this.selected)));
    ['raw', 'processed'].forEach(stage => this.el('log-' + stage).setAttribute('aria-pressed', String(stage === this.stage)));
    this.el('log-status').textContent = this.status(log);
    const empty = !Object.keys(log.series).length;
    this.el('log-csv').disabled = this.el('log-png').disabled = empty;
    const key = [this.selected, this.stage, log.version, log.state].join('|');
    if (!this.visible || (!force && key === this.rendered)) return;
    this.rendered = key;
    const duration = Math.max(10, log.duration);
    this.charts.forEach(({ graph, chart, note }) => {
      note.textContent = typeof graph.note === 'string' ? graph.note : graph.note[this.stage];
      chart.data.datasets.forEach((dataset, i) => {
        const series = graph.series[i][0];
        const name = graph.staged ? graph.id + '.' + this.stage + '.' + series : graph.id + '.' + series;
        dataset.data = decimate(log.series[name] || [], LOG_DISPLAY_POINTS);
      });
      chart.options.scales.xAxes[0].ticks.max = Math.ceil(duration);
      chart.update();
    });
  }

  fileName(extension) {
    const started = new Date(this.log.startedAt).toISOString().slice(0, 19).replace(/:/g, '-');
    return this.selected + '-flight-' + started + '.' + extension;
  }

  download(href, name) {
    const link = document.createElement('a');
    link.href = href;
    link.download = name;
    link.click();
  }

  downloadCsv() {
    const url = URL.createObjectURL(new Blob([this.log.csv()], { type: 'text/csv' }));
    this.download(url, this.fileName('csv'));
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  // One image of every graph as shown (current drone and stage), on white with titles.
  downloadPng() {
    this.refresh(true);
    const columns = 2, scale = window.devicePixelRatio || 1, titleHeight = 40 * scale;
    const width = Math.max(...this.charts.map(({ canvas }) => canvas.width));
    const height = Math.max(...this.charts.map(({ canvas }) => canvas.height)) + titleHeight;
    const header = 48 * scale;
    const image = document.createElement('canvas');
    image.width = width * columns;
    image.height = header + height * Math.ceil(this.charts.length / columns);
    const ctx = image.getContext('2d');
    ctx.fillStyle = '#fff';
    ctx.fillRect(0, 0, image.width, image.height);
    ctx.fillStyle = '#172337';
    ctx.font = '600 ' + 18 * scale + 'px ' + Chart.defaults.global.defaultFontFamily;
    ctx.fillText('Drone ' + this.selected.slice(-1) + ' · ' + this.stage + ' · ' + this.status(this.log), 12 * scale, 30 * scale);
    this.charts.forEach(({ graph, canvas, note }, i) => {
      const x = (i % columns) * width, y = header + Math.floor(i / columns) * height;
      ctx.fillStyle = '#172337';
      ctx.font = '600 ' + 14 * scale + 'px ' + Chart.defaults.global.defaultFontFamily;
      ctx.fillText(graph.title + ' (' + graph.unit + ')', x + 12 * scale, y + 18 * scale);
      ctx.fillStyle = '#58677b';
      ctx.font = 12 * scale + 'px ' + Chart.defaults.global.defaultFontFamily;
      ctx.fillText(note.textContent, x + 12 * scale, y + 34 * scale);
      ctx.drawImage(canvas, x, y + titleHeight);
    });
    this.download(image.toDataURL('image/png'), this.fileName('png'));
  }
}
