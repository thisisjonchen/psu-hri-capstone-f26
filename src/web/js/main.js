'use strict';

const xySources = [
  ['groundTruth', 'Ground truth', []],
  ['ukf', 'UKF', [6, 3]],
  ['camera', 'Camera pose', [2, 3]]
];

function configureXYChart(chart) {
  const xAxis = chart.options.scales.xAxes[0];
  xAxis.display = true;
  Object.assign(xAxis.ticks, { min: -1.6, max: 1.6, stepSize: 0.4, maxRotation: 0 });
  Object.assign(xAxis.scaleLabel, { display: true, labelString: 'X position (m)' });
  Object.assign(chart.options.scales.yAxes[0].ticks, { min: -1.6, max: 1.6, stepSize: 0.4 });
  chart.options.legend.display = false;
  chart.update();
}

class FleetMap {
  constructor() {
    const datasets = Object.values(drones).flatMap(drone => xySources.flatMap(([key, label, dash]) =>
      [0, 1, 2].map(i => ({
        droneId: drone.id, key: key + i, label: 'Drone ' + drone.id.slice(-1) + ' · ' + label,
        data: [], borderColor: drone.id === 'drone1' ? '#0072b2' : '#d55e00',
        borderDash: dash, borderWidth: 2.5, pointRadius: 0, fill: false, lineTension: 0
      }))));
    this.chart = new Chart(document.getElementById('fleet-xyChart').getContext('2d'), {
      type: 'line', data: { datasets }, options: {
        responsive: true, maintainAspectRatio: false, animation: { duration: 0 },
        scales: {
          xAxes: [{ type: 'linear', ticks: {}, scaleLabel: {} }],
          yAxes: [{ ticks: {}, scaleLabel: { display: true, labelString: 'Y position (m)' } }]
        }, legend: { display: false }
      }
    });
    configureXYChart(this.chart);
  }

  render() {
    this.chart.data.datasets.forEach(dataset => {
      dataset.data = drones[dataset.droneId].series[dataset.key].map(point => ({ ...point }));
    });
    this.chart.update();
  }

  clearDrone(id) {
    // Remove only the departed drone's markers.
    this.chart.data.datasets.filter(dataset => dataset.droneId === id).forEach(dataset => { dataset.data = []; });
    this.chart.update();
  }
}

// Each bridge uses the same topic names; all ROS and chart state belongs to its session.
class DroneSession {
  constructor(id) {
    this.id = id;
    this.host = id + '.local';
    this.detectionReceivedAt = null;
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
      if (this.field('camera').getAttribute('src') !== 'nocamera.jpg') {
        this.field('camera').src = 'nocamera.jpg';
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
      this.setDetection('Detector offline', 'offline');
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
      this.setDetection('Detector offline', 'offline');
    }
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
    this.startTime = null;
    this.latestTime = 0;
    this.positionMode = false;
    this.verticalHeightSamples = [];
    this.verticalSampleAt = null;
    this.detectionReceivedAt = null;
    this.setDetection('Detector offline', 'offline');
    ['battery', 'flightMode', 'height', 'speed', 'verticalSpeed'].forEach(name => {
      this.field(name).textContent = '—';
      this.field(name).classList.remove('alert-success', 'alert-danger');
    });
    this.field('camera').src = 'nocamera.jpg';
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
    subscribe('/pidrone/battery', 'pidrone_pkg/Battery', message => {
      if (!Number.isFinite(message.vbat)) return;
      const battery = this.field('battery');
      battery.textContent = message.vbat.toFixed(1) + ' V' + (message.vbat <= 11.3 ? ' EMPTY!' : '');
      battery.classList.toggle('alert-danger', message.vbat <= 11.3);
      battery.classList.toggle('alert-success', message.vbat > 11.3);
    });
    subscribe('/pidrone/mode', 'pidrone_pkg/Mode', message => { this.field('flightMode').textContent = message.mode; });
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
      this.updateXY('ukf', pose);
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
      this.updateXY('groundTruth', message.pose);
      this.render('height');
    });
    subscribe('/pidrone/picamera/pose', 'geometry_msgs/PoseStamped', message => { this.updateXY('camera', message.pose); });
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

  updateXY(key, pose) {
    const { position: p, orientation: q } = pose;
    if (![p.x, p.y, q.w, q.x, q.y, q.z].every(Number.isFinite)) return;
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
  setInterval(() => { Object.values(drones).forEach(drone => { drone.refreshDetection(); drone.refreshVerticalSpeed(); }); }, 250);
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
