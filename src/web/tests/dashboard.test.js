'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

// Test command routing without opening network sockets or contacting hardware.
function dashboard({ discover = true } = {}) {
  const listeners = {};
  const calibrationRequests = [];
  const timers = new Map();
  const maintenanceTimers = new Map();
  let nextTimer = 0;
  let now = 1000;
  const elements = new Map();
  function element(value = '') {
    const classes = new Set();
    return {
      value, textContent: '', dataset: {}, src: 'nocamera.svg', listeners: {},
      children: [],
      replaceChildren() { this.children = []; },
      appendChild(child) { this.children.push(child); },
      classList: {
        toggle(name, enabled) { enabled ? classes.add(name) : classes.delete(name); },
        remove(...names) { names.forEach(name => classes.delete(name)); },
        contains(name) { return classes.has(name); }
      },
      addEventListener(name, callback) { this.listeners[name] = callback; }, getContext() { return {}; },
      getAttribute(name) { return this[name]; }, setAttribute(name, value) { this[name] = value; }, closest() { return null; }
    };
  }
  for (const id of ['drone1', 'drone2']) {
    const fields = new Map();
    const html = fs.readFileSync(path.join(__dirname, '../index.html'), 'utf8');
    const panel = html.split('id="' + id + '"')[1].split('<section class="drone-dashboard"')[0];
    for (const match of panel.matchAll(/data-field="([^"]+)"/g)) fields.set(match[1], element());
    const root = element();
    root.querySelector = selector => fields.get(selector.match(/data-field="([^"]+)"/)[1]);
    elements.set(id, root);
    elements.set(id + '-camera', element());
    elements.set(id + '-detection', element());
  }
  for (const id of ['commandFeedback', 'fleetGrid', 'fleet-xyChart', 'controlsHeading', 'drone1-connectionStatus', 'drone2-connectionStatus', 'droneSelection',
    'room-status', 'room-fit', 'room-zoom-in', 'room-zoom-out', 'map-frame', 'map-placeholder', 'map-legend', 'map-sources',
    'target-preview', 'target-drone', 'target-x', 'target-y', 'target-submit', 'target-clear', 'target-send', 'target-stop', 'target-command-status', 'target-status', 'map-stream-status']) elements.set(id, element());
  elements.get('target-drone').value = 'drone1';
  for (const name of ['scan', 'measure', 'apply', 'cancel', 'status', 'preview']) elements.set('calibration-' + name, element());
  for (let i = 0; i < 4; i++) {
    elements.set('calibration-anchor-' + i, element());
  }
  for (const id of ['drone1', 'drone2']) {
    for (const status of [elements.get(id + '-connectionStatus'), elements.get(id).querySelector('[data-field="connectionStatus"]')]) {
      const icon = element(); icon.dataset.state = 'disconnected';
      const subtitle = element(); subtitle.textContent = 'Disconnected';
      status.querySelector = selector => selector === '[data-state-icon]' ? icon : subtitle;
    }
  }
  elements.get('droneSelection').value = 'drone1';
  elements.get('fleetGrid').dataset.count = '0';
  const document = {
    createElement: () => element(),
    getElementById: id => elements.get(id),
    querySelector: () => elements.get('droneSelection'),
    addEventListener: (name, callback) => { listeners[name] = callback; }
  };
  class Ros {
    constructor() { this.events = {}; this.isConnected = false; this.topics = []; }
    on(name, callback) { this.events[name] = callback; }
    connect(url) { this.url = url; }
    open() { this.isConnected = true; this.events.connection(); }
    close() { this.isConnected = false; this.events.close(); }
  }
  class Topic {
    constructor(options) { Object.assign(this, options); this.messages = []; this.ros.topics.push(this); }
    publish(message) { this.messages.push(JSON.parse(JSON.stringify(message))); }
    subscribe(callback) { this.callback = callback; }
    unsubscribe() { this.unsubscribed = true; }
    unadvertise() { this.unadvertised = true; }
    receive(message) { this.callback(message); }
  }
  class Chart {
    constructor(ctx, config) {
      Object.assign(this, config);
      this.chartArea = { left: 55, right: 455, top: 10, bottom: 410 };
      this.scales = {};
      [...this.options.scales.xAxes, ...this.options.scales.yAxes].forEach(axis => {
        this.scales[axis.id] = { options: axis,
          buildTicks() { this.min = axis.ticks.min; this.max = axis.ticks.max; this.ticks = [this.min, this.max]; },
          convertTicksToLabels() {} };
      });
    }
    update() { (this.plugins || []).forEach(plugin => { if (plugin.afterLayout) plugin.afterLayout(this); }); }
    resize() {}
  }
  Chart.defaults = { global: {} };
  const context = vm.createContext({
    AbortSignal,
    fetch(url, options = {}) {
      assert.equal(url, '/api/calibration');
      if (options.method === 'POST') {
        assert.equal(options.headers['X-Calibration-Request'], '1');
        calibrationRequests.push(JSON.parse(options.body));
        return Promise.resolve({ ok: true });
      }
      return new Promise(() => {}); // Status delivery is controlled by each test.
    },
    document, window: { addEventListener(name, callback) { listeners[name] = callback; } },
    console: { error() {} }, Chart,
    Quaternion: class { rotateVector(vector) { return vector; } },
    ROSLIB: { Ros, Topic, Message: class { constructor(data) { Object.assign(this, data); } } },
    Date: class extends Date { static now() { return now; } },
    setInterval(callback, delay) { const id = ++nextTimer; (delay === 250 || delay === 500 ? maintenanceTimers : timers).set(id, callback); return id; },
    clearInterval(id) { timers.delete(id); }
  });
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/anchor-calibration.js'), 'utf8'), context);
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/main.js'), 'utf8'), context);
  listeners.DOMContentLoaded();
  const api = vm.runInContext('({ drones, fleetMap, anchorCalibration, dispatch, heldKeys, connectSelected, currentTarget })', context);
  const receive = (drone, name, message) => drone.subscribers.find(topic => topic.name === name).receive(message);
  const connect = id => {
    api.drones[id].connect(); api.drones[id].ros.open();
    if (discover) receive(api.drones[id], '/pidrone/localization/anchors', anchorsAt(100));
    return api.drones[id];
  };
  const key = (name, value, target = element(), repeat = false) => listeners[name]({ key: value, target, repeat, preventDefault() {} });
  return { ...api, calibrationRequests, connect, receive, key, elements, timers, maintenanceTimers, listeners, element, advanceTime(ms) { now += ms; } };
}
const stamp = secs => ({ header: { stamp: { secs, nsecs: 0 } } });
const rangeAt = (secs, range) => ({ ...stamp(secs), range, min_range: 0.05, max_range: 1.2 });
const last = (drone, publisher) => drone.publishers[publisher].messages.at(-1);
const anchorsAt = (secs, points = [[0, 0, 2.25], [3, 0, 2.25], [3, 6, 2.25], [0, 6, 2.25]]) => ({
  header: { stamp: { secs, nsecs: 0 }, frame_id: 'map' },
  anchor_ids: ['1150', '111C', '1151', '0CA8'].slice(0, points.length),
  positions: points.map(([x, y, z]) => ({ x, y, z }))
});

test('room targets require live fused localization and explicit send, stay isolated, and stop on conflict', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.fleetMap.previewTarget('drone1', 1, 2);
  app.fleetMap.sendTarget();
  assert.equal(a.publishers.roomTarget.messages.length, 0);
  const pose = { header: { stamp: { secs: 100, nsecs: 0 }, frame_id: 'map' },
    pose: { pose: { position: { x: .5, y: 1, z: .4 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } } };
  app.receive(a, '/pidrone/mode', { mode: 'FLYING' });
  app.receive(a, '/pidrone/localization/fused', pose);
  app.receive(a, '/pidrone/localization/ready', { data: true });
  assert.equal(app.elements.get('target-send').disabled, false);
  app.fleetMap.sendTarget();
  assert.equal(a.publishers.roomTarget.messages.length, 1);
  assert.equal(b.publishers.roomTarget.messages.length, 0);
  assert.equal(last(a, 'roomTarget').header.frame_id, 'map');
  assert.deepEqual(last(a, 'roomTarget').pose.position, { x: 1, y: 2, z: .4 });
  app.elements.get('target-stop').listeners.click();
  assert.equal(a.publishers.cancelRoomTarget.messages.length, 1);
  app.advanceTime(600);
  a.refreshLocalization();
  app.fleetMap.sendTarget();
  assert.equal(a.publishers.roomTarget.messages.length, 1);
  assert.equal(a.fusedPose, null);
  assert.equal(b.field('fusedPosition').textContent, '—');
  app.receive(b, '/pidrone/localization/anchors', anchorsAt(101, [[0,0,2.25],[4,0,2.25],[4,6,2.25],[0,6,2.25]]));
  assert.equal(app.fleetMap.layoutUsable, false);
  assert.equal(b.publishers.cancelRoomTarget.messages.length, 1);
});

test('fusion faults and reference resets clear old markers and permit a new ROS clock epoch', () => {
  const app = dashboard(), a = app.connect('drone1');
  const pose = secs => ({ header: { stamp: { secs, nsecs: 0 }, frame_id: 'map' },
    pose: { pose: { position: { x: .5, y: 1, z: .4 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } } });
  app.receive(a, '/pidrone/localization/generation', { data: 0 });
  app.receive(a, '/pidrone/localization/fused', pose(100));
  app.receive(a, '/pidrone/localization/fusion_status', { status: [{ level: 1, message: 'UWB innovation rejected' }] });
  assert.equal(a.fusedPose, null);
  assert.equal(a.series.fused0.length, 0);
  app.receive(a, '/pidrone/localization/fused', pose(100));
  assert.equal(a.fusedPose, null);
  app.receive(a, '/pidrone/localization/generation', { data: 1 });
  app.receive(a, '/pidrone/localization/anchors', anchorsAt(50));
  assert.equal(app.fleetMap.layoutUsable, true);
  app.receive(a, '/pidrone/localization/fused', pose(50));
  assert.equal(a.fusedStamp, 50);
});

test('two connections keep telemetry, charts, clocks, and cameras isolated', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  assert.equal(a.ros.url, 'ws://drone1.local:9090');
  assert.equal(b.ros.url, 'ws://drone2.local:9090');
  app.receive(a, '/pidrone/battery', { vbat: 10.8 });
  app.receive(b, '/pidrone/battery', { vbat: 12.4 });
  assert.equal(a.field('battery').textContent, '10.8 V EMPTY!');
  assert.equal(b.field('battery').textContent, '12.4 V');
  app.receive(a, '/pidrone/range', { ...stamp(100), range: 0.2 });
  app.receive(b, '/pidrone/range', { ...stamp(900), range: 0.4 });
  assert.equal(a.heightChart.data.datasets[0].data[0].y, 0.2);
  assert.equal(b.heightChart.data.datasets[0].data[0].y, 0.4);
  assert.equal(a.startTime, 100);
  assert.equal(b.startTime, 900);
  assert.match(a.field('camera').src, /drone1.local:8080/);
  assert.match(b.field('camera').src, /drone2.local:8080/);
  assert.equal(app.timers.size, 2);
});

test('shared commands reach both bridges and individual commands only their target', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.dispatch('arm');
  assert.equal(last(a, 'mode').mode, 'ARMED');
  assert.equal(last(b, 'mode').mode, 'ARMED');
  app.dispatch('takeoff', 'drone2');
  assert.equal(last(a, 'mode').mode, 'ARMED');
  assert.equal(last(b, 'mode').mode, 'FLYING');
  for (const drone of [a, b]) app.receive(drone, '/pidrone/position_control', { data: true });
  app.dispatch('forward', 'both');
  for (const drone of [a, b]) {
    assert.equal(drone.publishers.positionMode.messages.length, 0);
    assert.deepEqual(last(drone, 'pose').position, { x: 0, y: 0.1, z: 0 });
  }
});

test('an offline partner blocks shared flight commands without queued or partial sends', () => {
  const app = dashboard();
  const a = app.connect('drone1');
  app.dispatch('takeoff', 'both');
  app.dispatch('forward', 'both');
  assert.equal(a.publishers.mode.messages.length, 0);
  assert.equal(a.publishers.pose.messages.length, 0);
  assert.match(app.elements.get('commandFeedback').textContent, /drone2 first/);
  app.dispatch('disarm', 'both');
  assert.equal(last(a, 'mode').mode, 'DISARMED');
  assert.equal(app.drones.drone2.publish('mode', { mode: 'FLYING' }), false);
});

test('each drone follows its reported position mode for keyboard movement', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.receive(a, '/pidrone/position_control', { data: true });
  app.receive(b, '/pidrone/position_control', { data: false });
  app.key('keydown', 'i');
  assert.equal(last(a, 'pose').position.y, 0.1);
  assert.equal(last(b, 'twist').linear.y, 0.1);
  assert.equal(a.positionMode, true);
  assert.equal(b.positionMode, false);
});

test('key release stops original recipients even after focus changes', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.elements.get('droneSelection').value = 'drone1';
  app.key('keydown', 'i');
  app.elements.get('droneSelection').value = 'drone2';
  const input = app.element(); input.closest = () => input;
  app.key('keyup', 'i', input);
  assert.equal(last(a, 'twist').linear.y, 0);
  assert.equal(last(b, 'twist').linear.y, 0);
  const sent = b.publishers.twist.messages.length;
  app.key('keydown', 'i', input);
  assert.equal(b.publishers.twist.messages.length, sent);
});

test('Space disarms all despite individual selection and form focus', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.elements.get('droneSelection').value = 'drone1';
  const input = app.element(); input.closest = () => input;
  app.key('keydown', ' ', input);
  assert.equal(last(a, 'mode').mode, 'DISARMED');
  assert.equal(last(b, 'mode').mode, 'DISARMED');
});

test('connection selection leaves shared targeting intact and window blur stops movement', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.key('keydown', 'i');
  app.elements.get('droneSelection').value = 'drone2';
  assert.equal(app.currentTarget(), 'both');
  app.listeners.blur();
  assert.equal(last(a, 'twist').linear.y, 0);
  assert.equal(last(b, 'twist').linear.y, 0);
  assert.equal(app.heldKeys.size, 0);
  app.key('keydown', 'a');
  app.listeners.blur();
  assert.equal(last(b, 'twist').angular.z, 0);
});

test('disconnect and reconnect clear only that drone and ignore stale callbacks', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  const oldRos = a.ros;
  const oldRange = a.subscribers.find(topic => topic.name === '/pidrone/range');
  a.connect();
  assert.equal(a.ros, oldRos);
  assert.equal(app.timers.size, 2);
  a.disconnect();
  assert.equal(app.timers.size, 1);
  assert.equal(b.connected, true);
  assert.equal(a.field('camera').src, 'nocamera.svg');
  app.connect('drone1');
  oldRange.receive({ ...stamp(500), range: 9 });
  oldRos.events.close();
  assert.equal(a.connected, true);
  assert.equal(a.series.range.length, 0);
  assert.equal(app.timers.size, 2);
  app.receive(a, '/pidrone/range', { ...stamp(1), range: 0.1 });
  assert.equal(a.startTime, 1);
});

test('unexpected close or error stops heartbeats and clears publishers', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  a.ros.close();
  b.ros.events.error(new Error('offline'));
  assert.equal(app.timers.size, 0);
  assert.equal(Object.keys(a.publishers).length, 0);
  assert.equal(Object.keys(b.publishers).length, 0);
  assert.equal(app.elements.get('drone2-connectionStatus').querySelector('[data-connection-text]').textContent, 'Connection error');
});

test('UKF analysis, speed updates, and bounded streams stay independent', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.receive(a, '/pidrone/ukf_stats', { ...stamp(100), error: 0.02, stddev: 0.03 });
  a.toggleAnalysis(app.element());
  assert.equal(a.heightChart.options.scales.yAxes[0].ticks.min, undefined);
  assert.equal(a.heightChart.options.scales.yAxes[0].ticks.max, undefined);
  assert.equal(a.heightChart.data.datasets[0].data[0].y, 0.02);
  assert.equal(b.analysis, false);
  app.receive(a, '/pidrone/picamera/twist', { ...stamp(101), twist: { linear: { x: 0.3, y: 0.4 } } });
  assert.equal(a.speedChart.data.datasets[0].data[0].y, 0.5);
  for (let secs = 102; secs < 115; secs++) app.receive(a, '/pidrone/range', { ...stamp(secs), range: secs / 100 });
  assert.ok(a.series.range.length <= 6);
  assert.equal(b.series.range.length, 0);
  a.toggleAnalysis(app.element());
  assert.equal(a.heightChart.data.datasets[0].data.at(-1).y, 1.14);
  assert.equal(a.heightChart.options.scales.yAxes[0].ticks.min, 0);
  assert.equal(a.heightChart.options.scales.yAxes[0].ticks.max, 0.6);
});

test('connecting the other drone preserves its partner and automatically targets both', () => {
  const app = dashboard();
  app.connectSelected();
  const firstRos = app.drones.drone1.ros;
  assert.equal(app.drones.drone2.ros, null);
  firstRos.open();
  assert.equal(app.currentTarget(), 'drone1');
  app.elements.get('droneSelection').value = 'drone2';
  app.connectSelected();
  assert.equal(app.drones.drone1.ros, firstRos);
  app.drones.drone2.ros.open();
  assert.equal(app.elements.get('fleetGrid').dataset.count, '2');
  assert.equal(app.currentTarget(), 'both');
  const secondRos = app.drones.drone2.ros;
  app.elements.get('droneSelection').value = 'drone1';
  app.connectSelected();
  assert.equal(app.drones.drone1.ros, firstRos);
  assert.equal(app.drones.drone2.ros, secondRos);
  assert.equal(app.currentTarget(), 'both');
});

test('a dropped partner stops movement until a fresh key press', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.key('keydown', 'i');
  a.ros.close();
  assert.equal(last(b, 'twist').linear.y, 0);
  app.key('keydown', 'i', app.element(), true);
  assert.equal(last(b, 'twist').linear.y, 0);
  app.key('keyup', 'i');
  app.key('keydown', 'i');
  assert.equal(last(b, 'twist').linear.y, 0.1);
});

test('the speed header shows horizontal speed and vertical direction while the graph keeps signed velocity', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.receive(a, '/pidrone/picamera/twist', { ...stamp(100), twist: { linear: { x: 0.3, y: 0.4, z: 99 } } });
  assert.equal(a.field('speed').textContent, '0.50 m/s');
  assert.equal(a.field('verticalSpeed').textContent, '—');
  app.receive(a, '/pidrone/range', rangeAt(100, 0.4));
  app.receive(a, '/pidrone/range', rangeAt(100.2, 0.36));
  assert.equal(a.field('speed').textContent, '0.50 m/s');
  assert.equal(a.field('verticalSpeed').textContent, '0.20 m/s ↓');
  assert.equal(a.speedChart.data.datasets[0].data[0].y, 0.5);
  assert.ok(Math.abs(a.speedChart.data.datasets[1].data[0].y + 0.2) < 1e-6);
  assert.equal(b.series.verticalSpeed.length, 0);
  assert.equal(b.field('verticalSpeed').textContent, '—');
  app.receive(a, '/pidrone/range', rangeAt(101, 0.36));
  app.receive(a, '/pidrone/range', rangeAt(101.2, 0.36));
  assert.equal(a.field('verticalSpeed').textContent, '0.00 m/s');
  app.receive(a, '/pidrone/range', rangeAt(101.3, NaN));
  assert.equal(a.field('verticalSpeed').textContent, '—');
  a.disconnect();
  assert.equal(a.field('speed').textContent, '—');
  assert.equal(a.field('verticalSpeed').textContent, '—');
  assert.equal(a.series.verticalSpeed.length, 0);
});

test('vertical speed follows timestamped height with smoothing and ignores repeated or older stamps', () => {
  const app = dashboard();
  const a = app.connect('drone1');
  const noise = [-0.001, 0.002, -0.002, 0.001, -0.001, 0.002, -0.001];
  noise.forEach((offset, i) => app.receive(a, '/pidrone/range', rangeAt(100 + i * 0.05, 0.2 + i * 0.05 * 0.2 + offset)));
  assert.ok(Math.abs(a.series.verticalSpeed.at(-1).y - 0.2) < 0.02);
  assert.match(a.field('verticalSpeed').textContent, /m\/s ↑$/);
  const value = a.field('verticalSpeed').textContent;
  const count = a.series.verticalSpeed.length;
  app.receive(a, '/pidrone/range', rangeAt(100.3, 1.1));
  app.receive(a, '/pidrone/range', rangeAt(100.1, 1.1));
  assert.equal(a.field('verticalSpeed').textContent, value);
  assert.equal(a.series.verticalSpeed.length, count);
});

test('vertical speed clears on stale, invalid, or interrupted range data and reconnect starts fresh', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  const send = (drone, time, height) => app.receive(drone, '/pidrone/range', rangeAt(time, height));
  send(a, 100, 0.2); send(a, 100.2, 0.24);
  send(b, 100, 0.3); send(b, 100.2, 0.28);
  assert.equal(a.field('verticalSpeed').textContent, '0.20 m/s ↑');
  assert.equal(b.field('verticalSpeed').textContent, '0.10 m/s ↓');
  app.advanceTime(501);
  a.refreshVerticalSpeed();
  assert.equal(a.field('verticalSpeed').textContent, '—');
  assert.equal(a.series.verticalSpeed.length, 0);
  assert.equal(b.field('verticalSpeed').textContent, '0.10 m/s ↓');
  send(a, 101, 0.4);
  assert.equal(a.field('verticalSpeed').textContent, '—');
  send(a, 101.2, 0.44);
  assert.equal(a.field('verticalSpeed').textContent, '0.20 m/s ↑');
  send(a, 101.3, 8);
  assert.equal(a.field('height').textContent, '—');
  assert.equal(a.field('verticalSpeed').textContent, '—');
  send(a, 102, 0.4); send(a, 102.2, 0.44);
  send(a, 103, 0.7);
  assert.equal(a.field('verticalSpeed').textContent, '—');
  a.disconnect();
  app.connect('drone1');
  send(a, 104, 0.2);
  assert.equal(a.field('verticalSpeed').textContent, '—');
  send(a, 104.2, 0.2);
  assert.equal(a.field('verticalSpeed').textContent, '0.00 m/s');
  app.receive(a, '/pidrone/range', rangeAt(0, 0.3));
  assert.equal(a.field('verticalSpeed').textContent, '—');
});

const poseAt = (x, y) => ({ position: { x, y, z: 0 }, orientation: { w: 1, x: 0, y: 0, z: 0 } });
const sharedMarker = (app, droneId, key) => app.fleetMap.chart.data.datasets.find(dataset => dataset.droneId === droneId && dataset.key === key);

const localizationAt = (seconds, x, y, z) => ({
  header: { stamp: { secs: seconds, nsecs: 0 }, frame_id: 'map' },
  pose: { pose: { position: { x, y, z }, orientation: { x: 0, y: 0, z: 0, w: 1 } }, covariance: Array(36).fill(0) }
});

test('UWB and ToF telemetry stays per drone and room coordinates remain visible', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.receive(a, '/pidrone/localization/pose', localizationAt(100, 2.8, 5.5, 0.4));
  app.receive(b, '/pidrone/localization/pose', localizationAt(100, 1, 2, 0.6));
  assert.match(a.field('localizationPosition').textContent, /X 2.80 · Y 5.50 · Z 0.40 m/);
  assert.match(b.field('localizationPosition').textContent, /Z 0.60 m/);
  assert.equal(sharedMarker(app, 'drone1', 'uwb1').data[0].x, 2.8);
  assert.equal(sharedMarker(app, 'drone2', 'uwb1').data[0].x, 1);
  assert.ok(app.fleetMap.chart.options.scales.yAxes[0].ticks.max >= 5.5);
  a.disconnect();
  assert.equal(sharedMarker(app, 'drone1', 'uwb1').data.length, 0);
  assert.equal(sharedMarker(app, 'drone2', 'uwb1').data[0].x, 1);
});

test('localization gaps retain a labeled last reading while stale map markers clear', () => {
  const app = dashboard();
  const a = app.connect('drone1');
  const send = (seconds, z = 0.4) => app.receive(a, '/pidrone/localization/pose', localizationAt(seconds, 1, 2, z));
  send(100);
  app.advanceTime(400);
  send(100);
  app.advanceTime(200);
  app.maintenanceTimers.forEach(callback => callback());
  const reading = 'X 1.00 · Y 2.00 · Z 0.40 m';
  assert.equal(a.field('localizationPosition').textContent, reading);
  assert.equal(a.field('localizationPosition').dataset.state, 'stale');
  assert.match(a.field('localizationStatus').textContent, /^Last reading · Localization stale$/);
  assert.equal(sharedMarker(app, 'drone1', 'uwb0').data.length, 0);
  send(101);
  for (const message of ['Sensor timestamps are not aligned', 'UWB missing or stale', 'ToF missing, invalid or stale']) {
    app.receive(a, '/pidrone/localization/status', { status: [{ level: 1, message }] });
    assert.equal(a.field('localizationStatus').textContent, 'Last reading · ' + message);
    assert.equal(a.field('localizationPosition').textContent, reading);
    assert.equal(a.field('localizationPosition').dataset.state, 'stale');
    assert.equal(sharedMarker(app, 'drone1', 'uwb0').data.length, 0);
    assert.equal(a.localizationReceivedAt, null);
  }
  send(102, NaN);
  assert.equal(a.field('localizationPosition').textContent, reading);
  assert.match(a.field('localizationStatus').textContent, /^Last reading · Invalid position$/);
  const unstamped = localizationAt(102, 9, 9, 9);
  delete unstamped.header.stamp;
  app.receive(a, '/pidrone/localization/pose', unstamped);
  assert.equal(a.field('localizationPosition').textContent, reading);
  assert.equal(a.localizationStamp, 101);
  send(102);
  assert.match(a.field('localizationPosition').textContent, /Z 0.40/);
  assert.equal(a.field('localizationPosition').dataset.state, 'ready');
  assert.match(a.field('localizationStatus').textContent, /^Ready/);
  a.disconnect();
  assert.equal(a.field('localizationPosition').textContent, '—');
  app.connect('drone1');
  send(1);
  assert.match(a.field('localizationPosition').textContent, /Z 0.40/);
});

test('shared map isolates pose streams and clears only the disconnected drone', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.receive(a, '/pidrone/picamera/pose', { header: { frame_id: 'map' }, pose: poseAt(-0.5, 0.2) });
  app.receive(b, '/pidrone/picamera/pose', { header: { frame_id: 'map' }, pose: poseAt(0.5, -0.2) });
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data[0].x, -0.5);
  assert.equal(sharedMarker(app, 'drone2', 'camera2').data[0].x, 0.5);
  app.receive(a, '/pidrone/state/ground_truth', { header: { ...stamp(100).header, frame_id: 'map' }, pose: poseAt(-0.4, 0.3) });
  assert.equal(sharedMarker(app, 'drone1', 'groundTruth2').data[0].x, -0.4);
  app.receive(b, '/pidrone/state/ukf_2d', { header: { ...stamp(100).header, frame_id: 'map' }, pose_with_covariance: { pose: poseAt(0.4, -0.3), covariance: Array(36).fill(0) } });
  assert.equal(sharedMarker(app, 'drone2', 'ukf2').data[0].x, 0.4);
  app.receive(a, '/pidrone/picamera/pose', { pose: poseAt(NaN, 0) });
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data[0].x, -0.5);
  const send = (drone, x) => app.receive(drone, '/pidrone/picamera/pose', { header: { frame_id: 'map' }, pose: poseAt(x, 0) });
  send(a, -0.4);
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data[0].x, -0.4);
  a.disconnect();
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data.length, 0);
  assert.equal(sharedMarker(app, 'drone2', 'camera2').data[0].x, 0.5);
  a.connect();
  a.ros.open();
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data.length, 0);
  app.receive(a, '/pidrone/localization/anchors', anchorsAt(1));
  send(a, -0.2);
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data[0].x, -0.2);
});

const tags = (tag_ids, zones, camera_online = true) => ({ tag_ids, zones, camera_online });

test('room bounds stay stable through movement, faults and resize with equal metric scale', () => {
  const app = dashboard(), map = app.fleetMap;
  const a = app.connect('drone1');
  const limits = () => JSON.stringify(map.chart.options.scales);
  const initial = limits();
  app.receive(a, '/pidrone/localization/pose', localizationAt(100, 100, -100, 0.4));
  assert.equal(limits(), initial);
  assert.match(app.elements.get('map-stream-status').textContent, /Outside current view/);
  app.receive(a, '/pidrone/localization/status', { status: [{ level: 1, message: 'No fix' }] });
  a.disconnect();
  assert.equal(limits(), initial);
  map.chart.chartArea.right = 855;
  map.render();
  const x = map.chart.scales['room-x'], y = map.chart.scales['room-y'];
  assert.ok(Math.abs(800 / (x.max - x.min) - 400 / (y.max - y.min)) < 1e-9);
  assert.ok(x.min <= -0.5 && x.max >= 3.5 && y.min <= -0.5 && y.max >= 6.5);
  app.elements.get('room-zoom-in').listeners.click();
  assert.ok(y.max - y.min < 7);
  app.elements.get('room-fit').listeners.click();
  assert.equal(y.max - y.min, 7);
});

test('room previews stay separate from telemetry, reject invalid targets and never send commands', () => {
  const app = dashboard(), map = app.fleetMap;
  const a = app.connect('drone1'), b = app.connect('drone2');
  const sent = [a, b].map(drone => Object.values(drone.publishers).map(topic => topic.messages.length));
  const submit = (id, x, y) => {
    app.elements.get('target-drone').value = id;
    app.elements.get('target-x').value = x;
    app.elements.get('target-y').value = y;
    app.elements.get('target-preview').listeners.submit({ preventDefault() {} });
  };
  submit('drone1', '1', '2');
  submit('drone2', '2', '4');
  for (const invalid of [['', '2'], ['Infinity', '2'], ['9', '2'], ['1', '-1']]) submit('drone1', ...invalid);
  assert.equal(map.targets.drone1.x, 1);
  assert.equal(map.targets.drone2.y, 4);
  const target = id => map.chart.data.datasets.find(d => d.targetDrone === id).data;
  assert.equal(target('drone1')[0].x, 1);
  assert.deepEqual([a, b].map(drone => Object.values(drone.publishers).map(topic => topic.messages.length)), sent);
  app.receive(a, '/pidrone/picamera/pose', { header: { frame_id: 'odom' }, pose: poseAt(1, 2) });
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data.length, 0);
  assert.match(app.elements.get('map-stream-status').textContent, /Hidden until aligned to map: Drone 1 Camera pose \(odom\)/);
  app.receive(a, '/pidrone/picamera/pose', { header: { frame_id: '/map' }, pose: poseAt(1, 2) });
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data[0].x, 1);
  app.receive(a, '/pidrone/picamera/pose', { pose: poseAt(1, 2) });
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data.length, 0);
  a.disconnect();
  assert.equal(app.elements.get('map-stream-status').textContent, '');
  assert.equal(target('drone1').length, 1);
  app.elements.get('target-clear').listeners.click();
  assert.equal(target('drone1').length, 0);
  assert.equal(target('drone2').length, 1);
  const invalid = anchorsAt(101);
  invalid.positions[3] = invalid.positions[0];
  app.receive(b, '/pidrone/localization/anchors', invalid);
  assert.equal(app.elements.get('room-status').textContent,
    'Drone 2: Check the four anchor positions. Map updates and target editing are paused. · Last observed layout retained');
  assert.equal(target('drone2').length, 0);
  assert.equal(map.targets.drone2.y, 4);
  app.receive(b, '/pidrone/localization/anchors', anchorsAt(102, [[0, 0, 2.25], [4, 0, 2.25], [4, 6, 2.25], [0, 6, 2.25]]));
  assert.equal(map.room.xMax, 4);
  assert.equal(target('drone2').length, 0);
});

test('anchor discovery waits for four, matches hardware IDs, and freezes on stale or conflicting reports', () => {
  const app = dashboard({ discover: false }), map = app.fleetMap;
  assert.equal(map.room, null);
  assert.equal(app.elements.get('target-submit').disabled, true);
  const a = app.connect('drone1'), b = app.connect('drone2');
  const send = (drone, message) => app.receive(drone, '/pidrone/localization/anchors', message);
  send(a, anchorsAt(100, [[0, 0, 2.25], [3, 0, 2.25], [3, 6, 2.25]]));
  assert.equal(map.room, null);
  send(a, anchorsAt(101));
  const original = map.room;
  app.receive(a, '/pidrone/localization/pose', localizationAt(100, 1, 2, 0.5));
  map.previewTarget('drone1', 1, 2);
  map.setZoom(1.25);
  const reordered = anchorsAt(102);
  reordered.anchor_ids.reverse(); reordered.positions.reverse();
  send(b, reordered);
  assert.equal(map.room, original);
  assert.equal(map.zoom, 1.25);
  assert.equal(map.targets.drone1.x, 1);
  const changed = anchorsAt(103);
  changed.positions[1].x = changed.positions[2].x = 4;
  send(b, changed);
  assert.equal(map.room, original);
  assert.equal(map.layoutUsable, false);
  assert.match(app.elements.get('room-status').textContent, /disagree/);
  assert.equal(app.elements.get('target-submit').disabled, true);
  send(a, changed);
  assert.equal(map.layoutUsable, true);
  assert.equal(map.room.xMax, 4);
  assert.equal(map.zoom, 1);
  assert.equal(Object.keys(map.targets).length, 0);
  assert.equal(sharedMarker(app, 'drone1', 'uwb0').data.length, 0);
  const agreed = map.room;
  app.advanceTime(1000);
  const partial = anchorsAt(104, [[0, 0, 2.25], [4, 0, 2.25], [4, 6, 2.25]]);
  send(a, partial);
  assert.equal(map.room, agreed);
  assert.equal(map.layoutUsable, true);
  app.advanceTime(600);
  // Repeated complete stamps and partial reports cannot renew the complete layout.
  send(b, changed);
  send(a, partial);
  app.maintenanceTimers.forEach(callback => callback());
  assert.equal(map.room, agreed);
  assert.equal(map.layoutUsable, false);
  assert.match(app.elements.get('room-status').textContent, /Last observed layout/);
  send(a, { ...changed, header: { ...changed.header, stamp: { secs: 105, nsecs: 0 } } });
  assert.equal(map.layoutUsable, true);
  const oldTopic = a.subscribers.find(topic => topic.name === '/pidrone/localization/anchors');
  a.disconnect();
  b.disconnect();
  app.connect('drone1');
  oldTopic.receive(anchorsAt(200));
  assert.equal(map.layoutUsable, false);
  send(a, anchorsAt(1));
  assert.equal(map.layoutUsable, true);
});

test('ROS tag results stay isolated and include tag zero and multiple tags', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  const topic = '/pidrone/apriltag/detections';
  app.receive(a, topic, tags([0, 2], ['PICKUP', 'BORDER']));
  app.receive(b, topic, tags([1], ['DROP_OFF']));
  assert.equal(a.field('detection').textContent, 'PICKUP · Tag 0, BORDER · Tag 2');
  assert.equal(b.field('detection').textContent, 'DROP_OFF · Tag 1');
  app.receive(a, topic, tags([], []));
  assert.equal(a.field('detection').textContent, 'No tag detected');
  assert.equal(a.field('detection').dataset.state, 'none');
  assert.equal(b.field('detection').textContent, 'DROP_OFF · Tag 1');
});

test('camera loss, malformed results, and a silent ROS detector clear stale tags', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  const topic = '/pidrone/apriltag/detections';
  const detected = tags([0], ['PICKUP']);
  for (const message of [tags([], [], false), tags([-1], ['UNKNOWN']), tags([0], []), tags([0], ['']), {}]) {
    app.receive(a, topic, detected);
    app.receive(a, topic, message);
    assert.equal(a.field('detection').textContent, 'Drone Offline');
  }
  app.receive(a, topic, detected);
  app.advanceTime(1600);
  app.receive(b, topic, tags([1], ['DROP_OFF']));
  for (const callback of app.maintenanceTimers.values()) callback();
  assert.equal(a.field('detection').textContent, 'Drone Offline');
  assert.equal(b.field('detection').textContent, 'DROP_OFF · Tag 1');
});

test('disconnect clears tags and old ROS callbacks cannot overwrite a new session', () => {
  const app = dashboard();
  const a = app.connect('drone1');
  const oldTopic = a.subscribers.find(topic => topic.name === '/pidrone/apriltag/detections');
  oldTopic.receive(tags([0], ['PICKUP']));
  a.disconnect();
  assert.equal(a.field('detection').textContent, 'Drone Offline');
  app.connect('drone1');
  oldTopic.receive(tags([0], ['PICKUP']));
  assert.equal(a.field('detection').textContent, 'Drone Offline');
  app.receive(a, '/pidrone/apriltag/detections', tags([1], ['DROP_OFF']));
  assert.equal(a.field('detection').textContent, 'DROP_OFF · Tag 1');
});

test('laptop calibration retains flight inhibition when its server becomes unavailable', () => {
  const app = dashboard(), drone = app.connect('drone2');
  app.receive(drone, '/pidrone/mode', { mode: 'DISARMED' });
  app.anchorCalibration.update({ stage: 'preview', message: 'Review', boards: [],
    reserved: true, grounded: true, preview: { positions: [], rms: 0 }, token: 'reviewed' });
  app.advanceTime(1600);
  app.anchorCalibration.refresh();
  assert.match(app.elements.get('calibration-status').textContent, /Reconnect the laptop/);
  app.anchorCalibration.request('apply');
  assert.equal(app.calibrationRequests.length, 0);
  app.dispatch('arm', 'drone2');
  assert.equal(drone.publishers.mode.messages.length, 0);
  assert.match(app.elements.get('commandFeedback').textContent, /inhibited/);
  app.dispatch('disarm', 'drone2');
  assert.equal(last(drone, 'mode').mode, 'DISARMED');
});

test('laptop calibration with only Drone 2 requires disarmed state and sends the reviewed layout through HTTP', () => {
  const app = dashboard(), service = app.connect('drone2');
  const boards = Array.from({ length: 4 }, (_, i) => ({ id: '000' + (i + 1), ble: 'uuid-' + i, active: true, initiator: i === 0 }));
  const state = { stage: 'idle', message: 'Select anchors', boards, reserved: false, grounded: true, preview: null, token: null };
  const status = () => app.anchorCalibration.update({ ...state });
  status();
  app.anchorCalibration.request('scan');
  assert.equal(app.calibrationRequests.length, 0);
  app.receive(service, '/pidrone/mode', { mode: 'DISARMED' });
  app.receive(service, '/pidrone/mode', { mode: 'ARMED' });
  app.anchorCalibration.request('scan');
  assert.equal(app.calibrationRequests.length, 0);
  app.receive(service, '/pidrone/mode', { mode: 'DISARMED' });
  status();
  app.anchorCalibration.request('scan');
  assert.deepEqual(app.calibrationRequests.at(-1), { action: 'scan' });
  assert.equal(service.publishers.calibrationRequest, undefined);
  status();
  for (let i = 0; i < 4; i++) app.elements.get('calibration-anchor-' + i).value = boards[i].ble;
  app.anchorCalibration.request('measure');
  assert.deepEqual(app.calibrationRequests.at(-1), { action: 'measure', anchors: boards.map(b => ({ ble: b.ble })) });
  Object.assign(state, { stage: 'preview', reserved: true, token: 'reviewed-token',
    preview: { positions: boards.map((p, i) => ({ ...p, x: i, y: i, z: 0 })), rms: .01 } });
  status();
  app.receive(service, '/pidrone/localization/calibration_locked', { data: true });
  assert.equal(app.fleetMap.layoutUsable, false);
  app.dispatch('arm');
  assert.equal(service.publishers.mode.messages.length, 0);
  app.elements.get('calibration-anchor-0').value = boards[1].ble;
  app.anchorCalibration.request('apply');
  assert.equal(app.calibrationRequests.length, 2);
  app.elements.get('calibration-anchor-0').value = boards[0].ble;
  app.anchorCalibration.request('apply');
  assert.deepEqual(app.calibrationRequests.at(-1), { action: 'apply', token: 'reviewed-token' });
  status();
  app.advanceTime(1600);
  app.anchorCalibration.refresh();
  assert.equal(app.elements.get('calibration-apply').disabled, true);
  app.anchorCalibration.request('apply');
  assert.equal(app.calibrationRequests.length, 3);
});
