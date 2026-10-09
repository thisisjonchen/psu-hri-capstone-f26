'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

// Test command routing without opening network sockets or contacting hardware.
function dashboard() {
  const listeners = {};
  const timers = new Map();
  const maintenanceTimers = new Map();
  let nextTimer = 0;
  let now = 1000;
  const elements = new Map();
  function element(value = '') {
    const classes = new Set();
    return {
      value, textContent: '', dataset: {}, src: 'nocamera.jpg', listeners: {},
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
  for (const id of ['commandFeedback', 'fleetGrid', 'fleet-xyChart', 'controlsHeading', 'drone1-connectionStatus', 'drone2-connectionStatus', 'droneSelection']) elements.set(id, element());
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
    constructor(ctx, config) { Object.assign(this, config); this.options.scales.xAxes[0].scaleLabel = {}; }
    update() {}
    resize() {}
  }
  Chart.defaults = { global: {} };
  const context = vm.createContext({
    document, window: { addEventListener(name, callback) { listeners[name] = callback; } },
    console: { error() {} }, Chart,
    Quaternion: class { rotateVector(vector) { return vector; } },
    ROSLIB: { Ros, Topic, Message: class { constructor(data) { Object.assign(this, data); } } },
    Date: class extends Date { static now() { return now; } },
    setInterval(callback, delay) { const id = ++nextTimer; (delay === 250 ? maintenanceTimers : timers).set(id, callback); return id; },
    clearInterval(id) { timers.delete(id); }
  });
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/main.js'), 'utf8'), context);
  listeners.DOMContentLoaded();
  const api = vm.runInContext('({ drones, fleetMap, dispatch, heldKeys, connectSelected, currentTarget })', context);
  const connect = id => { api.drones[id].connect(); api.drones[id].ros.open(); return api.drones[id]; };
  const receive = (drone, name, message) => drone.subscribers.find(topic => topic.name === name).receive(message);
  const key = (name, value, target = element(), repeat = false) => listeners[name]({ key: value, target, repeat, preventDefault() {} });
  return { ...api, connect, receive, key, elements, timers, maintenanceTimers, listeners, element, advanceTime(ms) { now += ms; } };
}
const stamp = secs => ({ header: { stamp: { secs, nsecs: 0 } } });
const rangeAt = (secs, range) => ({ ...stamp(secs), range, min_range: 0.05, max_range: 1.2 });
const last = (drone, publisher) => drone.publishers[publisher].messages.at(-1);

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
  assert.equal(a.field('camera').src, 'nocamera.jpg');
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

test('shared map combines both drones while their pose streams remain isolated', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.receive(a, '/pidrone/picamera/pose', { pose: poseAt(-0.5, 0.2) });
  app.receive(b, '/pidrone/picamera/pose', { pose: poseAt(0.5, -0.2) });
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data[0].x, -0.5);
  assert.equal(sharedMarker(app, 'drone2', 'camera2').data[0].x, 0.5);
  assert.equal(a.series.camera2[0].x, -0.5);
  assert.equal(b.series.camera2[0].x, 0.5);
  app.receive(a, '/pidrone/state/ground_truth', { ...stamp(100), pose: poseAt(-0.4, 0.3) });
  assert.equal(sharedMarker(app, 'drone1', 'groundTruth2').data[0].x, -0.4);
  app.receive(b, '/pidrone/state/ukf_2d', { ...stamp(100), pose_with_covariance: { pose: poseAt(0.4, -0.3), covariance: Array(36).fill(0) } });
  assert.equal(sharedMarker(app, 'drone2', 'ukf2').data[0].x, 0.4);
  app.receive(a, '/pidrone/picamera/pose', { pose: poseAt(NaN, 0) });
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data[0].x, -0.5);
});

test('shared map stays live and disconnection clears only the departed drone', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  const send = (drone, x) => app.receive(drone, '/pidrone/picamera/pose', { pose: poseAt(x, 0) });
  send(a, -0.5);
  send(b, 0.5);
  send(a, -0.4);
  assert.equal(a.series.camera2[0].x, -0.4);
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data[0].x, -0.4);
  send(b, 0.6);
  assert.equal(sharedMarker(app, 'drone2', 'camera2').data[0].x, 0.6);
  assert.equal(b.series.camera2[0].x, 0.6);
  a.disconnect();
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data.length, 0);
  assert.equal(sharedMarker(app, 'drone2', 'camera2').data[0].x, 0.6);
  a.connect();
  a.ros.open();
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data.length, 0);
  send(a, -0.2);
  assert.equal(sharedMarker(app, 'drone1', 'camera2').data[0].x, -0.2);
});

const tags = (tag_ids, zones, camera_online = true) => ({ tag_ids, zones, camera_online });

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
    assert.equal(a.field('detection').textContent, 'Detector offline');
  }
  app.receive(a, topic, detected);
  app.advanceTime(1600);
  app.receive(b, topic, tags([1], ['DROP_OFF']));
  for (const callback of app.maintenanceTimers.values()) callback();
  assert.equal(a.field('detection').textContent, 'Detector offline');
  assert.equal(b.field('detection').textContent, 'DROP_OFF · Tag 1');
});

test('disconnect clears tags and old ROS callbacks cannot overwrite a new session', () => {
  const app = dashboard();
  const a = app.connect('drone1');
  const oldTopic = a.subscribers.find(topic => topic.name === '/pidrone/apriltag/detections');
  oldTopic.receive(tags([0], ['PICKUP']));
  a.disconnect();
  assert.equal(a.field('detection').textContent, 'Detector offline');
  app.connect('drone1');
  oldTopic.receive(tags([0], ['PICKUP']));
  assert.equal(a.field('detection').textContent, 'Detector offline');
  app.receive(a, '/pidrone/apriltag/detections', tags([1], ['DROP_OFF']));
  assert.equal(a.field('detection').textContent, 'DROP_OFF · Tag 1');
});
