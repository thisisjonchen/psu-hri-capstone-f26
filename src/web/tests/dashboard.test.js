'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

// Test command routing without opening network sockets or contacting hardware.
function dashboard({ discover = true, calibrationFetch = null } = {}) {
  const listeners = {};
  const events = [];
  const retries = [];
  const calibrationRequests = [];
  const storage = new Map();
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
      focus() {}, setPointerCapture(id) { this.captured = id; },
      hasPointerCapture(id) { return this.captured === id; }, releasePointerCapture() { this.captured = null; },
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
  for (const id of ['fullscreen-toggle', 'commandFeedback', 'fleetGrid', 'fleet-xyChart', 'drone1-connectionStatus', 'drone2-connectionStatus', 'drone1-connectionToggle', 'drone2-connectionToggle',
    'room-status', 'room-fit', 'room-zoom-in', 'room-zoom-out', 'map-frame', 'map-placeholder', 'map-legend',
    'target-selection', 'target-clear', 'target-send', 'target-stop', 'target-command-status', 'target-status', 'map-stream-status']) elements.set(id, element());
  for (const name of ['scan', 'measure', 'apply', 'cancel', 'status', 'preview', 'selection', 'toggle', 'panel', 'form', 'success']) elements.set('calibration-' + name, element());
  elements.get('calibration-panel').hidden = true;
  elements.get('calibration-success').hidden = true;
  for (let i = 0; i < 4; i++) {
    elements.set('calibration-anchor-' + i, element());
  }
  for (const id of ['drone1', 'drone2']) {
    const status = elements.get(id + '-connectionStatus');
    const icon = element(), label = element(), action = element();
    status.querySelector = selector => selector === '[data-state-icon]' ? icon : selector === '[data-connection-label]' ? label : selector === '[data-connection-action]' ? action : null;
  }
  elements.get('fleetGrid').dataset.count = '0';
  const document = {
    createElement: () => element(),
    getElementById: id => elements.get(id),
    addEventListener: (name, callback) => { listeners[name] = callback; },
    dispatchEvent: event => events.push(event)
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
    advertise() { this.advertised = true; }
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
          convertTicksToLabels() {},
          getPixelForValue(value) { return axis.id === 'room-x'
            ? 55 + (value - this.min) / (this.max - this.min) * 400
            : 410 - (value - this.min) / (this.max - this.min) * 400; },
          getValueForPixel(value) { return axis.id === 'room-x'
            ? this.min + (value - 55) / 400 * (this.max - this.min)
            : this.min + (410 - value) / 400 * (this.max - this.min); } };
      });
    }
    update() { (this.plugins || []).forEach(plugin => { if (plugin.afterLayout) plugin.afterLayout(this); }); }
    resize() {}
  }
  Chart.defaults = { global: {} };
  Chart.helpers = { getRelativePosition: event => ({ x: event.clientX, y: event.clientY }) };
  const context = vm.createContext({
    AbortSignal,
    localStorage: { getItem: key => storage.get(key) || null, setItem: (key, value) => storage.set(key, value) },
    fetch(url, options = {}) {
      assert.equal(url, '/api/calibration');
      if (options.method === 'POST') {
        assert.equal(options.headers['X-Calibration-Request'], '1');
        const request = JSON.parse(options.body);
        if (request.action === 'status') {
          if (calibrationFetch) return calibrationFetch(request);
          return new Promise(() => {}); // Status delivery is controlled by each test.
        }
        calibrationRequests.push(request);
        if (calibrationFetch) return calibrationFetch(request);
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
    clearInterval(id) { timers.delete(id); },
    setTimeout(callback) { retries.push(callback); },
    clearTimeout() {},
    CustomEvent: class { constructor(type, options) { this.type = type; this.detail = options.detail; } }
  });
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/anchor-calibration.js'), 'utf8'), context);
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/main.js'), 'utf8'), context);
  listeners.DOMContentLoaded();
  const api = vm.runInContext('({ drones, fleetMap, anchorCalibration, dispatch, heldKeys, connectDrone, currentTarget, awaitSyncStart, confirmSyncStart, makeAprilTagPlan })', context);
  const receive = (drone, name, message) => drone.subscribers.find(topic => topic.name === name).receive(message);
  const connect = id => {
    api.drones[id].connect(); api.drones[id].ros.open();
    if (discover) receive(api.drones[id], '/pidrone/localization/anchors', anchorsAt(100));
    return api.drones[id];
  };
  const key = (name, value, target = element(), repeat = false) => {
    let prevented = false;
    listeners[name]({ key: value, target, repeat, preventDefault() { prevented = true; } });
    return prevented;
  };
  return { ...api, calibrationRequests, events, retries, connect, receive, key, elements, timers, maintenanceTimers, listeners, element, advanceTime(ms) { now += ms; } };
}
const stamp = secs => ({ header: { stamp: { secs, nsecs: 0 } } });
const rangeAt = (secs, range) => ({ ...stamp(secs), range, min_range: 0.05, max_range: 1.2 });
const last = (drone, publisher) => drone.publishers[publisher].messages.at(-1);
const anchorsAt = (secs, points = [[0, 0, 2.25], [3, 0, 2.25], [3, 6, 2.25], [0, 6, 2.25]]) => ({
  header: { stamp: { secs, nsecs: 0 }, frame_id: 'map' },
  anchor_ids: ['1150', '111C', '1151', '0CA8'].slice(0, points.length),
  positions: points.map(([x, y, z]) => ({ x, y, z }))
});

test('coordination selects solo or paired while disarmed and keeps the selection during staggered arming', () => {
  const app = dashboard();
  const a = app.connect('drone1');
  app.receive(a, '/pidrone/mode', { mode: 'DISARMED' });
  assert.equal(last(a, 'coordination').data, false);
  const b = app.connect('drone2');
  app.receive(b, '/pidrone/mode', { mode: 'DISARMED' });
  assert.equal(last(a, 'coordination').data, true);
  assert.equal(last(b, 'coordination').data, true);
  app.receive(a, '/pidrone/mode', { mode: 'ARMED' });
  assert.equal(last(b, 'coordination').data, true);
  b.disconnect();
  assert.equal(last(a, 'coordination').data, true);
  app.receive(a, '/pidrone/mode', { mode: 'DISARMED' });
  assert.equal(last(a, 'coordination').data, false);
  app.receive(a, '/pidrone/mode', { mode: 'ARMED' });
  b.connect(); b.ros.open();
  app.receive(b, '/pidrone/mode', { mode: 'DISARMED' });
  assert.equal(last(b, 'coordination').data, false);
  const count = b.publishers.coordination.messages.length;
  app.advanceTime(2000);
  app.maintenanceTimers.forEach(callback => callback());
  assert.equal(b.publishers.coordination.messages.length, count);
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
  app.anchorCalibration.update({ stage: 'error', message: 'Layout unverified', boards: [],
    selection: ['ws://drone1.local:9090', 'ws://drone2.local:9090'], reserved: true, dirty: true, grounded: false });
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

test('fusion gaps and resets retain display markers without live control data and permit a new ROS clock epoch', () => {
  const app = dashboard(), a = app.connect('drone1');
  const pose = secs => ({ header: { stamp: { secs, nsecs: 0 }, frame_id: 'map' },
    pose: { pose: { position: { x: .5, y: 1, z: .4 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } } });
  app.receive(a, '/pidrone/localization/generation', { data: 0 });
  app.receive(a, '/pidrone/localization/fused', pose(100));
  app.receive(a, '/pidrone/localization/fusion_status', { status: [{ level: 1, message: 'UWB innovation rejected' }] });
  assert.equal(a.fusedPose, null);
  assert.equal(a.series.fused0.length, 0);
  assert.equal(a.field('fusedPosition').textContent, 'X 0.50 · Y 1.00 · Z 0.40 m');
  assert.equal(sharedMarker(app, 'drone1', 'fused0').borderColor, '#0072b2');
  assert.equal(sharedMarker(app, 'drone1', 'fused0').data.length, 2);
  assert.equal(sharedMarker(app, 'drone1', 'fused0').stale, true);
  assert.equal(app.elements.get('map-stream-status').textContent, '');
  app.receive(a, '/pidrone/localization/fused', pose(100));
  assert.equal(a.fusedPose, null);
  app.receive(a, '/pidrone/localization/generation', { data: 1 });
  assert.equal(sharedMarker(app, 'drone1', 'fused0').data.length, 2);
  assert.equal(sharedMarker(app, 'drone1', 'fused0').stale, true);
  assert.equal(a.fusedPose, null);
  assert.equal(app.elements.get('target-send').disabled, true);
  app.receive(a, '/pidrone/localization/anchors', anchorsAt(50));
  assert.equal(app.fleetMap.layoutUsable, true);
  assert.equal(sharedMarker(app, 'drone1', 'fused0').stale, true);
  app.receive(a, '/pidrone/localization/fused', pose(50));
  assert.equal(a.fusedStamp, 50);
  assert.equal(sharedMarker(app, 'drone1', 'fused0').stale, false);
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

test('a synchronized start releases every Execute drone together once all are freshly ready', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  const starts = () => [a, b].map(drone => drone.publishers.routineStart.messages.length);
  assert.ok(a.publishers.routineStart.advertised && b.publishers.routineStart.advertised);
  for (const drone of [a, b]) {
    app.receive(drone, '/pidrone/mode', { mode: 'DISARMED' });
    app.receive(drone, '/pidrone/routine/ready', {}); // Earlier Execute must not count.
  }
  app.awaitSyncStart(['drone1', 'drone2']);
  app.receive(a, '/pidrone/routine/ready', {});
  assert.deepEqual(starts(), [0, 0]);
  app.advanceTime(600);
  app.receive(b, '/pidrone/routine/ready', {});
  assert.deepEqual(starts(), [0, 0]);  // drone1's ready is stale.
  app.receive(a, '/pidrone/routine/ready', {});
  assert.deepEqual(starts(), [0, 0]); // Wait for both HTTP launches to succeed too.
  app.confirmSyncStart();
  assert.deepEqual(starts(), [1, 1]);
  app.receive(a, '/pidrone/routine/ready', {});
  app.receive(b, '/pidrone/routine/ready', {});
  assert.deepEqual(starts(), [1, 1]);  // Released once per Execute.
  app.awaitSyncStart(['drone1', 'drone2']);
  app.awaitSyncStart(null); // A failed launch cancels release.
  app.confirmSyncStart();
  for (const drone of [a, b]) app.receive(drone, '/pidrone/routine/ready', {});
  assert.deepEqual(starts(), [1, 1]);
  app.awaitSyncStart(['drone1', 'drone2']);
  app.confirmSyncStart();
  app.dispatch('disarm', 'both');
  for (const drone of [a, b]) app.receive(drone, '/pidrone/routine/ready', {});
  assert.deepEqual(starts(), [1, 1]);
});

test('flight stages require both fresh participants in the current run and cancel on disconnect', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  const begin = () => {
    app.awaitSyncStart(['drone1', 'drone2'], ['forward', 'land']);
    for (const drone of [a, b]) {
      app.receive(drone, '/pidrone/mode', { mode: 'DISARMED' });
      app.receive(drone, '/pidrone/routine/ready', {});
    }
    app.confirmSyncStart();
    return JSON.parse(last(a, 'stageRelease').data).run;
  };
  const oldRun = begin(), run = begin();
  const ready = (drone, stage, token = run) => app.receive(drone, '/pidrone/routine/stage_ready',
    { data: JSON.stringify({ run: token, stage }) });
  const stage = drone => JSON.parse(last(drone, 'stageRelease').data).stage;
  ready(a, 'forward'); ready(b, 'forward');
  assert.equal(stage(a), 'start'); // Both must actually be flying.
  for (const drone of [a, b]) app.receive(drone, '/pidrone/mode', { mode: 'FLYING' });
  app.advanceTime(600);
  ready(a, 'forward');
  ready(b, 'forward', oldRun);
  assert.equal(stage(a), 'start'); // Stale readiness and earlier run cannot release.
  ready(b, 'forward');
  assert.equal(stage(a), 'forward'); assert.equal(stage(b), 'forward');
  ready(a, 'land'); ready(b, 'forward');
  assert.equal(stage(a), 'forward'); // One drone is still completing movement.
  ready(b, 'land');
  assert.equal(stage(a), 'land'); assert.equal(stage(b), 'land');
  const nextRun = begin();
  for (const drone of [a, b]) app.receive(drone, '/pidrone/mode', { mode: 'FLYING' });
  b.disconnect();
  ready(a, 'forward', nextRun);
  assert.equal(stage(a), 'start');
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
  app.key('keydown', 'i');
  const input = app.element(); input.closest = () => input;
  app.key('keyup', 'i', input);
  assert.equal(last(a, 'twist').linear.y, 0);
  assert.equal(last(b, 'twist').linear.y, 0);
  const sent = b.publishers.twist.messages.length;
  app.key('keydown', 'i', input);
  assert.equal(b.publishers.twist.messages.length, sent);
});

test('Space allows typing in forms and terminals, and disarms all outside them', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  const terminal = app.element(); terminal.closest = selector => selector === '.script-terminal' ? terminal : null;
  assert.equal(app.key('keydown', ' ', terminal), false);
  for (const tag of ['input', 'textarea', 'select']) {
    const input = app.element(); input.closest = selector => selector.split(', ').includes(tag) ? input : null;
    assert.equal(app.key('keydown', ' ', input), false);
  }
  const editable = app.element(); editable.isContentEditable = true;
  assert.equal(app.key('keydown', ' ', editable), false);
  assert.equal(last(a, 'mode'), undefined);
  assert.equal(last(b, 'mode'), undefined);
  assert.equal(app.key('keydown', ' '), true);
  assert.equal(last(a, 'mode').mode, 'DISARMED');
  assert.equal(last(b, 'mode').mode, 'DISARMED');
});

test('shared targeting stays intact and window blur stops movement', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.key('keydown', 'i');
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
  const oldTags = a.subscribers.find(topic => topic.name === '/pidrone/apriltag/detections');
  oldTags.receive(tags([0], ['PICKUP']));
  a.connect();
  assert.equal(a.ros, oldRos);
  assert.equal(app.timers.size, 2);
  a.disconnect();
  assert.equal(app.timers.size, 1);
  assert.equal(b.connected, true);
  assert.equal(a.field('camera').src, 'nocamera.svg');
  assert.equal(a.field('detection').textContent, 'No tag data');
  app.connect('drone1');
  oldRange.receive({ ...stamp(500), range: 9 });
  oldTags.receive(tags([0], ['PICKUP']));
  assert.equal(a.field('detection').textContent, 'No tag data');
  oldRos.events.close();
  assert.equal(a.connected, true);
  assert.equal(a.series.range.length, 0);
  assert.equal(app.timers.size, 2);
  app.receive(a, '/pidrone/range', { ...stamp(1), range: 0.1 });
  assert.equal(a.startTime, 1);
  app.receive(a, '/pidrone/apriltag/detections', tags([1], ['DROP_OFF']));
  assert.equal(a.field('detection').textContent, 'DROP_OFF · Tag 1');
});

test('unexpected close or error stops heartbeats and clears publishers', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  a.ros.close();
  b.ros.events.error(new Error('offline'));
  assert.equal(app.timers.size, 0);
  assert.equal(Object.keys(a.publishers).length, 0);
  assert.equal(Object.keys(b.publishers).length, 0);
  assert.equal(app.elements.get('drone2-connectionStatus').title, 'Connection error');
  assert.equal(app.elements.get('drone2-connectionStatus').querySelector('[data-connection-label]').textContent, 'Error');
});

test('bounded height streams stay independent', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  for (let secs = 102; secs < 115; secs++) app.receive(a, '/pidrone/range', { ...stamp(secs), range: secs / 100 });
  assert.ok(a.series.range.length <= 6);
  assert.equal(b.series.range.length, 0);
  assert.equal(a.heightChart.data.datasets[0].data.at(-1).y, 1.14);
  assert.equal(a.heightChart.options.scales.yAxes[0].ticks.min, 0);
  assert.equal(a.heightChart.options.scales.yAxes[0].ticks.max, 0.6);
});

test('connecting the other drone preserves its partner and automatically targets both', () => {
  const app = dashboard();
  app.listeners.click({ target: { closest: () => ({ dataset: { connectionToggle: 'drone1' } }) } });
  assert.deepEqual(app.events.map(event => [event.type, event.detail]), [['drone-connect', 'drone1']]);
  // Connect starts the nodes, so a refused rosbridge retries instead of failing.
  app.drones.drone1.ros.events.error(new Error('starting'));
  assert.equal(app.elements.get('drone1-connectionStatus').querySelector('[data-state-icon]').title, 'Waiting for nodes…');
  const deadline = app.drones.drone1.waitUntil;
  app.connectDrone('drone1');
  assert.equal(app.events.length, 1);
  assert.equal(app.drones.drone1.waitUntil, deadline);
  app.retries.pop()();
  const firstRos = app.drones.drone1.ros;
  assert.equal(app.drones.drone2.ros, null);
  assert.equal(app.elements.get('drone1-connectionToggle').textContent, 'Cancel connection');
  firstRos.open();
  assert.equal(app.elements.get('drone1-connectionToggle').hidden, true);
  assert.equal(app.drones.drone1.field('telemetry').hidden, false);
  assert.equal(app.elements.get('drone1-connectionStatus').disabled, false);
  assert.equal(app.elements.get('drone1-connectionStatus').querySelector('[data-connection-action]').textContent, 'Disconnect');
  assert.equal(app.currentTarget(), 'drone1');
  app.listeners.click({ target: { closest: () => ({ dataset: { connectionToggle: 'drone2' } }) } });
  assert.equal(app.drones.drone1.ros, firstRos);
  app.drones.drone2.ros.open();
  assert.equal(app.elements.get('fleetGrid').dataset.count, '2');
  assert.equal(app.currentTarget(), 'both');
  const secondRos = app.drones.drone2.ros;
  app.connectDrone('drone1');
  assert.equal(app.drones.drone1.ros, firstRos);
  assert.equal(app.drones.drone2.ros, secondRos);
  assert.equal(app.currentTarget(), 'both');
  app.listeners.click({ target: { closest: () => ({ dataset: { connectionToggle: 'drone1' } }) } });
  assert.equal(app.drones.drone1.ros, null);
  assert.equal(app.drones.drone2.ros, secondRos);
  assert.equal(app.elements.get('drone1-connectionToggle').textContent, 'Connect');
  assert.equal(app.elements.get('drone1-connectionToggle').hidden, false);
  assert.equal(app.drones.drone1.field('telemetry').hidden, true);
  assert.equal(app.elements.get('drone1-connectionStatus').disabled, true);
  assert.equal(app.events.at(-1).type, 'drone-disconnect');
});

test('startup deadline stops a hanging socket without affecting a connected partner', () => {
  const app = dashboard();
  const partner = app.connect('drone2');
  const before = partner.publishers.mode.messages.length;
  app.connectDrone('drone1');
  const drone = app.drones.drone1;
  const stalled = drone.ros;
  app.advanceTime(90000);
  app.maintenanceTimers.forEach(callback => callback());
  assert.equal(drone.ros, null);
  assert.equal(drone.waitUntil, 0);
  assert.equal(app.elements.get('drone1-connectionStatus').querySelector('[data-state-icon]').title, 'Connection timed out');
  assert.equal(app.events.at(-1).type, 'drone-connect-failed');
  stalled.open();
  assert.equal(drone.connected, false);
  assert.equal(partner.connected, true);
  assert.equal(partner.publishers.mode.messages.length, before);
  app.connectDrone('drone1');
  assert.notEqual(drone.ros, stalled);
  drone.ros.open();
  assert.equal(drone.connected, true);
});

test('startup retries retain their deadline and cannot restart after timeout or cancellation', () => {
  const app = dashboard();
  const drone = app.drones.drone1;
  app.connectDrone('drone1');
  const deadline = drone.waitUntil;
  for (let i = 0; i < 3; i++) {
    drone.ros.events.error(new Error('unreachable'));
    app.advanceTime(2000);
    app.retries.pop()();
    assert.equal(drone.waitUntil, deadline);
  }
  drone.ros.events.error(new Error('unreachable'));
  const retry = app.retries.pop();
  app.advanceTime(84000);
  retry();
  assert.equal(drone.ros, null);
  assert.equal(drone.waitUntil, 0);
  assert.equal(app.elements.get('drone1-connectionStatus').querySelector('[data-state-icon]').title, 'Connection timed out');
  app.connectDrone('drone1');
  drone.ros.events.error(new Error('unreachable'));
  const cancelled = app.retries.pop();
  drone.disconnect();
  cancelled();
  assert.equal(drone.ros, null);
  assert.equal(app.timers.size, 0);
});

test('terminal connection failure cancels startup but detaching a connected drone preserves telemetry', () => {
  const app = dashboard();
  const output = [];
  const context = vm.createContext({
    document: { addEventListener() {} }, drones: app.drones,
    location: { hostname: 'localhost', port: '8000' },
    WebSocket: class {
      static OPEN = 1; readyState = 1;
      constructor(url) { this.url = url; }
      close() { this.readyState = 3; }
    }
  });
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/routines.js'), 'utf8'), context);
  const Terminal = vm.runInContext('RoutineTerminal', context);
  const pane = Object.create(Terminal.prototype);
  Object.assign(pane, { id: 'drone1', socket: null, enabled: true, changed() {},
    term: { reset() {}, write(message) { output.push(message); }, cols: 80, rows: 24 }, fit: { fit() {} } });
  app.connectDrone('drone1');
  pane.attach(true);
  pane.socket.onclose();
  assert.equal(app.drones.drone1.ros, null);
  assert.equal(app.drones.drone1.waitUntil, 0);
  assert.equal(app.events.at(-1).type, 'drone-connect-failed');
  const drone = app.connect('drone1');
  const before = drone.publishers.mode.messages.length;
  pane.attach();
  const old = pane.socket;
  pane.nodes = true;
  pane.poll = () => {};
  pane.connectDrone();
  assert.equal(old.readyState, 3);
  assert.equal(pane.socket.url, 'ws://localhost:8001/terminal/drone1?start=1');
  const count = output.length;
  old.onmessage({ data: 'stale startup result' });
  assert.equal(output.length, count);
  pane.socket.onmessage({ data: 'current terminal' });
  assert.equal(output.at(-1), 'current terminal');
  Object.assign(pane, { scripts: [], nodes: true, running: false });
  assert.equal(pane.ready, true);
  pane.socket.close();
  assert.equal(pane.ready, false);
  pane.socket.onclose();
  assert.equal(drone.connected, true);
  assert.equal(drone.publishers.mode.messages.length, before);
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

test('the speed header uses controller velocity and keeps range-derived vertical speed separate', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.receive(a, '/pidrone/state', { ...stamp(100),
    twist_with_covariance: { twist: { linear: { x: -0.3, y: 0.4, z: 99 } } } });
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
  assert.equal(b.field('speed').textContent, '—');
  app.receive(a, '/pidrone/state', { ...stamp(100.3),
    twist_with_covariance: { twist: { linear: { x: 0, y: 0, z: 99 } } } });
  assert.equal(a.field('speed').textContent, '0.00 m/s');
  app.receive(a, '/pidrone/state', { ...stamp(100.4),
    twist_with_covariance: { twist: { linear: { x: NaN, y: 0, z: 99 } } } });
  assert.equal(a.field('speed').textContent, '—');
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
  app.receive(a, '/pidrone/localization/fused', localizationAt(100, 100, -100, 0.4));
  assert.equal(limits(), initial);
  assert.match(app.elements.get('map-stream-status').textContent, /Outside current view/);
  app.receive(a, '/pidrone/localization/fusion_status', { status: [{ level: 1, message: 'No fix' }] });
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
  const submit = (id, x, y) => map.previewTarget(id, x.trim() ? Number(x) : NaN, y.trim() ? Number(y) : NaN);
  submit('drone1', '1', '2');
  submit('drone2', '2', '4');
  for (const invalid of [['', '2'], ['Infinity', '2'], ['9', '2'], ['1', '-1']]) submit('drone1', ...invalid);
  assert.equal(map.targets.drone1.x, 1);
  assert.ok(Math.abs(map.targets.drone2.y - 4) < 1e-9);
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
  map.previewTarget('drone1', 1, 2);
  app.elements.get('target-clear').listeners.click();
  assert.equal(target('drone1').length, 0);
  assert.equal(target('drone2').length, 1);
  const invalid = anchorsAt(101);
  invalid.positions[3] = invalid.positions[0];
  app.receive(b, '/pidrone/localization/anchors', invalid);
  assert.equal(app.elements.get('room-status').textContent,
    'Drone 2: Check the four anchor positions. Map updates and target editing are paused. · Last observed layout retained');
  assert.equal(target('drone2').length, 0);
  assert.ok(Math.abs(map.targets.drone2.y - 4) < 1e-9);
  app.receive(b, '/pidrone/localization/anchors', anchorsAt(102, [[0, 0, 2.25], [4, 0, 2.25], [4, 6, 2.25], [0, 6, 2.25]]));
  assert.equal(map.room.xMax, 4);
  assert.equal(target('drone2').length, 0);
});

test('anchor discovery waits for four, matches hardware IDs, and freezes on stale or conflicting reports', () => {
  const app = dashboard({ discover: false }), map = app.fleetMap;
  assert.equal(map.room, null);
  assert.equal(app.elements.get('map-placeholder').hidden, false);
  assert.equal(app.elements.get('fleet-xyChart')['aria-disabled'], 'true');
  const a = app.connect('drone1'), b = app.connect('drone2');
  const send = (drone, message) => app.receive(drone, '/pidrone/localization/anchors', message);
  send(a, anchorsAt(100, [[0, 0, 2.25], [3, 0, 2.25], [3, 6, 2.25]]));
  assert.equal(map.room, null);
  send(a, anchorsAt(101));
  assert.equal(app.elements.get('map-placeholder').hidden, true);
  assert.equal(app.elements.get('room-status').textContent, '');
  assert.equal(app.elements.get('room-status').hidden, true);
  const original = map.room;
  app.receive(a, '/pidrone/localization/fused', localizationAt(100, 1, 2, 0.5));
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
  assert.equal(app.elements.get('map-placeholder').hidden, true);
  assert.equal(sharedMarker(app, 'drone1', 'fused0').data.length, 2);
  assert.equal(sharedMarker(app, 'drone1', 'fused0').stale, true);
  assert.match(app.elements.get('room-status').textContent, /disagree/);
  assert.equal(app.elements.get('fleet-xyChart')['aria-disabled'], 'true');
  send(a, changed);
  assert.equal(map.layoutUsable, true);
  assert.equal(map.room.xMax, 4);
  assert.equal(map.zoom, 1);
  assert.equal(Object.keys(map.targets).length, 0);
  assert.equal(sharedMarker(app, 'drone1', 'fused0').data.length, 0);
  app.receive(a, '/pidrone/localization/fused', localizationAt(101, 2, 3, 0.5));
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
  assert.equal(app.elements.get('map-placeholder').hidden, true);
  assert.equal(app.elements.get('map-frame').classList.contains('map-unavailable'), false);
  assert.equal(app.elements.get('fleet-xyChart').getAttribute('aria-hidden'), 'false');
  assert.equal(sharedMarker(app, 'drone1', 'fused0').data[0].x, 1.91);
  assert.equal(sharedMarker(app, 'drone1', 'fused0').stale, true);
  assert.equal(app.elements.get('target-send').disabled, true);
  assert.equal(app.elements.get('fleet-xyChart')['aria-disabled'], 'true');
  assert.equal(app.elements.get('room-fit').disabled, false);
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

test('verified calibration displays the room before a complete tag report and uses live positions', () => {
  const app = dashboard({ discover: false }), map = app.fleetMap, drone = app.connect('drone2');
  const report = anchorsAt(100, [[0, 0, 0], [3, 0, 0], [3, 6, 0], [0, 6, 0]]);
  const layout = { revision: 'saved-layout', frame: 'map', positions:
    report.positions.map((p, i) => ({ ...p, id: report.anchor_ids[i] })) };
  const state = { stage: 'complete', message: 'Layout saved and verified.', boards: [],
    selection: ['ws://drone2.local:9090'], grounded: true, reserved: false, layout };
  assert.equal(app.elements.get('map-placeholder').hidden, false);
  app.anchorCalibration.update(state);
  assert.equal(app.elements.get('map-placeholder').hidden, true);
  assert.equal(map.room.xMax, 3);
  assert.equal(map.room.yMax, 6);
  assert.equal(map.chart.data.datasets.find(d => d.key === 'anchors').data.length, 5);
  assert.equal(app.elements.get('room-status').hidden, true);
  map.previewTarget('drone2', 1, 2);
  app.receive(drone, '/pidrone/mode', { mode: 'FLYING' });
  const pose = { header: { stamp: { secs: 100, nsecs: 0 }, frame_id: 'map' },
    pose: { pose: { position: { x: 1, y: 2, z: .4 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } } };
  app.receive(drone, '/pidrone/localization/fused', pose);
  app.receive(drone, '/pidrone/localization/ready', { data: true });
  map.sendTarget();
  assert.equal(drone.publishers.roomTarget.messages.length, 0);
  app.receive(drone, '/pidrone/localization/anchors', {
    ...report, anchor_ids: report.anchor_ids.slice(0, 3), positions: report.positions.slice(0, 3) });
  app.receive(drone, '/pidrone/localization/fused', localizationAt(100, 1, 2, .4));
  assert.equal(sharedMarker(app, 'drone2', 'fused0').data.length, 2);
  map.sendTarget();
  assert.equal(drone.publishers.roomTarget.messages.length, 1);
  app.anchorCalibration.update(state);
  assert.equal(map.targets.drone2.x, 1);
  app.advanceTime(1600);
  app.maintenanceTimers.forEach(callback => callback());
  assert.equal(app.elements.get('map-placeholder').hidden, true);
  assert.equal(app.elements.get('target-send').disabled, true);
  assert.equal(sharedMarker(app, 'drone2', 'fused0').stale, true);
  const changed = anchorsAt(101, [[0, 0, 0], [4, 0, 0], [4, 6, 0], [0, 6, 0]]);
  app.receive(drone, '/pidrone/localization/anchors', changed);
  assert.equal(map.room.xMax, 3);
  assert.equal(map.layoutUsable, false);
  app.anchorCalibration.update({ ...state, layout: { ...layout, revision: 'resaved-layout',
    positions: changed.positions.map((p, i) => ({ ...p, id: changed.anchor_ids[i] })) } });
  assert.equal(map.room.xMax, 4);
  assert.equal(map.layoutUsable, true);
  assert.equal(Object.keys(map.targets).length, 0);
  assert.equal(sharedMarker(app, 'drone2', 'fused0').data.length, 0);
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
  assert.equal(a.field('detection').textContent, 'No AprilTag detected');
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
    assert.equal(a.field('detection').textContent, 'No tag data');
  }
  app.receive(a, topic, detected);
  app.advanceTime(1600);
  app.receive(b, topic, tags([1], ['DROP_OFF']));
  for (const callback of app.maintenanceTimers.values()) callback();
  assert.equal(a.field('detection').textContent, 'No tag data');
  assert.equal(b.field('detection').textContent, 'DROP_OFF · Tag 1');
});

test('scan, assignment, measurement and saving work independently of drones', async () => {
  const app = dashboard({ discover: false });
  app.elements.get('calibration-toggle').listeners.click();
  const boards = ['4d12', '980C', '9781', 'CA83'].map((id, i) => ({ id, ble: 'uuid-' + i, active: true, initiator: i === 0 }));
  const state = { stage: 'idle', message: 'Select anchors', boards: [], busy: false, preview: null, token: null };
  const status = () => app.anchorCalibration.update({ ...state });
  status();
  await app.anchorCalibration.request('scan');
  assert.deepEqual(app.calibrationRequests.at(-1), { action: 'scan' });
  state.boards = boards;
  status();
  for (let i = 0; i < 4; i++) {
    const select = app.elements.get('calibration-anchor-' + i);
    assert.equal(select.disabled, false);
    select.value = boards[i].ble;
    select.listeners.input();
  }
  await app.anchorCalibration.request('measure');
  assert.deepEqual(app.calibrationRequests.at(-1), { action: 'measure', anchors: boards.map(b => ({ ble: b.ble })) });
  Object.assign(state, { stage: 'preview', token: 'reviewed-token',
    preview: { positions: boards.map((p, i) => ({ ...p, x: i, y: i, z: 0 })), rms: .01 } });
  status();
  app.elements.get('calibration-anchor-0').value = boards[1].ble;
  await app.anchorCalibration.request('apply');
  assert.equal(app.calibrationRequests.length, 2);
  app.elements.get('calibration-anchor-0').value = boards[0].ble;
  const drone = app.connect('drone2');
  app.receive(drone, '/pidrone/mode', { mode: 'ARMED' });
  await app.anchorCalibration.request('apply');
  assert.deepEqual(app.calibrationRequests.at(-1), { action: 'apply', token: 'reviewed-token' });
  assert.equal(app.elements.get('calibration-panel').hidden, false);
  assert.equal(app.elements.get('calibration-success').hidden, true);
  status();
  app.advanceTime(1600);
  app.anchorCalibration.refresh();
  assert.equal(app.elements.get('calibration-apply').disabled, true);
  app.anchorCalibration.request('apply');
  assert.equal(app.calibrationRequests.length, 3);
  Object.assign(state, { stage: 'complete', preview: null, token: null,
    layout: { revision: 'reviewed-token', frame: 'map', verified: true, positions: state.preview.positions } });
  status();
  assert.equal(app.elements.get('calibration-success').hidden, false);
  assert.equal(app.elements.get('calibration-panel').hidden, false);
  const close = app.retries.at(-1);
  status();
  assert.equal(app.retries.at(-1), close);
  close();
  assert.equal(app.elements.get('calibration-panel').hidden, true);
  assert.equal(app.elements.get('calibration-toggle')['aria-expanded'], 'false');
  app.elements.get('calibration-toggle').listeners.click();
  status();
  assert.equal(app.elements.get('calibration-panel').hidden, false);
  assert.equal(app.elements.get('calibration-success').hidden, true);
});

test('room setup works without drones and reuses saved corners without a scan', async () => {
  const app = dashboard({ discover: false });
  const boards = ['4D12', '980C', '9781', 'CA83'].map((id, i) =>
    ({ id, ble: 'uuid-' + i, active: true, initiator: i === 0, pan: null }));
  const positions = boards.map((board, i) => ({ ...board, x: [0, 4, 4, 0][i], y: [0, 0, 3, 3][i], z: 0 }));
  const state = { stage: 'idle', message: 'Ready', boards, busy: false,
    layout: { revision: 'saved', frame: 'map', verified: false, positions } };
  app.anchorCalibration.update(state);
  assert.equal(app.elements.get('map-placeholder').hidden, true);
  assert.equal(app.elements.get('calibration-measure').disabled, false);
  assert.equal(app.fleetMap.layoutUsable, false);
  assert.equal(app.elements.get('room-status').hidden, true);
  await app.anchorCalibration.request('measure');
  assert.deepEqual(app.calibrationRequests, [{ action: 'measure', anchors: boards.map(b => ({ ble: b.ble })) }]);
  const drone = app.connect('drone2');
  app.receive(drone, '/pidrone/localization/anchors', {
    ...anchorsAt(100, positions.map(p => [p.x, p.y, p.z])), anchor_ids: boards.map(b => b.id) });
  assert.equal(app.fleetMap.layoutUsable, true);
  assert.equal(app.elements.get('target-send').disabled, true);
});

test('late status and request timeouts cannot unlock calibration or trigger duplicate requests', async () => {
  const polls = [];
  let rejectAction;
  const app = dashboard({ calibrationFetch: request => request.action === 'status'
    ? new Promise(resolve => polls.push(resolve))
    : new Promise((resolve, reject) => { rejectAction = reject; }) });
  const idle = { stage: 'idle', message: 'Ready', boards: [], busy: false };
  app.anchorCalibration.update(idle);
  const action = app.anchorCalibration.request('scan');
  polls.shift()({ ok: true, json: async () => idle });
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(app.elements.get('calibration-scan').disabled, true);
  await app.anchorCalibration.request('scan');
  rejectAction(Object.assign(new Error('Timeout'), { name: 'TimeoutError' }));
  await action;
  app.anchorCalibration.refresh();
  assert.match(app.elements.get('calibration-status').textContent, /may still be running/);
  assert.equal(app.elements.get('calibration-scan').disabled, true);
  await app.anchorCalibration.request('scan');
  assert.equal(app.calibrationRequests.length, 1);
  polls.shift()({ ok: true, json: async () => ({ ...idle, stage: 'scanning', busy: true,
    message: 'Checking Bluetooth anchor 0001...' }) });
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(app.elements.get('calibration-status').textContent, 'Checking Bluetooth anchor 0001...');
  assert.equal(app.elements.get('calibration-cancel').disabled, false);
});

function mapPointer(map, x, y, pointerId = 1) {
  const pixel = map.targetPixel({ x, y });
  return { button: 0, pointerId, clientX: pixel.x, clientY: pixel.y, preventDefault() {} };
}

test('dragging a drone previews its own target and sends only after explicit confirmation', () => {
  const app = dashboard(), map = app.fleetMap;
  const a = app.connect('drone1'), b = app.connect('drone2');
  const pose = { header: { stamp: { secs: 100, nsecs: 0 }, frame_id: 'map' },
    pose: { pose: { position: { x: 1, y: 2, z: .7 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } } };
  app.receive(b, '/pidrone/mode', { mode: 'FLYING' });
  app.receive(b, '/pidrone/localization/fused', pose);
  app.receive(b, '/pidrone/localization/ready', { data: true });
  const telemetry = JSON.stringify(b.series.fused0);
  const canvas = app.elements.get('fleet-xyChart');
  canvas.listeners.pointerdown(mapPointer(map, 1, 2));
  assert.equal(map.selectedTarget, 'drone2', 'the marker determines the drone, not the connection selector');
  canvas.listeners.pointermove(mapPointer(map, 2, 4));
  assert.equal(map.targets.drone2, undefined, 'moving is a draft, not a committed target');
  const target = map.chart.data.datasets.find(d => d.targetDrone === 'drone2');
  assert.ok(Math.abs(target.data[0].x - 2) < 1e-9);
  assert.ok(Math.abs(target.data[0].y - 4) < 1e-9);
  map.sendTarget();
  assert.equal(b.publishers.roomTarget.messages.length, 0);
  assert.equal(app.elements.get('target-send').disabled, true);
  canvas.listeners.pointerup(mapPointer(map, 2, 4));
  assert.ok(Math.abs(map.targets.drone2.x - 2) < 1e-9);
  assert.ok(Math.abs(map.targets.drone2.y - 4) < 1e-9);
  assert.equal(b.publishers.roomTarget.messages.length, 0, 'dropping never publishes');
  assert.equal(JSON.stringify(b.series.fused0), telemetry, 'telemetry stays at the measured position');
  assert.equal(app.elements.get('target-send').disabled, false);
  app.elements.get('target-send').listeners.click();
  assert.equal(b.publishers.roomTarget.messages.length, 1);
  assert.equal(a.publishers.roomTarget.messages.length, 0);
  assert.deepEqual(last(b, 'roomTarget').pose.position, { ...map.targets.drone2, z: .7 });
});

test('invalid drops and canceled pointer gestures preserve the previous destination', () => {
  const app = dashboard(), map = app.fleetMap, a = app.connect('drone1');
  map.previewTarget('drone1', 1, 2);
  const canvas = app.elements.get('fleet-xyChart');
  for (const end of ['outside', 'pointercancel', 'lostpointercapture', 'blur', 'Escape']) {
    canvas.listeners.pointerdown(mapPointer(map, 1, 2));
    canvas.listeners.pointermove(mapPointer(map, 2, 4));
    if (end === 'outside') canvas.listeners.pointerup(mapPointer(map, 9, 9));
    else if (end === 'Escape') canvas.listeners.keydown({ key: 'Escape' });
    else canvas.listeners[end]();
    assert.equal(map.drag, null);
    assert.equal(canvas.captured, null);
    assert.equal(map.targets.drone1.x, 1);
    assert.equal(map.targets.drone1.y, 2);
  }
  canvas.listeners.pointerdown(mapPointer(map, 1, 2));
  canvas.listeners.pointermove(mapPointer(map, 2, 4, 2));
  canvas.listeners.pointerup(mapPointer(map, 2, 4, 2));
  assert.equal(map.drag.point, null, 'a second pointer cannot move or release the active drag');
  canvas.listeners.pointercancel();
  assert.equal(a.publishers.roomTarget.messages.length, 0);
});

test('layout loss or replacement cancels an in-progress drag without restoring old targets', () => {
  const app = dashboard(), map = app.fleetMap, a = app.connect('drone1');
  map.previewTarget('drone1', 1, 2);
  map.startTargetDrag(mapPointer(map, 1, 2));
  map.moveTargetDrag(mapPointer(map, 2, 4));
  app.advanceTime(1600);
  map.refreshAnchors();
  assert.equal(map.drag, null);
  map.finishTargetDrag(mapPointer(map, 2, 4));
  assert.equal(map.targets.drone1.x, 1);
  app.receive(a, '/pidrone/localization/anchors', anchorsAt(101));
  map.startTargetDrag(mapPointer(map, 1, 2));
  map.moveTargetDrag(mapPointer(map, 2, 4));
  app.receive(a, '/pidrone/localization/anchors', anchorsAt(102, [[0, 0, 2.25], [4, 0, 2.25], [4, 6, 2.25], [0, 6, 2.25]]));
  assert.equal(map.drag, null);
  assert.equal(Object.keys(map.targets).length, 0);
  map.finishTargetDrag(mapPointer(map, 2, 4));
  assert.equal(Object.keys(map.targets).length, 0);
});

test('tap selection, overlapping targets, and keyboard adjustment work without coordinate fields', () => {
  const app = dashboard(), map = app.fleetMap;
  app.connect('drone1'); app.connect('drone2');
  map.previewTarget('drone1', 1, 2);
  map.previewTarget('drone2', 2, 2);
  map.startTargetDrag(mapPointer(map, 2, 2));
  map.finishTargetDrag(mapPointer(map, 2, 2));
  map.previewTarget('drone2', 1, 2);
  map.startTargetDrag(mapPointer(map, 1, 2));
  map.finishTargetDrag(mapPointer(map, 1, 2));
  assert.equal(map.selectedTarget, 'drone2');
  assert.equal(map.targets.drone2.y, 2);
  assert.equal(map.selectedTarget, 'drone2', 'overlapping markers follow the existing selection');
  let prevented = false, stopped = false;
  map.adjustTarget({ key: 'ArrowRight', shiftKey: true,
    preventDefault() { prevented = true; }, stopPropagation() { stopped = true; } });
  assert.ok(prevented && stopped);
  assert.equal(map.targets.drone2.x, 1.5);
  assert.equal(map.targets.drone1.x, 1);
  const html = fs.readFileSync(path.join(__dirname, '../index.html'), 'utf8');
  assert.doesNotMatch(html, /id="target-(drone|x|y|submit)"/);
});

test('mapped AprilTags follow changed positions across drones and clear on generation change', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  const tagMap = (x, count, time = 100) => ({
    header: { stamp: { secs: time, nsecs: 0 }, frame_id: 'map' },
    tag_ids: [0], zones: ['PICKUP'],
    positions: [{ x, y: 2, z: 0 }], sample_counts: [count]
  });
  app.receive(a, '/pidrone/apriltag/map', tagMap(1, 4));
  app.receive(b, '/pidrone/apriltag/map', tagMap(1.2, 6));
  const tags = app.fleetMap.mappedTags();
  assert.equal(tags.length, 1);
  assert.equal(tags[0].id, 0);
  assert.equal(tags[0].x, 1.2);
  assert.equal(app.fleetMap.chart.data.datasets.find(d => d.key === 'apriltags').data.length, 1);
  // A new location wins even when the other drone has more historical samples.
  app.receive(a, '/pidrone/apriltag/map', tagMap(2, 3, 101));
  assert.equal(app.fleetMap.mappedTags()[0].x, 2);
  // Updates to a different tag must not refresh the cached location of tag 0.
  const otherTag = tagMap(1.2, 6, 102);
  otherTag.tag_ids.push(1); otherTag.zones.push('DROP_OFF');
  otherTag.positions.push({ x: 3, y: 2, z: 0 }); otherTag.sample_counts.push(3);
  app.receive(b, '/pidrone/apriltag/map', otherTag);
  assert.equal(app.fleetMap.mappedTags()[0].x, 2);
  // Small moves must not be averaged back toward the other drone's old marker.
  app.receive(a, '/pidrone/apriltag/map', tagMap(1.4, 3, 103));
  assert.equal(app.fleetMap.mappedTags()[0].x, 1.4);
  app.receive(a, '/pidrone/apriltag/map', tagMap(1, 20, 100));
  assert.equal(app.fleetMap.mappedTags()[0].x, 1.4);
  app.receive(a, '/pidrone/localization/generation', { data: 0 });
  app.receive(a, '/pidrone/localization/generation', { data: 1 });
  assert.equal(app.fleetMap.tagReports.drone1, undefined);
  assert.equal(app.fleetMap.mappedTags().length, 2); // Drone 2's independent map remains.
  b.disconnect();
  assert.equal(app.fleetMap.mappedTags().length, 0);
});

test('tag route planning requires per-tag age, matching maps, and freezes the same plan for both drones', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  for (const [i, drone] of [a, b].entries()) {
    app.receive(drone, '/pidrone/mode', { mode: 'DISARMED' });
    app.receive(drone, '/pidrone/localization/generation', { data: 7 });
    app.receive(drone, '/pidrone/localization/fused', { header: { stamp: { secs: 100, nsecs: 0 }, frame_id: 'map' },
      pose: { pose: { position: { x: 1 + i * .75, y: 2, z: .05 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } } });
  }
  const report = { header: { stamp: { secs: 100, nsecs: 0 }, frame_id: 'map' }, generation: 7,
    tag_ids: [1, 2], zones: ['', ''], positions: [{ x: 1.5, y: 3, z: 0 }, { x: 1.5, y: 4, z: 0 }],
    sample_counts: [3, 3], observed_at: [{ secs: 99, nsecs: 0 }, { secs: 99, nsecs: 0 }] };
  app.receive(a, '/pidrone/apriltag/waypoints', {map: report, generation: report.generation, observed_at: report.observed_at});
  const plan = app.makeAprilTagPlan(['drone1', 'drone2'], [1, 2]);
  app.awaitSyncStart(['drone1', 'drone2'], ['tag-0', 'tag-1', 'land'], plan);
  for (const drone of [a, b]) app.receive(drone, '/pidrone/routine/ready', {});
  app.confirmSyncStart();
  assert.deepEqual(JSON.parse(last(a, 'stageRelease').data).plan, JSON.parse(last(b, 'stageRelease').data).plan);
  assert.equal(JSON.parse(last(a, 'stageRelease').data).plan.points[0].stamp, 99);
  assert.throws(() => app.makeAprilTagPlan(['drone1'], [1, 2]), /Connect only/);
  app.fleetMap.waypointReports.drone1.tags[0].observedAt = undefined;
  assert.throws(() => app.makeAprilTagPlan(['drone1', 'drone2'], [1, 2]), /confirmed observation/);
  app.fleetMap.waypointReports.drone1.tags[0].observedAt = 99;
  app.fleetMap.waypointReports.drone1.stamp = 250; // New snapshot cannot freshen old tag observations.
  assert.throws(() => app.makeAprilTagPlan(['drone1', 'drone2'], [1, 2]), /confirmed observation/);
  app.fleetMap.waypointReports.drone1.stamp = 100;
  app.fleetMap.waypointReports.drone1.generation = 6;
  assert.throws(() => app.makeAprilTagPlan(['drone1', 'drone2'], [1, 2]), /confirmed observation/);
});

test('AprilTag routine mode controls participant count and sets one synchronized stage per tag', async () => {
  let released, error, sends = [];
  const context = vm.createContext({ document: { addEventListener() {} },
    makeAprilTagPlan(ids, tags) { return { ids, tags }; },
    awaitSyncStart(ids, stages, plan) { released = { ids, stages, plan }; },
    confirmSyncStart() {}, feedback(message) { error = message; } });
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/routines.js'), 'utf8'), context);
  const Runner = vm.runInContext('RoutineRunner', context);
  for (const mode of ['single', 'both']) {
    const runner = Object.create(Runner.prototype);
    Object.assign(runner, { select: { value: 'prototype-1-apriltag.py' },
      args: { value: '--fly --drones ' + mode + ' --tags 1,2' },
      terminals: ['drone1', 'drone2'].map(id => ({ id, enabled: true, ready: true,
        execute() { sends.push(id); } })),
      async run(label, targets, action) { return Promise.allSettled(targets.map(action)); } });
    if (mode === 'single') {
      await runner.execute();
      assert.match(error, /exactly one/);
      assert.equal(sends.length, 0);
      runner.terminals[0].enabled = false; // Single works with Drone 2 as well.
    }
    await runner.execute();
    assert.deepEqual(Array.from(released.stages), ['tag-0', 'tag-1', 'land']);
    assert.equal(sends.length, mode === 'single' ? 1 : 2);
    sends = [];
    if (mode === 'both') {
      runner.terminals[1].enabled = false;
      await runner.execute();
      assert.match(error, /both drones/);
      assert.equal(sends.length, 0);
    }
  }
});
