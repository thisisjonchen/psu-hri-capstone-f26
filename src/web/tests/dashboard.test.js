'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function branchPicker(respond, confirm = () => true, connected = ['drone1', 'drone2']) {
  const elements = new Map(['drone-branches', 'branch-select', 'branch-apply', 'branch-clear',
    'branch-message', 'drone1-branch', 'drone2-branch'].map(id => [id, {
      value: '', textContent: '', dataset: {}, listeners: {},
      addEventListener(name, callback) { this.listeners[name] = callback; },
      replaceChildren(...options) { this.options = options; }
    }]));
  const requests = [];
  const restarts = [];
  const runner = { busy: false, render() {}, async restartTargets(ids) {
    restarts.push(Array.from(ids));
    return ids.map(() => ({ status: 'fulfilled', value: 'Nodes restarted and localized' }));
  } };
  const listeners = {};
  const drones = Object.fromEntries(['drone1', 'drone2'].map(id => [id, { connected: connected.includes(id) }]));
  const context = vm.createContext({ drones, testRunner: runner, document: { getElementById: id => elements.get(id),
    addEventListener: (name, callback) => { listeners[name] = callback; } },
    Option: class { constructor(text, value) { this.text = text; this.value = value; } },
    AbortController, setTimeout, clearTimeout, window: { confirm },
    fetch: async (url, options) => {
      requests.push({ url, ...options });
      return { ok: true, json: async () => JSON.parse(JSON.stringify(await respond(options))) };
    }
  });
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/routines.js'), 'utf8'), context);
  vm.runInContext('routineRunner = testRunner', context);
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/branches.js'), 'utf8'), context);
  return { picker: vm.runInContext('new DroneBranches()', context), elements, requests, drones, listeners, restarts, runner };
}

test('branch picker shows both revisions and switches both to a shared branch', async () => {
  let confirmed = false, prompt;
  const snapshot = { drone1: { branch: 'dual-hover', revision: '111', branches: ['dual-hover', 'new', 'only-one'] },
    drone2: { branch: 'dual-hover', revision: '222', branches: ['dual-hover', 'new'] } };
  const app = branchPicker(options => options.method === 'POST' ? { phase: 'build', drones: {
    drone1: { branch: 'new', revision: '333' }, drone2: { branch: 'new', revision: '333' }
  } } : { drones: snapshot }, message => { prompt = message; return confirmed; });
  app.picker.panel.open = true;
  await app.picker.panel.listeners.toggle();
  assert.equal(app.elements.get('drone1-branch').textContent, 'dual-hover · 111');
  assert.equal(app.elements.get('drone2-branch').textContent, 'dual-hover · 222');
  assert.deepEqual(app.picker.select.options.map(option => option.value), ['', 'dual-hover', 'new']);
  app.picker.select.value = 'new';
  app.picker.render();
  await app.picker.switchSelected();
  assert.equal(app.requests.length, 1, 'cancel must not update or restart anything');
  assert.deepEqual(app.restarts, []);
  assert.match(prompt, /Switch and restart Drone 1 and Drone 2 to new/);
  assert.match(prompt, /props off.*disarmed.*floor.*facing map \+Y/);
  confirmed = true;
  const change = app.picker.switchSelected();
  assert.equal(app.picker.apply.disabled, true);
  assert.equal(app.runner.busy, true, 'routine controls stay locked during update/build');
  await app.picker.switchSelected();
  await change;
  assert.equal(app.requests.length, 2, 'one status request and one mutation for both drones');
  assert.deepEqual(JSON.parse(app.requests[1].body), { action: 'switch', branch: 'new', drones: ['drone1', 'drone2'] });
  assert.equal(app.requests[1].headers['X-Routine-Request'], '1');
  assert.equal(app.elements.get('drone2-branch').textContent, 'new · 333');
  assert.match(app.picker.message.textContent, /updated and restarted/);
  assert.deepEqual(app.restarts, [['drone1', 'drone2']]);
  assert.equal(app.runner.busy, false);
});

test('branch picker switches and clears only the single connected drone', async () => {
  for (const id of ['drone1', 'drone2']) {
    let response = { drones: { [id]: { branch: 'old', revision: '111', branches: ['old', 'only-here'] } } };
    let prompt = '';
    const app = branchPicker(() => response, message => { prompt = message; return true; }, [id]);
    await app.picker.load();
    assert.equal(app.requests[0].url, '/api/branches?drone=' + id);
    assert.deepEqual(app.picker.select.options.map(option => option.value), ['', 'old', 'only-here']);
    assert.equal(app.picker.select.value, 'old');
    const other = id === 'drone1' ? 'drone2' : 'drone1';
    assert.equal(app.elements.get(other + '-branch').textContent, 'Not connected');
    app.picker.select.value = 'only-here';
    response = { phase: 'build', drones: { [id]: { branch: 'only-here', revision: '222' } } };
    await app.picker.switchSelected();
    assert.deepEqual(JSON.parse(app.requests[1].body), { action: 'switch', branch: 'only-here', drones: [id] });
    assert.match(app.picker.message.textContent, new RegExp('Drone ' + id.slice(-1) + ' updated'));
    assert.deepEqual(app.restarts, [[id]]);
    response = { drones: { [id]: { branch: 'only-here', revision: '222', branches: ['only-here'], dirty: true } } };
    await app.picker.load();
    response = { phase: 'clear', drones: { [id]: { dirty: false } } };
    await app.picker.clearSelected();
    assert.match(prompt, new RegExp('Clear local changes on Drone ' + id.slice(-1) + '\\?'));
    assert.doesNotMatch(prompt, new RegExp('Drone ' + other.slice(-1)));
    assert.deepEqual(JSON.parse(app.requests[3].body), { action: 'clear', drones: [id] });
    assert.equal(app.picker.apply.disabled, false);
    assert.deepEqual(app.restarts, [[id]], 'clearing local changes does not restart');
  }
});

test('branch targets refresh automatically as drones connect and disconnect', async () => {
  let response = { drones: { drone1: { branch: 'old', revision: '111', branches: ['old'] } } };
  const app = branchPicker(() => response, () => true, []);
  await app.picker.load();
  assert.equal(app.requests.length, 0);
  assert.equal(app.picker.apply.disabled, true);
  assert.equal(app.picker.clear.disabled, true);
  assert.match(app.picker.message.textContent, /Connect a drone/);
  app.drones.drone1.connected = true;
  await app.listeners['drone-connections-changed']();
  assert.equal(app.requests[0].url, '/api/branches?drone=drone1');
  assert.equal(app.picker.apply.disabled, false);
  response.drones.drone2 = { branch: 'old', revision: '111', branches: ['old'] };
  app.drones.drone2.connected = true;
  const loading = app.listeners['drone-connections-changed']();
  assert.equal(app.picker.apply.disabled, true);
  await app.picker.switchSelected();
  assert.ok(app.requests.every(request => request.method !== 'POST'), 'wait for fresh branches before switching');
  await loading;
  assert.equal(app.requests[1].url, '/api/branches?drone=drone1&drone=drone2');
  assert.equal(app.picker.apply.disabled, false);
  app.drones.drone1.connected = false;
  await app.picker.switchSelected();
  assert.equal(app.requests.length, 2, 'recheck targets at the click before sending');
  response = { drones: { drone2: { branch: 'old', revision: '111', branches: ['old'] } } };
  await app.listeners['drone-connections-changed']();
  assert.equal(app.picker.apply.disabled, false);
  assert.equal(app.requests[2].url, '/api/branches?drone=drone2');
  assert.equal(app.elements.get('drone1-branch').textContent, 'Not connected');
  app.drones.drone2.connected = false;
  await app.listeners['drone-connections-changed']();
  assert.equal(app.requests.length, 3, 'disconnecting the last drone needs no status request');
  assert.equal(app.picker.apply.disabled, true);
  assert.equal(app.picker.clear.disabled, true);
  assert.deepEqual(app.picker.select.options.map(option => option.value), ['']);
});

test('connection changes during a branch load discard the stale response and reload once', async () => {
  let finishLoad;
  const app = branchPicker(() => app.requests.length === 1 ? new Promise(resolve => { finishLoad = resolve; })
    : { drones: { drone2: { branch: 'current', revision: '222', branches: ['current'] } } }, () => true, ['drone1']);
  const loading = app.picker.load();
  app.drones.drone1.connected = false;
  await app.listeners['drone-connections-changed']();
  app.drones.drone2.connected = true;
  await app.listeners['drone-connections-changed']();
  assert.equal(app.requests.length, 1, 'connection changes queue one reload');
  finishLoad({ drones: { drone1: { branch: 'stale', revision: '111', branches: ['stale'] } } });
  await loading;
  assert.equal(app.requests.length, 2);
  assert.equal(app.requests[1].url, '/api/branches?drone=drone2');
  assert.equal(app.elements.get('drone1-branch').textContent, 'Not connected');
  assert.equal(app.elements.get('drone2-branch').textContent, 'current · 222');
  assert.deepEqual(app.picker.select.options.map(option => option.value), ['', 'current']);
  assert.equal(app.picker.apply.disabled, false);
});

test('connection refresh waits for a switch without changing its confirmed targets', async () => {
  let finishSwitch;
  const app = branchPicker(options => options.method === 'POST'
    ? new Promise(resolve => { finishSwitch = resolve; })
    : { drones: Object.fromEntries(Object.entries(app.drones).filter(([, drone]) => drone.connected)
      .map(([id]) => [id, { branch: 'new', revision: '222', branches: ['new'] }])) }, () => true, ['drone1']);
  await app.picker.load();
  const switching = app.picker.switchSelected();
  app.drones.drone2.connected = true;
  await app.listeners['drone-connections-changed']();
  assert.equal(app.requests.length, 2, 'do not reload during a switch');
  assert.deepEqual(JSON.parse(app.requests[1].body).drones, ['drone1']);
  finishSwitch({ phase: 'build', drones: { drone1: { branch: 'new', revision: '222' } } });
  await switching;
  assert.deepEqual(app.restarts, [['drone1']]);
  assert.equal(app.requests[2].url, '/api/branches?drone=drone1&drone=drone2');
  assert.equal(app.elements.get('drone2-branch').textContent, 'new · 222');
  assert.equal(app.picker.message.textContent, 'Drone 1 updated and restarted.');
  assert.equal(app.picker.apply.disabled, false);
});

test('branch picker blocks local changes and reports offline or partially switched drones', async () => {
  let response = { drones: {
    drone1: { branch: 'old', revision: '111', branches: ['new'], dirty: true },
    drone2: { branch: 'old', revision: '111', branches: ['new'] }
  } };
  const app = branchPicker(() => response);
  await app.picker.load();
  app.picker.select.value = 'new';
  app.picker.render();
  assert.equal(app.picker.apply.disabled, true);
  assert.match(app.picker.message.textContent, /Save or stash/);
  await app.picker.switchSelected();
  assert.equal(app.requests.length, 1);
  response.drones.drone1.dirty = false;
  await app.picker.load();
  response = { phase: 'switch', drones: { drone1: { branch: 'new', revision: '222' }, drone2: { error: 'offline' } } };
  await app.picker.switchSelected();
  assert.equal(app.elements.get('drone1-branch').textContent, 'new · 222');
  assert.equal(app.elements.get('drone2-branch').textContent, 'offline');
  assert.match(app.picker.message.textContent, /incomplete.*Reopen Branches/);
  assert.equal(app.picker.apply.disabled, true);
  assert.equal(app.picker.clear.disabled, true);
  assert.deepEqual(app.restarts, [], 'partial switches must not restart either drone');
});

test('build failures prevent restart and restart failures remain visible', async () => {
  const snapshot = { drones: { drone1: { branch: 'old', revision: '111', branches: ['new'] } } };
  let response = snapshot;
  const app = branchPicker(() => response, () => true, ['drone1']);
  await app.picker.load();
  app.picker.select.value = 'new';
  app.runner.busy = true;
  await app.picker.switchSelected();
  assert.equal(app.requests.length, 1, 'do not update during another routine action');
  app.runner.busy = false;
  response = { phase: 'build', drones: { drone1: { branch: 'new', revision: '222', error: 'compile failed' } } };
  await app.picker.switchSelected();
  assert.deepEqual(app.restarts, []);
  assert.equal(app.elements.get('drone1-branch').textContent, 'compile failed');
  assert.equal(app.runner.busy, false);
  response = snapshot;
  await app.picker.load();
  app.picker.select.value = 'new';
  response = { phase: 'build', drones: { drone1: { branch: 'new', revision: '222' } } };
  app.runner.restartTargets = async ids => {
    assert.deepEqual(Array.from(ids), ['drone1']);
    app.drones.drone1.connected = false;
    await app.listeners['drone-connections-changed']();
    return [{ status: 'rejected', reason: new Error('localization unavailable') }];
  };
  await app.picker.switchSelected();
  assert.equal(app.elements.get('drone1-branch').textContent, 'localization unavailable');
  assert.match(app.picker.message.textContent, /Restart incomplete/);
  assert.equal(app.runner.busy, false);
  assert.equal(app.picker.busy, false);
});

test('clearing local changes requires confirmation and enables switching only after success', async () => {
  let confirmed = false;
  let prompt = '';
  let response = { drones: {
    drone1: { branch: 'old', revision: '111', branches: ['old', 'new'], dirty: true },
    drone2: { branch: 'old', revision: '222', branches: ['old', 'new'], dirty: true }
  } };
  const app = branchPicker(() => response, message => { prompt = message; return confirmed; });
  await app.picker.load();
  assert.equal(app.picker.clear.disabled, false);
  await app.picker.clearSelected();
  assert.match(prompt, /Drone 1 and Drone 2/);
  assert.match(prompt, /permanently.*uncommitted.*untracked/);
  assert.equal(app.requests.length, 1, 'cancel must not send a mutation');
  confirmed = true;
  response = { phase: 'prepare', drones: { drone1: {}, drone2: { error: 'Drone must report DISARMED' } } };
  await app.picker.clearSelected();
  assert.match(app.picker.message.textContent, /No files changed/);
  assert.equal(app.picker.clear.disabled, true, 'reload required after a failed safety check');
  response = { drones: {
    drone1: { branch: 'old', revision: '111', branches: ['old', 'new'], dirty: true },
    drone2: { branch: 'old', revision: '222', branches: ['old', 'new'], dirty: true }
  } };
  app.picker.panel.open = false;
  await app.picker.panel.listeners.toggle();
  assert.equal(app.requests.length, 2, 'closing the popover does not reload branches');
  app.picker.panel.open = true;
  await app.picker.panel.listeners.toggle();
  response = { phase: 'clear', drones: {
    drone1: { branch: 'old', revision: '111', dirty: false },
    drone2: { branch: 'old', revision: '222', dirty: false }
  } };
  const clearing = app.picker.clearSelected();
  assert.equal(app.picker.clear.disabled, true);
  await app.picker.clearSelected();
  await clearing;
  assert.equal(app.requests.length, 4, 'a second click must not duplicate the clear request');
  assert.deepEqual(JSON.parse(app.requests[3].body), { action: 'clear', drones: ['drone1', 'drone2'] });
  assert.equal(app.elements.get('drone1-branch').textContent, 'old · 111');
  assert.equal(app.picker.apply.disabled, false);
  assert.equal(app.picker.clear.disabled, true);
  assert.equal(app.picker.message.textContent, 'Local changes cleared on Drone 1 and Drone 2.');
});

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
  for (const id of ['fullscreen-toggle', 'separation-guard', 'commandFeedback', 'fleetGrid', 'fleet-xyChart', 'drone1-connectionStatus', 'drone2-connectionStatus', 'drone1-connectionToggle', 'drone2-connectionToggle',
    'room-status', 'room-fit', 'room-zoom-in', 'room-zoom-out', 'map-frame', 'map-placeholder', 'map-legend',
    'target-selection', 'target-clear', 'target-send', 'target-stop', 'target-command-status', 'target-status', 'map-stream-status']) elements.set(id, element());
  for (const name of ['scan', 'measure', 'edit', 'apply', 'cancel', 'status', 'preview', 'selection', 'toggle', 'panel', 'form', 'success']) elements.set('calibration-' + name, element());
  elements.get('calibration-panel').hidden = true;
  elements.get('calibration-success').hidden = true;
  for (let i = 0; i < 4; i++) {
    elements.set('calibration-anchor-' + i, element());
    for (const name of ['x', 'y', 'label']) elements.set('calibration-' + name + '-' + i, element());
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
    requestAnimationFrame(callback) { callback(); },
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
    setInterval(callback, delay) { const id = ++nextTimer; (delay === 1000 ? timers : maintenanceTimers).set(id, callback); return id; },
    clearInterval(id) { timers.delete(id); },
    setTimeout(callback, delay) { callback.delay = delay; retries.push(callback); },
    clearTimeout() {},
    CustomEvent: class { constructor(type, options = {}) { this.type = type; this.detail = options.detail; } }
  });
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/anchor-calibration.js'), 'utf8'), context);
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/main.js'), 'utf8'), context);
  listeners.DOMContentLoaded();
  const api = vm.runInContext('({ drones, fleetMap, anchorCalibration, dispatch, heldKeys, connectDrone, currentTarget, awaitSyncStart, confirmSyncStart })', context);
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

function readyRoom(app, drone, seconds = 1) {
  app.receive(drone, '/pidrone/localization/fused', {
    header: { stamp: { secs: seconds, nsecs: 0 }, frame_id: 'map' },
    pose: { pose: { position: { x: 0, y: 0, z: .04 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }
  });
  app.receive(drone, '/pidrone/localization/ready', { data: true });
  app.maintenanceTimers.forEach(callback => callback());
}

test('automatic pairing requires fresh room positions and stays latched after arming', () => {
  const app = dashboard();
  const toggle = app.elements.get('separation-guard');
  toggle.checked = true;
  toggle.listeners.change();
  const a = app.connect('drone1');
  assert.equal(a.publishers.coordination.messages.length, 0, 'wait for a fresh flight mode');
  app.receive(a, '/pidrone/mode', { mode: 'DISARMED' });
  assert.equal(last(a, 'coordination').data, false);
  const b = app.connect('drone2');
  assert.equal(b.publishers.coordination.messages.length, 0, 'wait for the new drone’s flight mode');
  app.receive(b, '/pidrone/mode', { mode: 'DISARMED' });
  assert.equal(last(b, 'coordination').data, false, 'connected without UWB uses local flight');
  readyRoom(app, a);
  assert.equal(last(a, 'coordination').data, false, 'both room positions are required');
  readyRoom(app, b);
  assert.equal(last(a, 'coordination').data, true);
  assert.equal(last(b, 'coordination').data, true);
  app.receive(b, '/pidrone/localization/ready', { data: false });
  app.maintenanceTimers.forEach(callback => callback());
  assert.equal(last(a, 'coordination').data, false, 'unavailable UWB disables the guard before arming');
  app.advanceTime(600);
  readyRoom(app, a, 1.1);
  readyRoom(app, b, 1.1);
  assert.equal(last(a, 'coordination').data, true);
  app.advanceTime(1);
  app.receive(a, '/pidrone/localization/ready', { data: true });
  app.receive(b, '/pidrone/localization/ready', { data: true });
  app.maintenanceTimers.forEach(callback => callback());
  assert.equal(last(a, 'coordination').data, false, 'fresh readiness cannot renew an old position stamp');
  readyRoom(app, a, 1.601);
  readyRoom(app, b, 1.601);
  app.receive(a, '/pidrone/mode', { mode: 'ARMED' });
  app.receive(b, '/pidrone/localization/ready', { data: false });
  app.maintenanceTimers.forEach(callback => callback());
  assert.equal(last(a, 'coordination').data, true, 'UWB loss cannot disable protection after arming');
  b.disconnect();
  assert.equal(last(a, 'coordination').data, true);
  app.receive(a, '/pidrone/mode', { mode: 'DISARMED' });
  assert.equal(last(a, 'coordination').data, false);
  const count = a.publishers.coordination.messages.length;
  app.advanceTime(2000);
  app.maintenanceTimers.forEach(callback => callback());
  assert.equal(a.publishers.coordination.messages.length, count);
});

test('separation toggle defaults off, applies to both drones, and locks outside fresh disarmed mode', () => {
  const app = dashboard();
  const toggle = app.elements.get('separation-guard');
  assert.equal(toggle.checked, false);
  const a = app.connect('drone1'), b = app.connect('drone2');
  app.maintenanceTimers.forEach(callback => callback());
  assert.equal(toggle.disabled, true, 'unknown flight modes lock the switch');
  assert.equal(a.publishers.guardEnabled.messages.length, 0);
  for (const drone of [a, b]) app.receive(drone, '/pidrone/mode', { mode: 'DISARMED' });
  for (const drone of [a, b]) readyRoom(app, drone);
  assert.equal(toggle.disabled, false);
  for (const drone of [a, b]) {
    assert.equal(last(drone, 'guardEnabled').data, false);
    assert.equal(last(drone, 'coordination').data, false, 'fresh UWB must not enable a disabled guard');
  }
  toggle.checked = true;
  toggle.listeners.change();
  for (const drone of [a, b]) {
    assert.equal(last(drone, 'guardEnabled').data, true);
    assert.equal(last(drone, 'coordination').data, true);
  }
  toggle.checked = false;
  toggle.listeners.change();
  for (const drone of [a, b]) assert.equal(last(drone, 'guardEnabled').data, false);
  app.receive(a, '/pidrone/mode', { mode: 'ARMED' });
  assert.equal(toggle.disabled, true);
  toggle.checked = true;
  toggle.listeners.change();
  assert.equal(toggle.checked, false);
  assert.equal(last(a, 'guardEnabled').data, false);
  assert.equal(last(a, 'coordination').data, false);
  app.receive(a, '/pidrone/mode', { mode: 'FLYING' });
  b.disconnect();
  assert.equal(toggle.disabled, true);
  assert.equal(last(a, 'coordination').data, false);
  app.receive(a, '/pidrone/mode', { mode: 'DISARMED' });
  assert.equal(toggle.disabled, false);
  app.advanceTime(1501);
  toggle.checked = true;
  toggle.listeners.change();
  assert.equal(toggle.disabled, true, 'recheck mode freshness when toggled');
  assert.equal(toggle.checked, false);
  a.disconnect();
  app.connect('drone1');
  app.receive(a, '/pidrone/mode', { mode: 'DISARMED' });
  assert.equal(last(a, 'guardEnabled').data, false, 'reconnect retains the disabled choice');
  assert.equal(a.publishers.mode.messages.length, 0, 'guard changes never issue flight commands');
});

test('flight-state tooltip shows preflight refusal and expires grants at the source timestamp', () => {
  const app = dashboard();
  const toggle = app.elements.get('separation-guard');
  toggle.checked = true;
  toggle.listeners.change();
  const a = app.connect('drone1'), b = app.connect('drone2');
  for (const drone of [a, b]) app.receive(drone, '/pidrone/mode', { mode: 'DISARMED' });
  for (const drone of [a, b]) readyRoom(app, drone);
  const report = { ...stamp(1), mode: 'DISARMED', paired: true, ready: false, reason: 'uncertainty_too_large' };
  app.receive(a, '/pidrone/coordination/preflight', report);
  assert.match(a.field('flightMode').title, /blocked · uncertainty too large/);
  report.ready = true;
  report.reason = 'ready';
  app.receive(a, '/pidrone/coordination/preflight', report);
  assert.match(a.field('flightMode').title, /ready · paired/);
  app.advanceTime(400);
  app.receive(a, '/pidrone/coordination/preflight', report);
  app.advanceTime(101);
  app.maintenanceTimers.forEach(callback => callback());
  assert.equal(a.field('flightMode').title, 'Preflight unavailable');
  app.receive(a, '/pidrone/coordination/preflight', report);
  assert.equal(a.field('flightMode').title, 'Preflight unavailable');
  a.disconnect();
  assert.equal(a.field('flightMode').title, 'Preflight unavailable');
});

test('separation status comes from each onboard guard and expires on silence or disconnect', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  const status = drone => drone.field('coordination').textContent;
  assert.equal(status(a), 'Unavailable');
  app.receive(a, '/pidrone/coordination/status', { data: 'clear' });
  app.receive(b, '/pidrone/coordination/status', { data: 'disabled' });
  assert.equal(status(a), 'Active · clear');
  assert.equal(status(b), 'Disabled on drone');
  app.receive(a, '/pidrone/coordination/status', { data: 'uncertainty_too_large' });
  assert.equal(status(a), 'uncertainty too large');
  app.advanceTime(501);
  app.maintenanceTimers.forEach(callback => callback());
  for (const drone of [a, b]) assert.equal(status(drone), 'Unavailable');
  app.receive(a, '/pidrone/coordination/status', { data: 'clear' });
  app.receive(b, '/pidrone/coordination/status', { data: 'solo' });
  a.disconnect();
  assert.equal(status(a), 'Unavailable');
  assert.equal(status(b), 'Solo flight');
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

test('two connections keep telemetry, clocks, and cameras isolated', () => {
  const app = dashboard();
  const a = app.connect('drone1'), b = app.connect('drone2');
  assert.equal(a.ros.url, 'ws://drone1.local:9090');
  assert.equal(b.ros.url, 'ws://drone2.local:9090');
  app.receive(a, '/pidrone/battery', { vbat: 10.8 });
  app.receive(b, '/pidrone/battery', { vbat: 16.8 });
  assert.equal(a.field('battery').textContent, '10.8 V');
  assert.equal(b.field('battery').textContent, '16.8 V');
  for (const [voltage, percent] of [[14, 0], [15.4, 50], [16.8, 100], [17, 100]]) {
    app.receive(a, '/pidrone/battery', { vbat: voltage });
    assert.equal(a.field('battery').textContent, voltage.toFixed(1) + ' V');
    assert.equal(Number(a.field('batteryFill').getAttribute('width')), 13 * percent / 100);
    assert.equal(a.field('batteryStatus').title, voltage.toFixed(1) + ' V · ' + percent + '%');
  }
  app.receive(a, '/pidrone/battery', { vbat: NaN });
  assert.equal(a.field('battery').textContent, '17.0 V');
  app.receive(a, '/pidrone/range', { ...stamp(100), range: 0.2 });
  app.receive(b, '/pidrone/range', { ...stamp(900), range: 0.4 });
  assert.equal(a.field('height').textContent, '0.20 m');
  assert.equal(b.field('height').textContent, '0.40 m');
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

test('synchronized start waits for both routines and fresh preflight grants', () => {
  const app = dashboard();
  const toggle = app.elements.get('separation-guard');
  toggle.checked = true;
  toggle.listeners.change();
  const a = app.connect('drone1'), b = app.connect('drone2');
  const publishers = [a, b].map(drone => drone.publishers.routineStart);
  const starts = () => publishers.map(topic => topic.messages.length);
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
  for (const drone of [a, b]) readyRoom(app, drone, 1.6);
  app.confirmSyncStart();
  assert.deepEqual(starts(), [0, 0], 'routine readiness alone cannot release either drone');
  const report = { ...stamp(1.6), mode: 'DISARMED', paired: true, ready: true, reason: 'ready' };
  app.receive(a, '/pidrone/coordination/preflight', {
    ...report, ...stamp(0), ready: false, reason: 'flight_setup_unavailable'
  });
  app.receive(b, '/pidrone/coordination/preflight', report);
  assert.deepEqual(starts(), [0, 0], 'one blocked drone must hold both routines');
  assert.match(app.elements.get('commandFeedback').textContent, /drone1: Preflight blocked.*flight setup unavailable/);
  app.receive(a, '/pidrone/coordination/preflight', { ...report, ...stamp(1) });
  assert.deepEqual(starts(), [0, 0], 'a stale source timestamp cannot grant a start');
  app.receive(a, '/pidrone/coordination/preflight', { ...report, paired: false });
  assert.deepEqual(starts(), [0, 0], 'a grant for a different flight setup cannot release the run');
  app.receive(a, '/pidrone/coordination/preflight', report);
  assert.deepEqual(starts(), [1, 1]);
  for (const drone of [a, b]) assert.equal(last(drone, 'coordination').data, true);
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
  app.awaitSyncStart(['drone1', 'drone2']);
  app.confirmSyncStart();
  b.disconnect();
  app.receive(a, '/pidrone/routine/ready', {});
  assert.deepEqual(starts(), [1, 1]); // A lost partner must not release a partial start.
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
  assert.equal(app.drones.drone1.field('batteryStatus').hidden, false);
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
  assert.equal(app.drones.drone1.field('batteryStatus').hidden, true);
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

test('terminal lifecycle handles startup failure, detachment and restart reconnection', async () => {
  const app = dashboard();
  const output = [];
  let statusPolls = 0;
  const context = vm.createContext({
    document: { addEventListener() {} }, drones: app.drones,
    fetch() { statusPolls++; throw new Error('unexpected status poll'); },
    Date: class extends Date { static now() { return 1000; } },
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
  const result = { textContent: '', hidden: true };
  Object.assign(pane, { root: { querySelector() { return result; } },
    id: 'drone1', socket: null, enabled: true, changed() {},
    term: { reset() {}, write(message) { output.push(message); }, cols: 80, rows: 24 }, fit: { fit() {} } });
  app.connectDrone('drone1');
  pane.report('Failed: Warm reboot incomplete: Incomplete MSP reply.');
  pane.attach(true);
  pane.socket.onmessage({ data: '\x1b[2J' });
  assert.equal(result.textContent, 'Failed: Warm reboot incomplete: Incomplete MSP reply.');
  assert.equal(result.hidden, false);
  pane.report('Restarting nodes…');
  assert.equal(result.textContent, 'Restarting nodes…');
  const stalled = app.drones.drone1.ros;
  const deadline = app.drones.drone1.waitUntil;
  pane.started(true);
  assert.equal(app.drones.drone1.ros, null);
  assert.equal(app.drones.drone1.waitUntil, deadline);
  assert.equal(app.retries.at(-1).delay, 0, 'ready bridge should reconnect without the backoff');
  stalled.open();
  assert.equal(app.drones.drone1.connected, false);
  app.retries.pop()();
  const replacement = app.drones.drone1.ros;
  replacement.open();
  assert.equal(app.drones.drone1.connected, true);
  pane.started(true);
  assert.equal(app.drones.drone1.ros, replacement, 'startup must preserve an open bridge');
  app.drones.drone1.disconnect();
  app.connectDrone('drone1');
  app.drones.drone1.retry(app.drones.drone1.ros);
  assert.equal(app.drones.drone1.ros, null);
  pane.started(true);
  assert.equal(app.retries.at(-1).delay, 0, 'readiness should also replace an already queued retry');
  app.retries.pop()();
  assert.ok(app.drones.drone1.ros);
  app.drones.drone1.disconnect();
  app.connectDrone('drone1');
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

  pane.attach();
  const oldTerminal = pane.socket, oldRos = drone.ros;
  let finishRestart;
  pane.post = (path, body) => {
    assert.equal(path, '/api/nodes');
    assert.equal(body.action, 'restart');
    return new Promise(resolve => { finishRestart = resolve; });
  };
  const restart = pane.restart();
  await assert.rejects(pane.restart(), /already in progress/);
  await Terminal.prototype.poll.call(pane);
  assert.equal(statusPolls, 0, 'status polling pauses during restart');
  assert.equal(drone.connected, false);
  assert.equal(app.elements.get('drone1-connectionStatus').querySelector('[data-connection-label]').textContent, 'Restarting…');
  oldTerminal.onclose();
  oldRos.open();
  assert.equal(drone.connected, false, 'old callbacks cannot restore the interrupted connection');
  app.advanceTime(100000);
  app.maintenanceTimers.forEach(callback => callback());
  assert.notEqual(drone.waitUntil, 0, 'allow time for gyro checking and localization beyond the old 90-second limit');
  finishRestart({ output: 'Nodes restarted' });
  await restart;
  assert.equal(pane.restarting, false);
  assert.notEqual(drone.ros, oldRos);
  assert.notEqual(pane.socket, oldTerminal);
  drone.ros.open();
  assert.equal(drone.connected, true);
  assert.equal(pane.ready, false, 'reload the script list after restart');
  pane.scripts = [];
  assert.equal(pane.ready, true);
  assert.equal(drone.publishers.mode.messages.length, 0, 'reconnecting sends no flight command');

  const cancelled = pane.restart();
  drone.disconnect();
  finishRestart({ output: 'Nodes restarted' });
  await cancelled;
  assert.equal(drone.ros, null, 'cancelling prevents a late restart response from reconnecting');

  pane.post = async () => { throw new Error('Restart failed'); };
  await assert.rejects(pane.restart(), /Restart failed/);
  assert.equal(drone.waitUntil, 0);
  assert.equal(pane.restarting, false);
  assert.equal(app.elements.get('drone1-connectionStatus').title, 'Nodes failed to restart');
});

test('restart confirms the combined operation and blocks busy drones', async () => {
  let confirmed = false, prompt, restarts = [];
  const context = vm.createContext({ document: { addEventListener() {} },
    window: { confirm(message) { prompt = message; return confirmed; } } });
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/routines.js'), 'utf8'), context);
  const Runner = vm.runInContext('RoutineRunner', context);
  const runner = Object.create(Runner.prototype);
  Object.assign(runner, {
    terminals: ['drone1', 'drone2'].map(id => ({ id, enabled: true, nodes: true, running: false,
      restart() { restarts.push(id); } })),
    run(label, targets, action) { return Promise.all(targets.map(action)); }
  });
  await runner.restart();
  assert.deepEqual(restarts, []);
  assert.match(prompt, /^Restart drone1 and drone2\?/);
  assert.match(prompt, /powered for a few minutes/);
  assert.match(prompt, /props off/);
  assert.match(prompt, /disarmed.*floor.*facing map \+Y/);
  confirmed = true;
  runner.terminals[0].running = true;
  await runner.restart();
  assert.deepEqual(restarts, []);
  runner.terminals[0].running = false;
  await runner.restart();
  assert.deepEqual(restarts, ['drone1', 'drone2']);
  restarts = [];
  runner.terminals[0].running = true;
  await runner.restartTargets(['drone2']);
  assert.deepEqual(restarts, ['drone2'], 'branch restart must only affect its captured targets');
  assert.throws(() => runner.restartTargets(['drone1']), /Restart could not start/);
});

test('a stalled restart response releases the runner without retrying the command', async () => {
  const app = dashboard();
  let expire, requests = 0, cleared = false;
  const context = vm.createContext({ document: { addEventListener() {} }, drones: app.drones,
    AbortController, setTimeout(callback) { expire = callback; return 1; },
    clearTimeout() { cleared = true; },
    fetch(path, options) {
      requests++;
      return new Promise((resolve, reject) => options.signal.addEventListener('abort',
        () => reject(new Error('aborted'))));
    }
  });
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/routines.js'), 'utf8'), context);
  const Terminal = vm.runInContext('RoutineTerminal', context);
  const Runner = vm.runInContext('RoutineRunner', context);
  const pane = Object.assign(Object.create(Terminal.prototype), { id: 'drone1', report() {} });
  const runner = Object.assign(Object.create(Runner.prototype), { render() {} });
  const request = runner.run('Restarting', [pane], terminal => terminal.restart());
  assert.equal(runner.busy, true);
  expire();
  const results = await request;
  assert.equal(results[0].status, 'rejected');
  assert.match(results[0].reason.message, /may still be restarting.*Reconnect/);
  assert.equal(requests, 1);
  assert.equal(cleared, true);
  assert.equal(runner.busy, false);
  assert.equal(pane.restarting, false);
  assert.equal(app.drones.drone1.waitUntil, 0);
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
  assert.ok(Math.abs(parseFloat(a.field('verticalSpeed').textContent) - 0.2) < 0.02);
  assert.match(a.field('verticalSpeed').textContent, /m\/s ↑$/);
  const value = a.field('verticalSpeed').textContent;
  app.receive(a, '/pidrone/range', rangeAt(100.3, 1.1));
  app.receive(a, '/pidrone/range', rangeAt(100.1, 1.1));
  assert.equal(a.field('verticalSpeed').textContent, value);
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
  assert.match(app.elements.get('room-status').textContent, /saved calibration/);
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
  assert.deepEqual(app.calibrationRequests.at(-1), { action: 'apply', token: 'reviewed-token',
    positions: boards.map((p, i) => ({ id: p.id, x: i, y: i })) });
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

test('saved anchor coordinates can be adjusted without polling overwriting edits', async () => {
  const app = dashboard({ discover: false });
  const boards = ['980C', '9781', 'CA83', '4D12'].map((id, i) =>
    ({ id, ble: 'uuid-' + i, active: true, initiator: i === 3, pan: null }));
  const positions = boards.map((board, i) => ({ ...board, x: [0, 2, 4, 2][i], y: [0, 0, 6, 6][i], z: 0 }));
  const state = { stage: 'idle', message: 'Ready', boards, busy: false,
    layout: { revision: 'saved', frame: 'map', verified: false, positions } };
  app.anchorCalibration.update(state);
  await app.anchorCalibration.request('edit');
  assert.deepEqual(app.calibrationRequests.at(-1), { action: 'edit' });
  const preview = { ...state, stage: 'preview', token: 'edit-token', preview: { positions } };
  app.anchorCalibration.update(preview);
  const topLeft = app.elements.get('calibration-x-3');
  topLeft.value = '0';
  topLeft.listeners.input();
  app.anchorCalibration.update(preview);
  assert.equal(topLeft.value, '0');
  const topRight = app.elements.get('calibration-x-2');
  topRight.value = '';
  topRight.listeners.input();
  assert.equal(app.elements.get('calibration-apply').disabled, true);
  topRight.value = '2';
  topRight.listeners.input();
  await app.anchorCalibration.request('apply');
  assert.deepEqual(app.calibrationRequests.at(-1), { action: 'apply', token: 'edit-token',
    positions: boards.map((p, i) => ({ id: p.id, x: [0, 2, 2, 0][i], y: [0, 0, 6, 6][i] })) });
  assert.equal(app.calibrationRequests.length, 2);
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

test('routine execution synchronizes start for one or two drones', async () => {
  let released, sends = [];
  const context = vm.createContext({ document: { addEventListener() {} },
    awaitSyncStart(ids) { released = ids; }, confirmSyncStart() {} });
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../js/routines.js'), 'utf8'), context);
  const Runner = vm.runInContext('RoutineRunner', context);
  for (const [ids, target, expected] of [
    [['drone2'], 'all', ['drone2']],
    [['drone1', 'drone2'], 'all', ['drone1', 'drone2']],
    [['drone1', 'drone2'], 'drone1', ['drone1']],
    [['drone2'], 'drone1', []]
  ]) {
    released = null;
    const runner = Object.create(Runner.prototype);
    Object.assign(runner, { target: { value: target }, select: { value: 'prototype-1-forward.py' },
      args: { value: '--fly --distance .3' },
      terminals: ['drone1', 'drone2'].map(id => ({ id, enabled: ids.includes(id), ready: true,
        execute(script, args) { sends.push({ id, script, args }); } })),
      async run(label, targets, action) { return Promise.allSettled(targets.map(action)); } });
    await runner.execute();
    assert.deepEqual(released ? Array.from(released) : [], expected);
    assert.deepEqual(sends, expected.map(id => ({ id, script: 'prototype-1-forward.py', args: '--fly --distance .3' })));
    sends = [];
  }
});
