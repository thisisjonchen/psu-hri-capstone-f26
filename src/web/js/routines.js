'use strict';

function aprilTagOptions(args) {
  const options = { drones: 'single' }, words = args.trim().split(/\s+/);
  for (let i = 0; i < words.length; i++) {
    if (words[i] === '--fly') continue;
    const [flag, inline] = words[i].split('=');
    if (!['--drones', '--tags'].includes(flag)) throw new Error('Use --fly, --drones single|both, and --tags 1,2,3.');
    options[flag.slice(2)] = inline === undefined ? words[++i] : inline;
  }
  if (!['single', 'both'].includes(options.drones) || !/^\d+(,\d+)*$/.test(options.tags || '')) {
    throw new Error('Choose --drones single or both and an ordered --tags list.');
  }
  options.tags = options.tags.split(',').map(Number);
  if (options.tags.length > 12 || new Set(options.tags).size !== options.tags.length ||
      options.tags.some(id => !Number.isSafeInteger(id) || id > 2147483647)) {
    throw new Error('Use up to 12 distinct tag IDs.');
  }
  return options;
}

// Each terminal attaches to its drone's `pidrone` screen session through the laptop
// server. Connect opens it in start mode: the nodes start in the terminal, so their
// startup output is visible. Disconnect detaches; routines keep running on the drone.
class RoutineTerminal {
  constructor(id, changed) {
    this.id = id;
    this.changed = changed;
    this.root = document.querySelector('[data-terminal="' + id + '"]');
    this.enabled = false;
    this.term = null;
    this.fit = null;
    this.socket = null;
    this.scripts = null;
    this.nodes = false;
    this.running = false;
    this.polling = false;
  }

  setEnabled(enabled, start = false) {
    this.enabled = enabled;
    if (enabled) this.attach(start);
    else if (this.socket) this.socket.close();
    this.scripts = null;
    this.poll();
    this.changed();
  }

  connectDrone() {
    this.setEnabled(true, true);
  }

  attach(start = false) {
    if (this.socket) {
      const previous = this.socket;
      this.socket = null;
      previous.close();
    }
    if (!this.term) {
      this.term = new Terminal({ fontSize: 12, scrollback: 5000, convertEol: false,
        theme: { background: '#0f1726', foreground: '#e4eaf2' } });
      this.fit = new FitAddon.FitAddon();
      this.term.loadAddon(this.fit);
      this.term.open(this.root.querySelector('[data-field="terminal"]'));
      // Esc hands the keyboard back to the flight shortcuts (Space = Disarm all).
      this.term.attachCustomKeyEventHandler(event => {
        if (event.key !== 'Escape') return true;
        if (event.type === 'keydown') this.term.blur();
        return false;
      });
      // screen's alternate buffer has no scrollback, so xterm turns the wheel into arrow
      // keys that cycle shell history; leave the wheel to the page there.
      this.term.attachCustomWheelEventHandler(() => this.term.buffer.active.type !== 'alternate');
      // drones.open_terminal reports the node start's exit status as OSC 777.
      this.term.parser.registerOscHandler(777, status => { this.started(status === '0'); return true; });
      this.term.onData(input => this.send({ input }));
      this.term.onResize(({ cols, rows }) => this.send({ cols, rows }));
      this.resizeObserver = new ResizeObserver(() => this.fit.fit());
      this.resizeObserver.observe(this.root.querySelector('[data-field="terminal"]'));
    }
    this.term.reset();
    this.fit.fit();
    const socket = new WebSocket('ws://' + location.hostname + ':' + (Number(location.port) + 1) +
      '/terminal/' + this.id + (start ? '?start=1' : ''));
    socket.binaryType = 'arraybuffer';
    socket.onopen = () => socket.send(JSON.stringify({ cols: this.term.cols, rows: this.term.rows }));
    socket.onmessage = event => {
      if (this.socket !== socket || !this.enabled) return;
      this.term.write(typeof event.data === 'string' ? event.data : new Uint8Array(event.data));
    };
    socket.onclose = () => {
      if (this.socket !== socket) return;
      this.socket = null;
      const drone = drones[this.id];
      if (!drone.connected && drone.waitUntil) drone.failConnection('Connection error');
      this.term.write('\r\n\x1b[2m[detached; anything running on ' + this.id + ' keeps running]\x1b[0m\r\n');
      this.changed();
    };
    this.socket = socket;
  }

  started(ok) {
    this.nodes = ok;
    if (!ok) drones[this.id].failConnection('Nodes failed to start');
    this.changed();
  }

  send(message) {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) this.socket.send(JSON.stringify(message));
  }

  async poll() {
    if (!this.enabled || this.polling) return;
    this.polling = true;
    const socket = this.socket;
    try {
      const response = await fetch('/api/routine?drone=' + this.id);
      const body = await response.json();
      if (!this.enabled || this.socket !== socket) return;
      if (!response.ok) throw new Error(body.error || response.statusText);
      this.scripts = body.scripts;
      this.nodes = body.nodes;
      this.running = body.running;
    } catch (_) {
      if (!this.enabled || this.socket !== socket) return;
      this.scripts = null;
    } finally {
      this.polling = false;
      this.changed();
    }
  }

  get ready() {
    // The synchronized start reaches the routine over this drone's rosbridge.
    return this.enabled && !!this.scripts && this.nodes && !this.running &&
      this.socket?.readyState === WebSocket.OPEN &&
      drones[this.id].connected;
  }

  report(message) {
    if (this.term && message) this.term.write('\r\n[' + message + ']\r\n');
  }

  async post(path, body) {
    const response = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', 'X-Routine-Request': '1' },
      body: JSON.stringify(Object.assign({ drone: this.id }, body))
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || response.statusText);
    return result;
  }

  async nodeAction(action) {
    // Restarting ends the screen session and rosbridge, so reattach both afterwards.
    const telemetry = drones[this.id];
    const reconnect = action === 'restart' && !!telemetry.ros;
    if (action === 'restart' && this.socket) this.socket.close();
    const result = await this.post('/api/nodes', { action });
    this.nodes = true;
    if (!this.socket) this.attach();
    if (reconnect) telemetry.connect(true);
    return result.output;
  }

  async execute(script, args) {
    await this.post('/api/execute', { script, args });
    this.running = true;
    return 'Started ' + script;
  }
}

class RoutineRunner {
  constructor() {
    this.select = document.getElementById('routine-script');
    this.args = document.getElementById('routine-args');
    this.button = document.getElementById('routine-execute');
    this.restartButton = document.getElementById('nodes-restart');
    this.alignButton = document.getElementById('nodes-align');
    this.terminals = ['drone1', 'drone2'].map(id => new RoutineTerminal(id, () => this.render()));
    this.button.addEventListener('click', () => this.execute());
    this.restartButton.addEventListener('click', () => this.nodeAction('restart'));
    this.alignButton.addEventListener('click', () => this.nodeAction('align'));
    this.select.addEventListener('change', () => this.render());
    // The Connect and Disconnect buttons (main.js) also open and close the terminals.
    const terminal = id => this.terminals.find(candidate => candidate.id === id);
    document.addEventListener('drone-connect', event => terminal(event.detail).connectDrone());
    document.addEventListener('drone-disconnect', event => terminal(event.detail).setEnabled(false));
    document.addEventListener('drone-connect-failed', event => {
      const pane = terminal(event.detail.id);
      pane.setEnabled(false);
      pane.report(event.detail.message);
      this.render();
    });
    setInterval(() => this.terminals.forEach(terminal => terminal.poll()), 2000);
    this.render();
  }

  get active() {
    return this.terminals.filter(terminal => terminal.enabled);
  }

  render() {
    const lists = this.active.filter(terminal => terminal.scripts).map(terminal => terminal.scripts);
    // Offer only scripts every enabled drone has, so Execute runs the same file everywhere.
    const scripts = lists.length ? lists[0].filter(name => lists.every(list => list.includes(name))) : [];
    const selected = this.select.value;
    if (scripts.join() !== Array.from(this.select.options, option => option.value).filter(Boolean).join()) {
      const options = scripts.map(name => new Option(name, name));
      options.unshift(new Option(scripts.length ? 'Select routine' : 'Connect a drone', ''));
      this.select.replaceChildren(...options);
      this.select.value = scripts.includes(selected) ? selected : '';
    }
    const waiting = this.active.filter(terminal => !terminal.ready).map(terminal => terminal.id);
    const loaded = this.active.length > 0 && this.active.every(terminal => terminal.scripts);
    this.button.disabled = this.busy || !this.select.value || !this.active.length || waiting.length > 0;
    this.restartButton.disabled = this.alignButton.disabled = this.busy || !loaded ||
      !this.active.every(terminal => terminal.nodes && !terminal.running);
  }

  async run(label, targets, action) {
    this.busy = true;
    targets.forEach(terminal => terminal.report(label + '…'));
    this.render();
    const results = await Promise.allSettled(targets.map(action));
    this.busy = false;
    results.forEach((result, index) => {
      targets[index].report(result.status === 'fulfilled' ? result.value : 'Failed: ' + result.reason.message);
    });
    this.render();
    return results;
  }

  async execute() {
    const script = this.select.value;
    const args = this.args.value.trim();
    const targets = this.active;
    if (this.busy || !script || !targets.length || !targets.every(terminal => terminal.ready)) return;
    let stages = [], plan = null;
    if (script === 'prototype-1-apriltag.py') {
      try {
        const options = aprilTagOptions(args);
        if (targets.length !== (options.drones === 'both' ? 2 : 1)) {
          throw new Error('Connect ' + (options.drones === 'both' ? 'both drones' : 'exactly one drone') + ' for this mode.');
        }
        plan = makeAprilTagPlan(targets.map(terminal => terminal.id), options.tags);
        stages = options.tags.map((_, i) => 'tag-' + i).concat('land');
      } catch (error) { feedback(error.message); return; }
    }
    awaitSyncStart(targets.map(terminal => terminal.id), stages, plan);
    const results = await this.run('Starting ' + script, targets, terminal => terminal.execute(script, args));
    if (results.every(result => result.status === 'fulfilled')) confirmSyncStart();
    else awaitSyncStart(null);
  }

  nodeAction(action) {
    const targets = this.active;
    const names = targets.map(terminal => terminal.id).join(' and ');
    const prompts = { restart: 'Restart all nodes on ', align: 'Re-align map +Y on ' };
    const labels = { restart: 'Restarting nodes', align: 'Aligning heading' };
    // Fusion locks map +Y when its node starts, so confirm placement first.
    if (!window.confirm(prompts[action] + names + '?\n\nEach drone must be disarmed, stationary and ' +
        'facing map +Y: its heading is aligned now.' + (action === 'align' ? ' Only fusion restarts.' : ''))) return;
    this.run(labels[action], targets, terminal => terminal.nodeAction(action));
  }
}

document.addEventListener('DOMContentLoaded', () => { new RoutineRunner(); });
