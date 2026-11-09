'use strict';

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
    this.lastPollAt = 0;
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
    const drone = drones[this.id];
    if (!ok) drone.failConnection('Nodes failed to start');
    // Startup confirms rosbridge is listening. Replace a socket that may still
    // be waiting on an attempt made before the bridge was available.
    else if (this.enabled && drone.waitUntil && drone.ros && !drone.connected) drone.retry(drone.ros);
    this.changed();
  }

  send(message) {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) this.socket.send(JSON.stringify(message));
  }

  async poll() {
    if (!this.enabled || this.polling) return;
    if (this.running && Date.now() - this.lastPollAt < 10000) return;
    this.lastPollAt = Date.now();
    this.polling = true;
    const socket = this.socket;
    try {
      const response = await fetch('/api/routine?drone=' + this.id + (this.scripts && this.nodes ? '&running=1' : ''));
      const body = await response.json();
      if (!this.enabled || this.socket !== socket) return;
      if (!response.ok) throw new Error(body.error || response.statusText);
      if (body.scripts) this.scripts = body.scripts;
      if (body.nodes !== undefined) this.nodes = body.nodes;
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
    if (!message) return;
    const status = this.root.querySelector('[data-field="result"]');
    status.textContent = message;
    status.hidden = false;
    if (this.term) this.term.write('\r\n[' + message + ']\r\n');
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
    const restart = action === 'restart';
    let deadline;
    if (restart) {
      const socket = this.socket;
      this.socket = null;
      if (socket) socket.close();
      telemetry.disconnect('Restarting…');
      deadline = telemetry.waitUntil = Date.now() + 90000;
    }
    let result;
    try {
      result = await this.post('/api/nodes', { action });
    } catch (error) {
      if (restart && telemetry.waitUntil === deadline) telemetry.failConnection('Nodes failed to restart');
      throw error;
    }
    this.nodes = true;
    this.scripts = null;
    if (this.enabled && (!restart || telemetry.waitUntil === deadline)) {
      if (!this.socket) this.attach();
      if (restart) telemetry.connect(true);
    }
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
    this.target = document.getElementById('routine-target');
    this.args = document.getElementById('routine-args');
    this.button = document.getElementById('routine-execute');
    this.restartButton = document.getElementById('nodes-restart');
    this.alignButton = document.getElementById('nodes-align');
    this.terminals = ['drone1', 'drone2'].map(id => new RoutineTerminal(id, () => this.render()));
    this.button.addEventListener('click', () => this.execute());
    this.restartButton.addEventListener('click', () => this.nodeAction('restart'));
    this.alignButton.addEventListener('click', () => this.nodeAction('align'));
    this.select.addEventListener('change', () => this.render());
    this.target.addEventListener('change', () => this.render());
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

  get targets() {
    return this.active.filter(terminal => this.target.value === 'all' || terminal.id === this.target.value);
  }

  render() {
    const lists = this.targets.filter(terminal => terminal.scripts).map(terminal => terminal.scripts);
    // Offer only scripts available on every selected drone.
    const scripts = lists.length ? lists[0].filter(name => lists.every(list => list.includes(name))) : [];
    const selected = this.select.value;
    if (scripts.join() !== Array.from(this.select.options, option => option.value).filter(Boolean).join()) {
      const options = scripts.map(name => new Option(name, name));
      options.unshift(new Option(scripts.length ? 'Select routine' : 'Connect a drone', ''));
      this.select.replaceChildren(...options);
      this.select.value = scripts.includes(selected) ? selected : '';
    }
    const waiting = this.targets.filter(terminal => !terminal.ready).map(terminal => terminal.id);
    const loaded = this.active.length > 0 && this.active.every(terminal => terminal.scripts);
    this.button.disabled = this.busy || !this.select.value || !this.targets.length || waiting.length > 0;
    this.target.disabled = this.busy;
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
    const targets = this.targets;
    if (this.busy || !script || !targets.length || !targets.every(terminal => terminal.ready)) return;
    await this.run('Starting ' + script, targets, terminal => terminal.execute(script, args));
  }

  nodeAction(action) {
    const targets = this.active;
    const names = targets.map(terminal => terminal.id).join(' and ');
    if (this.busy || !targets.length || !targets.every(terminal => terminal.nodes && !terminal.running)) return;
    const prompts = { restart: 'Restart all nodes on ', align: 'Recalibrate gyro and relocalize ' };
    const labels = { restart: 'Restarting nodes', align: 'Rebooting FC, checking gyro and relocalizing' };
    const warmup = action === 'align' ? '\n\nLeave powered for a few minutes first. Props must be removed. Keep each drone untouched on the floor throughout the gyro calibration and relocalization. Only FC communication and localization restart; settings are unchanged.' : '';
    // Fusion locks map +Y when its node starts, so confirm placement first.
    if (!window.confirm(prompts[action] + names + '?\n\nEach drone must be disarmed, stationary and ' +
        'facing map +Y: its heading is aligned now.' + warmup)) return;
    return this.run(labels[action], targets, terminal => terminal.nodeAction(action));
  }
}

document.addEventListener('DOMContentLoaded', () => { new RoutineRunner(); });
