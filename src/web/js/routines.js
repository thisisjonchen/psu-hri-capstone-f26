'use strict';

// Each terminal attaches to its drone's `pidrone` screen session through the laptop
// server. Connect opens it in start mode: the nodes start in the terminal, so their
// startup output is visible. Closing or toggling off only detaches; routines keep
// running on the drone.
class RoutineTerminal {
  constructor(id, changed) {
    this.id = id;
    this.changed = changed;
    this.root = document.querySelector('[data-terminal="' + id + '"]');
    this.toggleButton = this.root.querySelector('[data-field="toggle"]');
    this.args = this.root.querySelector('[data-field="args"]');
    this.stateText = this.root.querySelector('[data-field="state"]');
    this.resultText = this.root.querySelector('[data-field="result"]');
    this.enabled = false;
    this.term = null;
    this.fit = null;
    this.socket = null;
    this.scripts = null;
    this.nodes = false;
    this.running = false;
    this.error = null;
    this.result = '';
    this.polling = false;
    this.toggleButton.addEventListener('click', () => this.setEnabled(!this.enabled));
  }

  setEnabled(enabled, start = false) {
    this.enabled = enabled;
    this.toggleButton.setAttribute('aria-pressed', String(enabled));
    if (enabled) this.attach(start);
    else if (this.socket) this.socket.close();
    this.scripts = null;
    this.error = null;
    this.result = '';
    this.poll();
    this.changed();
  }

  connectDrone() {
    // Already attached to running nodes: starting again would change nothing.
    if (!(this.enabled && this.socket && this.nodes)) this.setEnabled(true, true);
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
      window.addEventListener('resize', () => this.fit.fit());
    }
    this.term.reset();
    this.fit.fit();
    const socket = new WebSocket('ws://' + location.hostname + ':' + (Number(location.port) + 1) +
      '/terminal/' + this.id + (start ? '?start=1' : ''));
    socket.binaryType = 'arraybuffer';
    socket.onopen = () => socket.send(JSON.stringify({ cols: this.term.cols, rows: this.term.rows }));
    socket.onmessage = event => this.term.write(typeof event.data === 'string' ? event.data : new Uint8Array(event.data));
    socket.onclose = () => {
      if (this.socket !== socket) return;
      this.socket = null;
      this.term.write('\r\n\x1b[2m[detached; anything running on ' + this.id + ' keeps running]\x1b[0m\r\n');
      this.changed();
    };
    this.socket = socket;
  }

  started(ok) {
    this.nodes = ok;
    this.result = ok ? 'Nodes started; the routine window lists each node.' : 'Nodes failed to start; see the terminal.';
    if (!ok) drones[this.id].disconnect('Nodes failed to start');
    this.changed();
  }

  send(message) {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) this.socket.send(JSON.stringify(message));
  }

  async poll() {
    if (!this.enabled || this.polling) return;
    this.polling = true;
    try {
      const response = await fetch('/api/routine?drone=' + this.id);
      const body = await response.json();
      if (!response.ok) throw new Error(body.error || response.statusText);
      this.scripts = body.scripts;
      this.nodes = body.nodes;
      this.running = body.running;
      this.error = null;
    } catch (error) {
      this.scripts = null;
      this.error = error.message;
    } finally {
      this.polling = false;
      this.changed();
    }
  }

  get ready() {
    // The synchronized start reaches the routine over this drone's rosbridge.
    return this.enabled && !!this.scripts && this.nodes && !this.running && !!this.socket &&
      drones[this.id].connected;
  }

  render() {
    const state = !this.enabled ? 'Off' : this.error ? 'Offline: ' + this.error
      : !this.scripts ? 'Connecting…' : !this.nodes ? 'Nodes stopped' : this.running ? 'Running a routine'
      : !this.socket ? 'Detached' : 'Idle';
    this.stateText.textContent = state;
    this.stateText.dataset.state = !this.enabled ? 'off' : this.error ? 'error' : this.running ? 'running' : 'idle';
    this.resultText.textContent = this.enabled ? this.result : '';
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

  async execute(script) {
    await this.post('/api/execute', { script, args: this.args.value.trim() });
    this.running = true;
    return 'Started ' + script;
  }
}

class RoutineRunner {
  constructor() {
    this.select = document.getElementById('routine-script');
    this.button = document.getElementById('routine-execute');
    this.restartButton = document.getElementById('nodes-restart');
    this.alignButton = document.getElementById('nodes-align');
    this.status = document.getElementById('routine-status');
    this.terminals = ['drone1', 'drone2'].map(id => new RoutineTerminal(id, () => this.render()));
    this.button.addEventListener('click', () => this.execute());
    this.restartButton.addEventListener('click', () => this.nodeAction('restart'));
    this.alignButton.addEventListener('click', () => this.nodeAction('align'));
    this.select.addEventListener('change', () => this.render());
    // The Connect and Disconnect buttons (main.js) also open and close the terminals.
    const terminal = id => this.terminals.find(candidate => candidate.id === id);
    document.addEventListener('drone-connect', event => terminal(event.detail).connectDrone());
    document.addEventListener('drone-disconnect', event => terminal(event.detail).setEnabled(false));
    setInterval(() => this.terminals.forEach(terminal => terminal.poll()), 2000);
    this.render();
  }

  get active() {
    return this.terminals.filter(terminal => terminal.enabled);
  }

  render() {
    this.terminals.forEach(terminal => terminal.render());
    const lists = this.active.filter(terminal => terminal.scripts).map(terminal => terminal.scripts);
    // Offer only scripts every enabled drone has, so Execute runs the same file everywhere.
    const scripts = lists.length ? lists[0].filter(name => lists.every(list => list.includes(name))) : [];
    const selected = this.select.value;
    if (scripts.join() !== Array.from(this.select.options, option => option.value).filter(Boolean).join()) {
      const options = scripts.map(name => new Option(name, name));
      options.unshift(new Option(scripts.length ? 'Select a script' : 'Enable a drone terminal', ''));
      this.select.replaceChildren(...options);
      this.select.value = scripts.includes(selected) ? selected : '';
    }
    const differ = lists.length > 1 && lists.some(list => list.join() !== lists[0].join());
    const waiting = this.active.filter(terminal => !terminal.ready).map(terminal => terminal.id);
    const loaded = this.active.length > 0 && this.active.every(terminal => terminal.scripts);
    this.button.disabled = this.busy || !this.select.value || !this.active.length || waiting.length > 0;
    this.restartButton.disabled = this.alignButton.disabled = this.busy || !loaded ||
      !this.active.every(terminal => terminal.nodes && !terminal.running);
    if (!this.busy) {
      this.status.textContent = differ ? 'Drones have different routine files; check that both are on the same branch.'
        : waiting.length ? 'Waiting for ' + waiting.join(' and ') + ' (must be connected, attached and idle).' : '';
    }
  }

  async run(label, targets, action) {
    this.busy = true;
    this.status.textContent = label + ' on ' + targets.map(terminal => terminal.id).join(' and ') + '…';
    this.render();
    const results = await Promise.allSettled(targets.map(action));
    this.busy = false;
    results.forEach((result, index) => {
      targets[index].result = result.status === 'fulfilled' ? result.value : 'Failed: ' + result.reason.message;
    });
    const failed = results.map((result, index) => result.status === 'rejected'
      ? targets[index].id + ': ' + result.reason.message : null).filter(Boolean);
    this.status.textContent = failed.length ? label + ' failed — ' + failed.join('; ') : label + ' done.';
    this.render();
    return results;
  }

  async execute() {
    const script = this.select.value;
    const targets = this.active;
    if (this.busy || !script || !targets.length || !targets.every(terminal => terminal.ready)) return;
    const stages = script === 'prototype-1-forward.py' ? ['forward', 'land'] : [];
    awaitSyncStart(targets.map(terminal => terminal.id), stages);
    const results = await this.run('Starting ' + script, targets, terminal => terminal.execute(script));
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
