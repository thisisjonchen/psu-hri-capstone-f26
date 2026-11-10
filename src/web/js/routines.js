'use strict';

let routineRunner = null;

function confirmRestart(names, action = 'Restart') {
  // Fusion locks map +Y when its node starts, so confirm placement first.
  return window.confirm(action + ' ' + names + '?\n\nLeave powered for a few minutes first. ' +
    'Keep props off and drones disarmed, still on the floor and facing map +Y until complete.');
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
    this.restarting = false;
    this.lastPollAt = 0;
    this.reportTimer = null;
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
    let connectionError = null;
    socket.binaryType = 'arraybuffer';
    socket.onopen = () => socket.send(JSON.stringify({ cols: this.term.cols, rows: this.term.rows }));
    socket.onmessage = event => {
      if (this.socket !== socket || !this.enabled) return;
      // The server sends connection errors as text; drone output is binary.
      if (typeof event.data === 'string') {
        connectionError = event.data.trim();
        this.report(connectionError, 'error');
      } else this.term.write(new Uint8Array(event.data));
    };
    socket.onclose = () => {
      if (this.socket !== socket) return;
      this.socket = null;
      const drone = drones[this.id];
      if (!drone.connected && drone.waitUntil) drone.failConnection('Connection error');
      if (connectionError) this.report(connectionError, 'error');
      else if (this.enabled && drone.connected) this.report('Terminal disconnected; reconnecting…', 'progress');
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
    else if (this.enabled && drone.waitUntil && !drone.connected) drone.retry(drone.ros, 0);
    this.changed();
  }

  send(message) {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) this.socket.send(JSON.stringify(message));
  }

  async poll() {
    if (!this.enabled || this.polling || this.restarting) return;
    if (this.running && Date.now() - this.lastPollAt < 10000) return;
    this.lastPollAt = Date.now();
    this.polling = true;
    const socket = this.socket;
    try {
      const response = await fetch('/api/routine?drone=' + this.id + (this.scripts && this.nodes ? '&running=1' : ''));
      const body = await response.json();
      if (!this.enabled || this.socket !== socket || this.restarting) return;
      if (!response.ok) throw new Error(body.error || response.statusText);
      if (body.scripts) this.scripts = body.scripts;
      if (body.nodes !== undefined) this.nodes = body.nodes;
      this.running = body.running;
      // Restore only the terminal after SSH recovers; never replay node startup
      // or a routine command whose delivery may already have succeeded.
      if (!this.socket && this.nodes && drones[this.id].connected) this.attach();
    } catch (_) {
      if (!this.enabled || this.socket !== socket || this.restarting) return;
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

  report(message, state = 'info') {
    if (!message) return;
    clearTimeout(this.reportTimer);
    const status = this.root.querySelector('[data-field="result"]');
    status.textContent = message;
    status.title = message;
    status.dataset.state = state;
    status.hidden = false;
    this.reportTimer = setTimeout(() => {
      status.hidden = true;
      this.reportTimer = null;
    }, 8000);
  }

  async post(path, body, timeout = 0) {
    const controller = timeout ? new AbortController() : null;
    const timer = controller ? setTimeout(() => controller.abort(), timeout) : null;
    try {
      const response = await fetch(path, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Routine-Request': '1' },
        body: JSON.stringify(Object.assign({ drone: this.id }, body)),
        signal: controller?.signal
      });
      const result = await response.json();
      if (!response.ok) throw new Error(result.error || response.statusText);
      return result;
    } catch (error) {
      if (controller?.signal.aborted) throw new Error('Restart response timed out; the drone may still be restarting. Reconnect to inspect it before retrying.');
      throw error;
    } finally {
      if (timer !== null) clearTimeout(timer);
    }
  }

  async restart() {
    if (this.restarting) throw new Error('Restart is already in progress');
    this.restarting = true;
    // Restarting ends the screen session and rosbridge, so reattach both afterwards.
    const telemetry = drones[this.id];
    const socket = this.socket;
    this.socket = null;
    if (socket) socket.close();
    telemetry.disconnect('Restarting…');
    // Allow for the FC reboot, gyro check, full node startup and localization.
    const deadline = telemetry.waitUntil = Date.now() + 255000;
    let result;
    try {
      result = await this.post('/api/nodes', { action: 'restart' }, 240000);
    } catch (error) {
      if (telemetry.waitUntil === deadline) telemetry.failConnection('Nodes failed to restart');
      throw error;
    } finally {
      this.restarting = false;
    }
    this.nodes = true;
    this.scripts = null;
    if (this.enabled && telemetry.waitUntil === deadline) {
      if (!this.socket) this.attach();
      telemetry.connect(true);
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
    this.args = document.getElementById('routine-args');
    this.button = document.getElementById('routine-execute');
    this.restartButton = document.getElementById('nodes-restart');
    this.terminals = ['drone1', 'drone2'].map(id => new RoutineTerminal(id, () => this.render()));
    this.button.addEventListener('click', () => this.execute());
    this.restartButton.addEventListener('click', () => this.restart());
    this.select.addEventListener('change', () => this.render());
    // The Connect and Disconnect buttons (main.js) also open and close the terminals.
    const terminal = id => this.terminals.find(candidate => candidate.id === id);
    document.addEventListener('drone-connect', event => terminal(event.detail).connectDrone());
    document.addEventListener('drone-disconnect', event => terminal(event.detail).setEnabled(false));
    document.addEventListener('drone-connect-failed', event => {
      const pane = terminal(event.detail.id);
      pane.setEnabled(false);
      pane.report(event.detail.message, 'error');
      this.render();
    });
    setInterval(() => this.terminals.forEach(terminal => terminal.poll()), 2000);
    this.render();
  }

  get active() {
    return this.terminals.filter(terminal => terminal.enabled);
  }

  render() {
    const targets = this.active;
    const lists = targets.filter(terminal => terminal.scripts).map(terminal => terminal.scripts);
    // Offer only scripts available on every active drone.
    const scripts = lists.length ? lists[0].filter(name => lists.every(list => list.includes(name))) : [];
    const selected = this.select.value;
    if (scripts.join() !== Array.from(this.select.options, option => option.value).filter(Boolean).join()) {
      const options = scripts.map(name => new Option(name, name));
      options.unshift(new Option(scripts.length ? 'Select routine' : 'Connect a drone', ''));
      this.select.replaceChildren(...options);
      this.select.value = scripts.includes(selected) ? selected : '';
    }
    this.button.disabled = this.busy || !this.select.value || !targets.length || !targets.every(terminal => terminal.ready);
    this.restartButton.disabled = this.busy || !targets.length ||
      targets.some(terminal => terminal.running);
  }

  async run(label, targets, action) {
    this.busy = true;
    targets.forEach(terminal => terminal.report(label + '…', 'progress'));
    this.render();
    const results = await Promise.allSettled(targets.map(action));
    this.busy = false;
    results.forEach((result, index) => {
      const succeeded = result.status === 'fulfilled';
      targets[index].report(succeeded ? result.value : 'Failed: ' + result.reason.message,
        succeeded ? 'success' : 'error');
    });
    this.render();
    return results;
  }

  async execute() {
    const script = this.select.value;
    const args = this.args.value.trim();
    const targets = this.active;
    if (this.busy || !script || !targets.length || !targets.every(terminal => terminal.ready)) return;
    awaitSyncStart(targets.map(terminal => terminal.id));
    const results = await this.run('Starting ' + script, targets, terminal => terminal.execute(script, args));
    if (results.every(result => result.status === 'fulfilled')) confirmSyncStart();
    else awaitSyncStart(null);
  }

  restart() {
    const targets = this.active;
    const names = targets.map(terminal => terminal.id).join(' and ');
    if (this.busy || !targets.length || targets.some(terminal => terminal.running)) return;
    if (!confirmRestart(names)) return;
    return this.restartTargets(targets.map(terminal => terminal.id));
  }

  restartTargets(ids) {
    const targets = ids.map(id => this.terminals.find(terminal => terminal.id === id));
    if (this.busy || !targets.length || targets.some(terminal =>
      !terminal || terminal.running)) {
      throw new Error('Restart could not start. Check connections and stop running routines, then use Restart.');
    }
    return this.run('Restarting', targets, async terminal => terminal.enabled ? terminal.restart()
      : (await terminal.post('/api/nodes', { action: 'restart' }, 240000)).output);
  }
}

document.addEventListener('DOMContentLoaded', () => { routineRunner = new RoutineRunner(); });
