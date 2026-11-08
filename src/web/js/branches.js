'use strict';

class DroneBranches {
  constructor() {
    this.panel = document.getElementById('drone-branches');
    this.select = document.getElementById('branch-select');
    this.apply = document.getElementById('branch-apply');
    this.clear = document.getElementById('branch-clear');
    this.message = document.getElementById('branch-message');
    this.rows = ['drone1', 'drone2'].map(id => document.getElementById(id + '-branch'));
    this.records = {};
    this.targets = [];
    this.busy = false;
    this.reloadPending = false;
    this.panel.addEventListener('toggle', () => { if (this.panel.open) return this.load(); });
    this.select.addEventListener('change', () => this.render());
    this.apply.addEventListener('click', () => this.switchSelected());
    this.clear.addEventListener('click', () => this.clearSelected());
    document.addEventListener('drone-connections-changed', () => {
      this.reloadPending = true;
      return this.load();
    });
    this.render();
  }

  get connected() {
    return ['drone1', 'drone2'].filter(id => drones[id]?.connected);
  }

  get targetNames() {
    return this.targets.map(id => 'Drone ' + id.slice(-1)).join(' and ');
  }

  get ready() {
    return this.targets.length > 0 && this.targets.join() === this.connected.join() &&
      this.targets.every(id => Array.isArray(this.records[id]?.branches) && !this.records[id].error);
  }

  render() {
    this.rows.forEach((row, index) => {
      const id = 'drone' + (index + 1);
      const record = this.records[id];
      row.textContent = record?.error || (!drones[id]?.connected ? 'Not connected' : record?.branch
        ? record.branch + ' · ' + record.revision + (record.dirty ? ' · local changes' : '') : 'Not loaded');
      row.dataset.error = record?.error ? 'true' : 'false';
    });
    this.select.disabled = this.busy || !this.ready;
    const dirty = this.targets.some(id => this.records[id]?.dirty);
    this.apply.disabled = this.busy || !this.ready || !this.select.value || dirty;
    this.clear.disabled = this.busy || !this.ready || !dirty;
  }

  async request(options = {}, timeout = 60000) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeout);
    try {
      const query = options.method === 'POST' ? '' : '?' + this.targets.map(id => 'drone=' + encodeURIComponent(id)).join('&');
      const response = await fetch('/api/branches' + query, { ...options, signal: controller.signal });
      const body = await response.json();
      if (!response.ok) throw new Error(body.error || response.statusText);
      return body;
    } catch (error) {
      if (controller.signal.aborted) throw new Error('Request timed out.');
      throw error;
    } finally {
      clearTimeout(timer);
    }
  }

  async load() {
    if (this.busy) return;
    this.reloadPending = false;
    this.targets = this.connected;
    this.records = {};
    if (!this.targets.length) {
      this.select.replaceChildren(new Option('Select a branch', ''));
      this.message.textContent = 'Connect a drone.';
      this.render();
      return;
    }
    this.busy = true;
    this.message.textContent = 'Loading branches…';
    this.render();
    try {
      const previous = this.select.value;
      this.records = (await this.request()).drones;
      if (this.reloadPending) return;
      const records = this.targets.map(id => this.records[id]);
      const branches = this.ready ? records[0].branches.filter(name => records.every(record => record.branches.includes(name))) : [];
      this.select.replaceChildren(new Option('Select a branch', ''), ...branches.map(name => new Option(name, name)));
      const current = records.every(record => record?.branch === records[0]?.branch) ? records[0]?.branch : '';
      this.select.value = branches.includes(previous) ? previous : branches.includes(current) ? current : '';
      this.message.textContent = !this.ready ? 'Could not load branches. Reopen Branches to retry.'
        : records.some(record => record.dirty) ? 'Save or stash local changes, or clear them to switch.'
        : branches.length ? '' : 'No branches available.';
    } catch (error) {
      this.records = {};
      this.message.textContent = error.message + ' Reopen Branches to retry.';
    } finally {
      this.busy = false;
      if (this.reloadPending) return this.load();
      this.render();
    }
  }

  async switchSelected() {
    this.render();
    if (this.apply.disabled) return;
    if (!confirmRestart(this.targetNames + ' to ' + this.select.value,
      'Switch and restart')) return;
    return this.change('switch', this.select.value);
  }

  async clearSelected() {
    this.render();
    if (this.clear.disabled || !window.confirm('Clear local changes on ' + this.targetNames + '?\n\n' +
      'This permanently discards uncommitted changes and untracked files. ' +
      'Commits and ignored files are kept.')) return;
    return this.change('clear');
  }

  async change(action, branch) {
    if (routineRunner.busy) {
      this.message.textContent = 'Wait for the current action to finish.';
      return;
    }
    const clearing = action === 'clear';
    this.busy = true;
    routineRunner.busy = true;
    routineRunner.render();
    this.message.textContent = clearing ? 'Clearing local changes…' : 'Updating and building…';
    this.render();
    try {
      const result = await this.request({ method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Routine-Request': '1' },
        body: JSON.stringify({ action, branch, drones: this.targets }) }, clearing ? 180000 : 900000);
      Object.entries(result.drones).forEach(([id, record]) => {
        if (record.error) this.records[id] = record;
        else if (result.phase !== 'prepare') this.records[id] = { ...this.records[id], ...record };
      });
      this.message.textContent = result.phase === 'prepare' ? 'No files changed. Resolve the issue above, then reopen Branches.'
        : Object.values(result.drones).some(record => record.error) ? 'Action incomplete. Reopen Branches to verify.'
        : clearing ? 'Local changes cleared on ' + this.targetNames + '.'
        : this.targetNames + ' updated and built.';
      if (!clearing && result.phase === 'build' &&
          this.targets.every(id => result.drones[id] && !result.drones[id].error)) {
        this.message.textContent = 'Restarting…';
        routineRunner.busy = false;
        const restarted = await routineRunner.restartTargets(this.targets);
        restarted.forEach((result, index) => {
          if (result.status === 'rejected') this.records[this.targets[index]].error = result.reason.message;
        });
        this.message.textContent = restarted.some(result => result.status === 'rejected')
          ? 'Restart incomplete. Check terminal output.'
          : this.targetNames + ' updated and restarted.';
      }
    } catch (error) {
      this.records = {};
      this.message.textContent = error.message + ' Reopen Branches to verify.';
    } finally {
      routineRunner.busy = false;
      routineRunner.render();
      this.busy = false;
      if (this.reloadPending) {
        // Refresh connections after the action without hiding its result or errors.
        const message = this.message.textContent;
        const errors = Object.entries(this.records).filter(([, record]) => record.error);
        await this.load();
        errors.forEach(([id, record]) => {
          this.records[id] = { ...this.records[id], error: record.error };
        });
        this.message.textContent = message;
      }
      this.render();
    }
  }
}

document.addEventListener('DOMContentLoaded', () => { new DroneBranches(); });
