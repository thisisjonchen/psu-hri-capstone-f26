'use strict';

class DroneBranches {
  constructor() {
    this.panel = document.getElementById('drone-branches');
    this.select = document.getElementById('branch-select');
    this.apply = document.getElementById('branch-apply');
    this.refresh = document.getElementById('branch-refresh');
    this.clear = document.getElementById('branch-clear');
    this.message = document.getElementById('branch-message');
    this.rows = ['drone1', 'drone2'].map(id => document.getElementById(id + '-branch'));
    this.records = {};
    this.busy = false;
    this.panel.addEventListener('toggle', () => { if (this.panel.open && !this.busy) this.load(); });
    this.refresh.addEventListener('click', () => this.load());
    this.select.addEventListener('change', () => this.render());
    this.apply.addEventListener('click', () => this.switchBoth());
    this.clear.addEventListener('click', () => this.clearBoth());
    this.render();
  }

  get ready() {
    return ['drone1', 'drone2'].every(id => this.records[id]?.branches && !this.records[id].error);
  }

  render() {
    this.rows.forEach((row, index) => {
      const record = this.records['drone' + (index + 1)];
      row.textContent = record?.error || (record?.branch
        ? record.branch + ' · ' + record.revision + (record.dirty ? ' · local changes' : '') : 'Not loaded');
      row.dataset.error = record?.error ? 'true' : 'false';
    });
    this.select.disabled = this.busy || !this.ready;
    this.refresh.disabled = this.busy;
    const dirty = Object.values(this.records).some(record => record.dirty);
    this.apply.disabled = this.busy || !this.ready || !this.select.value || dirty;
    this.clear.disabled = this.busy || !this.ready || !dirty;
  }

  async request(options = {}, timeout = 60000) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeout);
    try {
      const response = await fetch('/api/branches', { ...options, signal: controller.signal });
      const body = await response.json();
      if (!response.ok) throw new Error(body.error || response.statusText);
      return body;
    } catch (error) {
      if (controller.signal.aborted) throw new Error('Request timed out. Refresh to verify both drones before retrying.');
      throw error;
    } finally {
      clearTimeout(timer);
    }
  }

  async load() {
    if (this.busy) return;
    this.busy = true;
    this.message.textContent = 'Loading branches…';
    this.render();
    try {
      const previous = this.select.value;
      this.records = (await this.request()).drones;
      const branches = this.ready ? this.records.drone1.branches.filter(name => this.records.drone2.branches.includes(name)) : [];
      this.select.replaceChildren(new Option('Select a branch', ''), ...branches.map(name => new Option(name, name)));
      const current = this.records.drone1?.branch === this.records.drone2?.branch ? this.records.drone1?.branch : '';
      this.select.value = branches.includes(previous) ? previous : branches.includes(current) ? current : '';
      this.message.textContent = !this.ready ? 'Could not load both drones. Check their connections, then Refresh.'
        : Object.values(this.records).some(record => record.dirty) ? 'Save or stash local changes, or clear them to switch.'
        : branches.length ? 'Keep both drones connected and disarmed while switching.' : 'No shared branches found on origin.';
    } catch (error) {
      this.records = {};
      this.message.textContent = error.message;
    } finally {
      this.busy = false;
      this.render();
    }
  }

  async switchBoth() {
    if (this.apply.disabled) return;
    return this.change('switch', this.select.value);
  }

  async clearBoth() {
    if (this.clear.disabled || !window.confirm('Clear local changes on Drone 1 and Drone 2?\n\n' +
      'This permanently discards all uncommitted changes and deletes untracked files in both repositories. ' +
      'Commits and ignored files are kept.')) return;
    return this.change('clear');
  }

  async change(action, branch) {
    const clearing = action === 'clear';
    this.busy = true;
    this.message.textContent = clearing ? 'Clearing local changes on both drones…' : 'Switching both drones to ' + branch + '…';
    this.render();
    try {
      const result = await this.request({ method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Routine-Request': '1' },
        body: JSON.stringify({ action, branch }) }, 180000);
      Object.entries(result.drones).forEach(([id, record]) => {
        if (record.error) this.records[id] = record;
        else if (result.phase !== 'prepare') this.records[id] = { ...this.records[id], ...record };
      });
      this.message.textContent = result.phase === 'prepare' ? 'No files changed. Resolve the issue above, then Refresh.'
        : Object.values(result.drones).some(record => record.error) ? 'Action incomplete. Refresh to verify both drones.'
        : clearing ? 'Local changes cleared on both drones.'
        : 'Both drones updated. Restart to load changes; rebuild first if native code changed.';
    } catch (error) {
      this.records = {};
      this.message.textContent = error.message + ' Refresh to verify both drones.';
    } finally {
      this.busy = false;
      this.render();
    }
  }
}

document.addEventListener('DOMContentLoaded', () => { new DroneBranches(); });
