'use strict';

class AnchorCalibration {
  constructor() {
    this.state = null;
    this.receivedAt = null;
    this.boardSignature = null;
    this.pendingAt = null;
    this.reports = {};
    this.droneId = null;
    ['scan', 'measure', 'apply', 'cancel'].forEach(action => {
      document.getElementById('calibration-' + action).addEventListener('click', () => this.request(action));
    });
    for (let i = 0; i < 4; i++) {
      document.getElementById('calibration-anchor-' + i).addEventListener('input', () => this.refresh());
    }
    this.refresh();
  }

  update(droneId, message) {
    try {
      const state = JSON.parse(message.data);
      if (!state || typeof state.stage !== 'string' || typeof state.message !== 'string' ||
          !Array.isArray(state.boards)) return;
      this.reports[droneId] = { state, receivedAt: Date.now() };
      if (this.droneId === droneId) this.pendingAt = null;
      this.refresh();
      if (fleetMap) fleetMap.refreshAnchors();
    } catch (error) { console.error('Invalid anchor calibration status', error); }
  }

  reportFresh(id) {
    const report = this.reports[id];
    return !!(drones[id] && drones[id].connected && report && Date.now() - report.receivedAt <= 1500);
  }

  selectCoordinator() {
    const active = state => state && (state.reserved || ['scanning', 'measuring', 'saving', 'verifying'].includes(state.stage));
    // Keep requests, previews and uncertain writes on the same service, even if it disconnects.
    const pinned = active(this.state) || (this.pendingAt !== null && Date.now() - this.pendingAt <= 3000);
    if (!pinned) {
      const available = Object.keys(drones).filter(id => this.reportFresh(id));
      const working = available.find(id => active(this.reports[id].state));
      const current = available.includes(this.droneId) && this.reports[this.droneId].state.grounded;
      const chosen = working || (current && this.droneId) ||
        available.find(id => this.reports[id].state.grounded) || available[0] || null;
      if (chosen !== this.droneId) {
        this.droneId = chosen;
        this.pendingAt = this.boardSignature = null;
      }
    }
    const report = this.reports[this.droneId];
    this.state = report ? report.state : null;
    this.receivedAt = report ? report.receivedAt : null;
    if (this.state) {
      const signature = this.droneId + JSON.stringify(this.state.boards);
      if (signature !== this.boardSignature) {
        this.boardSignature = signature;
        for (let i = 0; i < 4; i++) {
          const select = document.getElementById('calibration-anchor-' + i);
          const previous = select.value;
          select.replaceChildren();
          const empty = document.createElement('option');
          empty.value = ''; empty.textContent = 'Select anchor'; select.appendChild(empty);
          this.state.boards.forEach(board => {
            const option = document.createElement('option');
            option.value = board.ble;
            option.textContent = board.id + (board.initiator ? ' · Initiator' : '') +
              (!board.active ? ' · UWB inactive' : '');
            option.disabled = !board.active;
            select.appendChild(option);
          });
          select.value = this.state.boards.some(b => b.ble === previous) ? previous : '';
        }
      }
    }
  }

  fresh() {
    return this.reportFresh(this.droneId);
  }

  active() {
    return Object.values(this.reports).some(report => report.state.reserved);
  }

  grounded() {
    const connected = Object.values(drones).filter(drone => drone.connected);
    return this.fresh() && this.state.grounded === true && connected.length > 0 && connected.every(drone =>
      drone.field('flightMode').textContent === 'DISARMED' && drone.modeReceivedAt !== null &&
      Date.now() - drone.modeReceivedAt <= 500);
  }

  busy() {
    return (this.pendingAt !== null && Date.now() - this.pendingAt <= 3000) ||
      (this.state && ['scanning', 'measuring', 'saving', 'verifying'].includes(this.state.stage));
  }

  matchesPreview() {
    return !!(this.state && this.state.preview && this.state.preview.positions.every((p, i) =>
      document.getElementById('calibration-anchor-' + i).value === p.ble && p.z === 0));
  }

  request(action) {
    this.refresh();
    if (!this.fresh() || (action !== 'cancel' && (!this.grounded() || this.busy()))) return;
    const request = { action };
    if (action === 'measure') {
      request.anchors = Array.from({ length: 4 }, (_, i) => ({
        ble: document.getElementById('calibration-anchor-' + i).value
      }));
      if (new Set(request.anchors.map(a => a.ble)).size !== 4 ||
          request.anchors.some(a => !a.ble)) {
        document.getElementById('calibration-status').textContent = 'Select four different anchors.';
        return;
      }
    }
    if (action === 'apply') {
      if (this.state.stage !== 'preview' || !this.state.token || !this.matchesPreview()) return;
      request.token = this.state.token;
    }
    if (drones[this.droneId].publish('calibrationRequest', { data: JSON.stringify(request) })) {
      this.pendingAt = Date.now();
      this.refresh();
    }
  }

  refresh() {
    this.selectCoordinator();
    const fresh = this.fresh(), grounded = this.grounded(), busy = this.busy(), state = this.state;
    document.getElementById('calibration-scan').disabled = !grounded || busy || (state.reserved && !state.dirty);
    document.getElementById('calibration-measure').disabled = !grounded || busy || state.boards.length < 4;
    document.getElementById('calibration-apply').disabled = !grounded || busy || state.stage !== 'preview' || !state.token || !this.matchesPreview();
    document.getElementById('calibration-cancel').disabled = !fresh || !(busy || state.reserved);
    for (let i = 0; i < 4; i++) {
      document.getElementById('calibration-anchor-' + i).disabled = !grounded || busy;
    }
    document.getElementById('calibration-status').textContent = !fresh
      ? (state && state.reserved ? 'Reconnect Drone ' + this.droneId.slice(-1) + ' to finish or cancel calibration.'
        : 'Connect a drone and start its calibration service.')
      : !grounded ? (state.ground_reason || 'Disarm connected drones; fresh flight state is required.')
        : 'Drone ' + this.droneId.slice(-1) + ': ' + state.message;
    const preview = document.getElementById('calibration-preview');
    preview.hidden = !fresh || !state.preview;
    preview.textContent = fresh && state.preview ? state.preview.positions.map(p =>
      p.id + ': X ' + p.x.toFixed(2) + ', Y ' + p.y.toFixed(2) + ', Z ' + p.z.toFixed(2) + ' m').join('\n') +
      '\nRange fit RMS: ' + state.preview.rms.toFixed(3) + ' m' : '';
  }
}
