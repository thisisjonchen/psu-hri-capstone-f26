'use strict';

class AnchorCalibration {
  constructor() {
    this.state = null;
    this.receivedAt = null;
    this.boardSignature = null;
    this.pendingAt = null;
    ['scan', 'measure', 'apply', 'cancel'].forEach(action => {
      document.getElementById('calibration-' + action).addEventListener('click', () => this.request(action));
    });
    for (let i = 0; i < 4; i++) {
      document.getElementById('calibration-anchor-' + i).addEventListener('input', () => this.refresh());
    }
    this.refresh();
  }

  update(message) {
    try {
      const state = JSON.parse(message.data);
      if (!state || typeof state.stage !== 'string' || typeof state.message !== 'string' ||
          !Array.isArray(state.boards)) return;
      this.state = state;
      this.receivedAt = Date.now();
      this.pendingAt = null;
      const signature = JSON.stringify(state.boards);
      if (signature !== this.boardSignature) {
        this.boardSignature = signature;
        for (let i = 0; i < 4; i++) {
          const select = document.getElementById('calibration-anchor-' + i);
          const previous = select.value;
          select.replaceChildren();
          const empty = document.createElement('option');
          empty.value = ''; empty.textContent = 'Select anchor'; select.appendChild(empty);
          state.boards.forEach(board => {
            const option = document.createElement('option');
            option.value = board.ble;
            option.textContent = board.id + (board.initiator ? ' · Initiator' : '') +
              (!board.active ? ' · UWB inactive' : '');
            option.disabled = !board.active;
            select.appendChild(option);
          });
          select.value = state.boards.some(b => b.ble === previous) ? previous : '';
        }
      }
      this.refresh();
      if (fleetMap) fleetMap.refreshAnchors();
    } catch (error) { console.error('Invalid anchor calibration status', error); }
  }

  fresh() {
    return !!(drones.drone1.connected && this.state && this.receivedAt !== null &&
      Date.now() - this.receivedAt <= 1500);
  }

  grounded() {
    return this.fresh() && this.state.grounded === true && Object.values(drones).every(drone =>
      drone.connected && drone.field('flightMode').textContent === 'DISARMED' &&
      drone.modeReceivedAt !== null && Date.now() - drone.modeReceivedAt <= 500);
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
    if (drones.drone1.publish('calibrationRequest', { data: JSON.stringify(request) })) {
      this.pendingAt = Date.now();
      this.refresh();
    }
  }

  refresh() {
    const fresh = this.fresh(), grounded = this.grounded(), busy = this.busy(), state = this.state;
    document.getElementById('calibration-scan').disabled = !grounded || busy || (state.reserved && !state.dirty);
    document.getElementById('calibration-measure').disabled = !grounded || busy || state.boards.length < 4;
    document.getElementById('calibration-apply').disabled = !grounded || busy || state.stage !== 'preview' || !state.token || !this.matchesPreview();
    document.getElementById('calibration-cancel').disabled = !fresh || !(busy || state.reserved);
    for (let i = 0; i < 4; i++) {
      document.getElementById('calibration-anchor-' + i).disabled = !grounded || busy;
    }
    document.getElementById('calibration-status').textContent = !fresh
      ? 'Connect both drones and start the calibration service on Drone 1.'
      : !grounded ? (state.ground_reason || 'Disarm both drones; fresh flight state is required.') : state.message;
    const preview = document.getElementById('calibration-preview');
    preview.hidden = !fresh || !state.preview;
    preview.textContent = fresh && state.preview ? state.preview.positions.map(p =>
      p.id + ': X ' + p.x.toFixed(2) + ', Y ' + p.y.toFixed(2) + ', Z ' + p.z.toFixed(2) + ' m').join('\n') +
      '\nRange fit RMS: ' + state.preview.rms.toFixed(3) + ' m' : '';
  }
}
