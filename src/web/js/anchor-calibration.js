'use strict';

const anchorNames = Object.freeze({ '4D12': 'Tag 6', '980C': 'Tag 5', '9781': 'Tag 4', 'CA83': 'Tag 3' });
function anchorLabel(id) {
  const hardwareId = id.toUpperCase();
  return anchorNames[hardwareId] ? anchorNames[hardwareId] + ' (' + hardwareId + ')' : hardwareId;
}

class AnchorCalibration {
  constructor() {
    this.state = null;
    this.receivedAt = null;
    this.boardSignature = null;
    this.pendingAt = null;
    this.polling = false;
    ['scan', 'measure', 'apply', 'cancel'].forEach(action => {
      document.getElementById('calibration-' + action).addEventListener('click', () => this.request(action));
    });
    for (let i = 0; i < 4; i++) {
      document.getElementById('calibration-anchor-' + i).addEventListener('input', () => this.refresh());
    }
    this.refresh();
    this.poll();
    setInterval(() => this.poll(), 500);
  }

  update(state) {
    if (!state || typeof state.stage !== 'string' || typeof state.message !== 'string' ||
        !Array.isArray(state.boards)) return;
    this.state = state;
    this.receivedAt = Date.now();
    this.pendingAt = null;
    this.refresh();
    if (fleetMap) fleetMap.refreshAnchors();
  }

  async poll() {
    if (this.polling) return;
    this.polling = true;
    try {
      const response = await fetch('/api/calibration', {
        method: 'POST', cache: 'no-store',
        headers: { 'Content-Type': 'application/json', 'X-Calibration-Request': '1' },
        body: JSON.stringify({ action: 'status', bridges: this.selection() }),
        signal: AbortSignal.timeout(2000)
      });
      if (!response.ok) throw new Error('Calibration server unavailable');
      this.update(await response.json());
    } catch (error) {
      this.refresh();
    } finally {
      this.polling = false;
    }
  }

  updateBoards() {
    if (this.state) {
      const signature = JSON.stringify(this.state.boards);
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
            option.textContent = anchorLabel(board.id) + (!board.active ? ' · Unavailable' : '');
            option.disabled = !board.active;
            select.appendChild(option);
          });
          select.value = this.state.boards.some(b => b.ble === previous) ? previous : '';
        }
      }
    }
  }

  fresh() {
    return this.receivedAt !== null && Date.now() - this.receivedAt <= 1500;
  }

  selection() {
    return Object.values(drones).filter(drone => drone.connected)
      .map(drone => 'ws://' + drone.host + ':9090');
  }

  matchesSelection() {
    return !!(this.state && Array.isArray(this.state.selection) &&
      JSON.stringify(this.state.selection) === JSON.stringify(this.selection()));
  }

  grounded() {
    const connected = Object.values(drones).filter(drone => drone.connected);
    return this.fresh() && this.matchesSelection() && this.state.grounded === true && connected.length > 0 && connected.every(drone =>
      drone.field('flightMode').textContent === 'DISARMED' && drone.modeReceivedAt !== null &&
      Date.now() - drone.modeReceivedAt <= (this.state.flight_state_timeout_ms || 500));
  }

  busy() {
    return (this.pendingAt !== null && Date.now() - this.pendingAt <= 3000) ||
      (this.state && this.state.busy);
  }

  matchesPreview() {
    return !!(this.state && this.state.preview && this.state.preview.positions.every((p, i) =>
      document.getElementById('calibration-anchor-' + i).value === p.ble && p.z === 0));
  }

  async request(action) {
    this.refresh();
    if (!this.fresh() || (action !== 'cancel' && (!this.grounded() || this.busy()))) return;
    const request = { action, bridges: this.selection() };
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
    this.pendingAt = Date.now();
    this.refresh();
    try {
      const response = await fetch('/api/calibration', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Calibration-Request': '1' },
        body: JSON.stringify(request), signal: AbortSignal.timeout(3000)
      });
      if (!response.ok) throw new Error((await response.json()).error || 'Calibration request failed');
      await this.poll();
    } catch (error) {
      this.pendingAt = null;
      document.getElementById('calibration-status').textContent = error.message;
    }
  }

  refresh() {
    this.updateBoards();
    const fresh = this.fresh(), grounded = this.grounded(), busy = this.busy(), state = this.state;
    document.getElementById('calibration-scan').disabled = !grounded || busy || (state.reserved && !state.dirty);
    document.getElementById('calibration-measure').disabled = !grounded || busy || state.boards.length < 4;
    document.getElementById('calibration-apply').disabled = !grounded || busy || state.stage !== 'preview' || !state.token || !this.matchesPreview();
    document.getElementById('calibration-cancel').disabled = !fresh || !(busy || state.reserved);
    for (let i = 0; i < 4; i++) {
      document.getElementById('calibration-anchor-' + i).disabled = !grounded || busy;
    }
    document.getElementById('calibration-status').textContent = !fresh
      ? (state && state.reserved ? 'Reconnect the laptop calibration server to finish or cancel calibration.'
        : 'Open the dashboard through the laptop server: http://localhost:8000.')
      : !this.matchesSelection() ? 'Updating connected drones...'
        : !grounded ? (state.ground_reason || 'Waiting for drones to be ready.')
        : state.message;
    const preview = document.getElementById('calibration-preview');
    preview.hidden = !fresh || !state.preview;
    const corners = ['Top Left', 'Top Right', 'Bottom Left', 'Bottom Right'];
    // Present rows from top to bottom; the solver retains its counterclockwise order.
    preview.textContent = fresh && state.preview ? [3, 2, 0, 1]
      .map(index => state.preview.positions[index]).filter(Boolean).map((p, i) =>
        'Corner ' + (i + 1) + ' (' + corners[i] + ') · ' + anchorLabel(p.id) +
        ': X ' + p.x.toFixed(2) + ', Y ' + p.y.toFixed(2) + ' m').join('\n') : '';
  }
}
