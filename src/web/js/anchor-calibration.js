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
    this.pendingAction = null;
    this.requestEpoch = 0;
    this.requestError = null;
    this.awaitingStatus = false;
    this.saveToken = null;
    this.editorToken = null;
    this.previewSignature = null;
    this.closeTimer = null;
    document.getElementById('calibration-toggle').addEventListener('click', () => {
      this.setOpen(document.getElementById('calibration-panel').hidden);
    });
    this.remembered = [];
    try {
      const saved = JSON.parse(localStorage.getItem('uwb-anchor-corners'));
      if (Array.isArray(saved) && saved.length === 4 && saved.every(value => typeof value === 'string')) {
        this.remembered = saved;
      }
    } catch (_) { /* Calibration also works when browser storage is unavailable. */ }
    this.polling = false;
    ['scan', 'measure', 'edit', 'apply', 'cancel'].forEach(action => {
      document.getElementById('calibration-' + action).addEventListener('click', () => this.request(action));
    });
    for (let i = 0; i < 4; i++) {
      document.getElementById('calibration-anchor-' + i).addEventListener('input', () => {
        this.remembered = this.corners();
        try { localStorage.setItem('uwb-anchor-corners', JSON.stringify(this.remembered)); } catch (_) {}
        this.requestError = null;
        this.refresh();
      });
      ['x', 'y'].forEach(axis => {
        document.getElementById('calibration-' + axis + '-' + i).addEventListener('input', () => this.refresh());
      });
    }
    this.refresh();
    this.poll();
    setInterval(() => this.poll(), 500);
  }

  update(state) {
    if (!state || typeof state.stage !== 'string' || typeof state.message !== 'string' ||
        !Array.isArray(state.boards)) return;
    if (state.busy || (this.state && state.stage !== this.state.stage)) this.requestError = null;
    this.state = state;
    this.awaitingStatus = false;
    this.receivedAt = Date.now();
    this.refresh();
    if (fleetMap) fleetMap.updateCalibratedLayout(state.layout);
    if (this.saveToken && state.stage === 'complete' && state.layout &&
        state.layout.verified && state.layout.revision === this.saveToken) {
      this.saveToken = null;
      if (!document.getElementById('calibration-panel').hidden) {
        document.getElementById('calibration-form').hidden = true;
        document.getElementById('calibration-success').hidden = false;
        this.closeTimer = setTimeout(() => {
          this.setOpen(false);
          document.getElementById('calibration-toggle').focus();
        }, 1200);
      }
    }
    if (state.stage === 'error') this.saveToken = null;
  }

  setOpen(open) {
    clearTimeout(this.closeTimer);
    this.closeTimer = null;
    document.getElementById('calibration-panel').hidden = !open;
    document.getElementById('calibration-toggle').setAttribute('aria-expanded', String(open));
    document.getElementById('map-frame').classList.toggle('calibrating', open);
    document.getElementById('calibration-form').hidden = false;
    document.getElementById('calibration-success').hidden = true;
  }

  async poll() {
    if (this.polling || this.pendingAction) return;
    this.polling = true;
    const epoch = this.requestEpoch;
    try {
      const response = await fetch('/api/calibration', {
        method: 'POST', cache: 'no-store',
        headers: { 'Content-Type': 'application/json', 'X-Calibration-Request': '1' },
        body: JSON.stringify({ action: 'status' }),
        signal: AbortSignal.timeout(2000)
      });
      if (!response.ok) throw new Error('Calibration server unavailable');
      const state = await response.json();
      // A status request started before an action must not unlock its controls.
      if (epoch === this.requestEpoch) this.update(state);
    } catch (error) {
      if (epoch === this.requestEpoch) this.awaitingStatus = false;
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
          const saved = this.state.layout && this.state.layout.positions[i];
          const previous = select.value || this.remembered[i] || (saved && saved.ble) || '';
          select.replaceChildren();
          const empty = document.createElement('option');
          empty.value = ''; empty.textContent = 'Select anchor'; select.appendChild(empty);
          this.state.boards.forEach(board => {
            const option = document.createElement('option');
            option.value = board.ble;
            option.textContent = anchorLabel(board.id) +
              (!board.active ? ' · Unavailable' : '');
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

  busy() {
    return !!this.pendingAction || (this.state && this.state.busy);
  }

  corners() {
    return Array.from({ length: 4 }, (_, i) => document.getElementById('calibration-anchor-' + i).value);
  }

  selectionIssue() {
    if (!this.state || this.state.boards.filter(board => board.active).length < 4) {
      return 'Scan for at least four anchors with active UWB.';
    }
    const corners = this.corners();
    if (corners.some(value => !value)) return 'Select an anchor for each corner.';
    if (new Set(corners).size !== 4) return 'Select four different anchors.';
    const boards = corners.map(ble => this.state.boards.find(board => board.ble === ble));
    if (boards.some(board => !board || !board.active)) return 'An assigned anchor is unavailable; scan again or replace it.';
    if (boards.filter(board => board.initiator).length !== 1) return 'Select exactly one initiator and three other anchors.';
    const pans = new Set(boards.filter(board => board.pan != null).map(board => board.pan));
    if (pans.size > 1) return 'Select anchors on the same UWB network (PAN).';
    return '';
  }

  matchesPreview() {
    return !!(this.state && this.state.preview && this.state.preview.positions.length === 4 && this.state.preview.positions.every((p, i) =>
      document.getElementById('calibration-anchor-' + i).value === p.ble && p.z === 0));
  }

  editedPositions() {
    if (!this.state || !this.state.preview) return null;
    const positions = this.state.preview.positions.map((p, i) => {
      const values = ['x', 'y'].map(axis => document.getElementById('calibration-' + axis + '-' + i).value);
      return { id: p.id, x: values[0].trim() ? Number(values[0]) : NaN,
        y: values[1].trim() ? Number(values[1]) : NaN };
    });
    return positions.every(p => [p.x, p.y].every(v => Number.isFinite(v) && Math.abs(v) <= 1000))
      ? positions : null;
  }

  drawPositionPreview(positions) {
    const signature = JSON.stringify(positions);
    if (signature === this.previewSignature) return;
    this.previewSignature = signature;
    const svg = document.getElementById('calibration-position-map');
    svg.replaceChildren();
    const draw = (tag, attributes, text) => {
      const node = document.createElementNS('http://www.w3.org/2000/svg', tag);
      Object.entries(attributes).forEach(([key, value]) => node.setAttribute(key, value));
      if (text !== undefined) node.textContent = text;
      svg.appendChild(node);
      return node;
    };
    if (!positions) {
      draw('text', { x: 180, y: 120, 'text-anchor': 'middle' }, 'Enter X and Y to preview');
      return;
    }
    const left = 48, top = 20, width = 288, height = 180;
    const xs = positions.map(p => p.x), ys = positions.map(p => p.y);
    const dx = Math.max(...xs) - Math.min(...xs), dy = Math.max(...ys) - Math.min(...ys);
    const cx = (Math.max(...xs) + Math.min(...xs)) / 2, cy = (Math.max(...ys) + Math.min(...ys)) / 2;
    const margin = Math.max(0.15, Math.max(dx, dy) * 0.2);
    const scale = Math.max((dx + 2 * margin) / width, (dy + 2 * margin) / height);
    const xMin = cx - width * scale / 2, yMin = cy - height * scale / 2;
    const x = value => left + (value - xMin) / scale;
    const y = value => top + height - (value - yMin) / scale;
    for (let i = 0; i <= 4; i++) {
      const px = left + width * i / 4, py = top + height - height * i / 4;
      draw('line', { class: 'preview-grid', x1: px, x2: px, y1: top, y2: top + height });
      draw('line', { class: 'preview-grid', x1: left, x2: left + width, y1: py, y2: py });
      draw('text', { x: px, y: top + height + 16, 'text-anchor': 'middle' }, Number((xMin + width * scale * i / 4).toFixed(2)));
      draw('text', { x: left - 8, y: py + 4, 'text-anchor': 'end' }, Number((yMin + height * scale * i / 4).toFixed(2)));
    }
    draw('text', { x: left + width / 2, y: 234, 'text-anchor': 'middle' }, 'X (m)');
    draw('text', { transform: 'translate(12 110) rotate(-90)', 'text-anchor': 'middle' }, 'Y (m)');
    draw('polygon', { class: 'preview-outline', points: positions.map(p => x(p.x) + ',' + y(p.y)).join(' ') });
    positions.forEach(p => {
      const dot = draw('circle', { cx: x(p.x), cy: y(p.y), r: 5, 'data-anchor': p.id });
      const title = document.createElementNS('http://www.w3.org/2000/svg', 'title');
      title.textContent = anchorLabel(p.id) + ': ' + p.x + ', ' + p.y + ' m';
      dot.appendChild(title);
      draw('text', { class: 'preview-label', x: x(p.x) + (p.x <= cx ? -8 : 8),
        y: y(p.y) + (p.y <= cy ? 16 : -10), 'text-anchor': p.x <= cx ? 'end' : 'start' },
      anchorNames[p.id.toUpperCase()] || p.id.toUpperCase());
    });
  }

  async request(action) {
    this.refresh();
    if (this.pendingAction || !this.fresh()) return;
    if (document.getElementById('calibration-' + action).disabled) return;
    const request = { action };
    if (action === 'measure') {
      request.anchors = this.corners().map(ble => ({ ble }));
    }
    if (action === 'apply') {
      if (this.state.stage !== 'preview' || !this.state.token || !this.matchesPreview()) return;
      request.token = this.state.token;
      request.positions = this.editedPositions();
      if (!request.positions) return;
      this.saveToken = request.token;
    }
    this.pendingAction = action;
    this.requestEpoch++;
    this.requestError = null;
    this.refresh();
    try {
      const response = await fetch('/api/calibration', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Calibration-Request': '1' },
        body: JSON.stringify(request), signal: AbortSignal.timeout(3000)
      });
      if (!response.ok) throw new Error((await response.json()).error || 'Calibration request failed');
    } catch (error) {
      this.requestError = error.name === 'TimeoutError'
        ? 'Request timed out; the operation may still be running.' : error.message;
    } finally {
      this.pendingAction = null;
      // Wait for status after acceptance or an uncertain timeout before enabling another action.
      this.receivedAt = null;
      this.awaitingStatus = true;
      this.refresh();
      this.poll();
    }
  }

  refresh() {
    this.updateBoards();
    const fresh = this.fresh(), busy = this.busy(), state = this.state;
    if (state && state.preview && state.token !== this.editorToken) {
      this.editorToken = state.token;
      const corners = ['Bottom left', 'Bottom right', 'Top right', 'Top left'];
      state.preview.positions.forEach((p, i) => {
        document.getElementById('calibration-label-' + i).textContent = corners[i];
        document.getElementById('calibration-anchor-' + i).value = p.ble;
        ['x', 'y'].forEach(axis => {
          document.getElementById('calibration-' + axis + '-' + i).value = p[axis].toFixed(3);
        });
      });
    } else if (!state || !state.preview) {
      this.editorToken = null;
    }
    document.getElementById('calibration-scan').disabled = !fresh || busy || !!state.preview;
    document.getElementById('calibration-measure').disabled = !fresh || busy || !!this.selectionIssue();
    document.getElementById('calibration-edit').disabled = !fresh || busy || !state.layout || state.dirty || !!state.preview;
    document.getElementById('calibration-apply').disabled = !fresh || busy || state.stage !== 'preview' || !state.token || !this.matchesPreview() || !this.editedPositions();
    document.getElementById('calibration-cancel').disabled = !fresh || !!this.pendingAction || !(busy || state.preview || state.dirty);
    for (let i = 0; i < 4; i++) {
      document.getElementById('calibration-anchor-' + i).disabled = !fresh || busy;
      ['x', 'y'].forEach(axis => {
        document.getElementById('calibration-' + axis + '-' + i).disabled = !fresh || busy;
      });
    }
    const labels = { scan: 'Scanning…', measure: 'Measuring…', edit: 'Preparing…', apply: 'Saving…', cancel: 'Cancelling…' };
    const stages = { scanning: 'scan', measuring: 'measure', saving: 'apply' };
    const active = this.pendingAction || (state && state.busy && stages[state.stage]);
    const defaults = { scan: 'Scan', measure: 'Measure', edit: 'Adjust', apply: 'Save', cancel: 'Cancel' };
    Object.keys(defaults).forEach(action => {
      document.getElementById('calibration-' + action).textContent = active === action ? labels[action] : defaults[action];
    });
    const selectionStatus = document.getElementById('calibration-selection');
    const issue = state && state.boards.filter(board => board.active).length >= 4 ? this.selectionIssue() : '';
    selectionStatus.hidden = !issue;
    selectionStatus.textContent = issue || '';
    const status = document.getElementById('calibration-status');
    status.textContent = this.requestError || (!fresh
      ? (this.awaitingStatus ? 'Checking status…'
        : state && (state.preview || state.dirty) ? 'Reconnect the calibration server to finish or cancel.'
        : 'Calibration server offline.')
      : this.pendingAction ? labels[this.pendingAction] : state.message);
    status.hidden = !status.textContent;
    const preview = document.getElementById('calibration-preview');
    preview.hidden = !fresh || !state.preview;
    this.drawPositionPreview(fresh && this.matchesPreview() ? this.editedPositions() : null);
  }
}
