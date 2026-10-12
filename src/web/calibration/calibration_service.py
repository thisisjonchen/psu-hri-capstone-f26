"""Ground-only anchor calibration coordinated by the laptop dashboard server."""

import copy
import json
import logging
import os
import tempfile
from pathlib import Path
import fcntl
import threading
import uuid

from calibration.anchor_calibration import solve_layout
from calibration.calibration_guard import BridgeGuard, CONTROL_STATE_TIMEOUT
from calibration.pans_ble import PansBLE
from time import monotonic


class CalibrationJournal:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock_file = open(str(self.path) + '.lock', 'a')
        try:
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock_file.close()
            raise ValueError('Another laptop calibration service is using this recovery file')
        self.values = json.loads(self.path.read_text()) if self.path.exists() else {}
        if not isinstance(self.values, dict):
            raise ValueError('Invalid calibration recovery file; restore it before starting')

    def save(self, values):
        descriptor, temporary = tempfile.mkstemp(dir=self.path.parent, prefix='.calibration-')
        try:
            with os.fdopen(descriptor, 'w') as stream:
                json.dump(values, stream, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            self.values = copy.deepcopy(values)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def close(self):
        self.lock_file.close()


class AnchorCalibration:
    def __init__(self, bridges, state_path):
        self.journal = CalibrationJournal(state_path)
        saved = self.journal.values
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.busy = False
        self.dirty = bool(saved.get('incomplete_save', False))
        self.reserved = bool(saved.get('participants'))
        self.preview = None
        self.preview_at = None
        self.boards = saved.get('recovery_boards', []) if self.dirty else []
        self.state = {'stage': 'idle', 'message': 'Connect and disarm at least one drone to scan for anchors.',
                      'boards': self.boards, 'preview': None, 'token': None}
        if self.dirty:
            self.state.update(stage='error', message='Previous save was incomplete. Measure and apply again to recover.')
        owner = saved.get('owner', str(uuid.uuid4()))
        self.guard = BridgeGuard(bridges, owner, claimed=self.reserved, participants=saved.get('participants', []))
        self.persist()
        self.ble = PansBLE()
        self.stopping = threading.Event()
        self.worker = None
        self.timer = threading.Thread(target=self.maintain, daemon=True)
        self.timer.start()

    def persist(self):
        self.journal.save({'owner': self.guard.owner, 'incomplete_save': self.dirty,
                           'recovery_boards': self.boards if self.dirty else [],
                           'participants': self.guard.participants or []})

    def maintain(self):
        while not self.stopping.wait(0.25):
            self.tick()

    def close(self):
        self.stopping.set()
        self.cancel.set()
        self.timer.join()
        if self.worker is not None:
            self.worker.join()
        self.guard.close()
        self.ble.close()
        self.journal.close()

    def update(self, stage, message, **values):
        with self.lock:
            self.state.update(stage=stage, message=message, **values)

    def check(self, owned=True):
        if self.cancel.is_set() or self.stopping.is_set():
            raise ValueError('Calibration cancelled')
        self.guard.check(owned=owned)

    def request(self, request):
        if not isinstance(request, dict) or request.get('action') not in ('scan', 'measure', 'apply', 'cancel'):
            raise ValueError('Unknown calibration action')
        action = request['action']
        with self.lock:
            if action != 'cancel':
                self.guard.select(request.get('bridges'))
            if action == 'cancel':
                self.cancel.set()
                if self.busy:
                    return
            elif self.busy:
                raise ValueError('Calibration is already busy')
            self.busy = True
            if action != 'cancel':
                self.cancel.clear()
            self.worker = threading.Thread(target=self.run, args=(request,), daemon=True)
            self.worker.start()

    def release(self):
        if self.reserved:
            self.guard.reserve(False)
            self.reserved = False
            self.persist()
        self.preview = None
        self.preview_at = None
        self.state.update(preview=None, token=None)

    def mark_write(self):
        self.dirty = True
        self.persist()

    def run(self, request):
        try:
            action = request['action']
            if action == 'cancel':
                self.release()
                self.update('idle', 'Calibration cancelled. The saved layout is unverified.' if self.dirty
                            else 'Calibration cancelled. No new coordinates saved.')
                return
            if action == 'measure' or (action == 'scan' and self.dirty):
                self.guard.prepare()
                self.persist()
            self.check(owned=False)
            ble = self.ble
            if action == 'scan':
                if self.reserved and not self.dirty:
                    raise ValueError('Cancel the current preview before scanning again')
                if self.dirty:
                    self.guard.reserve(True)
                self.update('scanning', 'Scanning Bluetooth anchors...', preview=None, token=None)
                self.boards = ble.call('scan', lambda: self.check(self.dirty))
                message = 'Found %d anchors. Select an anchor for each corner.' % len(self.boards)
                if ble.scan_warnings:
                    message += ' Unreadable boards: ' + '; '.join(ble.scan_warnings)
                self.update('idle', message, boards=self.boards)
            elif action == 'measure':
                choices = request.get('anchors', [])
                if len(choices) != 4 or len(set(c['ble'] for c in choices)) != 4:
                    raise ValueError('Select four different anchors')
                known = {b['ble']: b for b in self.boards}
                boards = [dict(known[c['ble']], z=0.0) for c in choices]
                self.preview = None
                self.preview_at = None
                self.reserved = True  # Also recover an unconfirmed ownership request.
                try:
                    self.guard.reserve(True)
                except Exception:
                    self.reserved = self.dirty or self.guard.claimed
                    raise
                self.update('measuring', 'Measuring distances between anchors...', preview=None, token=None)
                rows = ble.call('measure', self.check, boards)
                preview = solve_layout(boards, rows)
                self.check()
                self.preview = preview
                self.preview_at = monotonic()
                self.update('preview', 'Review the layout, then save it to all four anchors.',
                            preview=preview, token=str(uuid.uuid4()))
            elif action == 'apply':
                if self.preview_at is not None and monotonic() - self.preview_at > 120 and not self.dirty:
                    self.release()
                if (self.preview is None or self.preview_at is None or monotonic() - self.preview_at > 120
                        or request.get('token') != self.state['token']):
                    raise ValueError('Preview expired or changed; measure again')
                self.check()
                self.update('saving', 'Saving and verifying anchor coordinates...')
                ble.call('save', self.check, self.preview['positions'], self.mark_write)
                self.check()
                # Keep the recovery warning until saved coordinates are verified.
                self.guard.reserve(False)
                self.dirty = self.reserved = False
                self.persist()
                self.preview = None
                self.preview_at = None
                self.update('complete', 'Layout saved and verified.',
                            token=None, preview=None)
        except Exception as error:
            logging.warning('Anchor calibration: %s', error)
            self.preview = None
            self.preview_at = None
            if self.cancel.is_set():
                try:
                    self.release()
                except Exception as release_error:
                    error = release_error
            # Unknown or partial coordinate writes require a successful retry.
            message = ('Calibration cancelled. The saved layout is unverified.'
                       if self.cancel.is_set() and self.dirty and not self.reserved else str(error))
            self.update('error', message, preview=None, token=None)
        finally:
            with self.lock:
                self.busy = False
                if not self.reserved and not self.guard.claimed:
                    self.guard.participants = None
                try:
                    self.persist()
                except OSError as error:
                    self.update('error', 'Cannot save calibration recovery state: %s' % error)

    def tick(self):
        with self.lock:
            if self.guard.claimed and not self.reserved and not self.busy:
                self.reserved = True
                self.update('error', 'Previous calibration is unfinished. Cancel or measure again.')
            if self.preview_at is not None and monotonic() - self.preview_at > 120 and not self.busy:
                self.request({'action': 'cancel'})

    def status(self, selection=None):
        with self.lock:
            if selection is not None:
                self.guard.select(selection)
            state = copy.deepcopy(self.state)
            state.update(reserved=self.reserved, dirty=self.dirty, busy=self.busy)
            state['selection'] = list(self.guard.selection)
            state['flight_state_timeout_ms'] = int(CONTROL_STATE_TIMEOUT * 1000)
            state['bridges'] = self.guard.connections()
            try:
                self.guard.check()
                state['grounded'] = True
            except ValueError as error:
                state['grounded'] = False
                state['ground_reason'] = str(error)
            return state
