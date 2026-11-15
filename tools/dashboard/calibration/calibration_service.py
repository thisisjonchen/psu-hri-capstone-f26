"""Bluetooth anchor calibration coordinated by the laptop dashboard server."""

import copy
import json
import logging
import os
import tempfile
from pathlib import Path
import threading
import uuid

from calibration.anchor_calibration import solve_layout, adjust_layout
from calibration.pans_ble import PansBLE
from time import monotonic


class CalibrationJournal:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock_file = open(str(self.path) + '.lock', 'a+b')
        try:
            if os.name == 'nt':
                import msvcrt
                self.lock_file.seek(0)
                msvcrt.locking(self.lock_file.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock_file.close()
            raise ValueError('Another laptop calibration service is using this recovery file')
        try:
            self.values = json.loads(self.path.read_text(encoding='utf-8')) if self.path.exists() else {}
            if not isinstance(self.values, dict):
                raise ValueError('Invalid calibration recovery file; restore it before starting')
        except Exception:
            self.close()
            raise

    def save(self, values):
        descriptor, temporary = tempfile.mkstemp(dir=self.path.parent, prefix='.calibration-')
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8', newline='\n') as stream:
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
    def __init__(self, state_path):
        self.journal = CalibrationJournal(state_path)
        saved = self.journal.values
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.busy = False
        self.dirty = bool(saved.get('incomplete_save', False))
        self.preview = None
        self.preview_at = None
        layout = saved.get('layout') if not self.dirty else None
        if layout:
            layout['verified'] = False
        self.boards = saved.get('recovery_boards', []) if self.dirty else saved.get('boards', [])
        self.state = {'stage': 'idle', 'message': '',
                      'boards': self.boards, 'preview': None, 'token': None, 'layout': layout}
        if self.dirty:
            self.state.update(stage='error', message='Previous save incomplete. Measure and save again.')
        self.persist()
        self.ble = PansBLE(progress=self.ble_progress)
        self.stopping = threading.Event()
        self.worker = None
        self.timer = threading.Thread(target=self.maintain, daemon=True)
        self.timer.start()

    def persist(self):
        self.journal.save({'incomplete_save': self.dirty,
                           'boards': self.boards, 'layout': self.state['layout'],
                           'recovery_boards': self.boards if self.dirty else []})

    def maintain(self):
        while not self.stopping.wait(0.25):
            self.tick()

    def close(self):
        self.stopping.set()
        self.cancel.set()
        self.timer.join()
        if self.worker is not None:
            self.worker.join()
        self.ble.close()
        self.journal.close()

    def update(self, stage, message, **values):
        with self.lock:
            self.state.update(stage=stage, message=message, **values)

    def ble_progress(self, message):
        with self.lock:
            self.state['message'] = message

    def check_cancelled(self):
        if self.cancel.is_set() or self.stopping.is_set():
            raise ValueError('Calibration cancelled')

    def request(self, request):
        if not isinstance(request, dict) or request.get('action') not in ('scan', 'measure', 'edit', 'apply', 'cancel'):
            raise ValueError('Unknown calibration action')
        action = request['action']
        with self.lock:
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

    def clear_preview(self):
        self.preview = None
        self.preview_at = None
        self.state.update(preview=None, token=None)

    def mark_write(self):
        with self.lock:
            self.state['layout'] = None
        self.dirty = True
        self.persist()

    def selected_boards(self, choices):
        addresses = [choice['ble'] for choice in choices]
        known = {board['ble']: board for board in self.boards if board['active']}
        if len(addresses) != 4 or len(set(addresses)) != 4:
            raise ValueError('Select four different anchors')
        if any(address not in known for address in addresses):
            raise ValueError('An assigned anchor is unavailable; scan again or replace it')
        boards = [known[address] for address in addresses]
        if len({board['id'] for board in boards}) != 4:
            raise ValueError('Select four distinct anchors')
        pans = {board['pan'] for board in boards if board['pan'] is not None}
        if sum(board['initiator'] for board in boards) != 1 or len(pans) > 1:
            raise ValueError('Select one initiator and three anchors on the same PAN')
        return [dict(board, z=0.0) for board in boards]

    def run(self, request):
        try:
            action = request['action']
            if action == 'cancel':
                self.clear_preview()
                self.update('idle', 'Cancelled. Layout unverified.' if self.dirty else 'Cancelled.')
                return
            self.check_cancelled()
            ble = self.ble
            if action == 'scan':
                if self.preview is not None:
                    raise ValueError('Cancel the current preview before scanning again')
                self.update('scanning', 'Scanning…', preview=None, token=None)
                self.boards = ble.call('scan', self.check_cancelled)
                message = '%d anchors found.' % len(self.boards)
                if not self.boards:
                    message = ('No anchors found. Check power and laptop Bluetooth, close other apps '
                               'connected to the anchors, then scan again.')
                if ble.scan_warnings:
                    message += ' Unreadable boards: ' + '; '.join(ble.scan_warnings)
                self.update('idle', message, boards=self.boards)
            elif action == 'measure':
                boards = self.selected_boards(request.get('anchors', []))
                self.preview = None
                self.preview_at = None
                self.update('measuring', 'Measuring…', preview=None, token=None)
                rows = ble.call('measure', self.check_cancelled, boards)
                logging.info('Anchor calibration ranges [meters, quality], corner order '
                             '[bottom left, bottom right, top right, top left] %s: %s',
                             [board['id'] for board in boards], json.dumps(rows, sort_keys=True))
                preview = solve_layout(boards, rows)
                self.check_cancelled()
                self.preview = preview
                self.preview_at = monotonic()
                self.update('preview', '', preview=preview, token=str(uuid.uuid4()))
            elif action == 'edit':
                if self.dirty:
                    raise ValueError('Previous save incomplete. Measure and save again.')
                boards = self.selected_boards(request.get('anchors', []))
                layout = self.state['layout']
                saved = {(p['ble'], p['id']): p for p in layout['positions']} if layout else {}
                positions = []
                for board in boards:
                    position = saved.get((board['ble'], board['id']), {})
                    positions.append(dict(board, x=position.get('x'), y=position.get('y')))
                self.preview = {'positions': positions}
                self.preview_at = monotonic()
                self.update('preview', '', preview=self.preview, token=str(uuid.uuid4()))
            elif action == 'apply':
                if (self.preview is None or self.preview_at is None or monotonic() - self.preview_at > 120
                        or request.get('token') != self.state['token']):
                    raise ValueError('Preview expired or changed. Select Measure or Adjust again.')
                positions = self.preview['positions']
                if 'positions' in request or any(p['x'] is None or p['y'] is None for p in positions):
                    positions = adjust_layout(positions, request.get('positions'))
                self.update('saving', 'Saving…')
                ble.call('save', self.check_cancelled, positions, self.mark_write)
                self.check_cancelled()
                # Keep the recovery warning until saved coordinates are verified.
                self.dirty = False
                layout = {'revision': self.state['token'], 'frame': 'map', 'verified': True,
                          'positions': copy.deepcopy(positions)}
                self.preview = None
                self.preview_at = None
                self.update('complete', 'Layout saved.',
                            token=None, preview=None, layout=layout)
        except Exception as error:
            logging.warning('Anchor calibration: %s', error)
            self.preview = None
            self.preview_at = None
            # Unknown or partial coordinate writes require a successful retry.
            message = ('Cancelled. Layout unverified.'
                       if self.cancel.is_set() and self.dirty else str(error))
            self.update('error', message, preview=None, token=None)
        finally:
            with self.lock:
                self.busy = False
                try:
                    self.persist()
                except OSError as error:
                    self.update('error', 'Cannot save calibration recovery state: %s' % error)

    def tick(self):
        with self.lock:
            if self.preview_at is not None and monotonic() - self.preview_at > 120 and not self.busy:
                self.request({'action': 'cancel'})

    def status(self):
        with self.lock:
            state = copy.deepcopy(self.state)
            state.update(dirty=self.dirty, busy=self.busy)
            return state
