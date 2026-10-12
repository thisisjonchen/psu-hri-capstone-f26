"""Observe reachable drones and retain calibration participants until release."""
from __future__ import division

import json
import threading
import uuid

from localization.uwb_locator import monotonic

LOCK = '/pidrone/localization/calibration_owner/request'
OWNER = '/pidrone/localization/calibration_owner/status'
ACK = '/pidrone/localization/calibration_locked'
ANCHORS = '/pidrone/localization/anchors'


class BridgeGuard(object):
    def __init__(self, urls, owner, claimed=False, participants=None):
        import websocket
        self.websocket = websocket
        if not 1 <= len(urls) <= 2 or len(set(urls)) != len(urls):
            raise ValueError('Configure one or two distinct drone rosbridge URLs')
        if participants and any(url not in urls for url in participants):
            raise ValueError('Restore the saved participant bridge addresses before calibration recovery')
        self.stop_event = threading.Event()
        self.lock = threading.RLock()
        self.owner = owner
        self.claimed = claimed
        self.participants = list(participants) if participants else None
        self.peers = [{'url': url, 'socket': None, 'mode': None, 'mode_at': 0,
                       'locked': None, 'ack_at': 0, 'owner': None, 'owner_at': 0, 'request': None, 'anchor_seq': 0,
                       'anchors': None, 'anchors_at': 0, 'anchor_stamp': None} for url in urls]
        for peer in self.peers:
            worker = threading.Thread(target=self.watch, args=(peer,))
            worker.daemon = True
            worker.start()

    def send(self, peer, message):
        with self.lock:
            if peer['socket'] is None:
                raise ValueError('Reconnect calibration participant %s' % peer['url'])
            peer['socket'].send(json.dumps(message))

    def watch(self, peer):
        websocket = self.websocket
        while not self.stop_event.is_set():
            socket = None
            try:
                socket = websocket.create_connection(peer['url'], timeout=1)
                with self.lock:
                    peer['socket'] = socket
                self.send(peer, {'op': 'advertise', 'topic': LOCK, 'type': 'std_msgs/String'})
                for topic, kind in (('/pidrone/mode', 'pidrone_pkg/Mode'), (ACK, 'std_msgs/Bool'),
                                    (OWNER, 'std_msgs/String'),
                                    (ANCHORS, 'pidrone_pkg/AnchorLayout')):
                    self.send(peer, {'op': 'subscribe', 'topic': topic, 'type': kind, 'queue_length': 1})
                while not self.stop_event.is_set():
                    try:
                        data = json.loads(socket.recv())
                    except websocket.WebSocketTimeoutException:
                        continue
                    if data.get('op') != 'publish':
                        continue
                    self.receive(peer, data.get('topic'), data.get('msg', {}))
            except Exception:
                pass  # Readiness below describes the unavailable peer; reconnect is bounded.
            finally:
                with self.lock:
                    peer['socket'] = None
                    peer['mode_at'] = peer['ack_at'] = peer['owner_at'] = peer['anchors_at'] = 0
                    peer['anchor_stamp'] = None
                if socket is not None:
                    socket.close()
            self.stop_event.wait(1)

    def receive(self, peer, topic, message):
        now = monotonic()
        with self.lock:
            if topic == '/pidrone/mode':
                peer['mode'], peer['mode_at'] = message.get('mode'), now
            elif topic == ACK and type(message.get('data')) is bool:
                peer['locked'], peer['ack_at'] = message['data'], now
            elif topic == OWNER and isinstance(message.get('data'), type(u'')):
                try:
                    status = json.loads(message['data'])
                    if isinstance(status['owner'], type(u'')) and isinstance(status['request'], type(u'')):
                        peer['owner'], peer['request'], peer['owner_at'] = status['owner'], status['request'], now
                        if status['owner'] == self.owner:
                            self.claimed = True
                except (ValueError, KeyError, TypeError):
                    pass
            elif topic == ANCHORS:
                try:
                    stamp = message['header']['stamp']
                    stamp = int(stamp['secs']) * 1000000000 + int(stamp['nsecs'])
                    if stamp <= 0 or (peer['anchor_stamp'] is not None and stamp <= peer['anchor_stamp']):
                        return
                    peer['anchor_stamp'] = stamp
                    peer['anchors'], peer['anchors_at'] = message, now
                    peer['anchor_seq'] += 1
                except (KeyError, TypeError, ValueError):
                    pass

    def selected(self, include_new=True):
        with self.lock:
            peers = [p for p in self.peers if
                     (self.participants is not None and p['url'] in self.participants) or
                     ((include_new or self.participants is None) and p['socket'] is not None)]
            if not peers:
                raise ValueError('Connect at least one drone rosbridge before calibration')
            return peers

    def prepare(self):
        with self.lock:
            # A retry can add a newly connected drone, but cannot drop an existing participant.
            self.participants = [p['url'] for p in self.selected()]
            return list(self.participants)

    def check(self, locked=False, allow_other=False):
        now = monotonic()
        with self.lock:
            for peer in self.selected(include_new=not allow_other):
                if peer['socket'] is None or not 0 <= now - peer['mode_at'] <= 0.5:
                    raise ValueError('Fresh flight state required from %s; start its flight controller' % peer['url'])
                if peer['mode'] != 'DISARMED':
                    raise ValueError('Disarm %s before calibration' % peer['url'])
                if not allow_other and (not 0 <= now - peer['owner_at'] <= 0.5 or peer['owner'] is None):
                    raise ValueError('Fresh calibration ownership required from %s; update its flight controller' % peer['url'])
                if not allow_other and peer['owner'] not in ('', self.owner):
                    raise ValueError('Anchor calibration is owned by another coordinator')
                if locked and (peer['owner'] != self.owner or peer['locked'] is not True
                               or not 0 <= now - peer['ack_at'] <= 0.5):
                    raise ValueError('Every participating flight controller must acknowledge the calibration lock; measure again if another drone joined')

    def set_locked(self, value):
        with self.lock:
            had_claim = self.claimed or any(p['owner'] == self.owner for p in self.peers)
        try:
            self.change_lock(value)
        except Exception:
            if value and self.claimed and not had_claim:
                # A competing coordinator may have won one master. Release only our claims.
                try:
                    self.change_lock(False)
                except Exception:
                    pass  # Keep the reservation if either release is unconfirmed.
            raise

    def change_lock(self, value):
        if value and self.participants is None:
            self.prepare()
        self.check(allow_other=not value)
        nonce = str(uuid.uuid4())
        with self.lock:
            if value:
                self.claimed = True
            peers = self.selected(include_new=False)
            for peer in peers:
                peer['ack_at'] = peer['owner_at'] = 0
                request = json.dumps({'owner': self.owner, 'action': 'acquire' if value else 'release', 'request': nonce})
                self.send(peer, {'op': 'publish', 'topic': LOCK, 'msg': {'data': request}})
        deadline = monotonic() + 3
        while monotonic() < deadline:
            # Wait for new acknowledgments after publishing; old ownership cannot confirm a change.
            now = monotonic()
            with self.lock:
                fresh = all(p['request'] == nonce and p['owner_at'] > 0 and p['ack_at'] > 0 and
                            0 <= now - p['owner_at'] <= 0.5 and 0 <= now - p['ack_at'] <= 0.5
                            for p in peers)
                if fresh:
                    self.check(allow_other=not value)
                    if all((p['owner'] == self.owner and p['locked'] is True) if value else
                           p['owner'] != self.owner for p in peers):
                        self.claimed = value
                        if not value:
                            self.participants = None
                        return
                for peer in peers:
                    if peer['socket'] is None or not 0 <= now - peer['mode_at'] <= 0.5 or peer['mode'] != 'DISARMED':
                        raise ValueError('Fresh disarmed flight state required from %s' % peer['url'])
            if self.stop_event.wait(0.05):
                break
        raise ValueError('Flight-controller calibration lock was not acknowledged by every participant')

    def sequences(self):
        with self.lock:
            return {p['url']: p['anchor_seq'] for p in self.selected(include_new=False)}

    def verify_layout(self, positions, baseline, guard, timeout=8):
        expected = {p['id']: p for p in positions}
        deadline = monotonic() + timeout
        while monotonic() < deadline:
            guard()
            matched = []
            with self.lock:
                for peer in self.selected(include_new=False):
                    sequence = baseline.get(peer['url'], peer['anchor_seq'])
                    report = peer['anchors'] or {}
                    ids, actual = report.get('anchor_ids', []), report.get('positions', [])
                    valid = (peer['anchor_seq'] > sequence and monotonic() - peer['anchors_at'] <= 0.5
                             and report.get('header', {}).get('frame_id', '').lstrip('/') == 'map'
                             and len(ids) == len(actual) == 4 and set(ids) == set(expected))
                    if valid:
                        valid = all(all(abs(float(point[axis]) - expected[key][axis]) <= 0.02
                                        for axis in ('x', 'y', 'z')) for key, point in zip(ids, actual))
                    matched.append(valid)
            if all(matched):
                return
            if self.stop_event.wait(0.1):
                break
        raise ValueError('Every participating tag must report the four newly saved anchors; flight stays inhibited')

    def close(self):
        self.stop_event.set()
        with self.lock:
            for peer in self.peers:
                if peer['socket'] is not None:
                    peer['socket'].close()
