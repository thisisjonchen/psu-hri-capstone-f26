"""Observe and lock both independent ROS masters through their rosbridges."""
from __future__ import division

import json
import threading

from localization.uwb_locator import monotonic

LOCK = '/pidrone/localization/calibration_lock'
ACK = '/pidrone/localization/calibration_locked'
ANCHORS = '/pidrone/localization/anchors'


class BridgeGuard(object):
    def __init__(self, urls):
        import websocket
        self.websocket = websocket
        if len(urls) != 2 or len(set(urls)) != 2:
            raise ValueError('Configure two distinct drone rosbridge URLs')
        self.stop_event = threading.Event()
        self.lock = threading.RLock()
        self.peers = [{'url': url, 'socket': None, 'mode': None, 'mode_at': 0,
                       'locked': None, 'ack_at': 0, 'anchor_seq': 0,
                       'anchors': None, 'anchors_at': 0, 'anchor_stamp': None} for url in urls]
        for peer in self.peers:
            worker = threading.Thread(target=self.watch, args=(peer,))
            worker.daemon = True
            worker.start()

    def send(self, peer, message):
        with self.lock:
            if peer['socket'] is None:
                raise ValueError('Connect both drone rosbridges before calibration')
            peer['socket'].send(json.dumps(message))

    def watch(self, peer):
        websocket = self.websocket
        while not self.stop_event.is_set():
            socket = None
            try:
                socket = websocket.create_connection(peer['url'], timeout=1)
                with self.lock:
                    peer['socket'] = socket
                self.send(peer, {'op': 'advertise', 'topic': LOCK, 'type': 'std_msgs/Bool', 'latch': True})
                for topic, kind in (('/pidrone/mode', 'pidrone_pkg/Mode'), (ACK, 'std_msgs/Bool'),
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
                    peer['mode_at'] = peer['ack_at'] = peer['anchors_at'] = 0
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

    def check(self, locked=False):
        now = monotonic()
        with self.lock:
            for peer in self.peers:
                if peer['socket'] is None or not 0 <= now - peer['mode_at'] <= 0.5:
                    raise ValueError('Fresh flight state required from both drones')
                if peer['mode'] != 'DISARMED':
                    raise ValueError('Disarm both drones before calibration')
                if locked and (peer['locked'] is not True or not 0 <= now - peer['ack_at'] <= 0.5):
                    raise ValueError('Both flight controllers must acknowledge the calibration lock')

    def set_locked(self, value):
        self.check()
        with self.lock:
            for peer in self.peers:
                peer['ack_at'] = 0
                self.send(peer, {'op': 'publish', 'topic': LOCK, 'msg': {'data': value}})
        deadline = monotonic() + 3
        while monotonic() < deadline:
            self.check()
            with self.lock:
                if all(p['locked'] is value and p['ack_at'] > 0 for p in self.peers):
                    return
            if self.stop_event.wait(0.05):
                break
        raise ValueError('Flight-controller calibration lock was not acknowledged on both drones')

    def sequences(self):
        with self.lock:
            return [p['anchor_seq'] for p in self.peers]

    def verify_layout(self, positions, baseline, guard, timeout=8):
        expected = {p['id']: p for p in positions}
        deadline = monotonic() + timeout
        while monotonic() < deadline:
            guard()
            matched = []
            with self.lock:
                for peer, sequence in zip(self.peers, baseline):
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
        raise ValueError('Both tags must report the four newly saved anchors; flight stays inhibited')

    def close(self):
        self.stop_event.set()
        with self.lock:
            for peer in self.peers:
                if peer['socket'] is not None:
                    peer['socket'].close()
